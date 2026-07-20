/**
 * @file    ec_parameter_menu.c
 * @brief   参数菜单 —— 运行时参数调整子菜单系统 - 实现文件
 * @details 本文件实现了运行时参数编辑菜单的核心功能。这是智能车调试和
 *          调参的重要工具，允许用户在 OLED 菜单界面中直接修改 PID 系数、
 *          速度限制、传感器阈值等关键参数，实现"所见即所得"的调试体验。
 *
 *          【核心机制：通过 void* 指针直接操作变量】
 *          参数菜单不维护自己的数据副本，而是通过 value 指针直接操作
 *          应用程序中实际使用的变量。这意味着：
 *          1. 用户修改参数后，控制算法立即使用新值
 *          2. 不需要"应用"或"同步"步骤
 *          3. 调参过程是实时生效的
 *
 *          【值读写抽象层】
 *          get_value() 和 set_value() 构成了一层抽象，统一了不同数据类型
 *          的访问方式。所有数据在内部都以 float 进行运算，在读写时进行
 *          类型转换。这种设计简化了参数编辑逻辑，但引入了一些类型转换的
 *          注意事项（见具体函数说明）。
 */
#include "framework/ec_parameter_menu.h"

#include <stdio.h>

/**
 * @brief 读取参数的当前值（统一以 float 返回）
 * @details 从 item->value 指针指向的内存中读取参数值，根据参数类型
 *          进行适当的类型转换后以 float 返回。
 *
 *          【类型转换注意事项】
 *          - int8_t/int16_t 先解引用再隐式转换为 float，可能丢失精度
 *            但用于显示和简单运算足够了
 *          - uint16_t 转换为 float 可能丢失精度（float 只有23位尾数，
 *            而 uint16_t 最大65535完全可表示，不会有精度损失）
 *          - float 直接读取并返回
 *          - bool 转换为 1.0f 或 0.0f
 *
 * @param item 参数条目指针
 * @return 当前参数值的 float 表示。无效参数返回 0.0f
 */
static float get_value(const ec_parameter_item_t *item)
{
    if (item == NULL || item->value == NULL) return 0.0f;
    switch (item->type)
    {
        case EC_PARAM_INT8: return (float)*(int8_t *)item->value;
        case EC_PARAM_INT16: return (float)*(int16_t *)item->value;
        case EC_PARAM_UINT16: return (float)*(uint16_t *)item->value;
        case EC_PARAM_FLOAT: return *(float *)item->value;
        case EC_PARAM_BOOL: return *(bool *)item->value ? 1.0f : 0.0f;
        default: return 0.0f;  /* EC_PARAM_ACTION 没有值可读 */
    }
}

/**
 * @brief 设置参数的值（从 float 转换后写入）
 * @details 将 float 格式的值经过类型转换后写入 item->value 指向的内存。
 *          写入前会进行范围裁剪，确保值在 [min_value, max_value] 区间内。
 *
 *          【范围裁剪策略】
 *          - 先裁剪再写入：如果用户试图将参数减小到 min 以下，会被自动
 *            拉到 min；同理，超过 max 则拉到 max
 *          - 这种"软限制"方式避免了参数越界导致的意外行为
 *
 *          【类型转换可能的风险】
 *          - float → int8_t：超出 [-128, 127] 的值会被截断
 *          - float → uint16_t：负数和超过 65535 的值会被截断
 *          - float → bool：大于等于 0.5 为 true，小于 0.5 为 false
 *          虽然范围裁剪提供了第一道防线，但类型转换本身也可能改变值，
 *          使用者需要注意参数的 min/max 设置与数据类型匹配。
 *
 * @param item 参数条目指针
 * @param value 要设置的 float 值
 */
static void set_value(ec_parameter_item_t *item, float value)
{
    if (item == NULL || item->value == NULL) return;
    /* 范围裁剪：确保值在允许范围内 */
    if (value < item->min_value) value = item->min_value;
    if (value > item->max_value) value = item->max_value;
    switch (item->type)
    {
        case EC_PARAM_INT8: *(int8_t *)item->value = (int8_t)value; break;
        case EC_PARAM_INT16: *(int16_t *)item->value = (int16_t)value; break;
        case EC_PARAM_UINT16: *(uint16_t *)item->value = (uint16_t)value; break;
        case EC_PARAM_FLOAT: *(float *)item->value = value; break;
        case EC_PARAM_BOOL: *(bool *)item->value = value >= 0.5f; break;
        default: break;     /* EC_PARAM_ACTION 没有值可设置 */
    }
}

/**
 * @brief 初始化参数菜单
 * @details 绑定参数条目数组到菜单控制器，初始为浏览模式(editing=false)，
 *          并设置脏标志(dirty=true)以触发首次显示。
 *
 * @param menu 参数菜单控制器指针
 * @param items 参数条目数组（由调用者提供，不拷贝数据）
 * @param count 参数条目数量
 */
void ec_parameter_menu_init(ec_parameter_menu_t *menu,
    ec_parameter_item_t *items, uint8_t count)
{
    if (menu == NULL) return;
    menu->items = items;
    menu->count = count;
    menu->selected = 0u;
    menu->editing = false;      /* 初始为浏览模式 */
    menu->dirty = true;         /* 首次显示需要刷新 */
}

/**
 * @brief 处理参数菜单的按键事件
 * @details 参数菜单有两种操作模式，按键在不同模式下行为不同：
 *
 *          【浏览模式 (editing = false)】
 *          ┌──────────┬──────────────────────────────┐
 *          │ 按键     │ 行为                          │
 *          ├──────────┼──────────────────────────────┤
 *          │ PREVIOUS │ 选中上一个参数                │
 *          │ NEXT     │ 选中下一个参数                │
 *          │ CONFIRM  │ 若为ACTION类型→执行动作        │
 *          │          │ 否则→切换到编辑模式            │
 *          └──────────┴──────────────────────────────┘
 *
 *          【编辑模式 (editing = true)】
 *          ┌──────────┬──────────────────────────────┐
 *          │ 按键     │ 行为                          │
 *          ├──────────┼──────────────────────────────┤
 *          │ PREVIOUS │ 参数值减小（减去 step）        │
 *          │ NEXT     │ 参数值增大（加上 step）        │
 *          │ CONFIRM  │ 退出编辑模式，回到浏览模式     │
 *          │ (BOOL)   │ PREVIOUS/NEXT 都切换 ON/OFF   │
 *          └──────────┴──────────────────────────────┘
 *
 *          【BOOL 类型的特殊处理】
 *          布尔类型在编辑模式下，PREVIOUS 和 NEXT 都执行相同的操作：
 *          切换值。这是因为布尔值只有两种状态，不需要步进增减的概念。
 *
 * @param menu 参数菜单控制器指针
 * @param key 按键类型
 */
void ec_parameter_menu_handle_key(ec_parameter_menu_t *menu, ec_menu_key_t key)
{
    ec_parameter_item_t *item;

    if (menu == NULL || menu->items == NULL || menu->count == 0u) return;
    item = &menu->items[menu->selected];

    if (key == EC_MENU_KEY_CONFIRM)
    {
        /* 确认键：处理动作类型或切换编辑模式 */
        if (item->type == EC_PARAM_ACTION)
        {
            /* ACTION 类型：执行动作（如保存参数到 Flash） */
            if (item->action != NULL) item->action(item->context);
        }
        else
        {
            /* 普通参数：切换编辑/浏览模式 */
            menu->editing = !menu->editing;
        }
    }
    else if (!menu->editing)
    {
        /* 浏览模式：在参数列表间移动 */
        if (key == EC_MENU_KEY_PREVIOUS)
            menu->selected = (menu->selected == 0u)
                ? (uint8_t)(menu->count - 1u) : (uint8_t)(menu->selected - 1u);
        else if (key == EC_MENU_KEY_NEXT)
            menu->selected = (uint8_t)((menu->selected + 1u) % menu->count);
    }
    else if (item->type == EC_PARAM_BOOL)
    {
        /* 编辑模式 + 布尔类型：翻转值 */
        set_value(item, get_value(item) < 0.5f ? 1.0f : 0.0f);
    }
    else if (key == EC_MENU_KEY_PREVIOUS)
    {
        /* 编辑模式 + 减小值 */
        set_value(item, get_value(item) - item->step);
    }
    else if (key == EC_MENU_KEY_NEXT)
    {
        /* 编辑模式 + 增大值 */
        set_value(item, get_value(item) + item->step);
    }
    menu->dirty = true;     /* 任何按键操作都标记为脏，需要刷新显示 */
}

/**
 * @brief 获取当前选中的参数条目
 * @details 返回当前选中参数的指针，供渲染函数读取参数名称和值进行显示。
 *
 * @param menu 参数菜单控制器指针
 * @return 参数条目指针，参数无效时返回 NULL
 */
const ec_parameter_item_t *ec_parameter_menu_current(const ec_parameter_menu_t *menu)
{
    if (menu == NULL || menu->items == NULL || menu->count == 0u) return NULL;
    return &menu->items[menu->selected];
}

/**
 * @brief 将参数值格式化为可读字符串
 * @details 根据参数类型选择合适的格式化方式：
 *
 *          【格式化规则】
 *          - EC_PARAM_ACTION：显示 "PRESS K3"（提示用户按确认执行动作）
 *          - EC_PARAM_BOOL：显示 "ON"（值 >= 0.5）或 "OFF"（值 < 0.5）
 *          - EC_PARAM_FLOAT：显示为保留2位小数的浮点数
 *          - EC_PARAM_INT8/INT16/UINT16：显示为有符号或无符号十进制整数
 *
 *          这种格式化的结果通常用于 OLED 或其他字符显示设备。
 *
 * @param item 参数条目指针
 * @param buffer 输出字符串缓冲区
 * @param capacity 缓冲区大小（包括 '\0' 终止符）
 */
void ec_parameter_menu_format_value(const ec_parameter_item_t *item,
    char *buffer, size_t capacity)
{
    if (buffer == NULL || capacity == 0u) return;
    if (item == NULL)
    {
        snprintf(buffer, capacity, "-");    /* 无效参数显示短横线 */
        return;
    }
    if (item->type == EC_PARAM_ACTION)
        snprintf(buffer, capacity, "PRESS K3");
    else if (item->type == EC_PARAM_BOOL)
        snprintf(buffer, capacity, "%s", get_value(item) > 0.5f ? "ON" : "OFF");
    else if (item->type == EC_PARAM_FLOAT)
        snprintf(buffer, capacity, "%.2f", (double)get_value(item));
    else
        snprintf(buffer, capacity, "%ld", (long)get_value(item));
}
