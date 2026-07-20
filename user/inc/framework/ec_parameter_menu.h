/**
 * @file    ec_parameter_menu.h
 * @brief   参数菜单 —— 运行时参数调整子菜单系统
 * @details 本模块实现了一个嵌套在菜单系统中的参数编辑子菜单，允许用户在
 *          运行时（Run-Time）调整各种系统参数，如 PID 系数、传感器阈值、
 *          速度限制等，而无需重新编译和烧录程序。
 *
 *          【设计模式：表单/属性页】
 *          参数菜单类似于 PC 软件中的"设置"或"属性"页面：
 *          1. 显示一个参数列表，用户可以用上下键翻阅
 *          2. 按确认键进入编辑模式，修改参数值
 *          3. 再次按确认键退出编辑模式
 *          4. 特殊的"动作(Action)"类型项在被确认时执行一个函数而非编辑值
 *
 *          【支持的参数类型】
 *          - EC_PARAM_INT8：8位有符号整数 (-128 ~ 127)
 *          - EC_PARAM_INT16：16位有符号整数 (-32768 ~ 32767)
 *          - EC_PARAM_UINT16：16位无符号整数 (0 ~ 65535)
 *          - EC_PARAM_FLOAT：单精度浮点数
 *          - EC_PARAM_BOOL：布尔值 (true/false)，显示为 ON/OFF
 *          - EC_PARAM_ACTION：动作类型，按确认执行函数（如保存参数到Flash）
 *
 *          【与 ec_menu 的关系】
 *          ec_parameter_menu 不是 ec_menu 的替代，而是它的补充：
 *          - ec_menu 是"外层菜单"，负责模式选择和启动/停止
 *          - ec_parameter_menu 是"内层菜单"，负责参数浏览和编辑
 *          - 两者共享相同的按键输入，但处理逻辑不同
 *
 *          【参数范围与步进】
 *          每个参数可以独立设置：
 *          - min_value/max_value：参数的允许范围，超出会被自动裁剪
 *          - step：每次按键调整的步进量，控制参数调节的精细度
 */
#ifndef EC_PARAMETER_MENU_H
#define EC_PARAMETER_MENU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "framework/ec_menu.h"

/**
 * @brief 参数类型枚举
 * @details 定义参数菜单支持的六种数据类型。每种类型在显示和编辑时
 *          使用不同的格式化/转换逻辑。
 */
typedef enum
{
    EC_PARAM_INT8 = 0,      /**< 8位有符号整数 */
    EC_PARAM_INT16,         /**< 16位有符号整数 */
    EC_PARAM_UINT16,        /**< 16位无符号整数 */
    EC_PARAM_FLOAT,         /**< 单精度浮点数 */
    EC_PARAM_BOOL,          /**< 布尔值（显示为 ON/OFF） */
    EC_PARAM_ACTION         /**< 动作类型，按确认立即执行操作 */
} ec_parameter_type_t;

/**
 * @brief 参数动作回调函数类型
 * @details 当参数类型为 EC_PARAM_ACTION 时，用户按确认键将调用此回调。
 *          典型用途：保存参数到 Flash、执行校准、重置为默认值等。
 * @param context 用户自定义上下文
 */
typedef void (*ec_parameter_action_fn)(void *context);

/**
 * @brief 参数条目描述结构体
 * @details 定义单个参数的所有属性，包括名称、类型、值的指针、范围限制等。
 *
 *          【关于 value 指针】
 *          value 是一个 void* 指针，指向实际存储参数值的变量。
 *          这样做的好处是：
 *          1. 参数菜单不拥有数据——它直接操作系统中已有的变量
 *          2. 当参数值被修改时，修改直接反映到原始变量上
 *          3. 其他模块（如控制算法）读取这些变量时，得到的就是用户调整后的值
 *
 *          【范围限制 (min/max)】
 *          使用 float 类型表示所有参数类型的范围限制，通过类型转换
 *          在 set_value 函数中处理具体的类型兼容性。
 */
typedef struct
{
    const char *name;               /**< 参数名称，用于显示 */
    ec_parameter_type_t type;       /**< 参数数据类型 */
    void *value;                    /**< 指向实际存储参数的变量的指针 */
    float min_value;                /**< 参数最小值，编辑时自动裁剪 */
    float max_value;                /**< 参数最大值，编辑时自动裁剪 */
    float step;                     /**< 调整步进量，每次按键增加或减少的值 */
    ec_parameter_action_fn action;  /**< 动作回调（仅 EC_PARAM_ACTION 类型使用） */
    void *context;                  /**< 传递给动作回调的上下文 */
} ec_parameter_item_t;

/**
 * @brief 参数菜单控制器结构体
 * @details 管理参数列表的浏览和编辑状态。
 *
 *          【两种操作模式】
 *          1. 浏览模式 (editing = false)：
 *             - 上下键：在参数列表中移动选中项
 *             - 确认键：进入编辑模式（如参数类型为 ACTION 则执行动作）
 *          2. 编辑模式 (editing = true)：
 *             - 上下键：增大/减小参数值
 *             - 确认键：退出编辑模式，保存修改
 *
 *          【dirty 标志】
 *          与 ec_menu 的 dirty 类似，标记参数发生了变更（选中项变动或值修改），
 *          通知外层刷新显示。
 */
typedef struct
{
    ec_parameter_item_t *items; /**< 参数条目数组指针（由调用者提供，不拷贝） */
    uint8_t count;              /**< 参数条目的数量 */
    uint8_t selected;           /**< 当前选中的参数索引 */
    bool editing;               /**< 编辑模式标志：true=正在编辑当前选中的参数 */
    bool dirty;                 /**< 脏标志，表示参数状态有变化需要刷新显示 */
} ec_parameter_menu_t;

/**
 * @brief 初始化参数菜单
 * @param menu 参数菜单控制器指针
 * @param items 参数条目数组（调用者需保证数组生命周期）
 * @param count 参数条目数量
 */
void ec_parameter_menu_init(ec_parameter_menu_t *menu,
    ec_parameter_item_t *items, uint8_t count);

/**
 * @brief 处理参数菜单的按键事件
 * @details 根据当前模式（浏览/编辑）和按键类型执行不同操作：
 *
 *          【浏览模式】
 *          - PREVIOUS/NEXT：在参数列表中移动选中项
 *          - CONFIRM：进入编辑模式，或（对 ACTION 类型）执行动作
 *
 *          【编辑模式】
 *          - PREVIOUS：减小参数值（减去 step）
 *          - NEXT：增大参数值（加上 step）
 *          - CONFIRM：退出编辑模式
 *          - 对于 BOOL 类型，PREVIOUS/NEXT 直接切换 ON/OFF
 *
 * @param menu 参数菜单控制器指针
 * @param key 按键类型
 */
void ec_parameter_menu_handle_key(ec_parameter_menu_t *menu, ec_menu_key_t key);

/**
 * @brief 获取当前选中的参数条目
 * @param menu 参数菜单控制器指针
 * @return 当前选中参数的指针，如无有效选中则返回 NULL
 */
const ec_parameter_item_t *ec_parameter_menu_current(const ec_parameter_menu_t *menu);

/**
 * @brief 格式化参数值为可显示的字符串
 * @details 根据参数类型将值格式化为适合在 OLED 上显示的文本：
 *          - EC_PARAM_FLOAT：格式为 "%.2f"（保留两位小数）
 *          - EC_PARAM_BOOL：显示为 "ON" 或 "OFF"
 *          - EC_PARAM_ACTION：显示为 "PRESS K3"
 *          - 其他整数类型：显示为十进制数 "%ld"
 * @param item 参数条目指针
 * @param buffer 输出缓冲区
 * @param capacity 缓冲区大小
 */
void ec_parameter_menu_format_value(const ec_parameter_item_t *item,
    char *buffer, size_t capacity);

#endif
