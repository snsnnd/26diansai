/**
 * @file    ec_mode_manager.c
 * @brief   模式管理器 —— 有限状态机驱动的工作模式切换框架 - 实现文件
 * @details 本文件实现了模式管理器的所有功能，包括模式的注册、选择、启停控制
 *          和运行驱动。管理器的核心是一个有限状态机，控制着整个系统的工作模式
 *          切换。
 *
 *          【状态机实现说明】
 *          管理器的状态转换受严格的条件约束：
 *          1. STOPPED → RUNNING：只有通过 ec_mode_manager_start() 显式启动
 *          2. RUNNING → STOPPED：通过 ec_mode_manager_stop() 显式停止，或
 *             在 stop 回调中检测到故障时进入 FAULT 状态
 *          3. RUNNING → FAULT：可以在模式的 run 或 stop 回调中设置 state = FAULT
 *          4. FAULT → STOPPED：通过 ec_mode_manager_stop() 尝试停止
 *
 *          【模式选择与启动的分离设计】
 *          selected 和 active 的分离是为了支持"预览→启动"的用户体验：
 *          - 用户可以在 STOPPED 状态下自由翻阅所有模式（修改 selected）
 *          - 按确认键后，selected 模式被启动（selected → active）
 *          - 运行中不能切换模式，必须先停止再选择
 */
#include "framework/ec_mode_manager.h"

#include <stddef.h>

/**
 * @brief 初始化模式管理器
 * @details 将所有字段清零，设置状态为 EC_MODE_STOPPED。
 *          管理器启动时没有任何模式注册。
 * @param manager 管理器实例指针
 */
void ec_mode_manager_init(ec_mode_manager_t *manager)
{
    if (manager == NULL) return;
    manager->count = 0u;
    manager->selected = 0u;
    manager->active = 0u;
    manager->state = EC_MODE_STOPPED;
}

/**
 * @brief 注册一个模式到管理器
 * @details 注册流程：
 *          1. 参数合法性检查（指针非空、名称非空、未满）
 *          2. 调用模式的 init 回调（如果提供了）
 *          3. init 成功 → 将模式复制到数组中，count++
 *          4. init 失败 → 返回 false，不注册
 *
 *          【为什么使用结构体拷贝而不是指针引用？】
 *          通过 *mode = *mode 将模式结构体拷贝到 mode 数组中，这样调用者
 *          可以释放或复用原始的 mode 变量，模式管理器拥有自己独立的数据副本。
 *
 *          【init 回调的设计意图】
 *          init 在注册时而非启动时调用，用于执行一次性的资源分配和检查。
 *          如果资源不足或硬件故障，init 返回 false，该模式就不会被注册，
 *          从而避免运行时出现意外错误。
 *
 * @param manager 管理器指针
 * @param mode 待注册的模式描述
 * @return true=注册成功
 */
bool ec_mode_manager_add(ec_mode_manager_t *manager, const ec_mode_t *mode)
{
    if (manager == NULL || mode == NULL || mode->name == NULL ||
        manager->count >= EC_MODE_MAX_COUNT)
    {
        return false;
    }
    /* 调用 init 回调进行资源初始化和可用性检查 */
    if (mode->init != NULL && !mode->init(mode->context))
    {
        return false;   /* 初始化失败，拒绝注册 */
    }
    /* 结构体拷贝：将模式数据复制到管理器数组中 */
    manager->modes[manager->count++] = *mode;
    return true;
}

/**
 * @brief 选择下一个模式（向下翻页）
 * @details 选中索引循环增加 (selected + 1) % count。
 *          只能在 STOPPED 状态下操作，防止运行时切换导致状态不一致。
 * @param manager 管理器指针
 */
void ec_mode_manager_select_next(ec_mode_manager_t *manager)
{
    if (manager == NULL || manager->state != EC_MODE_STOPPED || manager->count == 0u) return;
    manager->selected = (uint8_t)((manager->selected + 1u) % manager->count);
}

/**
 * @brief 选择上一个模式（向上翻页）
 * @details 选中索引循环减少。当在顶部时跳到底部 (count - 1)。
 *          只能在 STOPPED 状态下操作。
 * @param manager 管理器指针
 */
void ec_mode_manager_select_previous(ec_mode_manager_t *manager)
{
    if (manager == NULL || manager->state != EC_MODE_STOPPED || manager->count == 0u) return;
    manager->selected = (manager->selected == 0u)
        ? (uint8_t)(manager->count - 1u) : (uint8_t)(manager->selected - 1u);
}

/**
 * @brief 启动当前选中的模式
 * @details 启动一个模式的完整流程：
 *          1. 检查前置条件：管理器存在、处于 STOPPED 状态、有已注册的模式、
 *             选中索引有效
 *          2. 将 selected 赋值给 active——用户选中的模式成为"正在运行的"模式
 *          3. 设置 state 为 RUNNING
 *          4. 调用模式的 start 回调（如果提供了）
 *          5. 检查 start 后是否仍为 RUNNING（start 回调可能将状态设为 FAULT）
 *
 *          【为什么 start 回调可能改变状态？】
 *          start 回调中可能初始化硬件，如果硬件初始化失败，可以将 state
 *          设置为 EC_MODE_FAULT，表示硬件故障。此时 ec_mode_manager_start
 *          将返回 false，通知调用者启动失败。
 *
 * @param manager 管理器指针
 * @param now_ms 当前系统时间
 * @return true=启动成功且状态正常，false=启动失败或进入故障状态
 */
bool ec_mode_manager_start(ec_mode_manager_t *manager, uint32_t now_ms)
{
    ec_mode_t *mode;

    if (manager == NULL || manager->state != EC_MODE_STOPPED ||
        manager->count == 0u || manager->selected >= manager->count)
        return false;
    manager->active = manager->selected;    /* 选中→活跃：模式正式生效 */
    mode = &manager->modes[manager->active];
    manager->state = EC_MODE_RUNNING;        /* 先置为运行态 */
    if (mode->start != NULL) mode->start(now_ms, mode->context);  /* 调用启动回调 */
    return manager->state == EC_MODE_RUNNING; /* 检查是否保持运行状态 */
}

/**
 * @brief 停止当前正在运行的模式
 * @details 停止模式的完整流程：
 *          1. 检查前置条件（跳过已处于 STOPPED 的状态）
 *          2. 处理异常：如果 active 索引越界，直接设回 STOPPED
 *          3. 调用模式的 stop 回调（如果提供了），进行资源清理和硬件停止
 *          4. 除非 stop 回调中设为了 FAULT，否则将状态设为 STOPPED
 *
 *          【关于 FAULT 状态的保留】
 *          如果运行的模式检测到故障并在 stop 回调中将 state 设为 FAULT，
 *          管理器会保留这个 FAULT 状态，不自动恢复为 STOPPED。
 *          这样上层的错误处理逻辑可以感知到故障并进行相应处理。
 *          这是一种"故障冻结"设计模式。
 *
 * @param manager 管理器指针
 * @param now_ms 当前系统时间
 */
void ec_mode_manager_stop(ec_mode_manager_t *manager, uint32_t now_ms)
{
    ec_mode_t *mode;

    if (manager == NULL || manager->state == EC_MODE_STOPPED ||
        manager->count == 0u)
        return;
    if (manager->active >= manager->count)
    {
        /* active 索引异常越界，强制恢复 */
        manager->state = EC_MODE_STOPPED;
        return;
    }
    mode = &manager->modes[manager->active];
    if (mode->stop != NULL) mode->stop(now_ms, mode->context);  /* 调用停止回调 */
    /* 如果 stop 回调没有设置 FAULT 状态，则切换到 STOPPED */
    if (manager->state != EC_MODE_FAULT)
        manager->state = EC_MODE_STOPPED;
}

/**
 * @brief 运行当前活跃模式
 * @details 在管理器的 RUNNING 状态下，周期性调用此函数来执行当前模式的
 *          核心逻辑。这是模式管理器在运行时的"心跳"函数，应由调度器
 *          或主循环以固定周期调用。
 *
 * @param manager 管理器指针
 * @param now_ms 当前系统时间
 */
void ec_mode_manager_run(ec_mode_manager_t *manager, uint32_t now_ms)
{
    ec_mode_t *mode;

    if (manager == NULL || manager->state != EC_MODE_RUNNING ||
        manager->count == 0u || manager->active >= manager->count)
        return;
    mode = &manager->modes[manager->active];
    if (mode->run != NULL) mode->run(now_ms, mode->context);
}

/**
 * @brief 获取当前选中模式的名称
 * @details 用于在 OLED 屏幕上显示用户当前选中的模式名称。
 *          如果没有注册任何模式或者选中索引无效，返回 "NO TASK"。
 *
 * @param manager 管理器指针
 * @return 模式名称字符串
 */
const char *ec_mode_manager_selected_name(const ec_mode_manager_t *manager)
{
    if (manager == NULL || manager->count == 0u ||
        manager->selected >= manager->count)
        return "NO TASK";
    return manager->modes[manager->selected].name;
}

/**
 * @brief 获取当前正在运行的模式名称
 * @details 用于在 OLED 屏幕上显示当前正在运行的模式名。
 *          当没有模式在运行时，返回 "NO TASK"。
 *
 * @param manager 管理器指针
 * @return 模式名称字符串
 */
const char *ec_mode_manager_active_name(const ec_mode_manager_t *manager)
{
    if (manager == NULL || manager->count == 0u ||
        manager->active >= manager->count)
        return "NO TASK";
    return manager->modes[manager->active].name;
}
