/**
 * @file    ec_mode_manager.h
 * @brief   模式管理器 —— 有限状态机驱动的工作模式切换框架
 * @details 本模块实现了一个通用的工作模式（Mode）管理框架，类似于 Android
 *          Activity 或嵌入式系统中的状态机管理。它允许系统定义多个工作模式，
 *          每个模式有独立的生命周期（初始化的→启动→运行→停止），并支持
 *          运行时在模式之间切换。
 *
 *          【设计模式：有限状态机 + 策略模式】
 *          每个模式(ec_mode_t)是一个"策略"，包含四个生命周期函数：
 *          1. init：初始化（在注册时调用，只调用一次）
 *          2. start：启动（进入该模式时调用）
 *          3. run：运行（模式运行期间周期性调用）
 *          4. stop：停止（离开该模式时调用）
 *
 *          【状态转换图】
 *          初始化 → STOPPED → (用户选择模式) → RUNNING → (检测到故障) → FAULT
 *                      ↑                                    |
 *                      +----------(用户停止)----------------+
 *                      ↑                                    |
 *                      +----------(故障恢复)----------------+
 *
 *          【核心能力】
 *          - 支持最多 EC_MODE_MAX_COUNT(16) 个模式
 *          - 运行时模式选择（上/下翻）
 *          - 一键启动/停止当前选中模式
 *          - 故障状态检测与上报
 *          - 提供选中模式和活跃模式的名称查询
 *
 *          【典型应用场景】
 *          智能车可以通过模式管理器组织：
 *          - 待机模式(STANDBY)：等待启动指令，功耗最低
 *          - 寻迹模式(LINE_FOLLOW)：正常运行巡线算法
 *          - 遥控模式(RC_CONTROL)：手动遥控控制
 *          - 校准模式(CALIBRATE)：传感器校准
 *          - 演示模式(DEMO)：自动演示各种功能
 */
#ifndef EC_MODE_MANAGER_H
#define EC_MODE_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 最大支持的模式数量 */
#define EC_MODE_MAX_COUNT 16u

/**
 * @brief 模式初始化函数类型
 * @details 在模式注册时调用，用于初始化模式所需的资源。
 *          返回 false 表示初始化失败，该模式不会被注册。
 * @param context 用户自定义上下文
 * @return true=初始化成功，false=初始化失败
 */
typedef bool (*ec_mode_init_fn)(void *context);

/**
 * @brief 模式运行函数类型
 * @details 通用的模式生命周期函数签名，用于 start/run/stop 三个回调。
 * @param now_ms 当前系统时间（毫秒）
 * @param context 用户自定义上下文
 */
typedef void (*ec_mode_fn)(uint32_t now_ms, void *context);

/**
 * @brief 模式管理器状态枚举
 * @details 定义模式管理器的三种全局状态：
 *          - EC_MODE_STOPPED：停止状态，没有模式在运行，可以自由选择模式
 *          - EC_MODE_RUNNING：运行状态，当前有模式正在运行
 *          - EC_MODE_FAULT：故障状态，模式运行过程中检测到错误
 */
typedef enum
{
    EC_MODE_STOPPED = 0,    /**< 停止状态 - 空闲，可进行模式选择 */
    EC_MODE_RUNNING,        /**< 运行状态 - 有一个模式正在运行 */
    EC_MODE_FAULT           /**< 故障状态 - 运行中的模式报告了错误 */
} ec_mode_state_t;

/**
 * @brief 模式描述结构体
 * @details 定义一个工作模式。每个模式通过四个生命周期回调函数来管理：
 *
 *          【生命周期】
 *          注册 → init() [只调用一次]
 *                   ↓ (init 返回 true)
 *          等待 → start() [进入模式时调用]
 *                   ↓
 *          运行 → run()   [周期性调用，持续运行]
 *                   ↓
 *          停止 → stop()  [离开模式时调用]
 *
 * @note name 指针必须保持持久有效（通常指向字符串常量或静态字符串）
 */
typedef struct
{
    const char *name;           /**< 模式名称，用于显示和调试 */
    ec_mode_init_fn init;       /**< 初始化回调（注册时调用）。返回 false 阻止注册 */
    ec_mode_fn start;           /**< 启动回调（进入模式时调用）。用于配置硬件、初始化状态 */
    ec_mode_fn run;             /**< 运行回调（周期执行）。模式的核心逻辑 */
    ec_mode_fn stop;            /**< 停止回调（离开模式时调用）。用于清理资源、停止硬件 */
    void *context;              /**< 传递给所有回调函数的上下文指针 */
} ec_mode_t;

/**
 * @brief 模式管理器结构体
 * @details 管理所有模式的核心容器。采用静态数组存储模式，最多支持 16 个。
 *
 *          【重要字段说明】
 *          - selected：用户当前选中的模式（通过 select_next/select_previous 切换）
 *          - active：当前正在运行的模式（当 state=RUNNING 时有效）
 *          - state：管理器当前所处的状态
 *
 *          【模式选择与启动的分离】
 *          selected 和 active 可能是不同的模式：
 *          selected 是"用户高亮"的模式，active 是"正在运行"的模式。
 *          只有 state=STOPPED 时才能切换 selected。
 *          调用 start() 时，selected 复制到 active。
 */
typedef struct
{
    ec_mode_t modes[EC_MODE_MAX_COUNT]; /**< 模式数组，静态分配 */
    uint8_t count;                      /**< 已注册的模式数量 */
    uint8_t selected;                   /**< 当前选中的模式索引（通过上下键选择） */
    uint8_t active;                     /**< 当前正在运行的模式索引（仅 state=RUNNING 时有效） */
    ec_mode_state_t state;              /**< 管理器全局状态 */
} ec_mode_manager_t;

/**
 * @brief 初始化模式管理器
 * @param manager 指向模式管理器实例的指针，传入 NULL 则无操作
 */
void ec_mode_manager_init(ec_mode_manager_t *manager);

/**
 * @brief 注册一个模式到管理器
 * @details 将模式添加到管理器的模式数组中。注册时会调用模式的 init 回调，
 *          如果 init 返回 false，则注册失败。
 * @param manager 模式管理器指针
 * @param mode 待注册的模式描述结构体指针
 * @return true=注册成功，false=注册失败（参数无效、数组已满或 init 失败）
 */
bool ec_mode_manager_add(ec_mode_manager_t *manager, const ec_mode_t *mode);

/**
 * @brief 选择下一个模式（向下翻）
 * @details 将选中模式索引向前移动（循环），只能在 STOPPED 状态下操作。
 * @param manager 模式管理器指针
 */
void ec_mode_manager_select_next(ec_mode_manager_t *manager);

/**
 * @brief 选择上一个模式（向上翻）
 * @details 将选中模式索引向后移动（循环），只能在 STOPPED 状态下操作。
 * @param manager 模式管理器指针
 */
void ec_mode_manager_select_previous(ec_mode_manager_t *manager);

/**
 * @brief 启动当前选中的模式
 * @details 将当前选中的模式设置为运行状态：
 *          1. 将 selected 复制到 active
 *          2. 设置 state 为 RUNNING
 *          3. 调用模式的 start 回调
 * @param manager 模式管理器指针
 * @param now_ms 当前系统时间（毫秒）
 * @return true=启动成功，false=启动失败（无效状态、无模式等）
 */
bool ec_mode_manager_start(ec_mode_manager_t *manager, uint32_t now_ms);

/**
 * @brief 停止当前正在运行的模式
 * @details 停止正在运行的模式：
 *          1. 调用模式的 stop 回调（如果提供了）
 *          2. 设置 state 为 STOPPED（除非已经进入 FAULT 状态）
 * @param manager 模式管理器指针
 * @param now_ms 当前系统时间（毫秒）
 */
void ec_mode_manager_stop(ec_mode_manager_t *manager, uint32_t now_ms);

/**
 * @brief 运行当前活跃模式的核心逻辑
 * @details 在 RUNNING 状态下周期性调用此函数，驱动活跃模式的 run 回调。
 *          这是模式管理器的"心跳"函数。
 * @param manager 模式管理器指针
 * @param now_ms 当前系统时间（毫秒）
 */
void ec_mode_manager_run(ec_mode_manager_t *manager, uint32_t now_ms);

/**
 * @brief 获取当前选中模式的名称
 * @param manager 模式管理器指针
 * @return 模式名称字符串；如果没有注册任何模式则返回 "NO TASK"
 */
const char *ec_mode_manager_selected_name(const ec_mode_manager_t *manager);

/**
 * @brief 获取当前正在运行的模式名称
 * @param manager 模式管理器指针
 * @return 模式名称字符串；如果没有模式在运行则返回 "NO TASK"
 */
const char *ec_mode_manager_active_name(const ec_mode_manager_t *manager);

#endif
