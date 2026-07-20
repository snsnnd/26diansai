/**
 * @file    ec_menu.h
 * @brief   菜单系统 —— 基于模式管理器的交互菜单框架
 * @details 本模块实现了一个轻量级的交互菜单系统，将按键输入、模式管理器和
 *          OLED显示三者连接起来。它充当了"控制器"的角色：
 *
 *          【MVC 架构的简化实现】
 *          - Model（模型）：ec_mode_manager_t，管理各工作模式的状态和数据
 *          - View（视图）：ec_menu_render_fn，由用户实现的渲染回调，
 *            负责将当前菜单状态绘制到 OLED 显示屏上
 *          - Controller（控制器）：ec_menu_t，处理按键输入，驱动模型变化，
 *            并触发视图更新
 *
 *          【按键映射】
 *          - EC_MENU_KEY_PREVIOUS (KEY1) → 选择上一个模式
 *          - EC_MENU_KEY_NEXT (KEY2)     → 选择下一个模式
 *          - EC_MENU_KEY_CONFIRM (KEY3)  → 确认/切换（启动或停止模式）
 *
 *          【渲染优化】
 *          菜单使用 dirty 标志和最小渲染周期来实现按需刷新：
 *          - 当有按键事件时设置 dirty = true，立即触发渲染
 *          - 即使没有按键事件，也以 render_period_ms 为周期强制刷新
 *          - 这种"事件驱动 + 周期轮询"的组合确保了 UI 响应性和低功耗的平衡
 */
#ifndef EC_MENU_H
#define EC_MENU_H

#include "framework/ec_mode_manager.h"

/**
 * @brief 菜单按键枚举
 * @details 定义菜单系统使用的三个按键及其功能映射。
 *          这些值与 ec_keys 模块的按键编号相对应：
 *          - KEY1 = 1 → PREVIOUS（上一个）
 *          - KEY2 = 2 → NEXT（下一个）
 *          - KEY3 = 3 → CONFIRM（确认）
 */
typedef enum
{
    EC_MENU_KEY_PREVIOUS = 1,   /**< "上一个"键，向上选择模式 */
    EC_MENU_KEY_NEXT = 2,       /**< "下一个"键，向下选择模式 */
    EC_MENU_KEY_CONFIRM = 3     /**< "确认"键，启动/停止模式 */
} ec_menu_key_t;

/**
 * @brief 菜单渲染回调函数类型
 * @details 用户实现此回调来将菜单状态渲染到具体的显示设备（如 OLED）。
 *          回调函数需要根据管理器中的当前选中模式、运行状态等信息，
 *          绘制相应的界面。
 *
 * @param manager 模式管理器指针（只读，包含所有模式和状态信息）
 * @param now_ms 当前系统时间（毫秒）
 * @param context 用户自定义上下文（在 ec_menu_init 中传入）
 */
typedef void (*ec_menu_render_fn)(const ec_mode_manager_t *manager,
    uint32_t now_ms, void *context);

/**
 * @brief 菜单控制器结构体
 * @details 连接按键输入、模式管理和屏幕渲染的中央控制器。
 *
 *          【工作流程】
 *          1. ec_menu_handle_key() 处理按键事件 → 更新模式管理器
 *          2. 设置 dirty 标志 → 触发下次 ec_menu_update 刷新屏幕
 *          3. ec_menu_update() 检查 dirty 和时间 → 调用 render 回调
 *
 *          【周期性刷新机制】
 *          即使没有按键事件，菜单也会以 render_period_ms 为周期
 *          自动刷新屏幕。这对于显示实时数据（如速度、电池电压等）
 *          非常重要，确保用户看到的不是过时的信息。
 */
typedef struct
{
    ec_mode_manager_t *manager;         /**< 模式管理器指针，持有所有工作模式 */
    ec_menu_render_fn render;           /**< 渲染回调函数，具体绘制菜单界面 */
    void *render_context;               /**< 传递给渲染回调的上下文指针 */
    uint32_t render_period_ms;          /**< 最小渲染刷新周期（毫秒），
                                             即使无按键事件也按此周期刷新 */
    uint32_t last_render_ms;            /**< 上次渲染的时间戳，用于判断是否到了刷新时机 */
    bool dirty;                         /**< 脏标志。true=需要立即刷新（通常由按键事件触发） */
} ec_menu_t;

/**
 * @brief 初始化菜单控制器
 * @param menu 菜单控制器指针
 * @param manager 已初始化的模式管理器
 * @param render 菜单渲染回调函数
 * @param render_context 传递给渲染回调的上下文
 * @param render_period_ms 最小渲染周期。传0将使用默认值100ms
 */
void ec_menu_init(ec_menu_t *menu, ec_mode_manager_t *manager,
    ec_menu_render_fn render, void *render_context, uint32_t render_period_ms);

/**
 * @brief 处理菜单按键事件
 * @details 根据按键类型执行相应操作：
 *          - PREVIOUS：选择上一个模式（如不在运行状态）
 *          - NEXT：选择下一个模式（如不在运行状态）
 *          - CONFIRM：切换当前选中模式的启动/停止状态
 *
 *          无论何种按键，都会将 dirty 标志设为 true，触发下次刷新。
 *
 * @param menu 菜单控制器指针
 * @param key 按键类型（PREVIOUS/NEXT/CONFIRM）
 * @param now_ms 当前系统时间（毫秒）
 */
void ec_menu_handle_key(ec_menu_t *menu, ec_menu_key_t key, uint32_t now_ms);

/**
 * @brief 更新菜单显示
 * @details 检查是否需要刷新显示：
 *          1. dirty 标志为 true（有按键事件）
 *          2. 距离上次刷新超过 render_period_ms（周期性刷新）
 *          满足任一条件则调用 render 回调进行绘制。
 *
 * @param menu 菜单控制器指针
 * @param now_ms 当前系统时间（毫秒）
 */
void ec_menu_update(ec_menu_t *menu, uint32_t now_ms);

#endif
