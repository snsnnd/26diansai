/**
 * @file    ec_menu.c
 * @brief   菜单系统 —— 基于模式管理器的交互菜单框架 - 实现文件
 * @details 本文件实现了菜单控制器的核心逻辑，包括初始化、按键处理和显示更新。
 *          菜单系统是用户与智能车交互的主要接口，通过三个物理按键控制模式切换
 *          和参数调整，并通过 OLED 显示屏提供视觉反馈。
 *
 *          【交互逻辑总结】
 *          1. 用户按 PREVIOUS/NEXT 翻阅模式列表（仅在 STOPPED 状态）
 *          2. 用户按 CONFIRM 启动选中的模式
 *          3. 在模式运行中再次按 CONFIRM 停止模式
 *          4. 任何按键操作都会立即触发屏幕刷新
 *          5. 即使没有按键，屏幕也会定时刷新以显示实时数据
 */
#include "framework/ec_menu.h"

#include <stddef.h>

/**
 * @brief 初始化菜单控制器
 * @details 将菜单控制器与模式管理器、渲染函数绑定在一起。
 *          初始化时 dirty 设为 true，确保菜单在第一次 ec_menu_update
 *          调用时立即完成首次渲染，而不是等到按键事件或超时。
 *
 * @param menu 菜单控制器指针
 * @param manager 已初始化的模式管理器，菜单将操作此管理器
 * @param render 渲染回调，负责将菜单绘制到显示设备上
 * @param render_context 传递给渲染回调的自定义上下文
 * @param render_period_ms 最小渲染周期，如果传入0则使用100ms默认值
 */
void ec_menu_init(ec_menu_t *menu, ec_mode_manager_t *manager,
    ec_menu_render_fn render, void *render_context, uint32_t render_period_ms)
{
    if (menu == NULL) return;
    menu->manager = manager;
    menu->render = render;
    menu->render_context = render_context;
    /* 保护性编程：周期不能为0，否则可能在 ec_menu_update 中除零或造成过高的刷新频率 */
    menu->render_period_ms = (render_period_ms == 0u) ? 100u : render_period_ms;
    menu->last_render_ms = 0u;
    menu->dirty = true;     /* 初始化为脏，确保首次渲染立即执行 */
}

/**
 * @brief 处理菜单按键事件
 * @details 根据按键类型执行对应的模式管理器操作：
 *
 *          【CONFIRM（确认键）的逻辑】
 *          这是一个"启停切换"开关：
 *          - 如果当前有模式在运行 → 停止它
 *          - 如果当前没有模式运行 → 启动当前选中的模式
 *          这种设计类似于播放器的"播放/暂停"按钮，用同一个键控制两种状态。
 *
 *          【PREVIOUS/NEXT（选择键）的逻辑】
 *          委托给模式管理器的 select_previous/select_next 函数。
 *          这些函数内部会检查是否是 STOPPED 状态，如果不是则忽略操作。
 *
 *          【dirty 标志】
 *          任何按键事件都会设置 dirty = true，确保 ec_menu_update
 *          在下一次调用时会立即触发渲染，带给用户即时的视觉反馈。
 *
 * @param menu 菜单控制器指针
 * @param key 按键类型
 * @param now_ms 当前系统时间
 */
void ec_menu_handle_key(ec_menu_t *menu, ec_menu_key_t key, uint32_t now_ms)
{
    if (menu == NULL || menu->manager == NULL) return;

    if (key == EC_MENU_KEY_CONFIRM)
    {
        /* CONFIRM 键：运行/停止 切换 */
        if (menu->manager->state == EC_MODE_RUNNING)
            ec_mode_manager_stop(menu->manager, now_ms);   /* 停止当前模式 */
        else
            (void)ec_mode_manager_start(menu->manager, now_ms); /* 启动选中模式 */
    }
    else if (key == EC_MENU_KEY_PREVIOUS)
    {
        ec_mode_manager_select_previous(menu->manager);    /* 选择上一个模式 */
    }
    else if (key == EC_MENU_KEY_NEXT)
    {
        ec_mode_manager_select_next(menu->manager);        /* 选择下一个模式 */
    }
    menu->dirty = true;     /* 标记需要刷新显示 */
}

/**
 * @brief 更新菜单显示
 * @details 此函数实现了"事件驱动 + 周期刷新"的显示策略：
 *
 *          1. 事件驱动：当 dirty 标志为 true（有按键操作）时无条件刷新，
 *             确保用户操作有即时视觉反馈
 *          2. 周期刷新：即使没有按键事件，只要达到 render_period_ms 时间间隔，
 *             也会强制刷新。这对于显示实时数据（如速度、角度、状态等）
 *             至关重要——即使没有任何操作，屏幕上的数据也需要持续更新
 *
 *          刷新后清除 dirty 标志并记录本次刷新时间。
 *
 * @param menu 菜单控制器指针
 * @param now_ms 当前系统时间
 */
void ec_menu_update(ec_menu_t *menu, uint32_t now_ms)
{
    if (menu == NULL || menu->render == NULL) return;
    /* 检查是否需要刷新：dirty 标志 或 达到周期时间 */
    if (!menu->dirty && (uint32_t)(now_ms - menu->last_render_ms) < menu->render_period_ms) return;
    menu->last_render_ms = now_ms;
    menu->dirty = false;        /* 清除脏标志 */
    menu->render(menu->manager, now_ms, menu->render_context);  /* 执行渲染 */
}
