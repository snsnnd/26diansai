/**
 * @file    ec_time.c
 * @brief   系统时间管理模块 - 实现文件
 * @details 本模块基于 MSPM0G3507 的 TIMG0 定时器实现了 1ms 精度的系统时钟。
 *          全局毫秒计数器在定时器中断服务函数中递增，为整个系统提供统一的
 *          时间基准。
 *
 *          【时间基准的实现原理】
 *          1. pit_ms_init(PIT_TIM_G0, 1u, ...) 配置 TIMG0 定时器工作在
 *             周期性间隔定时器(PIT)模式，周期为 1ms
 *          2. 每次定时器溢出触发中断时，ec_time_tick ISR 被调用
 *          3. ISR 中递增 g_ec_time_ms 并调用可选的 tick hook
 *
 *          【中断安全性设计】
 *          - g_ec_time_ms 声明为 volatile，防止编译器优化导致读取到缓存值
 *          - ec_time_set_tick_hook 中禁用中断来保护对钩子指针的赋值操作
 *          - 普通的读取操作 (ec_time_ms) 不需要禁用中断，因为 uint32_t
 *            在 Cortex-M0+ 上是自然对齐的，读写是原子的
 */
#include "framework/ec_time.h"

#include "zf_common_headfile.h"

/**
 * 全局毫秒计数器。
 * 从系统初始化开始累计，每次定时器中断（1ms）递增1。
 * volatile 关键字防止编译器将其优化到寄存器中，确保每次读取都从内存获取最新值。
 */
static volatile uint32_t g_ec_time_ms;

/** 系统滴答钩子函数指针，在每次时钟滴答时被调用（ISR 上下文） */
static ec_time_tick_hook_t g_tick_hook;

/** 传递给滴答钩子的上下文指针 */
static void *g_tick_hook_context;

/**
 * @brief TIMG0 定时器中断服务函数
 * @details 这是系统的心脏——每 1ms 被硬件定时器触发一次。
 *          主要工作：
 *          1. 递增全局毫秒计数器 g_ec_time_ms
 *          2. 如果注册了 tick hook，则在 ISR 上下文中调用它
 *
 * @param event 硬件事件类型（本例中未使用）
 * @param context 用户上下文（本例中未使用）
 * @note 这是中断上下文！执行的代码必须极快且不可阻塞。
 *       如果 tick hook 执行时间过长，会影响系统的实时性。
 */
static void ec_time_tick(uint32 event, void *context)
{
    (void)event;
    (void)context;
    g_ec_time_ms++;                             /* 毫秒计数器递增 */
    if (g_tick_hook != NULL)
    {
        /* 调用用户注册的 tick hook，在中断上下文中执行 */
        g_tick_hook(g_ec_time_ms, g_tick_hook_context);
    }
}

/**
 * @brief 初始化系统时间模块
 * @details 完成以下初始化工作：
 *          1. 清零全局毫秒计数器
 *          2. 清空 tick hook
 *          3. 配置 TIMG0 定时器为 1ms 周期中断模式
 *
 * @note pit_ms_init 的第二个参数为 1 表示中断周期为 1ms。
 *       如果设置为 N，则周期为 N ms。
 *       该函数在系统启动早期被 ec_app_init 调用。
 */
void ec_time_init(void)
{
    g_ec_time_ms = 0u;
    g_tick_hook = NULL;
    g_tick_hook_context = NULL;
    /* 初始化 PIT 定时器，周期 1ms，绑定中断回调为 ec_time_tick */
    pit_ms_init(PIT_TIM_G0, 1u, ec_time_tick, NULL);
}

/**
 * @brief 获取当前系统时间（毫秒）
 * @return 自 ec_time_init 以来经过的毫秒数
 * @note 该函数非常轻量，只是简单地返回全局变量的值，
 *      适合在时间关键型代码中频繁调用。
 */
uint32_t ec_time_ms(void)
{
    return g_ec_time_ms;
}

/**
 * @brief 检查从 since_ms 到现在是否经过了至少 period_ms 时间
 * @details 【无符号回绕安全算法】
 *          该函数利用了 C 语言无符号整数减法的模运算特性：
 *          当 now_ms < since_ms（即发生了回绕）时，(now_ms - since_ms)
 *          在模 2^32 意义下仍然能正确计算出经过的时间。
 *
 *          例如：since_ms = 0xFFFFFFF0, now_ms = 0x00000010
 *          (0x00000010 - 0xFFFFFFF0) = 0x00000020 = 32ms（正确！）
 *
 * @param now_ms 当前时间戳（通常来自 ec_time_ms()）
 * @param since_ms 起始时间点
 * @param period_ms 需要判断的时间间隔
 * @return true 表示已经经过了指定时间，false 表示尚未到时间
 */
bool ec_time_elapsed(uint32_t now_ms, uint32_t since_ms, uint32_t period_ms)
{
    return (uint32_t)(now_ms - since_ms) >= period_ms;
}

/**
 * @brief 设置系统滴答钩子
 * @details 注册一个在每次系统滴答（1ms）时被调用的回调函数。
 *          该函数是中断安全的——它会在设置过程中禁用中断，
 *          防止在赋值过程中发生中断导致钩子状态不一致。
 *
 *          设置钩子和上下文指针的顺序很重要：
 *          先设置 context，再设置 hook 指针，这样即使 hook 在设置过程中
 *          被触发，也能拿到有效的 context（当然，在禁用中断的保护下，
 *          这种竞争条件实际上不会发生）。
 *
 * @param hook 钩子函数指针，传入 NULL 可取消之前的钩子
 * @param context 传递给钩子函数的上下文指针
 */
void ec_time_set_tick_hook(ec_time_tick_hook_t hook, void *context)
{
    uint32_t primask = __get_PRIMASK();

    /* 禁用中断，确保对钩子指针的赋值是原子的 */
    __disable_irq();
    g_tick_hook_context = context;
    g_tick_hook = hook;
    /* 恢复之前的中断状态（如果之前中断是使能的，则重新使能） */
    if (primask == 0u)
    {
        __enable_irq();
    }
}
