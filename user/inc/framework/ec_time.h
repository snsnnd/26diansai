/**
 * @file    ec_time.h
 * @brief   系统时间管理模块
 * @details 本模块提供了基于 TIMG0（通用定时器组0）的毫秒级系统时钟服务。
 *          它是整个智能车控制系统的"心跳"——所有周期性任务、延时判断、
 *          超时检测都依赖于此模块提供的统一时间基准。
 *
 *          【时间架构设计】
 *          1. 使用 PIT（Periodic Interval Timer，周期性间隔定时器）以 1ms 为周期
 *             产生定时中断，在中断服务函数中累加全局毫秒计数器 g_ec_time_ms。
 *          2. 提供可选的 tick hook（滴答钩子）机制，允许用户在每次时钟滴答时
 *             执行自定义代码（在中断上下文中！）。
 *          3. ec_time_ms() 返回自系统初始化以来的毫秒数，使用 uint32_t 类型，
 *             大约 49.7 天才会回绕一次，对智能车应用完全足够。
 *
 *          【注意事项】
 *          - ec_time_ms() 返回的是全局计数器的值，在中断中更新，因此读取时
 *            无需特殊保护（对 uint32_t 的读操作在 Cortex-M0+ 上是原子的）
 *          - Tick hook 在中断上下文中执行，必须严格遵守 ISR 的编写规则
 *          - ec_time_elapsed() 函数巧妙地利用了 unsigned 算术特性，能够正确
 *            处理 uint32_t 时间戳回绕的情况
 */
#ifndef EC_TIME_H
#define EC_TIME_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 系统滴答钩子函数类型
 * @details 这是一个在 TIMG0 定时器中断 ISR 上下文中执行的回调函数。
 *          每次系统毫秒计数器递增时，如果注册了 tick hook，该函数就会被调用。
 *
 *          【ISR 上下文的约束】
 *          在中断服务函数中执行时，必须遵守以下规则：
 *          - 非阻塞：不能包含等待、轮询或长时间循环
 *          - ISR 安全：不能调用 printf、delay 等非可重入函数
 *          - 不能获取锁或互斥量
 *          - 执行时间必须短且有界（通常在几微秒内完成）
 *          - 不能调用任何可能触发调度的函数
 *
 *          【单例模式】
 *          整个系统只能安装一个 tick hook，后续调用 ec_time_set_tick_hook()
 *          会覆盖之前的设置。
 *
 * @param now_ms 当前的系统毫秒计数器值
 * @param context 用户自定义上下文指针，在设置 hook 时传入
 */
typedef void (*ec_time_tick_hook_t)(uint32_t now_ms, void *context);

/**
 * @brief 初始化系统时间模块
 * @details 配置 TIMG0 定时器以 1ms 为周期产生中断，清零全局毫秒计数器。
 *          必须在任何使用时间功能的模块之前调用此函数。
 */
void ec_time_init(void);

/**
 * @brief 获取当前的系统毫秒计数值
 * @return 自 ec_time_init 调用以来经过的毫秒数（uint32_t，约49.7天回绕一次）
 * @note 在大多数 Cortex-M 处理器上，对 uint32_t 的读访问是原子的，
 *       因此不需要禁用中断来读取这个值。
 * @see ec_time_elapsed
 */
uint32_t ec_time_ms(void);

/**
 * @brief 判断从 since_ms 开始是否已经经过了 period_ms 时间
 * @details 此函数使用无符号整数算术的特性来正确处理时间戳回绕问题。
 *          (now_ms - since_ms) 即使在回绕场景下也能正确产生时间差，
 *          因为无符号减法在模 2^32 意义下是正确的。
 *
 *          例如：now_ms=0x00000002, since_ms=0xFFFFFFFE 时，
 *          (uint32_t)(2 - 0xFFFFFFFE) = 4，正确反映了经过4ms。
 *
 * @param now_ms 当前时间戳
 * @param since_ms 起始时间戳（被比较的基准时间点）
 * @param period_ms 需要判断的时间段长度（毫秒）
 * @return true 如果 (now_ms - since_ms) >= period_ms，即经过了指定的时间段
 * @note 适合在轮询循环中做非精确超时判断
 */
bool ec_time_elapsed(uint32_t now_ms, uint32_t since_ms, uint32_t period_ms);

/**
 * @brief 设置或替换系统滴答钩子回调
 * @details 注册一个在每次系统时钟滴答（1ms）时被调用的回调函数。
 *          该回调在定时器中断的 ISR 上下文中执行。
 *
 *          如果 hook 参数为 NULL，则取消已注册的 tick hook（实际实现中
 *          仍会设置，但执行 NULL 检查由调用者负责）。
 *
 * @param hook 回调函数指针，设置为 NULL 可取消之前的钩子
 * @param context 传递给回调函数的上下文指针
 * @note 此函数会禁用中断来确保对钩子指针的赋值操作是原子的
 */
void ec_time_set_tick_hook(ec_time_tick_hook_t hook, void *context);

#endif
