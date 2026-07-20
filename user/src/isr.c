/*********************************************************************************************************************
 * isr.c — 中断服务函数
 *
 * 逐飞 MSPM0G3507 开源库 ISR
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件实现了系统的三类关键中断处理：
 *   1. PIT（周期中断定时器） — 用于 PID 控制周期定时、任务调度等
 *   2. UART（串口收发） — 用于传感器数据接收和调试输出
 *   3. EXTI（外部中断/GPIO） — 用于编码器脉冲捕获和按键中断
 *
 * 所有外设中断均使用逐飞库的"回调函数"机制：
 *   在逐飞库初始化时注册回调函数和回调参数，
 *   ISR 中自动调用注册的回调函数，实现中断处理与业务逻辑的解耦。
 *
 * 设计特点：
 *   - UART/GPIO ISR 包含"drain limit"保护机制
 *   - 防止中断风暴导致主循环无法执行
 *   - 在 UART ISR 中直接调用逐飞库的 debug_interrupr_handler()
 *     实现 printf 输出和 uart_query_byte 的兼容
 ********************************************************************************************************************/

#include "isr.h"

/*=================================================== PIT 中断 ===================================================*/

/*
 * pit_irq_dispatch — PIT（周期中断定时器）通用中断分发函数。
 *
 * 逐飞库的 GPTIMER 定时器支持多种中断事件（装载、捕获、比较等），
 * 本函数仅处理 LOAD（装载/溢出）事件，调用对应的回调函数。
 *
 * 回调链：GPTIMER 硬件中断 → TIMAx_IRQHandler → pit_callback_list[index]()
 * 回调函数注册：pit_callback_list 和 pit_callback_ptr_list 由逐飞库管理
 *
 * @timer:  定时器外设寄存器基地址
 * @index:  定时器索引号（0~6 对应 TIMA0/1、TIMG0/6/7/8/12）
 */
static void pit_irq_dispatch(const GPTIMER_Regs *timer, uint8_t index)
{
    DL_TIMER_IIDX interrupt = DL_Timer_getPendingInterrupt(timer);

    if(interrupt == DL_TIMER_IIDX_LOAD && pit_callback_list[index] != NULL)
    {
        pit_callback_list[index](DL_TIMER_INTERRUPT_LOAD_EVENT, pit_callback_ptr_list[index]);
    }
}

/* PIT 中断处理函数 — 每个定时器对应一个独立的 IRQ Handler */

/* TIMA0: 通常用于电机 PWM 生成（CCP0~CCP3），也用作控制周期定时器 */
void TIMA0_IRQHandler (void)
{
    pit_irq_dispatch(TIMA0, 0);
}

/* TIMA1: 备用定时器，可用于辅助 PWM 或计时 */
void TIMA1_IRQHandler (void)
{
    pit_irq_dispatch(TIMA1, 1);
}

/* TIMG0: 通用定时器，可用于系统心跳或软件定时器 */
void TIMG0_IRQHandler (void)
{
    pit_irq_dispatch(TIMG0, 2);
}

/* TIMG6: 通用定时器 */
void TIMG6_IRQHandler (void)
{
    pit_irq_dispatch(TIMG6, 3);
}

/* TIMG7: 通用定时器 */
void TIMG7_IRQHandler (void)
{
    pit_irq_dispatch(TIMG7, 4);
}

/*
 * TIMG8: 编码器/定时器，在 line-car 工程中用作左轮编码器
 * （硬件 AB 正交解码模式）
 */
void TIMG8_IRQHandler (void)
{
    pit_irq_dispatch(TIMG8, 5);
}

/*
 * TIMG12: 编码器/定时器，在 line-car 工程中用作右轮编码器
 * （硬件 AB 正交解码模式）
 */
void TIMG12_IRQHandler (void)
{
    pit_irq_dispatch(TIMG12, 6);
}

/*=================================================== UART 中断 ===================================================*/

/*
 * UART_ISR_DRAIN_LIMIT — 单次 UART ISR 中最多处理的中断事件次数。
 *
 * 如果一次 UART ISR 中连续处理超过这个数量的事件，说明发生了"中断风暴"
 * （短时间内大量数据涌入，ISR 处理速度跟不上硬件 FIFO 填充速度）。
 * 此时应停止处理该 UART 的更多事件，让主循环有机会执行。
 * 这种"drain limit"机制是嵌入式系统中防止 ISR 饿死主循环的经典技巧。
 */
#define UART_ISR_DRAIN_LIMIT    (32u)

/*
 * UART 中断诊断计数器（volatile 修饰，因为 ISR 和主循环共享访问）。
 * 这些计数器由 isr_get_diagnostics() 读取。
 */
static volatile uint32_t g_uart_irq_count[4];       /* 各 UART 中断总触发次数 */
static volatile uint32_t g_uart_drain_limit_hits[4]; /* 各 UART 达到 drain limit 的次数 */
static volatile uint32_t g_gpio_irq_count;          /* GPIO 中断总触发次数 */
static volatile uint32_t g_gpio_event_count;        /* GPIO 实际处理的中断事件总数 */
static volatile uint32_t g_gpio_drain_limit_hits;   /* GPIO 达到 drain limit 的次数 */

/*
 * uart_irq_dispatch — UART 中断统一分发函数。
 *
 * MSPM0G3507 的 UART 中断控制器（IIDX）可以同时挂起多个中断事件，
 * 读取 IIDX 寄存器会自动清除对应事件标志。因此需要在 while 循环中
 * 反复读取 IIDX 直到没有更多待处理事件，或达到 drain limit。
 *
 * 中断处理流程：
 *   1. 递增该 UART 的中断计数
 *   2. 循环读取 UART 待处理中断（最多 UART_ISR_DRAIN_LIMIT 次）
 *   3. 根据中断类型分发：
 *      - TX: 调用逐飞库注册的 TX 回调（通常用于 DMA 发送完成通知）
 *      - RX: 调用逐飞库注册的 RX 回调 + 额外处理
 *      - 其他: 读取 IIDX 即可确认（清标志），不做额外处理
 *   4. 如果达到 drain limit，递增诊断计数（表示发生了中断风暴）
 *
 * 特殊 RX 处理逻辑：
 *   - 如果是调试串口的 RX 中断：
 *     调用 debug_interrupr_handler() 使逐飞库的 uart_query_byte() 正常工作
 *   - 如果是 UART1（索引 1）的 RX 中断：
 *     调用 wireless_module_uart_handler() 处理无线模块（蓝牙/HC05）的数据
 *
 * @uart:  UART 外设寄存器基地址
 * @index: UART 索引（0~3 对应 UART0~UART3）
 */
static void uart_irq_dispatch(UART_Regs *uart, uint8 index)
{
    uint8 serviced = 0u;           /* 本次 ISR 已处理的中断事件数 */
    DL_UART_IIDX interrupt;

    g_uart_irq_count[index]++;     /* 递增诊断计数 */

    /*
     * 循环处理所有待处理的中断事件。
     * DL_UART_getPendingInterrupt() 每次返回最高优先级的一个待处理中断，
     * 当没有更多待处理中断时返回 DL_UART_IIDX_NO_INTERRUPT。
     */
    while((serviced < UART_ISR_DRAIN_LIMIT)
        && ((interrupt = DL_UART_getPendingInterrupt(uart)) != DL_UART_IIDX_NO_INTERRUPT))
    {
        void_callback_uint32_ptr callback = uart_callback_list[index];

        serviced++;
        switch(interrupt)
        {
            case DL_UART_IIDX_TX:
            {
                /* 发送中断：TX FIFO 变空时触发 */
                if(NULL != callback)
                {
                    callback(UART_INTERRUPT_STATE_TX, uart_callback_ptr_list[index]);
                }
            }break;

            case DL_UART_IIDX_RX:
            {
                /* 接收中断：RX FIFO 接收到数据时触发 */

                /* 调用逐飞库的用户回调 */
                if(NULL != callback)
                {
                    callback(UART_INTERRUPT_STATE_RX, uart_callback_ptr_list[index]);
                }
                /*
                 * 调试串口特殊处理：
                 * 调用逐飞库的 debug_interrupr_handler() 来填充 debug 环形缓冲区，
                 * 这样上层 debug_printf() 和 uart_query_byte() 才能正常工作。
                 */
                if((uint8)DEBUG_UART_INDEX == index)
                {
#if DEBUG_UART_USE_INTERRUPT
                    debug_interrupr_handler();
#endif
                }
                /*
                 * UART1 特殊处理：
                 * 调用无线模块（如蓝牙 HC05）的数据处理函数。
                 * 这是逐飞库的无线模块框架的一部分。
                 */
                else if(1u == index)
                {
                    wireless_module_uart_handler();
                }
            }break;

            default:
            {
                /*
                 * 其他中断类型（如行状态错误、超时等）：
                 * 读取 IIDX 的动作已经清除了中断标志，
                 * 这里不做额外处理，继续轮询下一个待处理中断。
                 */
            }break;
        }
    }

    /* 如果达到 drain limit，说明发生了中断风暴，记录诊断信息 */
    if(serviced >= UART_ISR_DRAIN_LIMIT)
    {
        g_uart_drain_limit_hits[index]++;
    }
}

/*
 * UART IRQ Handlers — 各 UART 外设的中断入口。
 *
 * UART0: 调试串口 / 陀螺仪
 * UART1: 蓝牙 / T8 灰度传感器 / 备用
 * UART2: 蓝牙透明串口（line-car 调试端口）
 * UART3: MaixCam2 视觉模块通信
 *
 * 所有 UART 共享同一套分发逻辑，仅索引号不同。
 * 在 uart_irq_dispatch 中会根据索引号决定额外的处理逻辑。
 */

void UART0_IRQHandler (void)
{
    uart_irq_dispatch(UART0, 0u);   /* UART0, index=0 */
}

void UART1_IRQHandler (void)
{
    uart_irq_dispatch(UART1, 1u);   /* UART1, index=1 */
}

void UART2_IRQHandler (void)
{
    uart_irq_dispatch(UART2, 2u);   /* UART2, index=2 */
}

void UART3_IRQHandler (void)
{
    uart_irq_dispatch(UART3, 3u);   /* UART3, index=3 */
}

/*=================================================== EXTI 中断（GPIO 外部中断）===================================================*/

/*
 * GPIO_ISR_DRAIN_LIMIT — 单次 GPIO ISR 中最多处理的事件数。
 *
 * 比 UART 的 drain limit 更大（64 vs 32），因为 GPIO ISR 可能同时处理
 * 多个引脚的中断事件（编码器 A/B 相、按键等）。
 * GROUP1 中断聚合了 GPIOA 和 GPIOB 的全部引脚中断。
 */
#define GPIO_ISR_DRAIN_LIMIT    (64u)

/*
 * gpio_irq_dispatch_one — 处理一个 GPIO 组的单个中断事件。
 *
 * MSPM0G3507 的 GPIO 中断控制器使用 IIDX（Interrupt Index）寄存器
 * 指示当前待处理中断的引脚索引。读取 IIDX 的同时会自动清除该中断标志。
 *
 * 实现步骤：
 *   1. 读取该 GPIO 组的 CPU_INT.IIDX 寄存器获取中断引脚索引
 *   2. 如果 IIDX 为 0，表示没有更多待处理中断，返回 0
 *   3. 根据引脚索引计算中断事件类型（上升沿/下降沿/双边沿）
 *      - 索引 0~15 用 POLARITY15_0 寄存器
 *      - 索引 16~31 用 POLARITY31_16 寄存器
 *   4. 查找逐飞库注册的回调函数并调用
 *
 * @exti_group: GPIO 扩展中断组号（0=GPIOA, 1=GPIOB）
 * @return: 本次处理的事件数（0 或 1）
 */
static uint8 gpio_irq_dispatch_one(uint8 exti_group)
{
    uint32 register_temp = gpio_group[exti_group]->CPU_INT.IIDX;
    uint8 exti_index;
    uint8 exti_event;
    uint16 callback_index;

    if(register_temp == 0u)
    {
        return 0u;
    }

    exti_index = (uint8)(register_temp - 1u);
    if(15u >= exti_index)
    {
        exti_event = (uint8)((gpio_group[exti_group]->POLARITY15_0 >>
            ((exti_index % 16u) * 2u)) & 0x03u);
    }
    else
    {
        exti_event = (uint8)((gpio_group[exti_group]->POLARITY31_16 >>
            ((exti_index % 16u) * 2u)) & 0x03u);
    }
    callback_index = (uint16)(exti_group * GPIO_GROUP_PIN_NUMBER_MAX + exti_index);
    if(exti_callback_list[callback_index])
    {
        exti_callback_list[callback_index](exti_event,
            exti_callback_ptr_list[callback_index]);
    }
    return 1u;
}

/*
 * GROUP1_IRQHandler — GPIO 外部中断处理函数（GROUP1 中断线）。
 *
 * MSPM0G3507 的中断系统有分组概念，GPIOA 和 GPIOB 的外部中断
 * 都聚合在 GROUP1 中断线上。因此此函数需处理两个 GPIO 端口的所有中断。
 *
 * 处理策略：
 *   1. 先读取 GROUP1 的 IIDX 锁存器（确认中断来源）
 *   2. 优先处理 GPIOB（索引 1），因为 B4 被用作运行停止键
 *      需要及时响应（紧急制动）。
 *   3. 再处理 GPIOA（索引 0），编码器中断通常在此。
 *   4. 使用 drain limit 防止中断风暴。
 *
 * @note: 此函数的 serviced 递增逻辑与 UART 版本不同：
 *        每次循环可能处理多个事件（handled >= 1），
 *        因此用累计和而不是简单的递减。
 */
void GROUP1_IRQHandler (void)
{
    uint8 serviced = 0u;

    g_gpio_irq_count++;          /* 递增诊断计数器 */
    while(serviced < GPIO_ISR_DRAIN_LIMIT)
    {
        uint8 handled = 0u;

        /*
         * 读取 GROUP1 的中断待处理锁存器（IIDX）。
         * 这一步对于清除 GROUP 级别的中断标志是必要的。
         * 返回值在此不关心，因为之后会逐个 GPIO 组处理。
         */
        /* GROUP1 has its own IIDX latch in front of the GPIO port IIDX. */
        (void)DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

        /*
         * 先处理 GPIOB（组 1），再处理 GPIOA（组 0）。
         * 优先级原因是 B4 引脚被用作"运行停止键"（紧急制动），
         * 需要优先响应。编码器（在 GPIOA 上）虽然频率高但可以稍等。
         */
        /* B4 is the runtime emergency key; check GPIOB before encoder-heavy A. */
        handled += gpio_irq_dispatch_one(1u);   /* GPIOB */
        handled += gpio_irq_dispatch_one(0u);   /* GPIOA */

        /* 如果两个组都没有待处理事件，退出循环 */
        if(handled == 0u)
        {
            break;
        }
        serviced = (uint8)(serviced + handled);
        g_gpio_event_count += handled;          /* 更新诊断计数 */
    }
    if(serviced >= GPIO_ISR_DRAIN_LIMIT)
    {
        g_gpio_drain_limit_hits++;              /* 记录 drain limit 命中 */
    }
}

/*
 * isr_get_diagnostics — 读取 ISR 诊断信息（线程安全）。
 *
 * 在读取统计计数器时需要临时关中断，以防止 ISR 在读取过程中更新
 * 这些 volatile 变量导致读取到不一致的数据（特别是多个计数器需要一致性时）。
 *
 * 读取流程：
 *   1. 保存当前 PRIMASK 状态（全局中断使能状态）
 *   2. 读取 NVIC 和 GPIO 屏蔽寄存器状态
 *   3. 关中断（禁止 ISR 写入统计数据）
 *   4. 原子读取所有统计计数器到输出结构体
 *   5. 如果进入此函数时中断本来是开的，则恢复中断
 *
 * 这种"保存-关-读-恢复"模式是嵌入式系统中读取 ISR 共享数据的标准做法。
 *
 * @diagnostics: 输出参数，接收完整的 ISR 诊断数据
 */
void isr_get_diagnostics(isr_diagnostics_t *diagnostics)
{
    uint32_t primask;
    uint8_t i;

    if(diagnostics == NULL)
    {
        return;
    }

    /* 读取 PRIMASK 寄存器（用于后续判断是否需要恢复中断） */
    primask = __get_PRIMASK();
    diagnostics->primask = (uint8_t)primask;

    /* 读取 NVIC 和 GPIO 中断屏蔽状态（这些不需要关中断） */
    diagnostics->gpio_nvic_enabled =
        (NVIC->ISER[0] & (1u << (uint32_t)GPIOA_INT_IRQn)) != 0u ? 1u : 0u;
    diagnostics->gpioa_imask = GPIOA->CPU_INT.IMASK;
    diagnostics->gpiob_imask = GPIOB->CPU_INT.IMASK;

    /* 关中断，确保读取统计计数器时 ISR 不会修改它们 */
    __disable_irq();
    for(i = 0u; i < 4u; i++)
    {
        diagnostics->uart_irq_count[i] = g_uart_irq_count[i];
        diagnostics->uart_drain_limit_hits[i] = g_uart_drain_limit_hits[i];
    }
    diagnostics->gpio_irq_count = g_gpio_irq_count;
    diagnostics->gpio_event_count = g_gpio_event_count;
    diagnostics->gpio_drain_limit_hits = g_gpio_drain_limit_hits;

    /* 如果进入时中断是打开的，恢复中断 */
    if(primask == 0u)
    {
        __enable_irq();
    }
}
