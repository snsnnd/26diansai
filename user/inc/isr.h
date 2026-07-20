/*********************************************************************************************************************
 * isr.h — 中断服务程序（ISR）诊断接口
 *
 * 提供中断服务程序的诊断信息读取功能，包含：
 *   - UART 中断计数和 drain limit 命中次数（检测中断风暴）
 *   - GPIO 外部中断计数和事件计数
 *   - NVIC 使能状态和中断屏蔽状态
 *   - PRIMASK（全局中断屏蔽位）状态
 *
 * 这些诊断信息对调试中断相关的问题（如中断丢失、中断风暴、响应延迟）非常有用。
 ********************************************************************************************************************/

#ifndef _isr_h_
#define _isr_h_

#include "zf_common_headfile.h"

/**
 * isr_diagnostics_t — ISR 诊断数据结构体。
 *
 * @uart_irq_count:       各 UART 中断触发次数（索引 0~3 对应 UART0~UART3）
 * @uart_drain_limit_hits:各 UART 达到单次 ISR 最大处理次数（中断风暴检测）
 * @gpio_irq_count:       GPIO 中断（GROUP1_IRQHandler）总触发次数
 * @gpio_event_count:     GPIO 实际处理的事件总数（可能 > irq_count，说明单次 ISR 处理了多个事件）
 * @gpio_drain_limit_hits:GPIO ISR 达到最大处理次数的次数
 * @primask:              读取时的 PRIMASK 寄存器值（0=全局中断开，1=全局中断关）
 * @gpio_nvic_enabled:    GPIOA 中断在 NVIC 中是否使能（0/1）
 * @gpioa_imask:          GPIOA 的 CPU_INT.IMASK 寄存器值（各引脚中断屏蔽状态）
 * @gpiob_imask:          GPIOB 的 CPU_INT.IMASK 寄存器值
 */
typedef struct
{
    uint32_t uart_irq_count[4];          /* UART 中断计数，每个 UART 一个 */
    uint32_t uart_drain_limit_hits[4];   /* UART drain limit 命中次数 */
    uint32_t gpio_irq_count;             /* GPIO 中断总触发次数 */
    uint32_t gpio_event_count;           /* GPIO 实际处理的中断事件数 */
    uint32_t gpio_drain_limit_hits;      /* GPIO drain limit 命中次数 */
    uint8_t primask;                     /* PRIMASK 状态（全局中断标志） */
    uint8_t gpio_nvic_enabled;           /* GPIOA NVIC 使能标志 */
    uint32_t gpioa_imask;                /* GPIOA 中断屏蔽寄存器值 */
    uint32_t gpiob_imask;                /* GPIOB 中断屏蔽寄存器值 */
} isr_diagnostics_t;

/*
 * isr_get_diagnostics — 读取 ISR 诊断信息。
 *
 * 在读取时临时关中断以保证计数器数据的一致性（多个 UART 的 4 个计数需原子读取）。
 * 读取完成后恢复原始中断状态。
 *
 * @diagnostics: 输出参数，接收完整的 ISR 诊断数据
 */
void isr_get_diagnostics(isr_diagnostics_t *diagnostics);

#endif
