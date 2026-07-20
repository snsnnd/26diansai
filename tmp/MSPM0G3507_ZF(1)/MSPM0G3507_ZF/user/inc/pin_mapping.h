/*********************************************************************************************************************
 * pin_mapping.h - 立创·天猛星 MSPM0G3507 开发板引脚映射
 *
 * 依据用户提供的天猛星 MSPM0G3507 开发板原理图整理。
 * 逐飞库当前芯片枚举只支持 A/B 口，天猛星也只引出 PA/PB，因此不要使用旧主板里的 C/D 口宏。
 ********************************************************************************************************************/

#ifndef _PIN_MAPPING_H_
#define _PIN_MAPPING_H_

#include "zf_common_headfile.h"

#define PIN_LED                 B22     /* 用户 LED: PB22 -> R19 -> LED -> GND，高电平点亮 */

/*=================================================== 调试串口 / 下载 ===================================================*/

/* 原理图 CH340E: PA10/U0TX, PA11/U0RX；逐飞 debug 默认也正是 UART0_TX_A10 / UART0_RX_A11。 */
#define BOARD_DEBUG_UART        UART_0
#define BOARD_DEBUG_UART_TX     UART0_TX_A10
#define BOARD_DEBUG_UART_RX     UART0_RX_A11

/*=================================================== 云台项目默认接口 ===================================================*/

/* EMM 两轴电机共用 UART2: U2TX/U2RX = PB15/PB16。 */
#define BOARD_EMM_UART          UART_2
#define BOARD_EMM_UART_TX       UART2_TX_B15
#define BOARD_EMM_UART_RX       UART2_RX_B16

/* T8 灰度传感器 UART 查询口: U1TX/U1RX = PB4/PB5。 */
#define BOARD_T8_UART           UART_1
#define BOARD_T8_UART_TX        UART1_TX_B4
#define BOARD_T8_UART_RX        UART1_RX_B5

/* MaixCam2 视觉模块串口，默认复用 UART1；启用 T8 时需要重新分配。 */
#define BOARD_MAIXCAM_UART      UART_1
#define BOARD_MAIXCAM_UART_TX   UART1_TX_B4
#define BOARD_MAIXCAM_UART_RX   UART1_RX_B5
#define BOARD_MAIXCAM_BAUDRATE  115200u

#endif /* _PIN_MAPPING_H_ */
