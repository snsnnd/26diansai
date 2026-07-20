/*********************************************************************************************************************
 * pin_mapping.h — 立创·天猛星 MSPM0G3507 开发板引脚映射
 *
 * 依据用户提供的天猛星 MSPM0G3507 开发板原理图整理。
 * 逐飞库当前芯片枚举只支持 A/B 口，天猛星也只引出 PA/PB，
 * 因此不要使用旧主板里的 C/D 口宏。
 *
 * 本文件为不同应用配置文件（EC_APP_PROFILE）定义了标准化的引脚和串口别名，
 * 使得上层代码可以使用统一的宏名称（如 BOARD_DEBUG_UART）访问当前配置下的对应外设，
 * 无需关心具体是哪个 UART 或哪个引脚。
 ********************************************************************************************************************/

#ifndef _PIN_MAPPING_H_
#define _PIN_MAPPING_H_

#include "config.h"
#include "zf_common_headfile.h"

/*==================== 板载 LED 和 RGB 灯 ====================*/

/*
 * 天猛星开发板板载 3 个 LED 和 1 个 RGB 灯。
 * LED 为高电平点亮，RGB 使用共阳/共阴结构（由 PIN_RGB_ON_LEVEL 定义）。
 */
#define PIN_LED1                A30   /* LED1：红色，PA30 */
#define PIN_LED2                A7    /* LED2：绿色，PA7 */
#define PIN_LED3                B27   /* LED3：蓝色，PB27 */
#define PIN_RGB_G               B1    /* RGB 绿灯：PB1 */
#define PIN_RGB_R               B0    /* RGB 红灯：PB0 */
#define PIN_RGB_B               A29   /* RGB 蓝灯：PA29 */
#define PIN_RGB_ON_LEVEL        GPIO_LOW   /* RGB 点亮电平 */
#define PIN_RGB_OFF_LEVEL       GPIO_HIGH    /* RGB 熄灭电平 */

/* 应用功能别名 */
#define PIN_LED                 PIN_LED1                  /* 主指示 LED */
#define PIN_ENCODER_ACTIVITY_LED PIN_LED2                 /* 编码器活动指示 */

/*=================================================== 调试串口 / 下载 ===================================================*/

/*
 * 调试串口在不同应用配置下映射到不同的物理串口：
 *   - Line-car 模式：使用蓝牙串口（UART2），方便无线调试
 *   - 其他模式（硬件测试/空工程）：使用 UART1（A8/A9）
 */
/* Line-car debug/VOFA/tuning use the transparent Bluetooth UART. */
#if EC_APP_PROFILE == EC_APP_PROFILE_LINE_CAR
#define BOARD_DEBUG_UART        BLUETOOTH_UART          /* UART2 */
#define BOARD_DEBUG_UART_TX     BLUETOOTH_TX_PIN        /* PB15 */
#define BOARD_DEBUG_UART_RX     BLUETOOTH_RX_PIN        /* PB16 */
#else
#define BOARD_DEBUG_UART        UART_1                  /* UART1 */
#define BOARD_DEBUG_UART_TX     UART1_TX_A8             /* PA8 */
#define BOARD_DEBUG_UART_RX     UART1_RX_A9             /* PA9 */
#endif

/*=================================================== 云台项目默认接口 ===================================================*/

/*
 * EMM（Eletronic Micro Module）云台电机通信使用 UART2 半双工模式。
 * UART2 同时被蓝牙调试复用，但 EMM 云台项目与 line-car 项目不会同时编译，
 * 所以无冲突风险。
 */
/* EMM 两轴电机共用 UART2: U2TX/U2RX = PB15/PB16。 */
#define BOARD_EMM_UART          UART_2
#define BOARD_EMM_UART_TX       UART2_TX_B15
#define BOARD_EMM_UART_RX       UART2_RX_B16

/*
 * T8 灰度传感器默认使用 UART1（A8/A9）。
 * 在 line-car 模式中，调试串口被映射到蓝牙 UART2，
 * 因此 UART1 空闲出来可供 T8 使用。
 */
/* UART1 A8/A9 is retained as the hardware-test debug port. */
#define BOARD_T8_UART           UART_1
#define BOARD_T8_UART_TX        UART1_TX_A8
#define BOARD_T8_UART_RX        UART1_RX_A9

/*
 * MaixCam2（K210 视觉模块）使用 UART3 通信，波特率 115200。
 * 引脚使用 B2/B3，这些引脚也可用于 T8 的 I2C 模式，
 * 但 UART3 和 T8 I2C 不会同时使用。
 */
/* 用户指定: UART3 B2/B3 用于 MaixCam2 视觉通信。 */
#define BOARD_MAIXCAM_UART      UART_3
#define BOARD_MAIXCAM_UART_TX   UART3_TX_B2             /* PB2 */
#define BOARD_MAIXCAM_UART_RX   UART3_RX_B3             /* PB3 */
#define BOARD_MAIXCAM_BAUDRATE  115200u                  /* 通信波特率 115200 */

#endif /* _PIN_MAPPING_H_ */
