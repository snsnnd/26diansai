#ifndef _CONFIG_H_
#define _CONFIG_H_

/* 左轮 (电机2) */
#define MOTOR_L_IN1       PWM_TIM_A1_CH0_B17
#define MOTOR_L_IN2       PWM_TIM_G12_CH1_A25

/* 右轮 (电机1) */
#define MOTOR_R_IN1       PWM_TIM_A0_CH2_A15
#define MOTOR_R_IN2       PWM_TIM_A0_CH1_B20

#define MOTOR_PWM_FREQ  20000u

/* 编码器1 (左轮) */
#define ENCODER1_A_PIN   B24

/* 编码器2 (右轮) */
#define ENCODER2_A_PIN   A24

/* MG513: 13PPR, 1:30, A相双边沿测速 => 13 * 2 * 30 = 780 count/rev */
#define ENCODER_CPR      780

/* 按键 */
#define KEY1_PIN         B4
#define KEY2_PIN         A31
#define KEY3_PIN         A28

/* 蜂鸣器 (有源) */
#define BUZZER_PIN       A30

/* MPU6050 (软件 I2C) */
#define MPU6050_SCL      A11
#define MPU6050_SDA      A10

/* OLED (I2C SSD1306 128x64) */
#define OLED_SCL         A17
#define OLED_SDA         A16

/* 电池电压 (ADC) */
#define BAT_ADC          ADC0_CH0_A27

/* 循迹模块 (I2C) */
#define TRACE_SCL        B16
#define TRACE_SDA        B15

/* HC-05 蓝牙 */
#define HC05_EN_PIN      B3    /* EN脚, 拉高上电=AT模式 */

#endif
