/**
 * @file dt_motor.h
 * @brief 直流电机驱动头文件
 *        使用双路PWM控制一个直流电机（IN1/IN2），
 *        支持正反转、停止、急停和刹车模式。
 */

#ifndef _DT_MOTOR_H_
#define _DT_MOTOR_H_

#include "config.h"
#include "zf_common_headfile.h"

/** @brief PWM最大占空比（复用config.h中的定义） */
#define DT_MOTOR_DUTY_MAX   MOTOR_PWM_DUTY_MAX

/* 编译时检查：确保MOTOR_PWM_DUTY_MAX与PWM驱动使用的最大占空比一致 */
_Static_assert(DT_MOTOR_DUTY_MAX == PWM_DUTY_MAX,
    "MOTOR_PWM_DUTY_MAX must match the PWM driver scale");

/**
 * @brief 电机配置结构体
 *        定义一组H桥PWM引脚和频率
 */
typedef struct {
    pwm_channel_enum  in1_pin;   /**< H桥IN1引脚（正转PWM输入） */
    pwm_channel_enum  in2_pin;   /**< H桥IN2引脚（反转PWM输入） */
    uint32_t          pwm_freq;  /**< PWM载波频率（Hz） */
} dt_motor_config_t;

/**
 * @brief 初始化电机PWM引脚
 * @param cfg 电机配置结构体指针
 */
void dt_motor_init(dt_motor_config_t *cfg);

/**
 * @brief 设置电机转速（带符号，正数=正转，负数=反转，0=停止）
 * @param cfg 电机配置结构体指针
 * @param speed 目标速度，范围[-DT_MOTOR_DUTY_MAX, DT_MOTOR_DUTY_MAX]
 */
void dt_motor_set_speed(dt_motor_config_t *cfg, int16_t speed);

/**
 * @brief 电机停止（两路PWM均输出0）
 * @param cfg 电机配置结构体指针
 */
void dt_motor_stop(dt_motor_config_t *cfg);

/**
 * @brief 电机紧急停止（强制PWM输出低电平，不受占空比寄存器影响）
 * @param cfg 电机配置结构体指针
 */
void dt_motor_emergency_stop(dt_motor_config_t *cfg);

/**
 * @brief 电机刹车（两路PWM同时输出最大占空比，利用H桥同侧导通产生电磁制动）
 * @param cfg 电机配置结构体指针
 */
void dt_motor_brake(dt_motor_config_t *cfg);

#endif
