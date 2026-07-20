/**
 * @file dt_motor.c
 * @brief 直流电机驱动实现
 *        通过H桥双PWM控制方式驱动直流电机：
 *        正转：IN1=PWM, IN2=0
 *        反转：IN1=0, IN2=PWM
 *        刹车：IN1=PWM_MAX, IN2=PWM_MAX（H桥下管导通，绕组短路制动）
 *        急停：强制PWM输出低电平（通过硬件force_low功能）
 */

#include "driver/dt_motor.h"

/**
 * @brief 初始化电机控制PWM引脚
 *        两个引脚初始占空比均为0（电机停止）
 * @param cfg 电机配置结构体指针
 */
void dt_motor_init(dt_motor_config_t *cfg)
{
    if (cfg == NULL || cfg->pwm_freq == 0u) return;
    pwm_init(cfg->in1_pin, cfg->pwm_freq, 0); /* IN1初始占空比0 */
    pwm_init(cfg->in2_pin, cfg->pwm_freq, 0); /* IN2初始占空比0 */
}

/**
 * @brief 设置电机转速和方向
 *        speed>0: IN1输出PWM，IN2为0（正转）
 *        speed<0: IN1为0，IN2输出PWM（反转）
 *        speed=0: 电机停止（两路均为0）
 * @param cfg 电机配置结构体指针
 * @param speed 有符号速度值，范围[-DT_MOTOR_DUTY_MAX, DT_MOTOR_DUTY_MAX]
 * @note 传入的speed会被钳位到有效范围内
 */
void dt_motor_set_speed(dt_motor_config_t *cfg, int16_t speed)
{
    uint32_t duty;

    if (cfg == NULL) return;

    /* 钳位：确保speed不超出允许范围 */
    if (speed > (int16_t)DT_MOTOR_DUTY_MAX)
        speed = (int16_t)DT_MOTOR_DUTY_MAX;
    else if (speed < (int16_t)(-(int16_t)DT_MOTOR_DUTY_MAX))
        speed = (int16_t)(-(int16_t)DT_MOTOR_DUTY_MAX);

    /* 计算占空比绝对值 */
    duty = (uint32_t)(speed >= 0 ? speed : -speed);
    if (duty > DT_MOTOR_DUTY_MAX) duty = DT_MOTOR_DUTY_MAX;

    if (speed > 0)
    {
        /* 正转：IN2=0, IN1=PWM */
        pwm_set_duty(cfg->in2_pin, 0);
        pwm_set_duty(cfg->in1_pin, duty);
    }
    else if (speed < 0)
    {
        /* 反转：IN1=0, IN2=PWM */
        pwm_set_duty(cfg->in1_pin, 0);
        pwm_set_duty(cfg->in2_pin, duty);
    }
    else
    {
        /* 速度=0时停止 */
        dt_motor_stop(cfg);
    }
}

/**
 * @brief 电机停止（两路PWM占空比均设为0）
 *        电机自由滑行停止（无电磁制动效果）
 * @param cfg 电机配置结构体指针
 */
void dt_motor_stop(dt_motor_config_t *cfg)
{
    if (cfg == NULL) return;
    pwm_set_duty(cfg->in1_pin, 0);
    pwm_set_duty(cfg->in2_pin, 0);
}

/**
 * @brief 电机紧急停止
 *        调用PWM强制低电平输出，不受PWM占空比寄存器影响，
 *        可用于故障保护等需要立即切断电机驱动的场景
 * @param cfg 电机配置结构体指针
 */
void dt_motor_emergency_stop(dt_motor_config_t *cfg)
{
    if (cfg == NULL) return;
    pwm_force_low(cfg->in1_pin); /* 强制低电平 */
    pwm_force_low(cfg->in2_pin); /* 强制低电平 */
}

/**
 * @brief 电机电磁刹车
 *        两路PWM同时输出最大占空比，使H桥两个下管导通，
 *        电机绕组两端短路，利用反电动势产生电磁制动力矩，
 *        实现快速停车。
 * @param cfg 电机配置结构体指针
 */
void dt_motor_brake(dt_motor_config_t *cfg)
{
    if (cfg == NULL) return;
    pwm_set_duty(cfg->in1_pin, DT_MOTOR_DUTY_MAX);
    pwm_set_duty(cfg->in2_pin, DT_MOTOR_DUTY_MAX);
}
