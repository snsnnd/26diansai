#include "driver/dt_motor.h"

void dt_motor_init(dt_motor_config_t *cfg)
{
    pwm_init(cfg->in1_pin, cfg->pwm_freq, 0);
    pwm_init(cfg->in2_pin, cfg->pwm_freq, 0);
}

void dt_motor_set_speed(dt_motor_config_t *cfg, int16_t speed)
{
    uint32_t duty;

    if (speed > (int16_t)DT_MOTOR_DUTY_MAX)
        speed = (int16_t)DT_MOTOR_DUTY_MAX;
    else if (speed < (int16_t)(-(int16_t)DT_MOTOR_DUTY_MAX))
        speed = (int16_t)(-(int16_t)DT_MOTOR_DUTY_MAX);

    duty = (uint32_t)(speed >= 0 ? speed : -speed);
    if (duty > DT_MOTOR_DUTY_MAX) duty = DT_MOTOR_DUTY_MAX;

    if (speed > 0)
    {
        pwm_set_duty(cfg->in2_pin, 0);
        pwm_set_duty(cfg->in1_pin, duty);
    }
    else if (speed < 0)
    {
        pwm_set_duty(cfg->in1_pin, 0);
        pwm_set_duty(cfg->in2_pin, duty);
    }
    else
    {
        dt_motor_stop(cfg);
    }
}

void dt_motor_stop(dt_motor_config_t *cfg)
{
    pwm_set_duty(cfg->in1_pin, 0);
    pwm_set_duty(cfg->in2_pin, 0);
}

void dt_motor_brake(dt_motor_config_t *cfg)
{
    pwm_set_duty(cfg->in1_pin, DT_MOTOR_DUTY_MAX);
    pwm_set_duty(cfg->in2_pin, DT_MOTOR_DUTY_MAX);
}
