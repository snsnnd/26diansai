#ifndef _DT_MOTOR_H_
#define _DT_MOTOR_H_

#include "zf_common_headfile.h"

#define DT_MOTOR_DUTY_MAX   10000

typedef struct {
    pwm_channel_enum  in1_pin;
    pwm_channel_enum  in2_pin;
    uint32_t          pwm_freq;
} dt_motor_config_t;

void dt_motor_init(dt_motor_config_t *cfg);
void dt_motor_set_speed(dt_motor_config_t *cfg, int16_t speed);
void dt_motor_stop(dt_motor_config_t *cfg);
void dt_motor_brake(dt_motor_config_t *cfg);

#endif
