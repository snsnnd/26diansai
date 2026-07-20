#ifndef _MOTOR_TEST_H_
#define _MOTOR_TEST_H_

#include "zf_common_headfile.h"

typedef enum {
    MOTOR_TEST_MODE_PID = 0,
    MOTOR_TEST_MODE_OPEN_PWM,
    MOTOR_TEST_MODE_DEADZONE_POS,
    MOTOR_TEST_MODE_DEADZONE_NEG
} motor_test_mode_t;

typedef void (*motor_test_apply_cb_t)(int16_t left_cmd, int16_t right_cmd);

void motor_test_init(void);

uint8_t motor_test_is_pid_mode(void);
void motor_test_update_deadzone(uint32_t now_ms, int8_t motor_dir, float rpm_l, float rpm_r, motor_test_apply_cb_t apply_cb);

float *motor_test_target_rpm_ptr(void);
int16_t *motor_test_base_pwm_ptr(void);
float *motor_test_ff_k_ptr(void);
float *motor_test_kp_ptr(void);
float *motor_test_ki_ptr(void);
float *motor_test_kd_ptr(void);
uint8_t *motor_test_mode_ptr(void);
int16_t *motor_test_pwm_ptr(void);
int16_t *motor_test_deadzone_pwm_ptr(void);

float motor_test_target_rpm(void);
int16_t motor_test_base_pwm(void);
float motor_test_ff_k(void);
int16_t motor_test_feedforward_pwm(void);
float motor_test_kp(void);
float motor_test_ki(void);
float motor_test_kd(void);
uint8_t motor_test_mode(void);
int16_t motor_test_voltage_pwm(void);
void motor_test_set_voltage_pwm(int16_t pwm);

#endif
