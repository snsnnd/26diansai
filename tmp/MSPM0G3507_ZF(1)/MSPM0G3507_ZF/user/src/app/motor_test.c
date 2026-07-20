#include "app/motor_test.h"

static float g_target_rpm = 50.0f;
static int16_t g_base_pwm = 5400;
static float g_ff_k = 13.25f;
static float g_pid_kp = 8.0f;
static float g_pid_ki = 0.0f;
static float g_pid_kd = 0.0f;
static uint8_t g_test_mode = MOTOR_TEST_MODE_PID;
static int16_t g_test_pwm = 3000;
static int16_t g_deadzone_pwm = 0;
static int16_t g_voltage_pwm = 0;
static uint32_t g_last_deadzone_step = 0;
static uint8_t g_deadzone_done = 0;
static uint8_t g_last_test_mode = MOTOR_TEST_MODE_PID;

static float test_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t clamp_pwm_i16(int32_t value)
{
    if (value > 10000) return 10000;
    if (value < -10000) return -10000;
    return (int16_t)value;
}

void motor_test_init(void)
{
    g_target_rpm = 50.0f;
    g_base_pwm = 5400;
    g_ff_k = 13.25f;
    g_pid_kp = 8.0f;
    g_pid_ki = 0.0f;
    g_pid_kd = 0.0f;
    g_test_mode = MOTOR_TEST_MODE_PID;
    g_test_pwm = 3000;
    g_deadzone_pwm = 0;
    g_voltage_pwm = 0;
    g_last_deadzone_step = 0;
    g_deadzone_done = 0;
    g_last_test_mode = MOTOR_TEST_MODE_PID;
}

uint8_t motor_test_is_pid_mode(void)
{
    return g_test_mode == MOTOR_TEST_MODE_PID;
}

void motor_test_update_deadzone(uint32_t now_ms, int8_t motor_dir, float rpm_l, float rpm_r, motor_test_apply_cb_t apply_cb)
{
    int16_t cmd = 0;

    if (g_test_mode != g_last_test_mode)
    {
        g_last_test_mode = g_test_mode;
        g_deadzone_pwm = 0;
        g_deadzone_done = 0;
        g_last_deadzone_step = now_ms;
    }

    if (g_test_mode == MOTOR_TEST_MODE_OPEN_PWM)
    {
        cmd = clamp_pwm_i16((int32_t)g_test_pwm * motor_dir);
    }
    else if (g_test_mode == MOTOR_TEST_MODE_DEADZONE_POS || g_test_mode == MOTOR_TEST_MODE_DEADZONE_NEG)
    {
        int16_t sign = (g_test_mode == MOTOR_TEST_MODE_DEADZONE_POS) ? 1 : -1;

        if (!g_deadzone_done && (uint32_t)(now_ms - g_last_deadzone_step) >= 200u)
        {
            g_last_deadzone_step = now_ms;
            if (g_deadzone_pwm < 8000)
            {
                g_deadzone_pwm = (int16_t)(g_deadzone_pwm + 100);
            }
        }

        cmd = clamp_pwm_i16((int32_t)g_deadzone_pwm * sign);
        if (!g_deadzone_done && (test_absf(rpm_l) > 5.0f || test_absf(rpm_r) > 5.0f))
        {
            g_deadzone_done = 1;
        }
    }

    g_voltage_pwm = cmd;
    if (apply_cb != 0)
    {
        apply_cb(cmd, cmd);
    }
}

float *motor_test_target_rpm_ptr(void) { return &g_target_rpm; }
int16_t *motor_test_base_pwm_ptr(void) { return &g_base_pwm; }
float *motor_test_ff_k_ptr(void) { return &g_ff_k; }
float *motor_test_kp_ptr(void) { return &g_pid_kp; }
float *motor_test_ki_ptr(void) { return &g_pid_ki; }
float *motor_test_kd_ptr(void) { return &g_pid_kd; }
uint8_t *motor_test_mode_ptr(void) { return &g_test_mode; }
int16_t *motor_test_pwm_ptr(void) { return &g_test_pwm; }
int16_t *motor_test_deadzone_pwm_ptr(void) { return &g_deadzone_pwm; }

float motor_test_target_rpm(void) { return g_target_rpm; }
int16_t motor_test_base_pwm(void) { return g_base_pwm; }
float motor_test_ff_k(void) { return g_ff_k; }
float motor_test_kp(void) { return g_pid_kp; }
float motor_test_ki(void) { return g_pid_ki; }
float motor_test_kd(void) { return g_pid_kd; }
uint8_t motor_test_mode(void) { return g_test_mode; }
int16_t motor_test_voltage_pwm(void) { return g_voltage_pwm; }
void motor_test_set_voltage_pwm(int16_t pwm) { g_voltage_pwm = pwm; }

int16_t motor_test_feedforward_pwm(void)
{
    int32_t pwm = (int32_t)((float)g_base_pwm + g_target_rpm * g_ff_k);

    if (pwm > 10000) return 10000;
    if (pwm < 0) return 0;
    return (int16_t)pwm;
}
