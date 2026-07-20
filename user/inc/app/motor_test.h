#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTOR_TEST_STATE_IDLE = 0,
    MOTOR_TEST_STATE_RAW_LEFT,
    MOTOR_TEST_STATE_RAW_RIGHT,
    MOTOR_TEST_STATE_BOOST_LEFT_HIGH,
    MOTOR_TEST_STATE_BOOST_LEFT_LOW,
    MOTOR_TEST_STATE_BOOST_RIGHT_HIGH,
    MOTOR_TEST_STATE_BOOST_RIGHT_LOW,
    MOTOR_TEST_STATE_DEADZONE_LEFT,
    MOTOR_TEST_STATE_DEADZONE_PAUSE,
    MOTOR_TEST_STATE_DEADZONE_RIGHT,
    MOTOR_TEST_STATE_DONE
} motor_test_state_t;

typedef void (*motor_test_apply_fn)(int16_t left, int16_t right,
    uint32_t now_ms, void *context);
typedef uint32_t (*motor_test_edges_fn)(void *context);
typedef void (*motor_test_reset_fn)(void *context);

typedef struct
{
    motor_test_apply_fn apply;
    motor_test_edges_fn left_edges;
    motor_test_edges_fn right_edges;
    motor_test_reset_fn reset_left;
    motor_test_reset_fn reset_right;
    void *context;
    int16_t raw_left_pwm;
    int16_t raw_right_pwm;
    uint32_t raw_duration_ms;
    int16_t startup_boost_pwm;
    uint32_t startup_boost_duration_ms;
    int16_t startup_low_pwm;
    uint32_t startup_low_duration_ms;  /* 0表示低速阶段持续到外部停止 */
    int16_t deadzone_start_pwm;
    int16_t deadzone_step_pwm;
    int16_t deadzone_max_pwm;
    uint32_t deadzone_step_ms;
    uint32_t deadzone_pause_ms;
    uint32_t deadzone_edge_count;
} motor_test_config_t;

typedef struct
{
    motor_test_config_t config;
    motor_test_state_t state;
    int16_t current_pwm;
    int16_t left_threshold;
    int16_t right_threshold;
    uint32_t left_edge_count;
    uint32_t right_edge_count;
    uint32_t deadline_ms;
    uint32_t step_start_ms;
    uint32_t step_start_edges;
} motor_test_t;

bool motor_test_init(motor_test_t *test, const motor_test_config_t *config);
bool motor_test_start_raw_left(motor_test_t *test, uint32_t now_ms);
bool motor_test_start_raw_right(motor_test_t *test, uint32_t now_ms);
bool motor_test_start_boost_left(motor_test_t *test, uint32_t now_ms);
bool motor_test_start_boost_right(motor_test_t *test, uint32_t now_ms);
bool motor_test_start_deadzone(motor_test_t *test, uint32_t now_ms);
void motor_test_update(motor_test_t *test, uint32_t now_ms);
void motor_test_stop(motor_test_t *test, uint32_t now_ms);
bool motor_test_is_raw(const motor_test_t *test);
bool motor_test_is_boost(const motor_test_t *test);
bool motor_test_is_done(const motor_test_t *test);

#endif
