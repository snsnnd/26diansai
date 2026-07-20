#include "app/motor_test.h"

#include <stddef.h>

static bool motor_test_time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static void motor_test_apply(motor_test_t *test, int16_t left, int16_t right,
    uint32_t now_ms)
{
    if (test->config.apply != NULL)
    {
        test->config.apply(left, right, now_ms, test->config.context);
    }
}

bool motor_test_init(motor_test_t *test, const motor_test_config_t *config)
{
    if (test == NULL || config == NULL || config->apply == NULL ||
        config->raw_left_pwm <= 0 || config->raw_right_pwm <= 0 ||
        config->raw_duration_ms == 0u || config->startup_boost_pwm <= 0 ||
        config->startup_low_pwm <= 0 ||
        config->startup_low_pwm >= config->startup_boost_pwm ||
        config->startup_boost_duration_ms == 0u ||
        config->deadzone_start_pwm <= 0 ||
        config->deadzone_step_pwm <= 0 ||
        config->deadzone_max_pwm < config->deadzone_start_pwm ||
        config->startup_boost_pwm > config->deadzone_max_pwm ||
        config->deadzone_step_ms == 0u || config->deadzone_pause_ms == 0u ||
        config->deadzone_edge_count == 0u)
    {
        return false;
    }

    test->config = *config;
    test->state = MOTOR_TEST_STATE_IDLE;
    test->current_pwm = 0;
    test->left_threshold = 0;
    test->right_threshold = 0;
    test->left_edge_count = 0u;
    test->right_edge_count = 0u;
    test->deadline_ms = 0u;
    test->step_start_ms = 0u;
    test->step_start_edges = 0u;
    return true;
}

static bool motor_test_start_raw(motor_test_t *test, motor_test_state_t state,
    uint32_t now_ms)
{
    if (test == NULL || test->config.apply == NULL)
    {
        return false;
    }

    test->state = state;
    test->deadline_ms = now_ms + test->config.raw_duration_ms;
    test->current_pwm = (state == MOTOR_TEST_STATE_RAW_LEFT) ?
        test->config.raw_left_pwm : test->config.raw_right_pwm;
    return true;
}

bool motor_test_start_raw_left(motor_test_t *test, uint32_t now_ms)
{
    return motor_test_start_raw(test, MOTOR_TEST_STATE_RAW_LEFT, now_ms);
}

bool motor_test_start_raw_right(motor_test_t *test, uint32_t now_ms)
{
    return motor_test_start_raw(test, MOTOR_TEST_STATE_RAW_RIGHT, now_ms);
}

static bool motor_test_start_boost(motor_test_t *test,
    motor_test_state_t state, uint32_t now_ms)
{
    if (test == NULL || test->config.apply == NULL)
    {
        return false;
    }

    test->state = state;
    test->current_pwm = test->config.startup_boost_pwm;
    test->deadline_ms = now_ms + test->config.startup_boost_duration_ms;
    return true;
}

bool motor_test_start_boost_left(motor_test_t *test, uint32_t now_ms)
{
    return motor_test_start_boost(test, MOTOR_TEST_STATE_BOOST_LEFT_HIGH,
        now_ms);
}

bool motor_test_start_boost_right(motor_test_t *test, uint32_t now_ms)
{
    return motor_test_start_boost(test, MOTOR_TEST_STATE_BOOST_RIGHT_HIGH,
        now_ms);
}

bool motor_test_start_deadzone(motor_test_t *test, uint32_t now_ms)
{
    if (test == NULL || test->config.left_edges == NULL ||
        test->config.right_edges == NULL || test->config.reset_left == NULL ||
        test->config.reset_right == NULL)
    {
        return false;
    }

    test->config.reset_left(test->config.context);
    test->config.reset_right(test->config.context);
    test->state = MOTOR_TEST_STATE_DEADZONE_LEFT;
    test->current_pwm = test->config.deadzone_start_pwm;
    test->left_threshold = 0;
    test->right_threshold = 0;
    test->left_edge_count = 0u;
    test->right_edge_count = 0u;
    test->step_start_ms = now_ms;
    test->step_start_edges = 0u;
    test->deadline_ms = 0u;
    return true;
}

static void motor_test_finish_left(motor_test_t *test, uint32_t edges,
    int16_t threshold, uint32_t now_ms)
{
    test->left_threshold = threshold;
    test->left_edge_count = edges;
    motor_test_apply(test, 0, 0, now_ms);
    test->state = MOTOR_TEST_STATE_DEADZONE_PAUSE;
    test->deadline_ms = now_ms + test->config.deadzone_pause_ms;
}

static void motor_test_finish_right(motor_test_t *test, uint32_t edges,
    int16_t threshold, uint32_t now_ms)
{
    test->right_threshold = threshold;
    test->right_edge_count = edges;
    motor_test_apply(test, 0, 0, now_ms);
    test->state = MOTOR_TEST_STATE_DONE;
}

static void motor_test_update_deadzone_left(motor_test_t *test, uint32_t now_ms)
{
    uint32_t edges;

    motor_test_apply(test, (int16_t)-test->current_pwm, 0, now_ms);
    edges = test->config.left_edges(test->config.context);
    if (edges - test->step_start_edges >= test->config.deadzone_edge_count)
    {
        motor_test_finish_left(test, edges, test->current_pwm, now_ms);
    }
    else if ((uint32_t)(now_ms - test->step_start_ms) >=
        test->config.deadzone_step_ms)
    {
        if (test->current_pwm >= test->config.deadzone_max_pwm)
        {
            motor_test_finish_left(test, edges, -1, now_ms);
        }
        else
        {
            test->current_pwm = (int16_t)(test->current_pwm +
                test->config.deadzone_step_pwm);
            test->step_start_ms = now_ms;
            test->step_start_edges = edges;
        }
    }
}

static void motor_test_update_deadzone_right(motor_test_t *test, uint32_t now_ms)
{
    uint32_t edges;

    motor_test_apply(test, 0, (int16_t)-test->current_pwm, now_ms);
    edges = test->config.right_edges(test->config.context);
    if (edges - test->step_start_edges >= test->config.deadzone_edge_count)
    {
        motor_test_finish_right(test, edges, test->current_pwm, now_ms);
    }
    else if ((uint32_t)(now_ms - test->step_start_ms) >=
        test->config.deadzone_step_ms)
    {
        if (test->current_pwm >= test->config.deadzone_max_pwm)
        {
            motor_test_finish_right(test, edges, -1, now_ms);
        }
        else
        {
            test->current_pwm = (int16_t)(test->current_pwm +
                test->config.deadzone_step_pwm);
            test->step_start_ms = now_ms;
            test->step_start_edges = edges;
        }
    }
}

void motor_test_update(motor_test_t *test, uint32_t now_ms)
{
    if (test == NULL)
    {
        return;
    }

    switch (test->state)
    {
        case MOTOR_TEST_STATE_RAW_LEFT:
            if (motor_test_time_reached(now_ms, test->deadline_ms))
            {
                motor_test_apply(test, 0, 0, now_ms);
                test->state = MOTOR_TEST_STATE_DONE;
            }
            else
            {
                motor_test_apply(test, (int16_t)-test->current_pwm, 0, now_ms);
            }
            break;

        case MOTOR_TEST_STATE_RAW_RIGHT:
            if (motor_test_time_reached(now_ms, test->deadline_ms))
            {
                motor_test_apply(test, 0, 0, now_ms);
                test->state = MOTOR_TEST_STATE_DONE;
            }
            else
            {
                motor_test_apply(test, 0, (int16_t)-test->current_pwm, now_ms);
            }
            break;

        case MOTOR_TEST_STATE_BOOST_LEFT_HIGH:
            motor_test_apply(test, (int16_t)-test->current_pwm, 0, now_ms);
            if (motor_test_time_reached(now_ms, test->deadline_ms))
            {
                test->state = MOTOR_TEST_STATE_BOOST_LEFT_LOW;
                test->current_pwm = test->config.startup_low_pwm;
                test->deadline_ms = test->config.startup_low_duration_ms == 0u ?
                    0u : now_ms + test->config.startup_low_duration_ms;
            }
            break;

        case MOTOR_TEST_STATE_BOOST_LEFT_LOW:
            if (test->config.startup_low_duration_ms != 0u &&
                motor_test_time_reached(now_ms, test->deadline_ms))
            {
                motor_test_apply(test, 0, 0, now_ms);
                test->state = MOTOR_TEST_STATE_DONE;
            }
            else
            {
                motor_test_apply(test, (int16_t)-test->current_pwm, 0, now_ms);
            }
            break;

        case MOTOR_TEST_STATE_BOOST_RIGHT_HIGH:
            motor_test_apply(test, 0, (int16_t)-test->current_pwm, now_ms);
            if (motor_test_time_reached(now_ms, test->deadline_ms))
            {
                test->state = MOTOR_TEST_STATE_BOOST_RIGHT_LOW;
                test->current_pwm = test->config.startup_low_pwm;
                test->deadline_ms = test->config.startup_low_duration_ms == 0u ?
                    0u : now_ms + test->config.startup_low_duration_ms;
            }
            break;

        case MOTOR_TEST_STATE_BOOST_RIGHT_LOW:
            if (test->config.startup_low_duration_ms != 0u &&
                motor_test_time_reached(now_ms, test->deadline_ms))
            {
                motor_test_apply(test, 0, 0, now_ms);
                test->state = MOTOR_TEST_STATE_DONE;
            }
            else
            {
                motor_test_apply(test, 0, (int16_t)-test->current_pwm, now_ms);
            }
            break;

        case MOTOR_TEST_STATE_DEADZONE_LEFT:
            motor_test_update_deadzone_left(test, now_ms);
            break;

        case MOTOR_TEST_STATE_DEADZONE_PAUSE:
            motor_test_apply(test, 0, 0, now_ms);
            if (motor_test_time_reached(now_ms, test->deadline_ms))
            {
                test->config.reset_right(test->config.context);
                test->current_pwm = test->config.deadzone_start_pwm;
                test->step_start_ms = now_ms;
                test->step_start_edges = 0u;
                test->state = MOTOR_TEST_STATE_DEADZONE_RIGHT;
            }
            break;

        case MOTOR_TEST_STATE_DEADZONE_RIGHT:
            motor_test_update_deadzone_right(test, now_ms);
            break;

        case MOTOR_TEST_STATE_IDLE:
        case MOTOR_TEST_STATE_DONE:
        default:
            motor_test_apply(test, 0, 0, now_ms);
            break;
    }
}

void motor_test_stop(motor_test_t *test, uint32_t now_ms)
{
    if (test == NULL)
    {
        return;
    }

    motor_test_apply(test, 0, 0, now_ms);
    test->state = MOTOR_TEST_STATE_IDLE;
    test->current_pwm = 0;
}

bool motor_test_is_raw(const motor_test_t *test)
{
    return test != NULL && (test->state == MOTOR_TEST_STATE_RAW_LEFT ||
        test->state == MOTOR_TEST_STATE_RAW_RIGHT);
}

bool motor_test_is_boost(const motor_test_t *test)
{
    return test != NULL &&
        (test->state == MOTOR_TEST_STATE_BOOST_LEFT_HIGH ||
         test->state == MOTOR_TEST_STATE_BOOST_LEFT_LOW ||
         test->state == MOTOR_TEST_STATE_BOOST_RIGHT_HIGH ||
         test->state == MOTOR_TEST_STATE_BOOST_RIGHT_LOW);
}

bool motor_test_is_done(const motor_test_t *test)
{
    return test != NULL && test->state == MOTOR_TEST_STATE_DONE;
}
