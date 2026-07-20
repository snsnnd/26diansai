#include "lib/pid_controller.h"

static float pid_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float pid_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

void pid_init(PidController *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output_min = -1.0e6f;
    pid->output_max = 1.0e6f;
    pid->integral_min = -1.0e6f;
    pid->integral_max = 1.0e6f;
    pid->deadband = 0.0f;
    pid->derivative_lpf = 1.0f;
    pid->has_prev_error = false;
}

void pid_reset(PidController *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->has_prev_error = false;
}

void pid_set_gain(PidController *pid, float kp, float ki, float kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_limits(PidController *pid, float output_min, float output_max, float integral_min, float integral_max)
{
    if (pid == 0)
    {
        return;
    }

    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
}

void pid_set_deadband(PidController *pid, float deadband)
{
    if (pid == 0)
    {
        return;
    }

    pid->deadband = (deadband < 0.0f) ? -deadband : deadband;
}

void pid_set_derivative_lpf(PidController *pid, float alpha)
{
    if (pid == 0)
    {
        return;
    }

    pid->derivative_lpf = pid_clampf(alpha, 0.0f, 1.0f);
}

float pid_update(PidController *pid, float error, float dt_s)
{
    float derivative = 0.0f;
    float output;

    if (pid == 0 || dt_s <= 0.0f)
    {
        return 0.0f;
    }

    if (pid_absf(error) <= pid->deadband)
    {
        error = 0.0f;
    }

    pid->integral += error * dt_s;
    pid->integral = pid_clampf(pid->integral, pid->integral_min, pid->integral_max);

    if (pid->has_prev_error)
    {
        derivative = (error - pid->prev_error) / dt_s;
        derivative = pid->derivative_lpf * derivative + (1.0f - pid->derivative_lpf) * pid->prev_derivative;
    }

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = pid_clampf(output, pid->output_min, pid->output_max);

    pid->prev_error = error;
    pid->prev_derivative = derivative;
    pid->has_prev_error = true;

    return output;
}
