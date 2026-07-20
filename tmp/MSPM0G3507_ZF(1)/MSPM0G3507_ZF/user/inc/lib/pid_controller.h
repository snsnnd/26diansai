#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float prev_derivative;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;
    float derivative_lpf;
    bool has_prev_error;
} PidController;

void pid_init(PidController *pid);
void pid_reset(PidController *pid);
void pid_set_gain(PidController *pid, float kp, float ki, float kd);
void pid_set_limits(PidController *pid, float output_min, float output_max, float integral_min, float integral_max);
void pid_set_deadband(PidController *pid, float deadband);
void pid_set_derivative_lpf(PidController *pid, float alpha);
float pid_update(PidController *pid, float error, float dt_s);

#endif
