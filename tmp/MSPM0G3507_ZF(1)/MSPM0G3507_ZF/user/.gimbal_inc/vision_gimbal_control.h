#ifndef VISION_GIMBAL_CONTROL_H
#define VISION_GIMBAL_CONTROL_H

#include <stdint.h>

#include "gimbal.h"
#include "maixcam2_protocol.h"
#include "pid_controller.h"

typedef enum
{
    CTRL_STATE_IDLE = 0,
    CTRL_STATE_SEARCH,
    CTRL_STATE_COARSE_TRACK,
    CTRL_STATE_FINE_TRACK,
    CTRL_STATE_LOST_HOLD,
    CTRL_STATE_FAILSAFE,
} VisionControlState;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kff;
    float max_delta_deg;
    float deadband_px;
    int16_t large_error_px;
    uint32_t vision_timeout_ms;
    uint32_t lost_hold_ms;
    uint32_t control_period_ms;
} VisionGimbalConfig;

typedef struct
{
    VisionControlState state;
    VisionGimbalConfig config;
    PidController yaw_pid;
    PidController pitch_pid;
    uint32_t last_update_ms;
    uint32_t lost_start_ms;
    uint32_t last_control_ms;
    uint32_t control_count;
} VisionGimbalController;

void vision_gimbal_control_init(VisionGimbalController *controller);
void vision_gimbal_control_update(VisionGimbalController *controller, Gimbal *gimbal, uint32_t now_ms);
VisionControlState vision_gimbal_control_get_state(const VisionGimbalController *controller);

#endif
