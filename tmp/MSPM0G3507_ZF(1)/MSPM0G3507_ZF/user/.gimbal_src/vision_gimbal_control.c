#include "gimbal/vision_gimbal_control.h"

static float vg_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float vg_clampf(float value, float min_value, float max_value)
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

static VisionControlState decide_state(VisionGimbalController *controller, const MaixVisionTarget *target, uint32_t rx_time_ms, uint32_t now_ms)
{
    uint32_t age_ms = now_ms - rx_time_ms;
    int16_t abs_x;
    int16_t abs_y;

    if (age_ms > controller->config.vision_timeout_ms)
    {
        return CTRL_STATE_FAILSAFE;
    }

    if (!target->target_valid || target->vision_state == VISION_STATE_SEARCHING || target->vision_state == VISION_STATE_IDLE)
    {
        return CTRL_STATE_SEARCH;
    }

    if (target->vision_state == VISION_STATE_LOST)
    {
        if (controller->state != CTRL_STATE_LOST_HOLD)
        {
            controller->lost_start_ms = now_ms;
        }
        if ((now_ms - controller->lost_start_ms) <= controller->config.lost_hold_ms)
        {
            return CTRL_STATE_LOST_HOLD;
        }
        return CTRL_STATE_SEARCH;
    }

    abs_x = (target->error_x < 0) ? (int16_t)-target->error_x : target->error_x;
    abs_y = (target->error_y < 0) ? (int16_t)-target->error_y : target->error_y;

    if (abs_x > controller->config.large_error_px || abs_y > controller->config.large_error_px || target->vision_state == VISION_STATE_CANDIDATE)
    {
        return CTRL_STATE_COARSE_TRACK;
    }

    return CTRL_STATE_FINE_TRACK;
}

void vision_gimbal_control_init(VisionGimbalController *controller)
{
    if (controller == 0)
    {
        return;
    }

    controller->state = CTRL_STATE_IDLE;
    controller->config.kp = 0.006f;
    controller->config.ki = 0.0f;
    controller->config.kd = 0.001f;
    controller->config.kff = 0.0f;
    controller->config.max_delta_deg = 3.0f;
    controller->config.deadband_px = 8.0f;
    controller->config.large_error_px = 120;
    controller->config.vision_timeout_ms = 300u;
    controller->config.lost_hold_ms = 250u;
    controller->config.control_period_ms = 50u;
    controller->last_update_ms = 0u;
    controller->lost_start_ms = 0u;
    controller->last_control_ms = 0u;
    controller->control_count = 0u;

    pid_init(&controller->yaw_pid);
    pid_init(&controller->pitch_pid);
    pid_set_limits(&controller->yaw_pid, -controller->config.max_delta_deg, controller->config.max_delta_deg, -100.0f, 100.0f);
    pid_set_limits(&controller->pitch_pid, -controller->config.max_delta_deg, controller->config.max_delta_deg, -100.0f, 100.0f);
    pid_set_deadband(&controller->yaw_pid, controller->config.deadband_px);
    pid_set_deadband(&controller->pitch_pid, controller->config.deadband_px);
    pid_set_derivative_lpf(&controller->yaw_pid, 0.35f);
    pid_set_derivative_lpf(&controller->pitch_pid, 0.35f);
}

void vision_gimbal_control_update(VisionGimbalController *controller, Gimbal *gimbal, uint32_t now_ms)
{
    MaixVisionTarget target;
    uint32_t rx_time_ms;
    VisionControlState new_state;
    float dt_s;
    float yaw_cmd;
    float pitch_cmd;

    if (controller == 0 || gimbal == 0)
    {
        return;
    }

    if ((now_ms - controller->last_control_ms) < controller->config.control_period_ms)
    {
        return;
    }
    controller->last_control_ms = now_ms;

    if (!maixcam2_get_latest_target(&target, &rx_time_ms))
    {
        controller->state = CTRL_STATE_SEARCH;
        pid_reset(&controller->yaw_pid);
        pid_reset(&controller->pitch_pid);
        return;
    }

    new_state = decide_state(controller, &target, rx_time_ms, now_ms);
    if (new_state != controller->state)
    {
        pid_reset(&controller->yaw_pid);
        pid_reset(&controller->pitch_pid);
        controller->state = new_state;
    }

    dt_s = (controller->last_update_ms == 0u) ? 0.05f : ((float)(now_ms - controller->last_update_ms) / 1000.0f);
    if (dt_s <= 0.0f || dt_s > 0.2f)
    {
        dt_s = 0.05f;
    }
    controller->last_update_ms = now_ms;

    switch (controller->state)
    {
        case CTRL_STATE_COARSE_TRACK:
        case CTRL_STATE_FINE_TRACK:
            pid_set_gain(&controller->yaw_pid, controller->config.kp, controller->config.ki, controller->config.kd);
            pid_set_gain(&controller->pitch_pid, controller->config.kp, controller->config.ki, controller->config.kd);
            break;

        case CTRL_STATE_LOST_HOLD:
        case CTRL_STATE_SEARCH:
        case CTRL_STATE_FAILSAFE:
        case CTRL_STATE_IDLE:
        default:
            pid_reset(&controller->yaw_pid);
            pid_reset(&controller->pitch_pid);
            return;
    }

    yaw_cmd = pid_update(&controller->yaw_pid, (float)target.error_x, dt_s) + controller->config.kff * (float)target.vx;
    pitch_cmd = pid_update(&controller->pitch_pid, (float)target.error_y, dt_s) + controller->config.kff * (float)target.vy;
    yaw_cmd = vg_clampf(yaw_cmd, -controller->config.max_delta_deg, controller->config.max_delta_deg);
    pitch_cmd = vg_clampf(pitch_cmd, -controller->config.max_delta_deg, controller->config.max_delta_deg);

    if (vg_absf(yaw_cmd) > 0.001f || vg_absf(pitch_cmd) > 0.001f)
    {
        (void)gimbal_move_relative(gimbal, yaw_cmd, pitch_cmd);
        controller->control_count++;
    }
}

VisionControlState vision_gimbal_control_get_state(const VisionGimbalController *controller)
{
    return (controller == 0) ? CTRL_STATE_IDLE : controller->state;
}
