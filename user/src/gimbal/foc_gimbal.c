/**
 * @file foc_gimbal.c
 * @brief 无刷 FOC 云台控制器实现
 */
#include "gimbal/foc_gimbal.h"

FocGimbal g_foc_gimbal;

static float foc_clamp_angle(float deg)
{
    if (deg > 30.0f)
    {
        return 30.0f;
    }
    if (deg < -30.0f)
    {
        return -30.0f;
    }
    return deg;
}

FocGimbalStatus foc_gimbal_init(FocGimbal *gimbal)
{
    uint8_t motor_count;
    uint8_t retry;

    if (gimbal == NULL)
    {
        return FOC_GIMBAL_ERROR;
    }

    gimbal->uart        = FOC_GIMBAL_UART;
    gimbal->initialized = false;
    gimbal->enabled     = false;
    gimbal->run_mode    = FOC_GIMBAL_DEFAULT_RUN_MODE;
    gimbal->fault_count = 0u;

    uart_init(gimbal->uart, FOC_GIMBAL_UART_BAUD,
        FOC_GIMBAL_TX_PIN, FOC_GIMBAL_RX_PIN);

    for (retry = 0u; retry < 3u; retry++)
    {
        motor_count = 0u;
        if (foc_ping(gimbal->uart, 1000u, &motor_count) == ZF_TRUE
            && motor_count >= 2u)
        {
            break;
        }
        system_delay_ms(1000u);
    }

    if (motor_count < 2u)
    {
        return FOC_GIMBAL_ERROR_OFFLINE;
    }

    if (foc_session_enter(gimbal->uart, 500u) != ZF_TRUE)
    {
        return FOC_GIMBAL_ERROR_UART;
    }

    if (foc_set_control_mode(gimbal->uart, 0u, FOC_MODE_ANGLE,
            FOC_GIMBAL_CMD_TIMEOUT_MS) != ZF_TRUE)
    {
        return FOC_GIMBAL_ERROR_UART;
    }
    if (foc_set_control_mode(gimbal->uart, 1u, FOC_MODE_ANGLE,
            FOC_GIMBAL_CMD_TIMEOUT_MS) != ZF_TRUE)
    {
        return FOC_GIMBAL_ERROR_UART;
    }

    if (foc_set_run_mode(gimbal->uart, 0u, gimbal->run_mode,
            FOC_GIMBAL_CMD_TIMEOUT_MS) != ZF_TRUE)
    {
        return FOC_GIMBAL_ERROR_UART;
    }
    if (foc_set_run_mode(gimbal->uart, 1u, gimbal->run_mode,
            FOC_GIMBAL_CMD_TIMEOUT_MS) != ZF_TRUE)
    {
        return FOC_GIMBAL_ERROR_UART;
    }

    gimbal->initialized    = true;
    gimbal->yaw_target_deg  = 0.0f;
    gimbal->pitch_target_deg = 0.0f;
    gimbal->yaw_angle_deg   = 0.0f;
    gimbal->pitch_angle_deg = 0.0f;

    return FOC_GIMBAL_OK;
}

FocGimbalStatus foc_gimbal_enable(FocGimbal *gimbal, bool enable)
{
    if (gimbal == NULL)
    {
        return FOC_GIMBAL_ERROR;
    }
    if (!gimbal->initialized)
    {
        return FOC_GIMBAL_ERROR_NOT_INIT;
    }
    gimbal->enabled = enable;
    return FOC_GIMBAL_OK;
}

FocGimbalStatus foc_gimbal_stop(FocGimbal *gimbal)
{
    if (gimbal == NULL)
    {
        return FOC_GIMBAL_ERROR;
    }
    if (!gimbal->initialized)
    {
        return FOC_GIMBAL_ERROR_NOT_INIT;
    }
    gimbal->enabled = false;
    foc_hold(gimbal->uart, 0xFFu, FOC_GIMBAL_CMD_TIMEOUT_MS);
    return FOC_GIMBAL_OK;
}

FocGimbalStatus foc_gimbal_move_to(FocGimbal *gimbal,
    float yaw_deg, float pitch_deg)
{
    if (gimbal == NULL)
    {
        return FOC_GIMBAL_ERROR;
    }
    if (!gimbal->initialized || !gimbal->enabled)
    {
        return FOC_GIMBAL_ERROR_NOT_INIT;
    }

    yaw_deg   = foc_clamp_angle(yaw_deg);
    pitch_deg = foc_clamp_angle(pitch_deg);

    if (foc_set_dual_angle(gimbal->uart, yaw_deg, pitch_deg,
            FOC_GIMBAL_CMD_TIMEOUT_MS) != ZF_TRUE)
    {
        gimbal->fault_count++;
        return FOC_GIMBAL_ERROR_UART;
    }

    gimbal->yaw_target_deg   = yaw_deg;
    gimbal->pitch_target_deg = pitch_deg;
    return FOC_GIMBAL_OK;
}

FocGimbalStatus foc_gimbal_move_to_fast(FocGimbal *gimbal,
    float yaw_deg, float pitch_deg)
{
    if (gimbal == NULL)
    {
        return FOC_GIMBAL_ERROR;
    }
    if (!gimbal->initialized || !gimbal->enabled)
    {
        return FOC_GIMBAL_ERROR_NOT_INIT;
    }

    yaw_deg   = foc_clamp_angle(yaw_deg);
    pitch_deg = foc_clamp_angle(pitch_deg);

    foc_set_dual_angle_fast(gimbal->uart, yaw_deg, pitch_deg);

    gimbal->yaw_target_deg   = yaw_deg;
    gimbal->pitch_target_deg = pitch_deg;
    return FOC_GIMBAL_OK;
}

FocGimbalStatus foc_gimbal_hold(FocGimbal *gimbal)
{
    if (gimbal == NULL)
    {
        return FOC_GIMBAL_ERROR;
    }
    if (!gimbal->initialized)
    {
        return FOC_GIMBAL_ERROR_NOT_INIT;
    }

    if (foc_hold(gimbal->uart, 0xFFu, FOC_GIMBAL_CMD_TIMEOUT_MS)
        != ZF_TRUE)
    {
        return FOC_GIMBAL_ERROR_UART;
    }
    return FOC_GIMBAL_OK;
}

FocGimbalStatus foc_gimbal_hold_dual(FocGimbal *gimbal)
{
    return foc_gimbal_hold(gimbal);
}

FocGimbalStatus foc_gimbal_read_position(FocGimbal *gimbal,
    float *yaw_deg, float *pitch_deg)
{
    if (gimbal == NULL)
    {
        return FOC_GIMBAL_ERROR;
    }
    if (!gimbal->initialized)
    {
        return FOC_GIMBAL_ERROR_NOT_INIT;
    }

    if (yaw_deg != NULL)
    {
        *yaw_deg = gimbal->yaw_angle_deg;
    }
    if (pitch_deg != NULL)
    {
        *pitch_deg = gimbal->pitch_angle_deg;
    }
    return FOC_GIMBAL_OK;
}

FocGimbalStatus foc_gimbal_update_status(FocGimbal *gimbal,
    uint32_t now_ms)
{
    if (gimbal == NULL)
    {
        return FOC_GIMBAL_ERROR;
    }
    if (!gimbal->initialized)
    {
        return FOC_GIMBAL_ERROR_NOT_INIT;
    }
    if (now_ms - gimbal->last_status_ms < FOC_GIMBAL_STATUS_INTERVAL_MS)
    {
        return FOC_GIMBAL_OK;
    }

    gimbal->last_status_ms = now_ms;

    if (foc_get_all_status(gimbal->uart, FOC_GIMBAL_CMD_TIMEOUT_MS,
            &gimbal->m0_status, &gimbal->m1_status) != ZF_TRUE)
    {
        gimbal->fault_count++;
        return FOC_GIMBAL_ERROR_UART;
    }

    gimbal->yaw_angle_deg    = gimbal->m0_status.angle_deg;
    gimbal->pitch_angle_deg  = gimbal->m1_status.angle_deg;
    gimbal->yaw_velocity_dps = gimbal->m0_status.velocity_deg_s;
    gimbal->pitch_velocity_dps = gimbal->m1_status.velocity_deg_s;
    gimbal->yaw_error_deg    = gimbal->m0_status.error_deg;
    gimbal->pitch_error_deg  = gimbal->m1_status.error_deg;
    gimbal->sensor_valid     = gimbal->m0_status.online
                            && gimbal->m1_status.online
                            && (gimbal->m0_status.flags & FOC_FLAG_SENSOR_VALID) != 0u
                            && (gimbal->m1_status.flags & FOC_FLAG_SENSOR_VALID) != 0u;

    return FOC_GIMBAL_OK;
}

FocGimbalStatus foc_gimbal_set_run_mode(FocGimbal *gimbal,
    foc_run_mode_t mode)
{
    if (gimbal == NULL)
    {
        return FOC_GIMBAL_ERROR;
    }
    if (!gimbal->initialized)
    {
        return FOC_GIMBAL_ERROR_NOT_INIT;
    }

    if (foc_set_run_mode(gimbal->uart, 0u, mode,
            FOC_GIMBAL_CMD_TIMEOUT_MS) != ZF_TRUE)
    {
        return FOC_GIMBAL_ERROR_UART;
    }
    if (foc_set_run_mode(gimbal->uart, 1u, mode,
            FOC_GIMBAL_CMD_TIMEOUT_MS) != ZF_TRUE)
    {
        return FOC_GIMBAL_ERROR_UART;
    }

    gimbal->run_mode = mode;
    return FOC_GIMBAL_OK;
}

bool foc_gimbal_is_online(const FocGimbal *gimbal)
{
    if (gimbal == NULL)
    {
        return false;
    }
    return gimbal->initialized;
}

bool foc_gimbal_is_sensor_valid(const FocGimbal *gimbal)
{
    if (gimbal == NULL)
    {
        return false;
    }
    return gimbal->initialized && gimbal->sensor_valid;
}
