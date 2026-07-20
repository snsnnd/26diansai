/**
 * @file foc_gimbal.h
 * @brief 无刷电机 FOC 双轴云台控制器
 *
 * 本模块通过 UART 二进制协议控制 STM32F103 2804 无刷云台驱动板。
 * 与步进电机云台 (gimbal.h) 并行，API 风格一致。
 *
 * 电机映射: M0 = YAW (偏航), M1 = PITCH (俯仰)
 * 角度范围: ±30° (软限位)
 * 控制频率: 推荐 50~200Hz, 使用 foc_set_dual_angle_fast 做无应答高频控制
 *
 * @note 上电后需等待约 3 秒让 FOC 完成传感器校准和初始化
 */
#ifndef FOC_GIMBAL_H
#define FOC_GIMBAL_H

#include "gimbal/foc_protocol.h"
#include "zf_common_headfile.h"
#include "pin_mapping.h"

#include <stdbool.h>
#include <stdint.h>

/* ---- FOC 云台 UART 接口 ---- */
#ifndef FOC_GIMBAL_UART
#define FOC_GIMBAL_UART       UART_1
#endif

#ifndef FOC_GIMBAL_TX_PIN
#define FOC_GIMBAL_TX_PIN     UART1_TX_A8
#endif

#ifndef FOC_GIMBAL_RX_PIN
#define FOC_GIMBAL_RX_PIN     UART1_RX_A9
#endif

#ifndef FOC_GIMBAL_UART_BAUD
#define FOC_GIMBAL_UART_BAUD  115200u
#endif

/* ---- 默认参数 ---- */
#ifndef FOC_GIMBAL_DEFAULT_RUN_MODE
#define FOC_GIMBAL_DEFAULT_RUN_MODE  FOC_RUN_AUTO
#endif

#ifndef FOC_GIMBAL_STATUS_INTERVAL_MS
#define FOC_GIMBAL_STATUS_INTERVAL_MS  100u
#endif

#ifndef FOC_GIMBAL_INIT_TIMEOUT_MS
#define FOC_GIMBAL_INIT_TIMEOUT_MS     5000u
#endif

#ifndef FOC_GIMBAL_CMD_TIMEOUT_MS
#define FOC_GIMBAL_CMD_TIMEOUT_MS      100u
#endif

/* ---- 状态码 ---- */
typedef enum
{
    FOC_GIMBAL_OK = 0,
    FOC_GIMBAL_ERROR = -1,
    FOC_GIMBAL_ERROR_UART = -2,
    FOC_GIMBAL_ERROR_OFFLINE = -3,
    FOC_GIMBAL_ERROR_SENSOR = -4,
    FOC_GIMBAL_ERROR_LIMIT = -5,
    FOC_GIMBAL_ERROR_NOT_INIT = -6,
    FOC_GIMBAL_ERROR_TIMEOUT = -7,
    FOC_GIMBAL_ERROR_FAULT = -8,
} FocGimbalStatus;

/* ---- 云台实例 ---- */
typedef struct
{
    uart_index_enum    uart;
    bool               initialized;
    bool               enabled;
    bool               sensor_valid;
    foc_run_mode_t     run_mode;
    float              yaw_target_deg;
    float              pitch_target_deg;
    float              yaw_angle_deg;
    float              pitch_angle_deg;
    float              yaw_velocity_dps;
    float              pitch_velocity_dps;
    float              yaw_error_deg;
    float              pitch_error_deg;
    uint32_t           last_status_ms;
    uint32_t           last_cmd_ms;
    uint32_t           fault_count;
    foc_motor_status_t m0_status;
    foc_motor_status_t m1_status;
} FocGimbal;

extern FocGimbal g_foc_gimbal;

/* ---- API ---- */

FocGimbalStatus foc_gimbal_init(FocGimbal *gimbal);
FocGimbalStatus foc_gimbal_enable(FocGimbal *gimbal, bool enable);
FocGimbalStatus foc_gimbal_stop(FocGimbal *gimbal);
FocGimbalStatus foc_gimbal_move_to(FocGimbal *gimbal,
    float yaw_deg, float pitch_deg);
FocGimbalStatus foc_gimbal_move_to_fast(FocGimbal *gimbal,
    float yaw_deg, float pitch_deg);
FocGimbalStatus foc_gimbal_hold(FocGimbal *gimbal);
FocGimbalStatus foc_gimbal_hold_dual(FocGimbal *gimbal);
FocGimbalStatus foc_gimbal_read_position(FocGimbal *gimbal,
    float *yaw_deg, float *pitch_deg);
FocGimbalStatus foc_gimbal_update_status(FocGimbal *gimbal,
    uint32_t now_ms);
FocGimbalStatus foc_gimbal_set_run_mode(FocGimbal *gimbal,
    foc_run_mode_t mode);
bool           foc_gimbal_is_online(const FocGimbal *gimbal);
bool           foc_gimbal_is_sensor_valid(const FocGimbal *gimbal);

#endif
