#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdbool.h>
#include <stdint.h>

#include "emm_stepper.h"
#include "device/t8_gray_sensor.h"
#include "zf_common_headfile.h"
#include "pin_mapping.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Default wiring for LCSC Tianmengxing MSPM0G3507 board. */
#ifndef GIMBAL_EMM_UART
#define GIMBAL_EMM_UART BOARD_EMM_UART
#endif

#ifndef GIMBAL_EMM_UART_TX_PIN
#define GIMBAL_EMM_UART_TX_PIN BOARD_EMM_UART_TX
#endif

#ifndef GIMBAL_EMM_UART_RX_PIN
#define GIMBAL_EMM_UART_RX_PIN BOARD_EMM_UART_RX
#endif

#ifndef GIMBAL_T8_UART
#define GIMBAL_T8_UART BOARD_T8_UART
#endif

#ifndef GIMBAL_T8_UART_TX_PIN
#define GIMBAL_T8_UART_TX_PIN BOARD_T8_UART_TX
#endif

#ifndef GIMBAL_T8_UART_RX_PIN
#define GIMBAL_T8_UART_RX_PIN BOARD_T8_UART_RX
#endif

#ifndef GIMBAL_PITCH_MOTOR_ADDRESS
#define GIMBAL_PITCH_MOTOR_ADDRESS 1u   /* 电机1 = 上下 */
#endif

#ifndef GIMBAL_YAW_MOTOR_ADDRESS
#define GIMBAL_YAW_MOTOR_ADDRESS 2u     /* 电机2 = 左右 */
#endif

#ifndef GIMBAL_DEFAULT_MICROSTEP
#define GIMBAL_DEFAULT_MICROSTEP 16u
#endif

#ifndef GIMBAL_DEFAULT_SPEED_RPM
#define GIMBAL_DEFAULT_SPEED_RPM 300u
#endif

#ifndef GIMBAL_DEFAULT_ACCELERATION
#define GIMBAL_DEFAULT_ACCELERATION 50u
#endif

/* ================================================================
 *  PITCH pre-calibrated limits (encoder-based, persistent)
 *
 *  Encoder 0x31 is absolute — same physical position = same value
 *  regardless of power cycle.  Set USE_PRECALIB_PITCH to 1 to
 *  skip the auto-calibration routine and use these stored values.
 * ================================================================ */
#ifndef GIMBAL_USE_PRECALIB_PITCH
#define GIMBAL_USE_PRECALIB_PITCH 1
#endif

#ifndef GIMBAL_USE_PRECALIB_PITCH
#define GIMBAL_USE_PRECALIB_PITCH 1
#endif

/* === Encoder-based calibration constants (absolute, persistent) ===
   Measured by gimbal_calibrate_geared().  Fill in after first run.
   Encoder (0x31) reads 0-65535 counts → 0-360 deg.  Same value on
   every power-up for the same physical position. */
#ifndef GIMBAL_PITCH_RATIO
#define GIMBAL_PITCH_RATIO         4.0f
#endif
#ifndef GIMBAL_YAW_RATIO
#define GIMBAL_YAW_RATIO           8.0f
#endif
#ifndef GIMBAL_PITCH_BACK_ANGLE
#define GIMBAL_PITCH_BACK_ANGLE   -85.0f
#endif
/* Encoder degrees at the CW limit (stall) and at horizontal (after back-off).
   Set GIMBAL_USE_PRECALIB_PITCH=0 and run calibration once to measure these. */
#ifndef GIMBAL_PITCH_ENC_LIMIT
#define GIMBAL_PITCH_ENC_LIMIT     136.5f
#endif
#ifndef GIMBAL_PITCH_ENC_HORIZONTAL
#define GIMBAL_PITCH_ENC_HORIZONTAL 326.4f
#endif
#ifndef GIMBAL_YAW_ENC_CENTER
#define GIMBAL_YAW_ENC_CENTER      0.0f
#endif

typedef enum
{
    GIMBAL_OK = 0,
    GIMBAL_ERROR = -1,
    GIMBAL_ERROR_MOTOR = -2,
    GIMBAL_ERROR_SENSOR = -3,
    GIMBAL_ERROR_CALIB = -4,
} GimbalStatus;

/* ---- Calibration types ---- */

/* Geared calibration: maps motor encoder (fast, small gear)
   to actual gimbal angle (slow, large gear) via a ratio. */
typedef struct
{
    float enc_at_zero_deg;     /* encoder value when gimbal is at 0° (horizontal) */
    float enc_at_max_deg;      /* encoder value when gimbal is at max angle */
    float max_gimbal_deg;      /* actual max gimbal angle (e.g. 85°) */
    float gear_ratio;          /* motor_deg / gimbal_deg */
    bool  calibrated;
} GimbalGearedCalib;

typedef struct
{
    uint16_t explore_speed_rpm;    /* Slow jog speed for limit exploration (default 30) */
    uint8_t  explore_acceleration; /* Acceleration during exploration (default 5) */
    uint8_t  explore_attempts;     /* Number of rounds per axis (default 5) */
    uint32_t stall_check_ms;       /* Polling interval during exploration (default 50) */
    uint32_t stall_timeout_ms;     /* Max time per direction before giving up (default 15000) */
} GimbalCalibConfig;

typedef struct
{
    float   min_deg;          /* Lower limit angle */
    float   max_deg;          /* Upper limit angle */
    float   range_deg;        /* Total travel range */
    float   mid_deg;          /* Midpoint angle */
    bool    calibrated;       /* true after successful calibration */
} GimbalAxisCalib;

/* ---- Gimbal struct ---- */

typedef struct
{
    EmmDevice yaw;
    EmmDevice pitch;
    T8UartDevice sensor;
    float yaw_angle_deg;
    float pitch_angle_deg;
    float yaw_min_deg;
    float yaw_max_deg;
    float pitch_min_deg;
    float pitch_max_deg;
    uint16_t microstep;
    uint16_t speed_rpm;
    uint8_t acceleration;

    /* Calibration */
    GimbalCalibConfig calib_config;
    GimbalAxisCalib   calib_yaw;
    GimbalAxisCalib   calib_pitch;
    GimbalGearedCalib geared_pitch;
    GimbalGearedCalib geared_yaw;
    bool manual_mode;
} Gimbal;

extern Gimbal g_gimbal;

GimbalStatus gimbal_init(Gimbal *gimbal);
GimbalStatus gimbal_enable(Gimbal *gimbal, bool enable);
GimbalStatus gimbal_stop(Gimbal *gimbal);
GimbalStatus gimbal_zero_position(Gimbal *gimbal);
GimbalStatus gimbal_move_relative(Gimbal *gimbal, float yaw_delta_deg, float pitch_delta_deg);
GimbalStatus gimbal_move_to(Gimbal *gimbal, float yaw_deg, float pitch_deg);
GimbalStatus gimbal_update_from_t8(Gimbal *gimbal);
GimbalStatus gimbal_read_sensor(Gimbal *gimbal, uint8_t values[T8_SENSOR_COUNT], uint8_t *digital_bits);
void gimbal_debug_probe_emm_uart(void);

/* ---- Calibration API ---- */

/* Auto-calibrate both axes. Blocking call, takes ~15-40 s per axis.
   Prints detailed debug logs to the console. */
GimbalStatus gimbal_auto_calibrate(Gimbal *gimbal);

/* Geared calibration: user manually sets PITCH to 0° (straight forward),
   then the system explores both limits and computes the gear ratio.
   Call this once after mechanical assembly, then save the results. */
GimbalStatus gimbal_calibrate_geared(Gimbal *gimbal);

/* Calibrate a single axis. Internal use; also callable standalone.
   `motor`    – pointer to the EmmDevice (yaw or pitch)
   `axis_name`– "YAW" or "PITCH" for log output
   `result`   – output: filled with min/max/range/mid */
GimbalStatus gimbal_calibrate_axis(Gimbal *gimbal, EmmDevice *motor,
                                   const char *axis_name, GimbalAxisCalib *result);

/* ---- Position read ---- */

/* Read actual motor positions (realtime position 0x36) via forced-response.
   Returns the physical encoder-based angle in degrees. */
GimbalStatus gimbal_read_actual_position(Gimbal *gimbal, float *yaw_deg, float *pitch_deg);

/* ---- Soft limits ---- */

/* Apply calibration results as the active software limits. */
void gimbal_set_limits_from_calib(Gimbal *gimbal);

/* ---- Manual mode ---- */

/* Disable both motors so the gimbal can be rotated by hand. */
GimbalStatus gimbal_enter_manual_mode(Gimbal *gimbal);

/* Re-enable motors and read the current position to re-sync tracking. */
GimbalStatus gimbal_exit_manual_mode(Gimbal *gimbal);

#ifdef __cplusplus
}
#endif

#endif
