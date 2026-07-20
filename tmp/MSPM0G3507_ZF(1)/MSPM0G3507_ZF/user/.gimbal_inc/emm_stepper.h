#ifndef EMM_STEPPER_H
#define EMM_STEPPER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EMM_STEPPER_DEFAULT_BAUDRATE 115200u
#define EMM_STEPPER_DEFAULT_ADDRESS 1u
#define EMM_STEPPER_BROADCAST_ADDRESS 0u
#define EMM_STEPPER_MAX_RETRIES 3u
#define EMM_STEPPER_MAX_FRAME_SIZE 64u
#define EMM_STEPPER_RX_BUFFER_SIZE 256u
#define EMM_STEPPER_RX_READ_CHUNK 32u
#define EMM_STEPPER_DEFAULT_TIMEOUT_MS 20u
#define EMM_STEPPER_REACHED_TIMEOUT_MS 3000u
#define EMM_STEPPER_POLL_ATTEMPTS 4u
#define EMM_STEPPER_MATCH_ANY 0xFFu

typedef enum
{
    EMM_OK = 0,
    EMM_ERROR = -1,
    EMM_ERROR_INVALID_ARG = -2,
    EMM_ERROR_IO = -3,
    EMM_ERROR_TIMEOUT = -4,
    EMM_ERROR_BAD_RESPONSE = -5,
    EMM_ERROR_CHECKSUM = -6,
    EMM_ERROR_PARAM = -7,
    EMM_ERROR_FORMAT = -8,
    EMM_ERROR_OVERFLOW = -9,
    EMM_ERROR_NO_RESPONSE = -10,
} EmmStatus;

typedef enum
{
    EMM_CHECKSUM_FIXED = 0,
    EMM_CHECKSUM_XOR = 1,
    EMM_CHECKSUM_CRC8 = 2,
    EMM_CHECKSUM_MODBUS = 3,
    EMM_CHECKSUM_DMX512 = 4,
} EmmChecksumMode;

typedef enum
{
    EMM_DIRECTION_CW = 0,
    EMM_DIRECTION_CCW = 1,
} EmmDirection;

typedef enum
{
    EMM_SYNC_IMMEDIATE = 0,
    EMM_SYNC_BUFFERED = 1,
} EmmSyncFlag;

typedef enum
{
    EMM_STORE_NO = 0,
    EMM_STORE_YES = 1,
} EmmStoreFlag;

typedef enum
{
    EMM_MOTION_RELATIVE_LAST = 0,
    EMM_MOTION_ABSOLUTE = 1,
    EMM_MOTION_RELATIVE_CURRENT = 2,
} EmmMotionMode;

typedef enum
{
    EMM_HOME_NEAREST = 0,
    EMM_HOME_DIRECTION = 1,
    EMM_HOME_COLLISION = 2,
    EMM_HOME_LIMIT_SWITCH = 3,
    EMM_HOME_ABS_ZERO = 4,
    EMM_HOME_LAST_POWER_OFF = 5,
} EmmHomingMode;

typedef enum
{
    EMM_CONTROL_OPEN_LOOP = 0,
    EMM_CONTROL_CLOSED_LOOP = 1,
} EmmControlMode;

typedef enum
{
    EMM_MOTOR_09_DEG = 0x19,
    EMM_MOTOR_18_DEG = 0x32,
} EmmMotorType;

typedef enum
{
    EMM_FIRMWARE_X = 0,
    EMM_FIRMWARE_EMM = 1,
    EMM_FIRMWARE_EMM_TURBO = 2,
} EmmFirmwareType;

typedef enum
{
    EMM_BAUD_9600 = 0,
    EMM_BAUD_19200 = 1,
    EMM_BAUD_25000 = 2,
    EMM_BAUD_38400 = 3,
    EMM_BAUD_57600 = 4,
    EMM_BAUD_115200 = 5,
    EMM_BAUD_256000 = 6,
    EMM_BAUD_512000 = 7,
    EMM_BAUD_921600 = 8,
} EmmBaudRate;

typedef enum
{
    EMM_CAN_10K = 0,
    EMM_CAN_20K = 1,
    EMM_CAN_50K = 2,
    EMM_CAN_83K = 3,
    EMM_CAN_100K = 4,
    EMM_CAN_125K = 5,
    EMM_CAN_250K = 6,
    EMM_CAN_500K = 7,
    EMM_CAN_800K = 8,
    EMM_CAN_1M = 9,
} EmmCanRate;

typedef enum
{
    EMM_RESPONSE_NONE = 0,
    EMM_RESPONSE_RECEIVE = 1,
    EMM_RESPONSE_REACHED = 2,
    EMM_RESPONSE_BOTH = 3,
    EMM_RESPONSE_OTHER = 4,
} EmmResponseMode;

typedef enum
{
    EMM_STALL_DISABLE = 0,
    EMM_STALL_ENABLE = 1,
    EMM_STALL_AUTO_ZERO = 2,
} EmmStallProtect;

typedef enum
{
    EMM_PULSE_PORT_OFF = 0,
    EMM_PULSE_PORT_OPEN = 1,
    EMM_PULSE_PORT_FOC = 2,
    EMM_PULSE_PORT_ESI_RCO = 3,
    EMM_PULSE_PORT_PLR_ESI = 4,
} EmmPulsePortMode;

typedef enum
{
    EMM_SERIAL_PORT_OFF = 0,
    EMM_SERIAL_PORT_ESI_ALO = 1,
    EMM_SERIAL_PORT_UART = 2,
    EMM_SERIAL_PORT_CAN = 3,
    EMM_SERIAL_PORT_ULR_ESI = 4,
} EmmSerialPortMode;

typedef enum
{
    EMM_ENABLE_LEVEL_LOW = 0,
    EMM_ENABLE_LEVEL_HIGH = 1,
    EMM_ENABLE_LEVEL_HOLD = 2,
} EmmEnableLevel;

typedef enum
{
    EMM_DIR_LEVEL_CW = 0,
    EMM_DIR_LEVEL_CCW = 1,
} EmmDirLevel;

typedef size_t (*EmmWriteFn)(const uint8_t *data, size_t length, void *user_data);
typedef size_t (*EmmReadFn)(uint8_t *data, size_t length, uint32_t timeout_ms, void *user_data);
typedef void (*EmmFlushFn)(void *user_data);
typedef void (*EmmDelayFn)(uint32_t delay_ms, void *user_data);

typedef struct
{
    EmmWriteFn write;
    EmmReadFn read;
    EmmFlushFn flush_input;
    EmmFlushFn flush_output;
    EmmDelayFn delay_ms;
    void *user_data;
} EmmTransport;

typedef struct
{
    uint8_t address;
    EmmChecksumMode checksum_mode;
    uint32_t timeout_ms;
    uint32_t retry_delay_ms;
    uint8_t max_retries;
    EmmTransport transport;

    /* Write commands use EMM_RESPONSE_NONE by default (fire-and-forget).
       Read commands (emm_get_*_forced) temporarily switch to EMM_RESPONSE_RECEIVE
       to get a response, then restore the original mode.
       Use one EmmDevice instance per physical UART bus when reading. */
    uint32_t reached_timeout_ms;
    EmmResponseMode response_mode;
    bool strict_frame_check;
    bool auto_flush_before_write;
    bool auto_flush_before_read;      /* Clear RX buffer before forced reads */
    uint8_t poll_attempts;
    uint8_t rx_buffer[EMM_STEPPER_RX_BUFFER_SIZE];
    size_t rx_head;
    size_t rx_tail;
    size_t rx_overflow_count;
} EmmDevice;

typedef struct
{
    EmmDirection direction;
    uint16_t speed_rpm;
    uint8_t acceleration;
    EmmSyncFlag sync_flag;
} EmmJogParams;

typedef struct
{
    EmmDirection direction;
    uint16_t speed_rpm;
    uint8_t acceleration;
    uint32_t pulse_count;
    EmmMotionMode motion_mode;
    EmmSyncFlag sync_flag;
} EmmPositionParams;

typedef struct
{
    EmmHomingMode homing_mode;
    EmmDirection homing_direction;
    uint16_t homing_speed_rpm;
    uint32_t homing_timeout_ms;
    uint16_t collision_speed_rpm;
    uint16_t collision_current_ma;
    uint16_t collision_time_ms;
    bool auto_home;
} EmmHomingParams;

typedef struct
{
    uint16_t firmware_version;
    uint8_t hw_series;
    uint8_t hw_type;
    uint8_t hw_version;
} EmmVersionParams;

typedef struct
{
    uint16_t phase_resistance_mohm;
    uint16_t phase_inductance_uh;
} EmmMotorRHParams;

typedef struct
{
    uint32_t kp;
    uint32_t ki;
    uint32_t kd;
} EmmPIDParams;

typedef struct
{
    bool encoder_ready;
    bool calibrated;
    bool is_homing;
    bool homing_failed;
    bool over_temp;
    bool over_current;
} EmmHomingStatus;

typedef struct
{
    bool enabled;
    bool position_reached;
    bool stall_detected;
    bool stall_protected;
    bool left_limit;
    bool right_limit;
    bool power_off_flag;
} EmmMotorStatus;

typedef struct
{
    uint16_t bus_voltage_mv;
    uint16_t phase_current_ma;
    uint16_t encoder_value;
    int32_t target_position;
    int16_t realtime_speed_rpm;
    int32_t realtime_position;
    int32_t position_error;
    EmmHomingStatus homing_status;
    EmmMotorStatus motor_status;
} EmmSystemStatusParams;

typedef struct
{
    EmmMotorType motor_type;
    EmmPulsePortMode pulse_port_mode;
    EmmSerialPortMode serial_port_mode;
    EmmEnableLevel enable_level;
    EmmDirLevel dir_level;
    uint16_t microstep;
    bool microstep_interp;
    uint16_t open_loop_current_ma;
    uint16_t closed_loop_current_ma;
    uint16_t max_voltage;
    EmmBaudRate baud_rate;
    EmmCanRate can_rate;
    uint8_t motor_id;
    EmmChecksumMode checksum_mode;
    EmmResponseMode response_mode;
    EmmStallProtect stall_protect;
    uint16_t stall_speed_rpm;
    uint16_t stall_current_ma;
    uint16_t stall_time_ms;
    uint16_t position_window_x01deg;
} EmmConfigParams;

typedef struct
{
    bool store;
    EmmDirection direction;
    uint16_t speed_rpm;
    uint8_t acceleration;
    bool enable_en_control;
} EmmAutoRunParams;

typedef struct
{
    uint8_t address;
    uint8_t code;
    size_t length;
    uint8_t bytes[EMM_STEPPER_MAX_FRAME_SIZE];
} EmmRxFrame;

void emm_init(EmmDevice *device, const EmmTransport *transport, uint8_t address);
void emm_select_address(EmmDevice *device, uint8_t address);
void emm_set_checksum_mode(EmmDevice *device, EmmChecksumMode mode);
void emm_set_response_mode_local(EmmDevice *device, EmmResponseMode mode);
void emm_set_timeouts(EmmDevice *device, uint32_t command_timeout_ms, uint32_t reached_timeout_ms);
void emm_set_strict_frame_check(EmmDevice *device, bool enable);
void emm_set_auto_flush_before_write(EmmDevice *device, bool enable);
void emm_rx_clear(EmmDevice *device);
size_t emm_rx_available(const EmmDevice *device);
size_t emm_rx_overflow_count(const EmmDevice *device);
uint8_t emm_calculate_checksum(const uint8_t *data, size_t length, EmmChecksumMode mode);
EmmStatus emm_poll(EmmDevice *device, uint32_t timeout_ms);
EmmStatus emm_read_fixed_frame(EmmDevice *device, uint8_t expected_address, uint8_t expected_code, uint8_t *response, size_t response_length, uint32_t timeout_ms);
EmmStatus emm_read_dynamic_frame(EmmDevice *device, uint8_t expected_address, uint8_t expected_code, uint8_t *response, size_t response_capacity, size_t *response_length, uint32_t timeout_ms);
EmmStatus emm_read_any_frame(EmmDevice *device, EmmRxFrame *frame, uint32_t timeout_ms);
EmmStatus emm_wait_reached(EmmDevice *device, uint8_t expected_code, uint8_t *response, size_t response_length, uint32_t timeout_ms);
EmmStatus emm_send_raw_no_response(EmmDevice *device, const uint8_t *body, size_t body_length);
EmmStatus emm_send_raw(EmmDevice *device, const uint8_t *body, size_t body_length, uint8_t *response, size_t response_length);
EmmStatus emm_send_raw_dynamic(EmmDevice *device, const uint8_t *body, size_t body_length, uint8_t *response, size_t response_capacity, size_t *response_length);

EmmStatus emm_calibrate_encoder(EmmDevice *device);
EmmStatus emm_restart(EmmDevice *device);
EmmStatus emm_zero_position(EmmDevice *device);
EmmStatus emm_clear_protection(EmmDevice *device);
EmmStatus emm_factory_reset(EmmDevice *device);
EmmStatus emm_enable(EmmDevice *device, bool enable, EmmSyncFlag sync_flag);
EmmStatus emm_disable(EmmDevice *device, EmmSyncFlag sync_flag);
EmmStatus emm_jog(EmmDevice *device, const EmmJogParams *params);
EmmStatus emm_move_pulses(EmmDevice *device, const EmmPositionParams *params);
EmmStatus emm_move_degrees(EmmDevice *device, float degrees, uint16_t speed_rpm, uint8_t acceleration, EmmMotionMode motion_mode, uint16_t microstep, EmmSyncFlag sync_flag);
EmmStatus emm_move_revolutions(EmmDevice *device, float revolutions, uint16_t speed_rpm, uint8_t acceleration, EmmMotionMode motion_mode, uint16_t microstep, EmmSyncFlag sync_flag);
EmmStatus emm_stop(EmmDevice *device, EmmSyncFlag sync_flag);
EmmStatus emm_sync_move(EmmDevice *device);

EmmStatus emm_set_home_zero(EmmDevice *device, EmmStoreFlag store);
EmmStatus emm_home(EmmDevice *device, EmmHomingMode mode, EmmSyncFlag sync_flag);
EmmStatus emm_stop_home(EmmDevice *device);
EmmStatus emm_get_homing_status(EmmDevice *device, EmmHomingStatus *status);
EmmStatus emm_get_homing_params(EmmDevice *device, EmmHomingParams *params);
EmmStatus emm_set_homing_params(EmmDevice *device, const EmmHomingParams *params, EmmStoreFlag store);

EmmStatus emm_get_version(EmmDevice *device, EmmVersionParams *version);
EmmStatus emm_get_motor_rh(EmmDevice *device, EmmMotorRHParams *params);
EmmStatus emm_get_bus_voltage(EmmDevice *device, uint16_t *voltage_mv);
EmmStatus emm_get_bus_current(EmmDevice *device, uint16_t *current_ma);
EmmStatus emm_get_phase_current(EmmDevice *device, uint16_t *current_ma);
EmmStatus emm_get_encoder(EmmDevice *device, uint16_t *encoder);
EmmStatus emm_get_encoder_degrees(EmmDevice *device, float *degrees);
EmmStatus emm_get_pulse_count(EmmDevice *device, int32_t *pulse_count);
EmmStatus emm_get_target_position(EmmDevice *device, float *degrees);
EmmStatus emm_get_realtime_speed(EmmDevice *device, int16_t *speed_rpm);
EmmStatus emm_get_realtime_position(EmmDevice *device, float *degrees);
EmmStatus emm_get_position_error(EmmDevice *device, float *degrees);
EmmStatus emm_get_temperature(EmmDevice *device, int16_t *temperature_c);
EmmStatus emm_get_motor_status(EmmDevice *device, EmmMotorStatus *status);
EmmStatus emm_get_pid(EmmDevice *device, EmmPIDParams *params);
EmmStatus emm_get_config(EmmDevice *device, EmmConfigParams *params);
EmmStatus emm_get_system_status(EmmDevice *device, EmmSystemStatusParams *params);

EmmStatus emm_set_id(EmmDevice *device, uint8_t new_id, EmmStoreFlag store);
EmmStatus emm_set_microstep(EmmDevice *device, uint16_t microstep, EmmStoreFlag store);
EmmStatus emm_set_loop_mode(EmmDevice *device, EmmControlMode mode, EmmStoreFlag store);
EmmStatus emm_set_open_loop_current(EmmDevice *device, uint16_t current_ma, EmmStoreFlag store);
EmmStatus emm_set_closed_loop_current(EmmDevice *device, uint16_t current_ma, EmmStoreFlag store);
EmmStatus emm_set_pid(EmmDevice *device, const EmmPIDParams *params, EmmStoreFlag store);
EmmStatus emm_set_motor_direction(EmmDevice *device, EmmDirection direction, EmmStoreFlag store);
EmmStatus emm_set_position_window(EmmDevice *device, float window_deg, EmmStoreFlag store);
EmmStatus emm_set_heartbeat_time(EmmDevice *device, uint32_t time_ms, EmmStoreFlag store);
EmmStatus emm_set_auto_run(EmmDevice *device, const EmmAutoRunParams *params);
EmmStatus emm_set_config(EmmDevice *device, const EmmConfigParams *params, EmmStoreFlag store);
EmmStatus emm_set_scale_input(EmmDevice *device, bool enable, EmmStoreFlag store);
EmmStatus emm_set_lock_button(EmmDevice *device, bool lock, EmmStoreFlag store);
EmmStatus emm_broadcast_get_id(EmmDevice *device, uint8_t *motor_id);

/* ===== Forced-response read functions =====
   These temporarily override device->response_mode to EMM_RESPONSE_RECEIVE,
   perform the read, and restore the original mode. Use these when the device
   is normally in EMM_RESPONSE_NONE (fire-and-forget) but you need to read data. */
EmmStatus emm_get_realtime_position_forced(EmmDevice *device, float *degrees);
EmmStatus emm_get_encoder_forced(EmmDevice *device, uint16_t *encoder);
EmmStatus emm_get_motor_status_forced(EmmDevice *device, EmmMotorStatus *status);
EmmStatus emm_get_system_status_forced(EmmDevice *device, EmmSystemStatusParams *params);
EmmStatus emm_get_pulse_count_forced(EmmDevice *device, int32_t *pulse_count);

/* ===== Calibration helpers ===== */
/* Send jog command without waiting for a response (for fast polling during calibration). */
EmmStatus emm_jog_no_response(EmmDevice *device, const EmmJogParams *params);
/* Clear stall protection and re-enable the motor (recover from a locked state). */
EmmStatus emm_clear_stall_and_recover(EmmDevice *device);

#ifdef __cplusplus
}
#endif

#endif
