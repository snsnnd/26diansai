#include "gimbal/gimbal.h"
#include "gimbal/serial_rx_buffer.h"

#ifndef GIMBAL_PITCH_RATIO
#define GIMBAL_PITCH_RATIO   4.0f
#endif
#ifndef GIMBAL_YAW_RATIO
#define GIMBAL_YAW_RATIO     8.0f
#endif
#ifndef GIMBAL_PITCH_BACK_ANGLE
#define GIMBAL_PITCH_BACK_ANGLE  -85.0f
#endif

#define GIMBAL_UART_READ_POLL_MS 1u
#define GIMBAL_T8_CENTER_X 3.5f
#define GIMBAL_T8_DEADBAND 0.18f
#define GIMBAL_T8_TRACK_GAIN_DEG 0.7f

typedef struct
{
    uart_index_enum uart;
} GimbalUartContext;

static GimbalUartContext EmmUartContext = { GIMBAL_EMM_UART };
static uint8_t EmmRxStorage[128];
static SerialRxBuffer EmmRxBuffer;

/* ---- Half-duplex TX pin control ----
 * EMM42 uses a single-wire half-duplex bus.  After the MSPM0 transmits,
 * the TX pin (PB15, AF2) must be floated (switched to input) so the
 * motor can drive the bus for its response.  Before the next write we
 * restore the pin to UART TX alternate function. */

/* Extract the raw GPIO pin from the compound UART-pin enum.
   GIMBAL_EMM_UART_TX == UART2_TX_B15 → B15 (0x2F). */
#define EMM_TX_GPIO_PIN  ((gpio_pin_enum)(GIMBAL_EMM_UART_TX_PIN & UART_PIN_INDEX_MASK))

static void emm_tx_set_input(void)
{
    /* Floating input – pin is high-Z, motor can drive the bus. */
    gpio_init(EMM_TX_GPIO_PIN, GPI, GPI_FLOATING_IN, GPIO_LOW);
}

static void emm_tx_set_output(void)
{
    /* Restore UART TX alternate function (AF2, push-pull). */
    afio_init(EMM_TX_GPIO_PIN, GPO, GPIO_AF2, GPO_AF_PUSH_PULL);
}

static void emm_uart_rx_callback(uint32 state, void *ptr)
{
    GimbalUartContext *context = (GimbalUartContext *)ptr;
    uint8 byte;

    if (context == NULL || (state & UART_INTERRUPT_STATE_RX) == 0u)
    {
        return;
    }

    while (uart_query_byte(context->uart, &byte) == ZF_TRUE)
    {
        (void)serial_rx_buffer_push(&EmmRxBuffer, byte);
    }
}

static const char *emm_status_name(EmmStatus status)
{
    switch (status)
    {
        case EMM_OK: return "ok";
        case EMM_ERROR_INVALID_ARG: return "invalid-arg";
        case EMM_ERROR_IO: return "io";
        case EMM_ERROR_TIMEOUT: return "timeout/no-response";
        case EMM_ERROR_BAD_RESPONSE: return "bad-response";
        case EMM_ERROR_CHECKSUM: return "checksum";
        case EMM_ERROR_PARAM: return "param";
        case EMM_ERROR_FORMAT: return "format";
        case EMM_ERROR_OVERFLOW: return "overflow";
        case EMM_ERROR_NO_RESPONSE: return "no-response-mode";
        case EMM_ERROR: return "error";
        default: return "unknown";
    }
}

Gimbal g_gimbal;

static float clamp_float(float value, float min_value, float max_value)
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

static void delay_ms(uint32_t delay_ms_value, void *user_data)
{
    (void)user_data;
    system_delay_ms(delay_ms_value);
}

/* Read actual encoder-based angle (0-360° per revolution).
   Uses code 0x31 (encoder count) rather than 0x36 (realtime position)
   because 0x36 returns a commanded/computed value that can drift
   when the motor is stalled or underloaded. */
static EmmStatus read_encoder_deg(EmmDevice *motor, float *deg)
{
    uint16_t encoder;
    EmmStatus s = emm_get_encoder_forced(motor, &encoder);
    if (s == EMM_OK && deg != 0)
    {
        *deg = ((float)encoder * 360.0f) / 65536.0f;
    }
    return s;
}

static size_t zf_uart_write_adapter(const uint8_t *data, size_t length, void *user_data)
{
    GimbalUartContext *context = (GimbalUartContext *)user_data;

    if (context == NULL || data == NULL)
    {
        return 0u;
    }

    /* Half-duplex: restore TX pin to UART output before writing. */
    emm_tx_set_output();

    uart_write_buffer(context->uart, data, (uint32)length);

    /* Wait until the last byte has been fully shifted out.
       DL_UART_isBusy returns true while TX FIFO or shift register has data. */
    while (DL_UART_isBusy(UART2))
    {
        /* spin */
    }

    /* Float the TX pin so the motor can drive the bus for its reply. */
    emm_tx_set_input();

    return length;
}

static size_t zf_uart_read_adapter(uint8_t *data, size_t length, uint32_t timeout_ms, void *user_data)
{
    size_t count = 0u;
    uint32_t elapsed_ms = 0u;

    (void)user_data;

    if (data == NULL)
    {
        return 0u;
    }

    while (count < length)
    {
        if (serial_rx_buffer_pop(&EmmRxBuffer, &data[count]))
        {
            count++;
            continue;
        }

        if (elapsed_ms >= (timeout_ms == 0u ? 1u : timeout_ms))
        {
            break;
        }

        system_delay_ms(GIMBAL_UART_READ_POLL_MS);
        elapsed_ms += GIMBAL_UART_READ_POLL_MS;
    }

    return count;
}

static void zf_uart_flush_input_adapter(void *user_data)
{
    uint8 byte;

    (void)user_data;

    serial_rx_buffer_clear(&EmmRxBuffer);
    while (uart_query_byte(GIMBAL_EMM_UART, &byte) == ZF_TRUE)
    {
    }
}

static void zf_uart_flush_output_adapter(void *user_data)
{
    (void)user_data;
}

static EmmStatus move_axis_to(EmmDevice *motor, uint8_t address, float current_deg, float target_deg, uint16_t speed_rpm, uint8_t acceleration, uint16_t microstep)
{
    emm_select_address(motor, address);
    return emm_move_degrees(
        motor,
        target_deg - current_deg,
        speed_rpm,
        acceleration,
        EMM_MOTION_RELATIVE_CURRENT,
        microstep,
        EMM_SYNC_IMMEDIATE);
}

static bool find_line_center(const uint8_t values[T8_SENSOR_COUNT], float *center)
{
    uint16_t sum = 0u;
    uint16_t weighted_sum = 0u;

    for (uint8_t i = 0u; i < T8_SENSOR_COUNT; ++i)
    {
        uint8_t weight = (uint8_t)(255u - values[i]);
        sum = (uint16_t)(sum + weight);
        weighted_sum = (uint16_t)(weighted_sum + (uint16_t)(weight * i));
    }

    if (sum == 0u)
    {
        return false;
    }

    *center = (float)weighted_sum / (float)sum;
    return true;
}

GimbalStatus gimbal_init(Gimbal *gimbal)
{
    EmmTransport emm_transport;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    uart_init(GIMBAL_EMM_UART, EMM_STEPPER_DEFAULT_BAUDRATE, GIMBAL_EMM_UART_TX_PIN, GIMBAL_EMM_UART_RX_PIN);
    serial_rx_buffer_init(&EmmRxBuffer, EmmRxStorage, sizeof(EmmRxStorage));
    uart_set_callback(GIMBAL_EMM_UART, emm_uart_rx_callback, &EmmUartContext);
    uart_set_interrupt_config(GIMBAL_EMM_UART, UART_INTERRUPT_CONFIG_RX_ENABLE);

    /* Half-duplex: float TX pin initially so motor can send unsolicited data. */
    emm_tx_set_input();

    emm_transport.write = zf_uart_write_adapter;
    emm_transport.read = zf_uart_read_adapter;
    emm_transport.flush_input = zf_uart_flush_input_adapter;
    emm_transport.flush_output = zf_uart_flush_output_adapter;
    emm_transport.delay_ms = delay_ms;
    emm_transport.user_data = &EmmUartContext;

    emm_init(&gimbal->yaw, &emm_transport, GIMBAL_YAW_MOTOR_ADDRESS);
    gimbal->yaw.timeout_ms = 80u;
    gimbal->yaw.retry_delay_ms = 5u;
    gimbal->yaw.auto_flush_before_read = true;

    emm_init(&gimbal->pitch, &emm_transport, GIMBAL_PITCH_MOTOR_ADDRESS);
    gimbal->pitch.timeout_ms = 80u;
    gimbal->pitch.retry_delay_ms = 5u;
    gimbal->pitch.auto_flush_before_read = true;

    gimbal->yaw_angle_deg = 0.0f;
    gimbal->pitch_angle_deg = 0.0f;
    gimbal->yaw_min_deg = -179.0f;
    gimbal->yaw_max_deg =  179.0f;

#if GIMBAL_USE_PRECALIB_PITCH
    {
        gimbal->pitch_min_deg = GIMBAL_PITCH_BACK_ANGLE;
        gimbal->pitch_max_deg = 0.0f;
        if (gimbal->pitch_min_deg > gimbal->pitch_max_deg)
        {
            float t = gimbal->pitch_min_deg;
            gimbal->pitch_min_deg = gimbal->pitch_max_deg;
            gimbal->pitch_max_deg = t;
        }
    }
#else
    gimbal->pitch_min_deg = -35.0f;
    gimbal->pitch_max_deg =  35.0f;
#endif
    gimbal->microstep = GIMBAL_DEFAULT_MICROSTEP;
    gimbal->speed_rpm = GIMBAL_DEFAULT_SPEED_RPM;
    gimbal->acceleration = GIMBAL_DEFAULT_ACCELERATION;

    /* Default calibration config */
    gimbal->calib_config.explore_speed_rpm    = 30u;
    gimbal->calib_config.explore_acceleration = 20u;
    gimbal->calib_config.explore_attempts     = 5u;
    gimbal->calib_config.stall_check_ms       = 200u;
    gimbal->calib_config.stall_timeout_ms     = 15000u;

    gimbal->calib_yaw.calibrated   = false;
    gimbal->calib_pitch.calibrated = false;
    gimbal->geared_pitch.calibrated = false;
    gimbal->geared_yaw.calibrated   = false;
    gimbal->manual_mode = false;

    return GIMBAL_OK;
}

GimbalStatus gimbal_enable(Gimbal *gimbal, bool enable)
{
    EmmStatus yaw_status;
    EmmStatus pitch_status;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    yaw_status = emm_enable(&gimbal->yaw, enable, EMM_SYNC_IMMEDIATE);

    system_delay_ms(5u);

    pitch_status = emm_enable(&gimbal->pitch, enable, EMM_SYNC_IMMEDIATE);


    
    if (yaw_status != EMM_OK || pitch_status != EMM_OK)
    {
        printf("[GIMBAL] enable detail: yaw=%s(%d) pitch=%s(%d)\r\n",
            emm_status_name(yaw_status), yaw_status,
            emm_status_name(pitch_status), pitch_status);
        return GIMBAL_ERROR_MOTOR;
    }

    return GIMBAL_OK;
}

void gimbal_debug_probe_emm_uart(void)
{
    const uint8_t command[] = { 0x01u, 0xF3u, 0xABu, 0x01u, 0x00u, 0x6Bu };
    uint8_t byte;
    uint8_t rx_count = 0u;

    printf("[GIMBAL] UART2 probe tx: 01 F3 AB 01 00 6B\r\n");
    zf_uart_flush_input_adapter(&EmmUartContext);
    uart_write_buffer(GIMBAL_EMM_UART, command, sizeof(command));

    printf("[GIMBAL] UART2 irq rx:");
    for (uint32_t elapsed_ms = 0u; elapsed_ms < 500u; elapsed_ms++)
    {
        while (serial_rx_buffer_pop(&EmmRxBuffer, &byte))
        {
            printf(" %X", byte);
            rx_count++;
        }
        system_delay_ms(1u);
    }

    if (rx_count == 0u)
    {
        printf(" none");
    }
    printf(" overflow=%lu\r\n", (unsigned long)serial_rx_buffer_overflow_count(&EmmRxBuffer));
}

GimbalStatus gimbal_stop(Gimbal *gimbal)
{
    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    (void)emm_stop(&gimbal->yaw, EMM_SYNC_IMMEDIATE);
    (void)emm_stop(&gimbal->pitch, EMM_SYNC_IMMEDIATE);
    return GIMBAL_OK;
}

GimbalStatus gimbal_zero_position(Gimbal *gimbal)
{
    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    if (emm_zero_position(&gimbal->yaw) != EMM_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }
    if (emm_zero_position(&gimbal->pitch) != EMM_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }

    gimbal->yaw_angle_deg = 0.0f;
    gimbal->pitch_angle_deg = 0.0f;
    return GIMBAL_OK;
}

GimbalStatus gimbal_move_relative(Gimbal *gimbal, float yaw_delta_deg, float pitch_delta_deg)
{
    float cur_yaw, cur_pitch;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    gimbal_read_actual_position(gimbal, &cur_yaw, &cur_pitch);
    return gimbal_move_to(gimbal, cur_yaw + yaw_delta_deg, cur_pitch + pitch_delta_deg);
}

GimbalStatus gimbal_move_to(Gimbal *gimbal, float yaw_deg, float pitch_deg)
{
    float target_yaw, target_pitch;
    float cur_enc, target_enc, delta;
    int32_t pulses;

    if (gimbal == NULL) return GIMBAL_ERROR;

    target_yaw   = clamp_float(yaw_deg, gimbal->yaw_min_deg, gimbal->yaw_max_deg);
    target_pitch = clamp_float(pitch_deg, gimbal->pitch_min_deg, gimbal->pitch_max_deg);

    /* PITCH: convert gimbal deg back to target encoder, then delta */
    if (read_encoder_deg(&gimbal->pitch, &cur_enc) == EMM_OK)
    {
        target_enc = GIMBAL_PITCH_ENC_HORIZONTAL + target_pitch * GIMBAL_PITCH_RATIO;
        delta = target_enc - cur_enc;
        while (delta > 180.0f)  delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        pulses = (int32_t)(delta / 360.0f * (float)(200u * gimbal->microstep));
        {
            EmmPositionParams mp;
            mp.direction   = (pulses < 0) ? EMM_DIRECTION_CCW : EMM_DIRECTION_CW;
            mp.speed_rpm   = gimbal->speed_rpm;
            mp.acceleration = gimbal->acceleration;
            mp.pulse_count  = (uint32_t)(pulses < 0 ? -pulses : pulses);
            mp.motion_mode  = EMM_MOTION_RELATIVE_CURRENT;
            mp.sync_flag    = EMM_SYNC_IMMEDIATE;
            (void)emm_move_pulses(&gimbal->pitch, &mp);
        }
    }
    system_delay_ms(100u);

    /* YAW: convert gimbal deg back to target encoder, then delta */
    if (read_encoder_deg(&gimbal->yaw, &cur_enc) == EMM_OK)
    {
        target_enc = GIMBAL_YAW_ENC_CENTER + target_yaw * GIMBAL_YAW_RATIO;
        delta = target_enc - cur_enc;
        while (delta > 180.0f)  delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        pulses = (int32_t)(delta / 360.0f * (float)(200u * gimbal->microstep));
        {
            EmmPositionParams mp;
            mp.direction   = (pulses < 0) ? EMM_DIRECTION_CCW : EMM_DIRECTION_CW;
            mp.speed_rpm   = gimbal->speed_rpm;
            mp.acceleration = gimbal->acceleration;
            mp.pulse_count  = (uint32_t)(pulses < 0 ? -pulses : pulses);
            mp.motion_mode  = EMM_MOTION_RELATIVE_CURRENT;
            mp.sync_flag    = EMM_SYNC_IMMEDIATE;
            (void)emm_move_pulses(&gimbal->yaw, &mp);
        }
    }

    gimbal->yaw_angle_deg   = target_yaw;
    gimbal->pitch_angle_deg = target_pitch;
    return GIMBAL_OK;
}

GimbalStatus gimbal_read_sensor(Gimbal *gimbal, uint8_t values[T8_SENSOR_COUNT], uint8_t *digital_bits)
{
    if (gimbal == NULL || values == NULL)
    {
        return GIMBAL_ERROR;
    }

    if (t8_uart_get_gray8_all(&gimbal->sensor, values) != T8_OK)
    {
        return GIMBAL_ERROR_SENSOR;
    }

    if (digital_bits != NULL)
    {
        (void)t8_uart_get_digital(&gimbal->sensor, digital_bits);
    }

    return GIMBAL_OK;
}

GimbalStatus gimbal_update_from_t8(Gimbal *gimbal)
{
    uint8_t values[T8_SENSOR_COUNT];
    float center;
    float error;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    if (gimbal_read_sensor(gimbal, values, NULL) != GIMBAL_OK)
    {
        return GIMBAL_ERROR_SENSOR;
    }

    if (!find_line_center(values, &center))
    {
        return GIMBAL_OK;
    }

    error = center - GIMBAL_T8_CENTER_X;
    if (error > -GIMBAL_T8_DEADBAND && error < GIMBAL_T8_DEADBAND)
    {
        return GIMBAL_OK;
    }

    return gimbal_move_relative(gimbal, error * GIMBAL_T8_TRACK_GAIN_DEG, 0.0f);
}

/* ================================================================
 *  Internal helper – move a single axis to an absolute position
 * ================================================================ */
static GimbalStatus gimbal_move_axis_to_position(EmmDevice *motor, float target_deg,
                                                  uint16_t speed_rpm, uint8_t acceleration,
                                                  uint16_t microstep, bool is_pitch)
{
    float current_enc;
    float delta_deg;
    (void)is_pitch;

    if (read_encoder_deg(motor, &current_enc) != EMM_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }

    delta_deg = target_deg - current_enc;
    if (delta_deg > 180.0f)  { delta_deg -= 360.0f; }
    if (delta_deg < -180.0f) { delta_deg += 360.0f; }

    if (delta_deg > -0.05f && delta_deg < 0.05f)
    {
        return GIMBAL_OK;
    }

    return (emm_move_degrees(motor, delta_deg, speed_rpm, acceleration,
                             EMM_MOTION_RELATIVE_CURRENT, microstep,
                             EMM_SYNC_IMMEDIATE) == EMM_OK)
                ? GIMBAL_OK
                : GIMBAL_ERROR_MOTOR;
}

/* ================================================================
 *  Calibration
 * ================================================================ */

GimbalStatus gimbal_calibrate_axis(Gimbal *gimbal, EmmDevice *motor,
                                   const char *axis_name, GimbalAxisCalib *result)
{
    float positives[5];
    float negatives[5];
    float pos;
    float prev_pos;
    uint8_t stuck_cnt;
    EmmMotorStatus st;
    int32_t pos_d;   /* position as degrees × 10, fixed-point for console */

    printf("\r\n[CALIB] ========================================\r\n");
    printf("[CALIB]  Calibrating %s (Addr=%u)\r\n", axis_name, (unsigned int)motor->address);
    printf("[CALIB]  Speed: %u RPM | Rounds: %u\r\n",
           (unsigned int)gimbal->calib_config.explore_speed_rpm,
           (unsigned int)gimbal->calib_config.explore_attempts);
    printf("[CALIB]  Stall check: %lu ms | Timeout: %lu ms\r\n",
           (unsigned long)gimbal->calib_config.stall_check_ms,
           (unsigned long)gimbal->calib_config.stall_timeout_ms);
    printf("[CALIB] ========================================\r\n\r\n");

    /* === Direction test (one-shot, user observes) === */
    printf("[CALIB] [%s] *** DIR TEST: small CW move ***\r\n", axis_name);
    {
        float before_test = 0.0f;
        float after_test = 0.0f;
        EmmJogParams test_jog;
        EmmStatus read_sts;

        /* Read initial position (motor must be stopped) */
        (void)emm_stop(motor, EMM_SYNC_IMMEDIATE);
        system_delay_ms(200u);
        read_sts = read_encoder_deg(motor, &before_test);
        {
            int32_t d = (int32_t)(before_test * 10.0f);
            printf("[CALIB] [%s] init pos read: %s(%d) val=%ld.%ld deg\r\n", axis_name,
                   (read_sts == EMM_OK) ? "ok" : "FAIL", (int)read_sts,
                   (long)(d / 10), (long)((d < 0 ? -d : d) % 10));
        }

        test_jog.direction    = EMM_DIRECTION_CW;
        test_jog.speed_rpm    = gimbal->calib_config.explore_speed_rpm;
        test_jog.acceleration = gimbal->calib_config.explore_acceleration;
        test_jog.sync_flag    = EMM_SYNC_IMMEDIATE;
        (void)emm_jog_no_response(motor, &test_jog);
        system_delay_ms(150u);
        (void)emm_stop(motor, EMM_SYNC_IMMEDIATE);
        system_delay_ms(200u);

        read_sts = read_encoder_deg(motor, &after_test);
        {
            int32_t d = (int32_t)(after_test * 10.0f);
            int32_t dd = (int32_t)((after_test - before_test) * 10.0f);
            printf("[CALIB] [%s] after move: %s(%d) val=%ld.%ld deg\r\n", axis_name,
                   (read_sts == EMM_OK) ? "ok" : "FAIL", (int)read_sts,
                   (long)(d / 10), (long)((d < 0 ? -d : d) % 10));
            printf("[CALIB] [%s] delta: %ld.%ld deg\r\n", axis_name,
                   (long)(dd / 10), (long)((dd < 0 ? -dd : dd) % 10));
        }
        printf("[CALIB] [%s] *** Watch the motor direction! ***\r\n", axis_name);
        printf("[CALIB] [%s] If direction is wrong, use emm_set_motor_direction() to flip\r\n\r\n",
               axis_name);
    }

    system_delay_ms(1500u);  /* Give user time to observe */

    /* === 5-round exploration === */
    for (uint8_t round = 1u; round <= gimbal->calib_config.explore_attempts; ++round)
    {
        uint32_t elapsed;

        printf("[CALIB] --- 第 %u/%u 轮 ---\r\n",
               (unsigned int)round,
               (unsigned int)gimbal->calib_config.explore_attempts);

        /* ----- Positive (CW) exploration ----- */
        printf("[CALIB] [%s] >>> CW exploration, speed=%uRPM <<<\r\n",
               axis_name, (unsigned int)gimbal->calib_config.explore_speed_rpm);

        /* Start CW jog once, then poll while running.
           Reading position/status during motion works on EMM42;
           we only stop when stall is detected or timeout reached. */
        {
            EmmJogParams jog_cw;
            jog_cw.direction    = EMM_DIRECTION_CW;
            jog_cw.speed_rpm    = gimbal->calib_config.explore_speed_rpm;
            jog_cw.acceleration = gimbal->calib_config.explore_acceleration;
            jog_cw.sync_flag    = EMM_SYNC_IMMEDIATE;
            (void)emm_jog_no_response(motor, &jog_cw);
        }

        elapsed = 0u;
        stuck_cnt = 0u;
        prev_pos = 999.0f;
        while (1)
        {
            EmmStatus pos_sts, sta_sts;

            system_delay_ms(gimbal->calib_config.stall_check_ms);
            elapsed += gimbal->calib_config.stall_check_ms;

            /* Read while motor is running (no stop needed) */
            pos_sts = read_encoder_deg(motor, &pos);
            system_delay_ms(10u);
            sta_sts = emm_get_motor_status_forced(motor, &st);

            if (pos_sts != EMM_OK || sta_sts != EMM_OK)
            {
                printf("[CALIB] [%s] t=%lums rd err pos=%d sts=%d\r\n",
                       axis_name, (unsigned long)elapsed,
                       (int)pos_sts, (int)sta_sts);
                continue;
            }

            pos_d = (int32_t)(pos * 10.0f);
            printf("[CALIB] [%s] t=%lu.%lus pos=%ld.%ld st=0x%02X(E=%d)\r\n",
                   axis_name,
                   (unsigned long)(elapsed / 1000u),
                   (unsigned long)((elapsed % 1000u) / 100u),
                   (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10),
                   (unsigned)((st.enabled ? 1u : 0u) | (st.stall_detected ? 4u : 0u) | (st.stall_protected ? 8u : 0u)),
                   (int)st.enabled);

            /* Detect limit: stall, or motor disabled itself */
            if (st.stall_detected || st.stall_protected || !st.enabled)
            {
                const char *reason = "STALL";
                if (!st.enabled && !st.stall_detected && !st.stall_protected) reason = "DISABLED";
                printf("[CALIB] [%s] *** %s! pos=%ld.%ld deg ***\r\n",
                       axis_name, reason,
                       (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                positives[round - 1u] = pos;
                break;
            }

            /* Detect motor stuck (position not changing) */
            {
                float diff = pos - prev_pos;
                if (diff < 0.0f) { diff = -diff; }
                if (diff < 1.0f)
                {
                    stuck_cnt++;
                    if (stuck_cnt >= 3u)
                    {
                        printf("[CALIB] [%s] *** STUCK! pos=%ld.%ld deg (no movement) ***\r\n",
                               axis_name, (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                        positives[round - 1u] = pos;
                        break;
                    }
                }
                else
                {
                    stuck_cnt = 0u;
                }
            }
            prev_pos = pos;

            if (elapsed > gimbal->calib_config.stall_timeout_ms)
            {
                printf("[CALIB] [%s] *** CW timeout(%lu), pos=%ld.%ld deg ***\r\n",
                       axis_name,
                       (unsigned long)gimbal->calib_config.stall_timeout_ms,
                       (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                positives[round - 1u] = pos;
                break;
            }
        }

        /* Stop, clear protection, re-enable */
        (void)emm_stop(motor, EMM_SYNC_IMMEDIATE);
        system_delay_ms(100u);
        printf("[CALIB] [%s] CW limit[%u]: %ld.%ld deg\r\n",
               axis_name, (unsigned int)round,
               (long)((int32_t)(positives[round - 1u] * 10.0f) / 10),
               (long)((int32_t)(positives[round - 1u] * 10.0f) < 0
                       ? -(int32_t)(positives[round - 1u] * 10.0f) % 10
                       : (int32_t)(positives[round - 1u] * 10.0f) % 10));

        /* Clear stall with retry + verification */
        {
            EmmMotorStatus clr_status;
            uint8_t clr_retry;
            for (clr_retry = 0u; clr_retry < 5u; ++clr_retry)
            {
                printf("[CALIB] [%s] clear stall attempt %u...\r\n",
                       axis_name, (unsigned int)(clr_retry + 1u));
                (void)emm_clear_stall_and_recover(motor);
                system_delay_ms(150u);
                if (emm_get_motor_status_forced(motor, &clr_status) == EMM_OK)
                {
                    if (!clr_status.stall_detected && !clr_status.stall_protected)
                    {
                        printf("[CALIB] [%s] stall cleared OK\r\n", axis_name);
                        break;
                    }
                    printf("[CALIB] [%s] stall still active (det=%d prot=%d)\r\n",
                           axis_name, (int)clr_status.stall_detected,
                           (int)clr_status.stall_protected);
                }
            }
        }

        /* Back off from CW limit before reverse exploration */
        {
            float backoff_pos;
            EmmStatus rd;
            printf("[CALIB] [%s] backing off from CW limit (-20 deg)...\r\n", axis_name);
            (void)emm_move_degrees(motor, -20.0f,
                                   gimbal->calib_config.explore_speed_rpm,
                                   gimbal->calib_config.explore_acceleration,
                                   EMM_MOTION_RELATIVE_CURRENT,
                                   gimbal->microstep,
                                   EMM_SYNC_IMMEDIATE);
            system_delay_ms(800u);
            (void)emm_stop(motor, EMM_SYNC_IMMEDIATE);
            system_delay_ms(100u);
            rd = read_encoder_deg(motor, &backoff_pos);
            {
                int32_t bpd = (int32_t)(backoff_pos * 10.0f);
                printf("[CALIB] [%s] after back-off pos: %ld.%ld deg (%s)\r\n",
                       axis_name, (long)(bpd / 10), (long)((bpd < 0 ? -bpd : bpd) % 10),
                       (rd == EMM_OK) ? "ok" : "err");
            }
        }
        system_delay_ms(gimbal->calib_config.stall_check_ms);

        /* ----- Reverse exploration (uses absolute position moves) -----
           Jog direction byte may not switch correctly on this motor,
           so we use absolute position moves: step toward 0 deg from
           the stall point.  The motor's internal PID computes the
           right direction automatically. */
        printf("[CALIB] [%s] <<< REV exploration (abs pos steps) <<<\r\n",
               axis_name);

        elapsed = 0u;
        stuck_cnt = 0u;
        prev_pos = 999.0f;  /* force first position update to be "moving" */
        while (1)
        {
            EmmStatus pos_sts;
            float target;

            /* Read current position */
            (void)emm_stop(motor, EMM_SYNC_IMMEDIATE);
            system_delay_ms(30u);
            pos_sts = read_encoder_deg(motor, &pos);

            if (pos_sts != EMM_OK)
            {
                printf("[CALIB] [%s] t=%lu.%lus read err=%d\r\n",
                       axis_name,
                       (unsigned long)(elapsed / 1000u),
                       (unsigned long)((elapsed % 1000u) / 100u),
                       (int)pos_sts);
                system_delay_ms(gimbal->calib_config.stall_check_ms);
                elapsed += gimbal->calib_config.stall_check_ms;
                continue;
            }

            /* Read status to check for stall */
            {
                EmmStatus sta_sts;
                sta_sts = emm_get_motor_status_forced(motor, &st);
                pos_d = (int32_t)(pos * 10.0f);
                printf("[CALIB] [%s] t=%lu.%lus pos=%ld.%ld st=0x%02X(E=%d)\r\n",
                       axis_name,
                       (unsigned long)(elapsed / 1000u),
                       (unsigned long)((elapsed % 1000u) / 100u),
                       (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10),
                       (unsigned)((st.enabled ? 1u : 0u) | (st.stall_detected ? 4u : 0u) | (st.stall_protected ? 8u : 0u)),
                       (int)st.enabled);
                (void)pos_sts; (void)sta_sts; /* suppress unused warnings */

                /* Detect limit: stall, motor disabled, or stuck */
                if (st.stall_detected || st.stall_protected || !st.enabled)
                {
                    const char *reason = !st.enabled ? "DISABLED" : "STALL";
                    printf("[CALIB] [%s] *** %s! pos=%ld.%ld deg ***\r\n",
                           axis_name, reason,
                           (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                    negatives[round - 1u] = pos;
                    break;
                }

                /* Stuck detection */
                {
                    float diff = pos - prev_pos;
                    if (diff < 0.0f) { diff = -diff; }
                    if (diff < 1.0f)
                    {
                        stuck_cnt++;
                        if (stuck_cnt >= 3u)
                        {
                            printf("[CALIB] [%s] *** STUCK! pos=%ld.%ld deg (no movement) ***\r\n",
                                   axis_name, (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                            negatives[round - 1u] = pos;
                            break;
                        }
                    }
                    else
                    {
                        stuck_cnt = 0u;
                    }
                }
                prev_pos = pos;
            }

            if (elapsed > gimbal->calib_config.stall_timeout_ms)
            {
                printf("[CALIB] [%s] *** REV timeout(%lu), pos=%ld.%ld deg ***\r\n",
                       axis_name,
                       (unsigned long)gimbal->calib_config.stall_timeout_ms,
                       (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                negatives[round - 1u] = pos;
                break;
            }

            /* Move toward the opposite side: step toward negative by
               subtracting a chunk each iteration.  The motor PID will
               figure out the correct direction to reach the target. */
            target = pos - 15.0f;  /* 15 deg step away from CW limit */
            if (target < -180.0f) { target = -180.0f; } /* safety floor */
            {
                int32_t td = (int32_t)(target * 10.0f);
                printf("[CALIB] [%s]   moving to %ld.%ld deg...\r\n", axis_name,
                       (long)(td / 10), (long)((td < 0 ? -td : td) % 10));
            }
            (void)emm_move_degrees(motor, target - pos,
                                   gimbal->calib_config.explore_speed_rpm,
                                   gimbal->calib_config.explore_acceleration,
                                   EMM_MOTION_RELATIVE_CURRENT,
                                   gimbal->microstep,
                                   EMM_SYNC_IMMEDIATE);

            system_delay_ms(gimbal->calib_config.stall_check_ms);
            elapsed += gimbal->calib_config.stall_check_ms;
        }

        (void)emm_stop(motor, EMM_SYNC_IMMEDIATE);
        system_delay_ms(100u);
        printf("[CALIB] [%s] CCW limit[%u]: %ld.%ld deg\r\n",
               axis_name, (unsigned int)round,
               (long)((int32_t)(negatives[round - 1u] * 10.0f) / 10),
               (long)((int32_t)(negatives[round - 1u] * 10.0f) < 0
                       ? -(int32_t)(negatives[round - 1u] * 10.0f) % 10
                       : (int32_t)(negatives[round - 1u] * 10.0f) % 10));
        printf("[CALIB] [%s] clear stall & recover...\r\n", axis_name);
        (void)emm_clear_stall_and_recover(motor);
        system_delay_ms(gimbal->calib_config.stall_check_ms);

        /* Move back toward midpoint */
        {
            float mid = (positives[round - 1u] + negatives[round - 1u]) / 2.0f;
            int32_t md = (int32_t)(mid * 10.0f);
            printf("[CALIB] [%s] return to mid %ld.%ld deg...\r\n", axis_name,
                   (long)(md / 10), (long)((md < 0 ? -md : md) % 10));
            (void)gimbal_move_axis_to_position(motor, mid,
                                               gimbal->calib_config.explore_speed_rpm,
                                               gimbal->calib_config.explore_acceleration,
                                               gimbal->microstep, true);
        }
        system_delay_ms(500u);
        printf("\r\n");
    }

    /* === Summarize === */
    {
        float pos_min = positives[0], pos_max = positives[0];
        float neg_min = negatives[0], neg_max = negatives[0];

        for (uint8_t i = 1u; i < gimbal->calib_config.explore_attempts; ++i)
        {
            if (positives[i] < pos_min) { pos_min = positives[i]; }
            if (positives[i] > pos_max) { pos_max = positives[i]; }
            if (negatives[i] < neg_min)  { neg_min  = negatives[i]; }
            if (negatives[i] > neg_max)  { neg_max  = negatives[i]; }
        }

        printf("[CALIB] ========================================\r\n");
        printf("[CALIB] [%s] Calib done! %u rounds:\r\n",
               axis_name, (unsigned int)gimbal->calib_config.explore_attempts);
        printf("[CALIB]   CW (enc): ");
        for (uint8_t i = 0u; i < gimbal->calib_config.explore_attempts; ++i)
        {
            int32_t v = (int32_t)(positives[i] * 10.0f);
            printf("%c%ld.%ld ",
                   (v < 0) ? '-' : '+',
                   (long)((v < 0 ? -v : v) / 10),
                   (long)((v < 0 ? -v : v) % 10));
        }
        printf("\r\n");
        printf("[CALIB]   CCW (enc): ");
        for (uint8_t i = 0u; i < gimbal->calib_config.explore_attempts; ++i)
        {
            int32_t v = (int32_t)(negatives[i] * 10.0f);
            printf("%c%ld.%ld ",
                   (v < 0) ? '-' : '+',
                   (long)((v < 0 ? -v : v) / 10),
                   (long)((v < 0 ? -v : v) % 10));
        }
        printf("\r\n");

        /* CW = upper limit, CCW = lower limit in mechanical space.
           If encoder wrapped (CW < CCW in raw encoder), compensate. */
        {
            float cw  = pos_max;
            float ccw = neg_min;

            if (cw < ccw)
            {
                cw += 360.0f;  /* CW exploration wrapped around 360°→0° */
            }

            result->range_deg = cw - ccw;
            result->mid_deg   = (cw + ccw) / 2.0f;
            if (result->mid_deg >= 360.0f) { result->mid_deg -= 360.0f; }

            /* Store limits: lower=CCW, upper=CW (may be >360 for internal use) */
            result->min_deg = ccw;
            result->max_deg = cw;

            printf("[CALIB]   CCW limit: %ld.%ld deg (enc)\r\n",
                   (long)((int32_t)(ccw * 10.0f) / 10),
                   (long)((int32_t)(ccw * 10.0f) < 0 ? -(int32_t)(ccw * 10.0f) % 10
                                                     : (int32_t)(ccw * 10.0f) % 10));
            printf("[CALIB]   CW  limit: %ld.%ld deg (enc%s)\r\n",
                   (long)((int32_t)(pos_max * 10.0f) / 10),
                   (long)((int32_t)(pos_max * 10.0f) < 0 ? -(int32_t)(pos_max * 10.0f) % 10
                                                         : (int32_t)(pos_max * 10.0f) % 10),
                   (pos_max < neg_min) ? ", wrapped +360" : "");
            {
                int32_t rg = (int32_t)(result->range_deg * 10.0f);
                int32_t md = (int32_t)(result->mid_deg * 10.0f);
                printf("[CALIB]   range: %ld.%ld deg  mid(enc): %ld.%ld deg\r\n",
                       (long)(rg / 10), (long)((rg < 0 ? -rg : rg) % 10),
                       (long)(md / 10), (long)((md < 0 ? -md : md) % 10));
            }
        }
        result->calibrated = true;

        printf("[CALIB] ========================================\r\n\r\n");
    }

    /* Move to midpoint as final resting position */
    {
        float mid = result->mid_deg;
        int32_t md = (int32_t)(mid * 10.0f);
        printf("[CALIB] [%s] final move to mid %ld.%ld deg...\r\n", axis_name,
               (long)(md / 10), (long)((md < 0 ? -md : md) % 10));
        (void)gimbal_move_axis_to_position(motor, mid,
                                           gimbal->calib_config.explore_speed_rpm,
                                           gimbal->calib_config.explore_acceleration,
                                           gimbal->microstep, true);
        system_delay_ms(500u);
    }

    return GIMBAL_OK;
}

GimbalStatus gimbal_auto_calibrate(Gimbal *gimbal)
{
    GimbalStatus status;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    printf("\r\n");
    printf("[CALIB] ========================================\r\n");
    printf("[CALIB]  Gimbal auto-calibration start\r\n");
    printf("[CALIB]  PITCH(m1/up-dn): stall  YAW(m2/L-R): fixed +/-180\r\n");
    printf("[CALIB] ========================================\r\n\r\n");

    /* Enable both motors first */
    printf("[CALIB] enable both motors...\r\n");
    (void)emm_enable(&gimbal->yaw, true, EMM_SYNC_IMMEDIATE);
    system_delay_ms(50u);
    (void)emm_enable(&gimbal->pitch, true, EMM_SYNC_IMMEDIATE);
    system_delay_ms(200u);
    /* Verify YAW is alive */
    {
        float ty;
        EmmMotorStatus ts;
        if (read_encoder_deg(&gimbal->yaw, &ty) == EMM_OK
            && emm_get_motor_status_forced(&gimbal->yaw, &ts) == EMM_OK)
        {
            printf("[CALIB] YAW check: enc=%ld.%ld deg en=%d\r\n",
                   (long)((int32_t)(ty * 10.0f) / 10),
                   (long)((int32_t)(ty * 10.0f) < 0 ? -(int32_t)(ty * 10.0f) % 10
                                                   : (int32_t)(ty * 10.0f) % 10),
                   (int)ts.enabled);
        }
        else
        {
            printf("[CALIB] YAW check: NO RESPONSE (addr=%u)\r\n",
                   (unsigned int)GIMBAL_YAW_MOTOR_ADDRESS);
        }
    }

    /* ---- PITCH: stall-based calibration ---- */
    printf("[CALIB] >>> Calibrating PITCH (m1, up/down, stall) <<<\r\n");
    status = gimbal_calibrate_axis(gimbal, &gimbal->pitch, "PITCH", &gimbal->calib_pitch);
    if (status != GIMBAL_OK)
    {
        printf("[CALIB] PITCH 校准失败!\r\n");
        /* Don't abort — YAW can still be set */
    }

    system_delay_ms(500u);

    /* ---- YAW: 360° motor, no mechanical stop, use fixed ±180° ---- */
    printf("[CALIB] >>> YAW (m2, left/right, 360 no-stop, fixed +/-180) <<<\r\n");
    gimbal->calib_yaw.min_deg    = -180.0f;
    gimbal->calib_yaw.max_deg    =  180.0f;
    gimbal->calib_yaw.range_deg  =  360.0f;
    gimbal->calib_yaw.mid_deg    =    0.0f;
    gimbal->calib_yaw.calibrated = true;
    printf("[CALIB] YAW  fixed limit: -180.0 ~ +180.0 deg\r\n");

    /* Apply calibration as active limits */
    gimbal_set_limits_from_calib(gimbal);

    /* ---- Return both axes to center, then zero encoder at midpoint ---- */
    printf("\r\n[CALIB] --- 回中 & 设零[?:e7]�� ---\r\n");

    /* PITCH: move to midpoint, then zero encoder there */
    {
        float pm = gimbal->calib_pitch.mid_deg;
        printf("[CALIB] PITCH goto mid(%ld.%ld)...\r\n",
               (long)((int32_t)(pm * 10.0f) / 10),
               (long)((int32_t)(pm * 10.0f) < 0 ? -(int32_t)(pm * 10.0f) % 10
                                               : (int32_t)(pm * 10.0f) % 10));
        (void)gimbal_move_axis_to_position(&gimbal->pitch, pm,
                                           gimbal->calib_config.explore_speed_rpm,
                                           gimbal->calib_config.explore_acceleration,
                                           gimbal->microstep, true);
        system_delay_ms(500u);
        printf("[CALIB] PITCH zero position at midpoint...\r\n");
        {
            EmmResponseMode saved = gimbal->pitch.response_mode;
            gimbal->pitch.response_mode = EMM_RESPONSE_RECEIVE;
            EmmStatus zs = emm_zero_position(&gimbal->pitch);
            gimbal->pitch.response_mode = saved;
            printf("[CALIB] PITCH zero result: %s(%d)\r\n",
                   (zs == EMM_OK) ? "ok" : "FAIL", (int)zs);
        }
        system_delay_ms(100u);
        gimbal->pitch_angle_deg = 0.0f;
    }

    /* YAW: move to 0° (fixed range) */
    printf("[CALIB] YAW  goto 0...\r\n");
    (void)gimbal_move_axis_to_position(&gimbal->yaw, 0.0f,
                                       gimbal->calib_config.explore_speed_rpm,
                                       gimbal->calib_config.explore_acceleration,
                                       gimbal->microstep, false);
    system_delay_ms(500u);
    /* Check if YAW moved */
    {
        float ty;
        if (read_encoder_deg(&gimbal->yaw, &ty) == EMM_OK)
        {
            printf("[CALIB] YAW after goto 0: enc=%ld.%ld deg\r\n",
                   (long)((int32_t)(ty * 10.0f) / 10),
                   (long)((int32_t)(ty * 10.0f) < 0 ? -(int32_t)(ty * 10.0f) % 10
                                                   : (int32_t)(ty * 10.0f) % 10));
        }
    }

    /* Sync software tracking to actual positions */
    {
        float y, p;
        if (gimbal_read_actual_position(gimbal, &y, &p) == GIMBAL_OK)
        {
            gimbal->yaw_angle_deg   = y;
            gimbal->pitch_angle_deg = p;
        }
    }

    printf("\r\n[CALIB] ========================================\r\n");
    printf("[CALIB]  Calibration complete!\r\n");
    {
        int32_t pmin = (int32_t)(gimbal->calib_pitch.min_deg * 10.0f);
        int32_t pmax = (int32_t)(gimbal->calib_pitch.max_deg * 10.0f);
        int32_t prng = (int32_t)(gimbal->calib_pitch.range_deg * 10.0f);
        printf("[CALIB]  PITCH: %ld.%ld ~ %ld.%ld deg (range %ld.%ld)\r\n",
               (long)(pmin / 10), (long)((pmin < 0 ? -pmin : pmin) % 10),
               (long)(pmax / 10), (long)((pmax < 0 ? -pmax : pmax) % 10),
               (long)(prng / 10), (long)((prng < 0 ? -prng : prng) % 10));
    }
    printf("[CALIB]  YAW:   -180.0 ~ +180.0 deg (range 360.0)\r\n");
    printf("[CALIB] ========================================\r\n\r\n");

    return GIMBAL_OK;
}

/* ================================================================
 *  Position reading
 * ================================================================ */

GimbalStatus gimbal_read_actual_position(Gimbal *gimbal, float *yaw_deg, float *pitch_deg)
{
    float y, p;
    EmmStatus ys, ps;

    if (gimbal == NULL || yaw_deg == NULL || pitch_deg == NULL)
    {
        return GIMBAL_ERROR;
    }

    ys = read_encoder_deg(&gimbal->yaw, &y);
    ps = read_encoder_deg(&gimbal->pitch, &p);

    if (ys == EMM_OK)
    {
        float rel = y - GIMBAL_YAW_ENC_CENTER;
        while (rel > 180.0f)  rel -= 360.0f;
        while (rel < -180.0f) rel += 360.0f;
        *yaw_deg = rel / GIMBAL_YAW_RATIO;
    }
    if (ps == EMM_OK)
    {
        float rel = p - GIMBAL_PITCH_ENC_HORIZONTAL;
        while (rel > 180.0f)  rel -= 360.0f;
        while (rel < -180.0f) rel += 360.0f;
        *pitch_deg = rel / GIMBAL_PITCH_RATIO;
    }

    {
        int32_t yd = (int32_t)(*yaw_deg * 10.0f);
        int32_t pd = (int32_t)(*pitch_deg * 10.0f);
        printf("[GIMBAL] actual pos: Yaw=%ld.%ld (%s)  Pitch=%ld.%ld (%s)\r\n",
               (long)(yd / 10), (long)((yd < 0 ? -yd : yd) % 10),
               (ys == EMM_OK) ? "ok" : "err",
               (long)(pd / 10), (long)((pd < 0 ? -pd : pd) % 10),
               (ps == EMM_OK) ? "ok" : "err");
    }

    return (ys == EMM_OK && ps == EMM_OK) ? GIMBAL_OK : GIMBAL_ERROR_MOTOR;
}

/* ================================================================
 *  Soft limits from calibration
 * ================================================================ */

void gimbal_set_limits_from_calib(Gimbal *gimbal)
{
    if (gimbal == NULL) { return; }

    if (gimbal->calib_yaw.calibrated)
    {
        gimbal->yaw_min_deg = gimbal->calib_yaw.min_deg + 1.0f;
        gimbal->yaw_max_deg = gimbal->calib_yaw.max_deg - 1.0f;
        printf("[GIMBAL] YAW  soft limit: %ld.%ld ~ %ld.%ld deg (rel)\r\n",
               (long)((int32_t)(gimbal->yaw_min_deg * 10.0f) / 10),
               (long)((int32_t)(gimbal->yaw_min_deg * 10.0f) < 0 ? -(int32_t)(gimbal->yaw_min_deg * 10.0f) % 10 : (int32_t)(gimbal->yaw_min_deg * 10.0f) % 10),
               (long)((int32_t)(gimbal->yaw_max_deg * 10.0f) / 10),
               (long)((int32_t)(gimbal->yaw_max_deg * 10.0f) < 0 ? -(int32_t)(gimbal->yaw_max_deg * 10.0f) % 10 : (int32_t)(gimbal->yaw_max_deg * 10.0f) % 10));
    }

    if (gimbal->calib_pitch.calibrated)
    {
        float half_range = gimbal->calib_pitch.range_deg * 0.5f - 1.0f;
        if (half_range < 0.0f) { half_range = 0.0f; }
        gimbal->pitch_min_deg = -half_range;
        gimbal->pitch_max_deg =  half_range;
        printf("[GIMBAL] PITCH soft limit: %ld.%ld ~ %ld.%ld deg (rel, mid=0)\r\n",
               (long)((int32_t)(gimbal->pitch_min_deg * 10.0f) / 10),
               (long)((int32_t)(gimbal->pitch_min_deg * 10.0f) < 0 ? -(int32_t)(gimbal->pitch_min_deg * 10.0f) % 10 : (int32_t)(gimbal->pitch_min_deg * 10.0f) % 10),
               (long)((int32_t)(gimbal->pitch_max_deg * 10.0f) / 10),
               (long)((int32_t)(gimbal->pitch_max_deg * 10.0f) < 0 ? -(int32_t)(gimbal->pitch_max_deg * 10.0f) % 10 : (int32_t)(gimbal->pitch_max_deg * 10.0f) % 10));
    }
}

/* ================================================================
 *  Manual mode
 * ================================================================ */

GimbalStatus gimbal_enter_manual_mode(Gimbal *gimbal)
{
    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    printf("[GIMBAL] *** 进入手动模式 - 电机断电，可自由转动 ***\r\n");

    (void)emm_disable(&gimbal->yaw, EMM_SYNC_IMMEDIATE);
    system_delay_ms(10u);
    (void)emm_disable(&gimbal->pitch, EMM_SYNC_IMMEDIATE);

    gimbal->manual_mode = true;
    return GIMBAL_OK;
}

GimbalStatus gimbal_exit_manual_mode(Gimbal *gimbal)
{
    float yaw_pos, pitch_pos;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    printf("[GIMBAL] *** 退出手动模式 - 重新使能电机 ***\r\n");

    (void)emm_enable(&gimbal->yaw, true, EMM_SYNC_IMMEDIATE);
    system_delay_ms(50u);
    (void)emm_enable(&gimbal->pitch, true, EMM_SYNC_IMMEDIATE);
    system_delay_ms(100u);

    /* Read current positions to re-sync software tracking */
    if (gimbal_read_actual_position(gimbal, &yaw_pos, &pitch_pos) == GIMBAL_OK)
    {
        gimbal->yaw_angle_deg   = yaw_pos;
        gimbal->pitch_angle_deg = pitch_pos;
    {
        int32_t yd = (int32_t)(yaw_pos * 10.0f);
        int32_t pd = (int32_t)(pitch_pos * 10.0f);
        printf("[GIMBAL] pos synced: Yaw=%ld.%ld Pitch=%ld.%ld\r\n",
               (long)(yd / 10), (long)((yd < 0 ? -yd : yd) % 10),
               (long)(pd / 10), (long)((pd < 0 ? -pd : pd) % 10));
    }
    }

    gimbal->manual_mode = false;
    return GIMBAL_OK;
}


/* ================================================================
 *  Geared calibration – match ZDT_X42S Python reference approach.
 *
 *  PITCH: jog CW toward limit -> stall -> zero at limit ->
 *         move back by BACK_ANGLE to horizontal -> zero at horizontal.
 *  YAW:   just zero at current position.
 * ================================================================ */

static void wait_for_enter(void)
{
    printf("... 按回车键继续\r\n");
    for (;;)
    {
        uint8 ch;
        if (uart_query_byte(DEBUG_UART_INDEX, &ch) == ZF_TRUE)
        {
            if (ch == '\r' || ch == '\n') { break; }
        }
        system_delay_ms(10u);
    }
    system_delay_ms(200u);
}

GimbalStatus gimbal_calibrate_geared(Gimbal *gimbal)
{
    float cur_deg;
    EmmMotorStatus st;
    uint32_t elapsed;
    int32_t back_pulses;
    float back_motor_deg;

    if (gimbal == NULL) { return GIMBAL_ERROR; }

    printf("\r\n");
    printf("[GEARED] ========================================\r\n");
    printf("[GEARED]  PITCH ratio=%.1f  YAW ratio=%.1f\r\n",
           (double)GIMBAL_PITCH_RATIO, (double)GIMBAL_YAW_RATIO);
    printf("[GEARED] ========================================\r\n\r\n");

    /* ================================================================
     *  PITCH calibration
     * ================================================================ */
    printf("[GEARED] >>> PITCH: jog CW to limit, then back %.1f deg <<<\r\n",
           (double)GIMBAL_PITCH_BACK_ANGLE);
    printf("[GEARED] >>> 确保云台在安全位置，按回车开始 <<<\r\n");
    wait_for_enter();

    (void)emm_enable(&gimbal->pitch, true, EMM_SYNC_IMMEDIATE);
    system_delay_ms(200u);

    /* Jog CW */
    printf("[GEARED] jogging CW at %d RPM...\r\n",
           (int)gimbal->calib_config.explore_speed_rpm);
    {
        EmmJogParams jog;
        jog.direction    = EMM_DIRECTION_CW;
        jog.speed_rpm    = gimbal->calib_config.explore_speed_rpm;
        jog.acceleration = gimbal->calib_config.explore_acceleration;
        jog.sync_flag    = EMM_SYNC_IMMEDIATE;
        (void)emm_jog_no_response(&gimbal->pitch, &jog);
    }

    /* Poll for stall */
    elapsed = 0u;
    while (1)
    {
        system_delay_ms(gimbal->calib_config.stall_check_ms);
        elapsed += gimbal->calib_config.stall_check_ms;

        if (emm_get_motor_status_forced(&gimbal->pitch, &st) != EMM_OK)
            continue;

        if (st.stall_detected || st.stall_protected || !st.enabled)
        {
            printf("[GEARED] PITCH stop at t=%lu.%lus (st=0x%02X)\r\n",
                   (unsigned long)(elapsed / 1000u),
                   (unsigned long)((elapsed % 1000u) / 100u),
                   (unsigned)((st.enabled ? 1u : 0u) | (st.stall_detected ? 4u : 0u) | (st.stall_protected ? 8u : 0u)));
            break;
        }
        if (elapsed > gimbal->calib_config.stall_timeout_ms)
        {
            printf("[GEARED] PITCH timeout, abort\r\n");
            (void)emm_stop(&gimbal->pitch, EMM_SYNC_IMMEDIATE);
            return GIMBAL_ERROR_CALIB;
        }
    }

    /* Stop and clear */
    (void)emm_stop(&gimbal->pitch, EMM_SYNC_IMMEDIATE);
    system_delay_ms(100u);

    /* Read encoder at limit */
    {
        float enc_limit;
        read_encoder_deg(&gimbal->pitch, &enc_limit);
        printf("[GEARED] PITCH CW limit enc: %ld.%ld deg\r\n",
               (long)((int32_t)(enc_limit * 10.0f) / 10),
               (long)((int32_t)(enc_limit * 10.0f) < 0 ? -(int32_t)(enc_limit * 10.0f) % 10
                                                     : (int32_t)(enc_limit * 10.0f) % 10));
    }

    (void)emm_clear_stall_and_recover(&gimbal->pitch);
    system_delay_ms(300u);

    /* Move back to horizontal (relative move by back_angle * ratio motor deg) */
    back_motor_deg = GIMBAL_PITCH_BACK_ANGLE * GIMBAL_PITCH_RATIO;
    back_pulses = (int32_t)(back_motor_deg / 360.0f * (float)(200u * gimbal->microstep));
    if (back_pulses < 0) back_pulses = -back_pulses;

    printf("[GEARED] back motor=%.1f deg, pulses=%ld\r\n",
           (double)back_motor_deg, (long)back_pulses);

    {
        EmmPositionParams mp;
        mp.direction   = (GIMBAL_PITCH_BACK_ANGLE < 0.0f) ? EMM_DIRECTION_CCW : EMM_DIRECTION_CW;
        mp.speed_rpm   = gimbal->calib_config.explore_speed_rpm;
        mp.acceleration = gimbal->calib_config.explore_acceleration;
        mp.pulse_count  = (uint32_t)back_pulses;
        mp.motion_mode  = EMM_MOTION_RELATIVE_CURRENT;
        mp.sync_flag    = EMM_SYNC_IMMEDIATE;
        (void)emm_move_pulses(&gimbal->pitch, &mp);
    }

    /* Wait for position reached */
    {
        uint32_t wait_ms = 0u;
        while (wait_ms < 15000u)
        {
            system_delay_ms(100u);
            wait_ms += 100u;
            if (emm_get_motor_status_forced(&gimbal->pitch, &st) == EMM_OK)
            {
                if (st.position_reached) break;
            }
        }
        system_delay_ms(500u);
    }

    /* Read encoder at horizontal position (do NOT zero_position,
       encoder values are the persistent absolute reference). */
    read_encoder_deg(&gimbal->pitch, &cur_deg);
    printf("[GEARED] PITCH horizontal enc: %ld.%ld deg\r\n",
           (long)((int32_t)(cur_deg * 10.0f) / 10),
           (long)((int32_t)(cur_deg * 10.0f) < 0 ? -(int32_t)(cur_deg * 10.0f) % 10
                                                 : (int32_t)(cur_deg * 10.0f) % 10));
    printf("[GEARED] >>> COPY these #defines into gimbal.h: <<<\r\n");
    printf("[GEARED] #define GIMBAL_PITCH_ENC_HORIZONTAL %.1ff\r\n",
           (double)cur_deg);
    printf("[GEARED] #define GIMBAL_PITCH_ENC_LIMIT     ... (from above)\r\n");

    gimbal->geared_pitch.gear_ratio = GIMBAL_PITCH_RATIO;
    gimbal->geared_pitch.enc_at_zero_deg = cur_deg;
    gimbal->geared_pitch.calibrated = true;

    gimbal->pitch_min_deg = GIMBAL_PITCH_BACK_ANGLE;
    gimbal->pitch_max_deg = 0.0f;
    if (gimbal->pitch_min_deg > gimbal->pitch_max_deg)
    {
        float t = gimbal->pitch_min_deg;
        gimbal->pitch_min_deg = gimbal->pitch_max_deg;
        gimbal->pitch_max_deg = t;
    }
    printf("[GEARED] PITCH limits: %.1f ~ %.1f gimbal deg\r\n",
           (double)gimbal->pitch_min_deg, (double)gimbal->pitch_max_deg);

    /* ================================================================
     *  YAW calibration — just record encoder at center
     * ================================================================ */
    printf("\r\n[GEARED] >>> YAW: 摆到正前方，按回车 <<<\r\n");
    wait_for_enter();

    (void)emm_enable(&gimbal->yaw, true, EMM_SYNC_IMMEDIATE);
    system_delay_ms(200u);

    read_encoder_deg(&gimbal->yaw, &cur_deg);
    printf("[GEARED] YAW center enc: %ld.%ld deg\r\n",
           (long)((int32_t)(cur_deg * 10.0f) / 10),
           (long)((int32_t)(cur_deg * 10.0f) < 0 ? -(int32_t)(cur_deg * 10.0f) % 10
                                                 : (int32_t)(cur_deg * 10.0f) % 10));
    printf("[GEARED] >>> #define GIMBAL_YAW_ENC_CENTER %.1ff <<<\r\n",
           (double)cur_deg);

    gimbal->geared_yaw.gear_ratio = GIMBAL_YAW_RATIO;
    gimbal->geared_yaw.enc_at_zero_deg = cur_deg;
    gimbal->geared_yaw.calibrated = true;
    gimbal->yaw_min_deg = -179.0f;
    gimbal->yaw_max_deg =  179.0f;

    printf("[GEARED] YAW done, ratio=%.1f\r\n", (double)GIMBAL_YAW_RATIO);
    printf("[GEARED] ========================================\r\n");
    printf("[GEARED]  Calibration complete!\r\n");
    printf("[GEARED]  Copy the #define lines above into gimbal.h\r\n");
    printf("[GEARED]  PITCH ratio=%.1f  YAW ratio=%.1f\r\n",
           (double)GIMBAL_PITCH_RATIO, (double)GIMBAL_YAW_RATIO);
    printf("[GEARED] ========================================\r\n\r\n");

    return GIMBAL_OK;
}
