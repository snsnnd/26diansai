#include "gimbal/emm_stepper.h"

#define EMM_STATUS_FIXED_CHECKSUM 0x6Bu
#define EMM_STATUS_SUCCESS 0x02u
#define EMM_STATUS_PARAM_ERROR 0xE2u
#define EMM_STATUS_FORMAT_ERROR 0xEEu

#define EMM_CODE_CAL_ENCODER 0x06u
#define EMM_CODE_RESTART 0x08u
#define EMM_CODE_ZERO_POSITION 0x0Au
#define EMM_CODE_CLEAR_PROTECTION 0x0Eu
#define EMM_CODE_FACTORY_RESET 0x0Fu
#define EMM_CODE_GET_VERSION 0x1Fu
#define EMM_CODE_GET_MOTOR_RH 0x20u
#define EMM_CODE_GET_PID 0x21u
#define EMM_CODE_GET_HOME_PARAM 0x22u
#define EMM_CODE_GET_BUS_VOLTAGE 0x24u
#define EMM_CODE_GET_BUS_CURRENT 0x26u
#define EMM_CODE_GET_PHASE_CURRENT 0x27u
#define EMM_CODE_GET_ENCODER 0x31u
#define EMM_CODE_GET_PULSE_COUNT 0x32u
#define EMM_CODE_GET_TARGET_POSITION 0x33u
#define EMM_CODE_GET_REALTIME_SPEED 0x35u
#define EMM_CODE_GET_REALTIME_POSITION 0x36u
#define EMM_CODE_GET_POSITION_ERROR 0x37u
#define EMM_CODE_GET_TEMPERATURE 0x39u
#define EMM_CODE_GET_MOTOR_STATUS 0x3Au
#define EMM_CODE_GET_HOME_STATUS 0x3Bu
#define EMM_CODE_GET_CONFIG 0x42u
#define EMM_CODE_GET_SYS_STATUS 0x43u
#define EMM_CODE_SET_OPEN_LOOP_CURRENT 0x44u
#define EMM_CODE_SET_CLOSED_LOOP_CURRENT 0x45u
#define EMM_CODE_SET_LOOP_MODE 0x46u
#define EMM_CODE_SET_CONFIG 0x48u
#define EMM_CODE_SET_PID 0x4Au
#define EMM_CODE_SET_HOME_PARAM 0x4Cu
#define EMM_CODE_SET_SCALE_INPUT 0x4Fu
#define EMM_CODE_SET_HEARTBEAT_TIME 0x68u
#define EMM_CODE_SET_MICROSTEP 0x84u
#define EMM_CODE_SET_HOME_ZERO 0x93u
#define EMM_CODE_HOME 0x9Au
#define EMM_CODE_STOP_HOME 0x9Cu
#define EMM_CODE_SET_ID 0xAEu
#define EMM_CODE_SET_LOCK_BUTTON 0xD0u
#define EMM_CODE_SET_POSITION_WINDOW 0xD1u
#define EMM_CODE_SET_MOTOR_DIRECTION 0xD4u
#define EMM_CODE_ENABLE 0xF3u
#define EMM_CODE_JOG 0xF6u
#define EMM_CODE_SET_AUTO_RUN 0xF7u
#define EMM_CODE_POSITION 0xFDu
#define EMM_CODE_ESTOP 0xFEu
#define EMM_CODE_SYNC_MOVE 0xFFu
#define EMM_CODE_BROADCAST_GET_ID 0x15u

#define EMM_PROTOCOL_CAL_ENCODER 0x45u
#define EMM_PROTOCOL_RESTART 0x97u
#define EMM_PROTOCOL_ZERO_POSITION 0x6Du
#define EMM_PROTOCOL_CLEAR_PROTECTION 0x52u
#define EMM_PROTOCOL_FACTORY_RESET 0x5Fu
#define EMM_PROTOCOL_ENABLE 0xABu
#define EMM_PROTOCOL_ESTOP 0x98u
#define EMM_PROTOCOL_SYNC_MOVE 0x66u
#define EMM_PROTOCOL_SET_HOME_ZERO 0x88u
#define EMM_PROTOCOL_STOP_HOME 0x48u
#define EMM_PROTOCOL_SET_HOME_PARAM 0xAEu
#define EMM_PROTOCOL_GET_CONFIG 0x6Cu
#define EMM_PROTOCOL_GET_SYS_STATUS 0x7Au
#define EMM_PROTOCOL_SET_MICROSTEP 0x8Au
#define EMM_PROTOCOL_SET_ID 0x4Bu
#define EMM_PROTOCOL_SET_OPEN_LOOP_CURRENT 0x33u
#define EMM_PROTOCOL_SET_CLOSED_LOOP_CURRENT 0x66u
#define EMM_PROTOCOL_SET_LOOP_MODE 0xA6u
#define EMM_PROTOCOL_SET_CONFIG 0xD1u
#define EMM_PROTOCOL_SET_PID 0xC3u
#define EMM_PROTOCOL_SET_AUTO_RUN 0x1Cu
#define EMM_PROTOCOL_SET_SCALE_INPUT 0x71u
#define EMM_PROTOCOL_SET_MOTOR_DIRECTION 0x60u
#define EMM_PROTOCOL_SET_LOCK_BUTTON 0xB3u
#define EMM_PROTOCOL_SET_POSITION_WINDOW 0x07u
#define EMM_PROTOCOL_SET_HEARTBEAT_TIME 0x38u

static const uint8_t EmmCrc8Table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

static uint16_t read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint32_t read_u32_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
}

static int32_t read_signed_prefix(const uint8_t *data, size_t length)
{
    int32_t sign = (data[0] == 1u) ? -1 : 1;
    uint32_t value = 0u;

    for (size_t i = 1u; i < length; ++i)
    {
        value = (value << 8) | data[i];
    }

    return sign * (int32_t)value;
}

uint8_t emm_calculate_checksum(const uint8_t *data, size_t length, EmmChecksumMode mode);

static void write_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

static void write_u32_be(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;
}


static size_t emm_rx_next_index(size_t index)
{
    return (index + 1u) % EMM_STEPPER_RX_BUFFER_SIZE;
}

size_t emm_rx_available(const EmmDevice *device)
{
    if (device == 0)
    {
        return 0u;
    }
    if (device->rx_head >= device->rx_tail)
    {
        return device->rx_head - device->rx_tail;
    }
    return EMM_STEPPER_RX_BUFFER_SIZE - device->rx_tail + device->rx_head;
}

size_t emm_rx_overflow_count(const EmmDevice *device)
{
    return (device == 0) ? 0u : device->rx_overflow_count;
}

void emm_rx_clear(EmmDevice *device)
{
    if (device == 0)
    {
        return;
    }
    device->rx_head = 0u;
    device->rx_tail = 0u;
}

static bool emm_rx_push(EmmDevice *device, uint8_t byte)
{
    size_t next;
    if (device == 0)
    {
        return false;
    }

    next = emm_rx_next_index(device->rx_head);
    if (next == device->rx_tail)
    {
        /* Drop the oldest byte. Newer bytes are more likely to complete the
           current frame during high-frequency polling. */
        device->rx_tail = emm_rx_next_index(device->rx_tail);
        device->rx_overflow_count++;
    }

    device->rx_buffer[device->rx_head] = byte;
    device->rx_head = next;
    return true;
}

static bool emm_rx_peek(const EmmDevice *device, size_t offset, uint8_t *byte)
{
    size_t count;
    size_t index;

    if (device == 0 || byte == 0)
    {
        return false;
    }

    count = emm_rx_available(device);
    if (offset >= count)
    {
        return false;
    }

    index = (device->rx_tail + offset) % EMM_STEPPER_RX_BUFFER_SIZE;
    *byte = device->rx_buffer[index];
    return true;
}

static void emm_rx_drop(EmmDevice *device, size_t length)
{
    size_t count;

    if (device == 0)
    {
        return;
    }

    count = emm_rx_available(device);
    if (length > count)
    {
        length = count;
    }

    device->rx_tail = (device->rx_tail + length) % EMM_STEPPER_RX_BUFFER_SIZE;
}

static void emm_rx_copy(const EmmDevice *device, size_t offset, uint8_t *data, size_t length)
{
    for (size_t i = 0u; i < length; ++i)
    {
        uint8_t value = 0u;
        (void)emm_rx_peek(device, offset + i, &value);
        data[i] = value;
    }
}

static bool emm_match_byte(uint8_t actual, uint8_t expected)
{
    return expected == EMM_STEPPER_MATCH_ANY || actual == expected;
}

EmmStatus emm_poll(EmmDevice *device, uint32_t timeout_ms)
{
    uint8_t chunk[EMM_STEPPER_RX_READ_CHUNK];
    size_t total = 0u;
    uint8_t loops = 0u;

    if (device == 0 || device->transport.read == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    do
    {
        size_t count = device->transport.read(chunk, sizeof(chunk), (loops == 0u) ? timeout_ms : 0u, device->transport.user_data);
        if (count == 0u)
        {
            break;
        }
        for (size_t i = 0u; i < count; ++i)
        {
            (void)emm_rx_push(device, chunk[i]);
        }
        total += count;
        loops++;
    } while (loops < device->poll_attempts);

    return (total > 0u) ? EMM_OK : EMM_ERROR_TIMEOUT;
}

static EmmStatus emm_try_parse_fixed_from_rx(EmmDevice *device,
                                             uint8_t expected_address,
                                             uint8_t expected_code,
                                             uint8_t *response,
                                             size_t response_length)
{
    size_t count;

    if (device == 0 || response == 0 || response_length < 3u || response_length > EMM_STEPPER_MAX_FRAME_SIZE)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    count = emm_rx_available(device);
    if (count < response_length)
    {
        return EMM_ERROR_TIMEOUT;
    }

    for (size_t offset = 0u; offset + response_length <= count; ++offset)
    {
        uint8_t address = 0u;
        uint8_t code = 0u;
        uint8_t frame[EMM_STEPPER_MAX_FRAME_SIZE];
        uint8_t expected_checksum;

        (void)emm_rx_peek(device, offset, &address);
        (void)emm_rx_peek(device, offset + 1u, &code);

        if (!emm_match_byte(address, expected_address))
        {
            continue;
        }
        if (device->strict_frame_check && !emm_match_byte(code, expected_code))
        {
            continue;
        }

        emm_rx_copy(device, offset, frame, response_length);
        expected_checksum = emm_calculate_checksum(frame, response_length - 1u, device->checksum_mode);
        if (frame[response_length - 1u] != expected_checksum)
        {
            continue;
        }

        for (size_t i = 0u; i < response_length; ++i)
        {
            response[i] = frame[i];
        }
        emm_rx_drop(device, offset + response_length);
        return EMM_OK;
    }

    /* Discard only obvious noise before the first possible address. Valid but
       currently unmatched frames are retained for emm_read_any_frame(). */
    while (emm_rx_available(device) > 0u)
    {
        uint8_t address = 0u;
        (void)emm_rx_peek(device, 0u, &address);
        if (expected_address == EMM_STEPPER_MATCH_ANY || address == expected_address)
        {
            break;
        }
        if (address == EMM_STEPPER_BROADCAST_ADDRESS || address == EMM_STEPPER_MATCH_ANY)
        {
            break;
        }
        emm_rx_drop(device, 1u);
    }

    return EMM_ERROR_TIMEOUT;
}

static EmmStatus emm_try_parse_dynamic_from_rx(EmmDevice *device,
                                               uint8_t expected_address,
                                               uint8_t expected_code,
                                               uint8_t *response,
                                               size_t response_capacity,
                                               size_t *response_length)
{
    size_t count;

    if (device == 0 || response == 0 || response_length == 0 || response_capacity < 5u)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    count = emm_rx_available(device);
    if (count < 5u)
    {
        return EMM_ERROR_TIMEOUT;
    }

    for (size_t offset = 0u; offset + 5u <= count; ++offset)
    {
        uint8_t address = 0u;
        uint8_t code = 0u;
        uint8_t total_u8 = 0u;
        size_t total;
        uint8_t frame[EMM_STEPPER_MAX_FRAME_SIZE];
        uint8_t expected_checksum;

        (void)emm_rx_peek(device, offset, &address);
        (void)emm_rx_peek(device, offset + 1u, &code);
        (void)emm_rx_peek(device, offset + 2u, &total_u8);
        total = (size_t)total_u8;

        if (!emm_match_byte(address, expected_address))
        {
            continue;
        }
        if (device->strict_frame_check && !emm_match_byte(code, expected_code))
        {
            continue;
        }
        if (total < 5u || total > EMM_STEPPER_MAX_FRAME_SIZE)
        {
            continue;
        }
        if (total > response_capacity)
        {
            return EMM_ERROR_BAD_RESPONSE;
        }
        if (offset + total > count)
        {
            continue;
        }

        emm_rx_copy(device, offset, frame, total);
        expected_checksum = emm_calculate_checksum(frame, total - 1u, device->checksum_mode);
        if (frame[total - 1u] != expected_checksum)
        {
            continue;
        }

        for (size_t i = 0u; i < total; ++i)
        {
            response[i] = frame[i];
        }
        *response_length = total;
        emm_rx_drop(device, offset + total);
        return EMM_OK;
    }

    return EMM_ERROR_TIMEOUT;
}

EmmStatus emm_read_fixed_frame(EmmDevice *device,
                               uint8_t expected_address,
                               uint8_t expected_code,
                               uint8_t *response,
                               size_t response_length,
                               uint32_t timeout_ms)
{
    EmmStatus status;
    uint8_t attempts;

    if (device == 0 || response == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    status = emm_try_parse_fixed_from_rx(device, expected_address, expected_code, response, response_length);
    if (status == EMM_OK || status != EMM_ERROR_TIMEOUT)
    {
        return status;
    }

    attempts = (device->poll_attempts == 0u) ? 1u : device->poll_attempts;
    for (uint8_t i = 0u; i < attempts; ++i)
    {
        status = emm_poll(device, timeout_ms);
        if (status != EMM_OK && status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        status = emm_try_parse_fixed_from_rx(device, expected_address, expected_code, response, response_length);
        if (status == EMM_OK || status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        if (timeout_ms == 0u)
        {
            break;
        }
    }

    return EMM_ERROR_TIMEOUT;
}

EmmStatus emm_read_dynamic_frame(EmmDevice *device,
                                 uint8_t expected_address,
                                 uint8_t expected_code,
                                 uint8_t *response,
                                 size_t response_capacity,
                                 size_t *response_length,
                                 uint32_t timeout_ms)
{
    EmmStatus status;
    uint8_t attempts;

    if (device == 0 || response == 0 || response_length == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    status = emm_try_parse_dynamic_from_rx(device, expected_address, expected_code, response, response_capacity, response_length);
    if (status == EMM_OK || status != EMM_ERROR_TIMEOUT)
    {
        return status;
    }

    attempts = (device->poll_attempts == 0u) ? 1u : device->poll_attempts;
    for (uint8_t i = 0u; i < attempts; ++i)
    {
        status = emm_poll(device, timeout_ms);
        if (status != EMM_OK && status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        status = emm_try_parse_dynamic_from_rx(device, expected_address, expected_code, response, response_capacity, response_length);
        if (status == EMM_OK || status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        if (timeout_ms == 0u)
        {
            break;
        }
    }

    return EMM_ERROR_TIMEOUT;
}

EmmStatus emm_read_any_frame(EmmDevice *device, EmmRxFrame *frame, uint32_t timeout_ms)
{
    uint8_t response[EMM_STEPPER_MAX_FRAME_SIZE];
    size_t response_length = 0u;
    EmmStatus status;

    if (device == 0 || frame == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    status = emm_read_dynamic_frame(device, EMM_STEPPER_MATCH_ANY, EMM_STEPPER_MATCH_ANY,
                                    response, sizeof(response), &response_length, timeout_ms);
    if (status != EMM_OK)
    {
        status = emm_read_fixed_frame(device, EMM_STEPPER_MATCH_ANY, EMM_STEPPER_MATCH_ANY,
                                      response, 4u, timeout_ms);
        if (status != EMM_OK)
        {
            return status;
        }
        response_length = 4u;
    }

    frame->address = response[0];
    frame->code = response[1];
    frame->length = response_length;
    for (size_t i = 0u; i < response_length; ++i)
    {
        frame->bytes[i] = response[i];
    }
    return EMM_OK;
}

EmmStatus emm_wait_reached(EmmDevice *device, uint8_t expected_code, uint8_t *response, size_t response_length, uint32_t timeout_ms)
{
    if (device == 0 || response == 0 || response_length < 4u)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    return emm_read_fixed_frame(device, device->address, expected_code, response, response_length, timeout_ms);
}

static EmmStatus emm_write_body_once(EmmDevice *device, const uint8_t *body, size_t body_length)
{
    uint8_t command[EMM_STEPPER_MAX_FRAME_SIZE];

    if (device == 0 || body == 0 || device->transport.write == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    if (body_length + 1u > sizeof(command))
    {
        return EMM_ERROR_INVALID_ARG;
    }

    if (device->auto_flush_before_write)
    {
        emm_rx_clear(device);
        if (device->transport.flush_input != 0)
        {
            device->transport.flush_input(device->transport.user_data);
        }
    }
    if (device->transport.flush_output != 0)
    {
        device->transport.flush_output(device->transport.user_data);
    }

    for (size_t i = 0u; i < body_length; ++i)
    {
        command[i] = body[i];
    }
    command[body_length] = emm_calculate_checksum(body, body_length, device->checksum_mode);

    if (device->transport.write(command, body_length + 1u, device->transport.user_data) != body_length + 1u)
    {
        return EMM_ERROR_IO;
    }
    return EMM_OK;
}

static EmmStatus simple_status_from_response(const uint8_t *response)
{
    if (response[2] == EMM_STATUS_SUCCESS)
    {
        return EMM_OK;
    }
    if (response[2] == EMM_STATUS_PARAM_ERROR)
    {
        return EMM_ERROR_PARAM;
    }
    if (response[2] == EMM_STATUS_FORMAT_ERROR)
    {
        return EMM_ERROR_FORMAT;
    }
    return EMM_ERROR_BAD_RESPONSE;
}

static bool emm_body_is_broadcast_no_response(const uint8_t *body, size_t body_length)
{
    if (body == 0 || body_length < 2u)
    {
        return false;
    }
    if (body[0] != EMM_STEPPER_BROADCAST_ADDRESS)
    {
        return false;
    }
    return body[1] != EMM_CODE_BROADCAST_GET_ID;
}

static EmmStatus send_simple(EmmDevice *device, const uint8_t *body, size_t body_length)
{
    uint8_t response[4];
    EmmStatus status;

    if (device == 0 || body == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    if (device->response_mode == EMM_RESPONSE_NONE || emm_body_is_broadcast_no_response(body, body_length))
    {
        return emm_send_raw_no_response(device, body, body_length);
    }

    status = emm_send_raw(device, body, body_length, response, sizeof(response));
    if (status != EMM_OK)
    {
        return status;
    }
    return simple_status_from_response(response);
}

static EmmStatus send_motion(EmmDevice *device, const uint8_t *body, size_t body_length)
{
    uint8_t response[4];
    EmmStatus status;
    uint8_t expected_address;
    uint8_t expected_code;

    if (device == 0 || body == 0 || body_length < 2u)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    expected_address = (body[0] == EMM_STEPPER_BROADCAST_ADDRESS) ? EMM_STEPPER_MATCH_ANY : body[0];
    expected_code = body[1];

    status = emm_write_body_once(device, body, body_length);
    if (status != EMM_OK)
    {
        return status;
    }

    if (device->response_mode == EMM_RESPONSE_NONE || body[0] == EMM_STEPPER_BROADCAST_ADDRESS)
    {
        return EMM_OK;
    }

    if (device->response_mode == EMM_RESPONSE_REACHED)
    {
        status = emm_read_fixed_frame(device, expected_address, expected_code, response, sizeof(response), device->reached_timeout_ms);
        return (status == EMM_OK) ? simple_status_from_response(response) : status;
    }

    status = emm_read_fixed_frame(device, expected_address, expected_code, response, sizeof(response), device->timeout_ms);
    if (status != EMM_OK)
    {
        return status;
    }
    status = simple_status_from_response(response);
    if (status != EMM_OK)
    {
        return status;
    }

    if (device->response_mode == EMM_RESPONSE_BOTH)
    {
        status = emm_read_fixed_frame(device, expected_address, expected_code, response, sizeof(response), device->reached_timeout_ms);
        return (status == EMM_OK) ? simple_status_from_response(response) : status;
    }

    return EMM_OK;
}

static EmmStatus send_read(EmmDevice *device, uint8_t code, uint8_t *response, size_t response_length)
{
    uint8_t body[2];
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = code;
    return emm_send_raw(device, body, sizeof(body), response, response_length);
}

static void parse_homing_status(uint8_t data, EmmHomingStatus *status)
{
    status->encoder_ready = (data & 0x01u) != 0u;
    status->calibrated = (data & 0x02u) != 0u;
    status->is_homing = (data & 0x04u) != 0u;
    status->homing_failed = (data & 0x08u) != 0u;
    status->over_temp = (data & 0x10u) != 0u;
    status->over_current = (data & 0x20u) != 0u;
}

static void parse_motor_status(uint8_t data, EmmMotorStatus *status)
{
    status->enabled = (data & 0x01u) != 0u;
    status->position_reached = (data & 0x02u) != 0u;
    status->stall_detected = (data & 0x04u) != 0u;
    status->stall_protected = (data & 0x08u) != 0u;
    status->left_limit = (data & 0x10u) != 0u;
    status->right_limit = (data & 0x20u) != 0u;
    status->power_off_flag = (data & 0x80u) != 0u;
}

static void encode_homing_params(uint8_t *data, const EmmHomingParams *params)
{
    data[0] = (uint8_t)params->homing_mode;
    data[1] = (uint8_t)params->homing_direction;
    write_u16_be(&data[2], params->homing_speed_rpm);
    write_u32_be(&data[4], params->homing_timeout_ms);
    write_u16_be(&data[8], params->collision_speed_rpm);
    write_u16_be(&data[10], params->collision_current_ma);
    write_u16_be(&data[12], params->collision_time_ms);
    data[14] = params->auto_home ? 1u : 0u;
}

static void decode_homing_params(const uint8_t *data, EmmHomingParams *params)
{
    params->homing_mode = (EmmHomingMode)data[0];
    params->homing_direction = (EmmDirection)data[1];
    params->homing_speed_rpm = read_u16_be(&data[2]);
    params->homing_timeout_ms = read_u32_be(&data[4]);
    params->collision_speed_rpm = read_u16_be(&data[8]);
    params->collision_current_ma = read_u16_be(&data[10]);
    params->collision_time_ms = read_u16_be(&data[12]);
    params->auto_home = data[14] != 0u;
}

static void encode_config(uint8_t *data, const EmmConfigParams *params)
{
    data[0] = (uint8_t)params->motor_type;
    data[1] = (uint8_t)params->pulse_port_mode;
    data[2] = (uint8_t)params->serial_port_mode;
    data[3] = (uint8_t)params->enable_level;
    data[4] = (uint8_t)params->dir_level;
    data[5] = (params->microstep == 256u) ? 0u : (uint8_t)params->microstep;
    data[6] = params->microstep_interp ? 1u : 0u;
    data[7] = 0u;
    write_u16_be(&data[8], params->open_loop_current_ma);
    write_u16_be(&data[10], params->closed_loop_current_ma);
    write_u16_be(&data[12], params->max_voltage);
    data[14] = (uint8_t)params->baud_rate;
    data[15] = (uint8_t)params->can_rate;
    data[16] = params->motor_id;
    data[17] = (uint8_t)params->checksum_mode;
    data[18] = (uint8_t)params->response_mode;
    data[19] = (uint8_t)params->stall_protect;
    write_u16_be(&data[20], params->stall_speed_rpm);
    write_u16_be(&data[22], params->stall_current_ma);
    write_u16_be(&data[24], params->stall_time_ms);
    write_u16_be(&data[26], params->position_window_x01deg);
}

static void decode_config(const uint8_t *data, EmmConfigParams *params)
{
    params->motor_type = (EmmMotorType)data[0];
    params->pulse_port_mode = (EmmPulsePortMode)data[1];
    params->serial_port_mode = (EmmSerialPortMode)data[2];
    params->enable_level = (EmmEnableLevel)data[3];
    params->dir_level = (EmmDirLevel)data[4];
    params->microstep = (data[5] == 0u) ? 256u : (uint16_t)data[5];
    params->microstep_interp = data[6] != 0u;
    params->open_loop_current_ma = read_u16_be(&data[8]);
    params->closed_loop_current_ma = read_u16_be(&data[10]);
    params->max_voltage = read_u16_be(&data[12]);
    params->baud_rate = (EmmBaudRate)data[14];
    params->can_rate = (EmmCanRate)data[15];
    params->motor_id = data[16];
    params->checksum_mode = (EmmChecksumMode)data[17];
    params->response_mode = (EmmResponseMode)data[18];
    params->stall_protect = (EmmStallProtect)data[19];
    params->stall_speed_rpm = read_u16_be(&data[20]);
    params->stall_current_ma = read_u16_be(&data[22]);
    params->stall_time_ms = read_u16_be(&data[24]);
    params->position_window_x01deg = read_u16_be(&data[26]);
}

void emm_init(EmmDevice *device, const EmmTransport *transport, uint8_t address)
{
    if (device == 0)
    {
        return;
    }

    device->address = address;
    device->checksum_mode = EMM_CHECKSUM_FIXED;
    device->timeout_ms = EMM_STEPPER_DEFAULT_TIMEOUT_MS;
    device->retry_delay_ms = 0u;
    device->max_retries = EMM_STEPPER_MAX_RETRIES;
    device->reached_timeout_ms = EMM_STEPPER_REACHED_TIMEOUT_MS;
    device->response_mode = EMM_RESPONSE_NONE;
    device->strict_frame_check = true;
    device->auto_flush_before_write = false;
    device->auto_flush_before_read = false;
    device->poll_attempts = EMM_STEPPER_POLL_ATTEMPTS;
    device->rx_head = 0u;
    device->rx_tail = 0u;
    device->rx_overflow_count = 0u;

    if (transport != 0)
    {
        device->transport = *transport;
    }
    else
    {
        device->transport.write = 0;
        device->transport.read = 0;
        device->transport.flush_input = 0;
        device->transport.flush_output = 0;
        device->transport.delay_ms = 0;
        device->transport.user_data = 0;
    }
}

void emm_select_address(EmmDevice *device, uint8_t address)
{
    if (device != 0)
    {
        device->address = address;
    }
}

void emm_set_response_mode_local(EmmDevice *device, EmmResponseMode mode)
{
    if (device != 0)
    {
        device->response_mode = mode;
    }
}

void emm_set_timeouts(EmmDevice *device, uint32_t command_timeout_ms, uint32_t reached_timeout_ms)
{
    if (device != 0)
    {
        device->timeout_ms = command_timeout_ms;
        device->reached_timeout_ms = reached_timeout_ms;
    }
}

void emm_set_strict_frame_check(EmmDevice *device, bool enable)
{
    if (device != 0)
    {
        device->strict_frame_check = enable;
    }
}

void emm_set_auto_flush_before_write(EmmDevice *device, bool enable)
{
    if (device != 0)
    {
        device->auto_flush_before_write = enable;
    }
}

void emm_set_checksum_mode(EmmDevice *device, EmmChecksumMode mode)
{
    if (device != 0)
    {
        device->checksum_mode = mode;
    }
}

uint8_t emm_calculate_checksum(const uint8_t *data, size_t length, EmmChecksumMode mode)
{
    uint8_t checksum = 0u;

    if (mode == EMM_CHECKSUM_FIXED || data == 0 || length == 0u)
    {
        return EMM_STATUS_FIXED_CHECKSUM;
    }

    if (mode == EMM_CHECKSUM_XOR)
    {
        for (size_t i = 0u; i < length; ++i)
        {
            checksum ^= data[i];
        }
        return checksum;
    }

    if (mode == EMM_CHECKSUM_CRC8)
    {
        checksum = data[0];
        for (size_t i = 1u; i < length; ++i)
        {
            checksum = EmmCrc8Table[checksum ^ data[i]];
        }
        return checksum;
    }

    return EMM_STATUS_FIXED_CHECKSUM;
}

EmmStatus emm_send_raw_no_response(EmmDevice *device, const uint8_t *body, size_t body_length)
{
    return emm_write_body_once(device, body, body_length);
}

EmmStatus emm_send_raw(EmmDevice *device, const uint8_t *body, size_t body_length, uint8_t *response, size_t response_length)
{
    EmmStatus status;
    uint8_t expected_address;
    uint8_t expected_code;

    if (device == 0 || body == 0 || response == 0 || body_length < 2u || response_length < 3u)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    if (device->response_mode == EMM_RESPONSE_NONE || emm_body_is_broadcast_no_response(body, body_length))
    {
        return emm_send_raw_no_response(device, body, body_length);
    }

    expected_address = (body[0] == EMM_STEPPER_BROADCAST_ADDRESS) ? EMM_STEPPER_MATCH_ANY : body[0];
    expected_code = body[1];

    for (uint8_t tries = 0u; tries < device->max_retries; ++tries)
    {
        status = emm_write_body_once(device, body, body_length);
        if (status != EMM_OK)
        {
            return status;
        }

        status = emm_read_fixed_frame(device, expected_address, expected_code, response, response_length, device->timeout_ms);
        if (status == EMM_OK)
        {
            return EMM_OK;
        }
        if (status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        if (device->transport.delay_ms != 0 && device->retry_delay_ms != 0u)
        {
            device->transport.delay_ms(device->retry_delay_ms, device->transport.user_data);
        }
    }

    return EMM_ERROR_TIMEOUT;
}

EmmStatus emm_send_raw_dynamic(EmmDevice *device, const uint8_t *body, size_t body_length, uint8_t *response, size_t response_capacity, size_t *response_length)
{
    EmmStatus status;
    uint8_t expected_address;
    uint8_t expected_code;

    if (device == 0 || body == 0 || response == 0 || response_length == 0 || body_length < 2u || response_capacity < 5u)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    if (device->response_mode == EMM_RESPONSE_NONE || emm_body_is_broadcast_no_response(body, body_length))
    {
        return emm_send_raw_no_response(device, body, body_length);
    }

    expected_address = (body[0] == EMM_STEPPER_BROADCAST_ADDRESS) ? EMM_STEPPER_MATCH_ANY : body[0];
    expected_code = body[1];

    for (uint8_t tries = 0u; tries < device->max_retries; ++tries)
    {
        status = emm_write_body_once(device, body, body_length);
        if (status != EMM_OK)
        {
            return status;
        }

        status = emm_read_dynamic_frame(device, expected_address, expected_code, response, response_capacity, response_length, device->timeout_ms);
        if (status == EMM_OK)
        {
            return EMM_OK;
        }
        if (status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        if (device->transport.delay_ms != 0 && device->retry_delay_ms != 0u)
        {
            device->transport.delay_ms(device->retry_delay_ms, device->transport.user_data);
        }
    }

    return EMM_ERROR_TIMEOUT;
}

EmmStatus emm_calibrate_encoder(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_CAL_ENCODER, EMM_PROTOCOL_CAL_ENCODER };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_restart(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_RESTART, EMM_PROTOCOL_RESTART };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_zero_position(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_ZERO_POSITION, EMM_PROTOCOL_ZERO_POSITION };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_clear_protection(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_CLEAR_PROTECTION, EMM_PROTOCOL_CLEAR_PROTECTION };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_factory_reset(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_FACTORY_RESET, EMM_PROTOCOL_FACTORY_RESET };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_enable(EmmDevice *device, bool enable, EmmSyncFlag sync_flag)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[5] = { device->address, EMM_CODE_ENABLE, EMM_PROTOCOL_ENABLE, enable ? 1u : 0u, (uint8_t)sync_flag };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_disable(EmmDevice *device, EmmSyncFlag sync_flag)
{
    return emm_enable(device, false, sync_flag);
}

EmmStatus emm_jog(EmmDevice *device, const EmmJogParams *params)
{
    uint8_t body[7];
    if (device == 0 || params == 0 || params->speed_rpm > 3000u)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = EMM_CODE_JOG;
    body[2] = (uint8_t)params->direction;
    write_u16_be(&body[3], params->speed_rpm);
    body[5] = params->acceleration;
    body[6] = (uint8_t)params->sync_flag;
    return send_motion(device, body, sizeof(body));
}

EmmStatus emm_move_pulses(EmmDevice *device, const EmmPositionParams *params)
{
    uint8_t body[12];
    if (device == 0 || params == 0 || params->speed_rpm > 3000u)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = EMM_CODE_POSITION;
    body[2] = (uint8_t)params->direction;
    write_u16_be(&body[3], params->speed_rpm);
    body[5] = params->acceleration;
    write_u32_be(&body[6], params->pulse_count);
    body[10] = (uint8_t)params->motion_mode;
    body[11] = (uint8_t)params->sync_flag;
    return send_motion(device, body, sizeof(body));
}

EmmStatus emm_move_degrees(EmmDevice *device, float degrees, uint16_t speed_rpm, uint8_t acceleration, EmmMotionMode motion_mode, uint16_t microstep, EmmSyncFlag sync_flag)
{
    float pulses_float;
    int32_t pulses;
    EmmPositionParams params;

    if (microstep == 0u)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    pulses_float = degrees * (float)(200u * microstep) / 360.0f;
    pulses = (pulses_float >= 0.0f) ? (int32_t)(pulses_float + 0.5f) : (int32_t)(pulses_float - 0.5f);
    if (pulses == 0 && degrees != 0.0f)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    params.direction = (pulses < 0) ? EMM_DIRECTION_CCW : EMM_DIRECTION_CW;
    params.speed_rpm = speed_rpm;
    params.acceleration = acceleration;
    params.pulse_count = (pulses < 0) ? (uint32_t)(-pulses) : (uint32_t)pulses;
    params.motion_mode = motion_mode;
    params.sync_flag = sync_flag;
    return emm_move_pulses(device, &params);
}

EmmStatus emm_move_revolutions(EmmDevice *device, float revolutions, uint16_t speed_rpm, uint8_t acceleration, EmmMotionMode motion_mode, uint16_t microstep, EmmSyncFlag sync_flag)
{
    return emm_move_degrees(device, revolutions * 360.0f, speed_rpm, acceleration, motion_mode, microstep, sync_flag);
}

EmmStatus emm_stop(EmmDevice *device, EmmSyncFlag sync_flag)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[4] = { device->address, EMM_CODE_ESTOP, EMM_PROTOCOL_ESTOP, (uint8_t)sync_flag };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_sync_move(EmmDevice *device)
{
    uint8_t body[3] = { EMM_STEPPER_BROADCAST_ADDRESS, EMM_CODE_SYNC_MOVE, EMM_PROTOCOL_SYNC_MOVE };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_home_zero(EmmDevice *device, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[4] = { device->address, EMM_CODE_SET_HOME_ZERO, EMM_PROTOCOL_SET_HOME_ZERO, (uint8_t)store };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_home(EmmDevice *device, EmmHomingMode mode, EmmSyncFlag sync_flag)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[4] = { device->address, EMM_CODE_HOME, (uint8_t)mode, (uint8_t)sync_flag };
    return send_motion(device, body, sizeof(body));
}

EmmStatus emm_stop_home(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_STOP_HOME, EMM_PROTOCOL_STOP_HOME };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_get_homing_status(EmmDevice *device, EmmHomingStatus *status)
{
    uint8_t response[4];
    EmmStatus result;
    if (status == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_HOME_STATUS, response, sizeof(response));
    if (result == EMM_OK)
    {
        parse_homing_status(response[2], status);
    }
    return result;
}

EmmStatus emm_get_homing_params(EmmDevice *device, EmmHomingParams *params)
{
    uint8_t response[18];
    EmmStatus result;
    if (params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_HOME_PARAM, response, sizeof(response));
    if (result == EMM_OK)
    {
        decode_homing_params(&response[2], params);
    }
    return result;
}

EmmStatus emm_set_homing_params(EmmDevice *device, const EmmHomingParams *params, EmmStoreFlag store)
{
    uint8_t body[19];
    if (device == 0 || params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_HOME_PARAM;
    body[2] = EMM_PROTOCOL_SET_HOME_PARAM;
    body[3] = (uint8_t)store;
    encode_homing_params(&body[4], params);
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_get_version(EmmDevice *device, EmmVersionParams *version)
{
    uint8_t response[7];
    EmmStatus result;
    uint16_t hw_info;
    if (version == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_VERSION, response, sizeof(response));
    if (result == EMM_OK)
    {
        version->firmware_version = read_u16_be(&response[2]);
        hw_info = read_u16_be(&response[4]);
        version->hw_series = (uint8_t)((hw_info >> 12) & 0x0Fu);
        version->hw_type = (uint8_t)((hw_info >> 8) & 0x0Fu);
        version->hw_version = (uint8_t)(hw_info & 0xFFu);
    }
    return result;
}

EmmStatus emm_get_motor_rh(EmmDevice *device, EmmMotorRHParams *params)
{
    uint8_t response[7];
    EmmStatus result;
    if (params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_MOTOR_RH, response, sizeof(response));
    if (result == EMM_OK)
    {
        params->phase_resistance_mohm = read_u16_be(&response[2]);
        params->phase_inductance_uh = read_u16_be(&response[4]);
    }
    return result;
}

EmmStatus emm_get_bus_voltage(EmmDevice *device, uint16_t *voltage_mv)
{
    uint8_t response[5];
    EmmStatus result = send_read(device, EMM_CODE_GET_BUS_VOLTAGE, response, sizeof(response));
    if (result == EMM_OK && voltage_mv != 0) { *voltage_mv = read_u16_be(&response[2]); }
    return (voltage_mv == 0) ? EMM_ERROR_INVALID_ARG : result;
}

EmmStatus emm_get_bus_current(EmmDevice *device, uint16_t *current_ma)
{
    uint8_t response[5];
    EmmStatus result = send_read(device, EMM_CODE_GET_BUS_CURRENT, response, sizeof(response));
    if (result == EMM_OK && current_ma != 0) { *current_ma = read_u16_be(&response[2]); }
    return (current_ma == 0) ? EMM_ERROR_INVALID_ARG : result;
}

EmmStatus emm_get_phase_current(EmmDevice *device, uint16_t *current_ma)
{
    uint8_t response[5];
    EmmStatus result = send_read(device, EMM_CODE_GET_PHASE_CURRENT, response, sizeof(response));
    if (result == EMM_OK && current_ma != 0) { *current_ma = read_u16_be(&response[2]); }
    return (current_ma == 0) ? EMM_ERROR_INVALID_ARG : result;
}

EmmStatus emm_get_encoder(EmmDevice *device, uint16_t *encoder)
{
    uint8_t response[5];
    EmmStatus result = send_read(device, EMM_CODE_GET_ENCODER, response, sizeof(response));
    if (result == EMM_OK && encoder != 0) { *encoder = read_u16_be(&response[2]); }
    return (encoder == 0) ? EMM_ERROR_INVALID_ARG : result;
}

EmmStatus emm_get_encoder_degrees(EmmDevice *device, float *degrees)
{
    uint16_t encoder;
    EmmStatus result;
    if (degrees == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = emm_get_encoder(device, &encoder);
    if (result == EMM_OK)
    {
        *degrees = ((float)encoder * 360.0f) / 65536.0f;
    }
    return result;
}

EmmStatus emm_get_pulse_count(EmmDevice *device, int32_t *pulse_count)
{
    uint8_t response[8];
    EmmStatus result = send_read(device, EMM_CODE_GET_PULSE_COUNT, response, sizeof(response));
    if (result == EMM_OK && pulse_count != 0) { *pulse_count = read_signed_prefix(&response[2], 5u); }
    return (pulse_count == 0) ? EMM_ERROR_INVALID_ARG : result;
}

EmmStatus emm_get_target_position(EmmDevice *device, float *degrees)
{
    uint8_t response[8];
    EmmStatus result = send_read(device, EMM_CODE_GET_TARGET_POSITION, response, sizeof(response));
    if (result == EMM_OK && degrees != 0) { *degrees = ((float)read_signed_prefix(&response[2], 5u) * 360.0f) / 65536.0f; }
    return (degrees == 0) ? EMM_ERROR_INVALID_ARG : result;
}

EmmStatus emm_get_realtime_speed(EmmDevice *device, int16_t *speed_rpm)
{
    uint8_t response[6];
    int16_t sign;
    EmmStatus result = send_read(device, EMM_CODE_GET_REALTIME_SPEED, response, sizeof(response));
    if (result == EMM_OK && speed_rpm != 0)
    {
        sign = (response[2] == 1u) ? -1 : 1;
        *speed_rpm = (int16_t)(sign * (int16_t)read_u16_be(&response[3]));
    }
    return (speed_rpm == 0) ? EMM_ERROR_INVALID_ARG : result;
}

EmmStatus emm_get_realtime_position(EmmDevice *device, float *degrees)
{
    uint8_t response[8];
    EmmStatus result = send_read(device, EMM_CODE_GET_REALTIME_POSITION, response, sizeof(response));
    if (result == EMM_OK && degrees != 0) { *degrees = ((float)read_signed_prefix(&response[2], 5u) * 360.0f) / 65536.0f; }
    return (degrees == 0) ? EMM_ERROR_INVALID_ARG : result;
}

EmmStatus emm_get_position_error(EmmDevice *device, float *degrees)
{
    uint8_t response[8];
    EmmStatus result = send_read(device, EMM_CODE_GET_POSITION_ERROR, response, sizeof(response));
    if (result == EMM_OK && degrees != 0) { *degrees = ((float)read_signed_prefix(&response[2], 5u) * 360.0f) / 65536.0f; }
    return (degrees == 0) ? EMM_ERROR_INVALID_ARG : result;
}

EmmStatus emm_get_temperature(EmmDevice *device, int16_t *temperature_c)
{
    uint8_t response[5];
    int16_t sign;
    EmmStatus result = send_read(device, EMM_CODE_GET_TEMPERATURE, response, sizeof(response));
    if (result == EMM_OK && temperature_c != 0)
    {
        sign = (response[2] == 0u) ? -1 : 1;
        *temperature_c = (int16_t)(sign * (int16_t)response[3]);
    }
    return (temperature_c == 0) ? EMM_ERROR_INVALID_ARG : result;
}

EmmStatus emm_get_motor_status(EmmDevice *device, EmmMotorStatus *status)
{
    uint8_t response[4];
    EmmStatus result;
    if (status == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_MOTOR_STATUS, response, sizeof(response));
    if (result == EMM_OK)
    {
        parse_motor_status(response[2], status);
    }
    return result;
}

EmmStatus emm_get_pid(EmmDevice *device, EmmPIDParams *params)
{
    uint8_t response[15];
    EmmStatus result;
    if (params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_PID, response, sizeof(response));
    if (result == EMM_OK)
    {
        params->kp = read_u32_be(&response[2]);
        params->ki = read_u32_be(&response[6]);
        params->kd = read_u32_be(&response[10]);
    }
    return result;
}

EmmStatus emm_get_config(EmmDevice *device, EmmConfigParams *params)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_GET_CONFIG, EMM_PROTOCOL_GET_CONFIG };
    uint8_t response[40];
    size_t response_length = 0u;
    EmmStatus result;
    if (params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = emm_send_raw_dynamic(device, body, sizeof(body), response, sizeof(response), &response_length);
    if (result == EMM_OK && response_length >= 33u)
    {
        decode_config(&response[4], params);
    }
    return result;
}

EmmStatus emm_get_system_status(EmmDevice *device, EmmSystemStatusParams *params)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_GET_SYS_STATUS, EMM_PROTOCOL_GET_SYS_STATUS };
    uint8_t response[40];
    size_t response_length = 0u;
    EmmStatus result;
    const uint8_t *data;
    if (params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = emm_send_raw_dynamic(device, body, sizeof(body), response, sizeof(response), &response_length);
    if (result == EMM_OK && response_length >= 31u)
    {
        data = &response[4];
        params->bus_voltage_mv = read_u16_be(&data[0]);
        params->phase_current_ma = read_u16_be(&data[2]);
        params->encoder_value = read_u16_be(&data[4]);
        params->target_position = read_signed_prefix(&data[6], 5u);
        params->realtime_speed_rpm = (int16_t)(((data[11] == 1u) ? -1 : 1) * (int16_t)read_u16_be(&data[12]));
        params->realtime_position = read_signed_prefix(&data[14], 5u);
        params->position_error = read_signed_prefix(&data[19], 5u);
        parse_homing_status(data[24], &params->homing_status);
        parse_motor_status(data[25], &params->motor_status);
    }
    return result;
}

EmmStatus emm_set_id(EmmDevice *device, uint8_t new_id, EmmStoreFlag store)
{
    uint8_t body[5];
    EmmStatus result;
    if (device == 0 || new_id == 0u)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_ID;
    body[2] = EMM_PROTOCOL_SET_ID;
    body[3] = (uint8_t)store;
    body[4] = new_id;
    result = send_simple(device, body, sizeof(body));
    if (result == EMM_OK)
    {
        device->address = new_id;
    }
    return result;
}

EmmStatus emm_set_microstep(EmmDevice *device, uint16_t microstep, EmmStoreFlag store)
{
    uint8_t body[5];
    if (device == 0 || microstep == 0u || microstep > 256u)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_MICROSTEP;
    body[2] = EMM_PROTOCOL_SET_MICROSTEP;
    body[3] = (uint8_t)store;
    body[4] = (microstep == 256u) ? 0u : (uint8_t)microstep;
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_loop_mode(EmmDevice *device, EmmControlMode mode, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[5] = { device->address, EMM_CODE_SET_LOOP_MODE, EMM_PROTOCOL_SET_LOOP_MODE, (uint8_t)store, (uint8_t)mode };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_open_loop_current(EmmDevice *device, uint16_t current_ma, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[6] = { device->address, EMM_CODE_SET_OPEN_LOOP_CURRENT, EMM_PROTOCOL_SET_OPEN_LOOP_CURRENT, (uint8_t)store, 0u, 0u };
    if (current_ma > 5000u) { return EMM_ERROR_INVALID_ARG; }
    write_u16_be(&body[4], current_ma);
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_closed_loop_current(EmmDevice *device, uint16_t current_ma, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[6] = { device->address, EMM_CODE_SET_CLOSED_LOOP_CURRENT, EMM_PROTOCOL_SET_CLOSED_LOOP_CURRENT, (uint8_t)store, 0u, 0u };
    if (current_ma > 5000u) { return EMM_ERROR_INVALID_ARG; }
    write_u16_be(&body[4], current_ma);
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_pid(EmmDevice *device, const EmmPIDParams *params, EmmStoreFlag store)
{
    uint8_t body[16];
    if (device == 0 || params == 0) { return EMM_ERROR_INVALID_ARG; }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_PID;
    body[2] = EMM_PROTOCOL_SET_PID;
    body[3] = (uint8_t)store;
    write_u32_be(&body[4], params->kp);
    write_u32_be(&body[8], params->ki);
    write_u32_be(&body[12], params->kd);
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_motor_direction(EmmDevice *device, EmmDirection direction, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[5] = { device->address, EMM_CODE_SET_MOTOR_DIRECTION, EMM_PROTOCOL_SET_MOTOR_DIRECTION, (uint8_t)store, (uint8_t)direction };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_position_window(EmmDevice *device, float window_deg, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[6] = { device->address, EMM_CODE_SET_POSITION_WINDOW, EMM_PROTOCOL_SET_POSITION_WINDOW, (uint8_t)store, 0u, 0u };
    uint16_t window = (uint16_t)(window_deg * 10.0f);
    write_u16_be(&body[4], window);
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_heartbeat_time(EmmDevice *device, uint32_t time_ms, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[8] = { device->address, EMM_CODE_SET_HEARTBEAT_TIME, EMM_PROTOCOL_SET_HEARTBEAT_TIME, (uint8_t)store, 0u, 0u, 0u, 0u };
    write_u32_be(&body[4], time_ms);
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_auto_run(EmmDevice *device, const EmmAutoRunParams *params)
{
    uint8_t body[9];
    if (device == 0 || params == 0) { return EMM_ERROR_INVALID_ARG; }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_AUTO_RUN;
    body[2] = EMM_PROTOCOL_SET_AUTO_RUN;
    body[3] = params->store ? 1u : 0u;
    body[4] = (uint8_t)params->direction;
    write_u16_be(&body[5], params->speed_rpm);
    body[7] = params->acceleration;
    body[8] = params->enable_en_control ? 1u : 0u;
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_config(EmmDevice *device, const EmmConfigParams *params, EmmStoreFlag store)
{
    uint8_t body[32];
    if (device == 0 || params == 0) { return EMM_ERROR_INVALID_ARG; }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_CONFIG;
    body[2] = EMM_PROTOCOL_SET_CONFIG;
    body[3] = (uint8_t)store;
    encode_config(&body[4], params);
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_scale_input(EmmDevice *device, bool enable, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[5] = { device->address, EMM_CODE_SET_SCALE_INPUT, EMM_PROTOCOL_SET_SCALE_INPUT, (uint8_t)store, enable ? 1u : 0u };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_set_lock_button(EmmDevice *device, bool lock, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[5] = { device->address, EMM_CODE_SET_LOCK_BUTTON, EMM_PROTOCOL_SET_LOCK_BUTTON, (uint8_t)store, lock ? 1u : 0u };
    return send_simple(device, body, sizeof(body));
}

EmmStatus emm_broadcast_get_id(EmmDevice *device, uint8_t *motor_id)
{
    uint8_t body[2] = { EMM_STEPPER_BROADCAST_ADDRESS, EMM_CODE_BROADCAST_GET_ID };
    uint8_t response[4];
    EmmStatus result;
    if (motor_id == 0) { return EMM_ERROR_INVALID_ARG; }
    result = emm_send_raw(device, body, sizeof(body), response, sizeof(response));
    if (result == EMM_OK)
    {
        *motor_id = response[2];
    }
    return result;
}

/* ================================================================
 *  Forced-response read helpers
 *
 *  These temporarily override device->response_mode to
 *  EMM_RESPONSE_RECEIVE, perform the read, and restore the
 *  original mode.  Use when the device normally runs in
 *  EMM_RESPONSE_NONE (fire-and-forget writes) but you need
 *  actual data from the motor.
 * ================================================================ */

static EmmStatus emm_forced_read(EmmDevice *device,
                                 const uint8_t *body, size_t body_length,
                                 uint8_t *response, size_t response_length)
{
    EmmResponseMode saved_mode;
    uint8_t raw[64];
    size_t raw_len;
    size_t echo_len;
    size_t i;
    uint32_t waited_ms;
    uint8_t expected_addr;
    uint8_t expected_code;

    if (device == 0 || body == 0 || response == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    saved_mode = device->response_mode;
    device->response_mode = EMM_RESPONSE_RECEIVE;

    /* Flush stale RX data. */
    if (device->auto_flush_before_read)
    {
        emm_rx_clear(device);
        if (device->transport.flush_input != 0)
        {
            device->transport.flush_input(device->transport.user_data);
        }
    }

    /* Write command (echo goes into RX buffer). */
    {
        EmmStatus ws = emm_write_body_once(device, body, body_length);
        if (ws != EMM_OK)
        {
            device->response_mode = saved_mode;
            return ws;
        }
    }

    expected_addr = body[0];
    expected_code = body[1];
    echo_len     = body_length + 1u;  /* body + checksum */

    /* Wait for echo + response to arrive, then read raw bytes. */
    if (device->transport.delay_ms != 0)
    {
        device->transport.delay_ms(10u, device->transport.user_data);
    }

    /* Drain transport-level RX into raw[] buffer. */
    raw_len = 0u;
    waited_ms = 0u;
    while (waited_ms < device->timeout_ms)
    {
        size_t n;
        if (device->transport.read == 0) { break; }
        n = device->transport.read(&raw[raw_len],
                    sizeof(raw) - raw_len,
                    5u,  /* short timeout per poll */
                    device->transport.user_data);
        if (n > 0u) { raw_len += n; }
        if (raw_len >= echo_len + response_length) { break; }
        if (device->transport.delay_ms != 0)
        {
            device->transport.delay_ms(1u, device->transport.user_data);
        }
        waited_ms += 6u;  /* 5ms read timeout + 1ms delay */
    }

    /* Search for the real response (skip echo bytes).
       The echo is always [addr, code, 0x6B] and arrives first
       (MCU receives its own TX on the half-duplex bus).
       We skip the echo by its known length, then find the
       real response after it. */
    {
        size_t search_start = 0u;

        /* If the buffer starts with the echo pattern, skip it.
           Echo = body bytes + checksum, length = echo_len. */
        if (raw_len >= echo_len
            && raw[0] == expected_addr
            && raw[1] == expected_code
            && raw[2] == EMM_STATUS_FIXED_CHECKSUM)
        {
            search_start = echo_len;
        }

        for (i = search_start; i + response_length <= raw_len; i++)
        {
            if (raw[i] != expected_addr) { continue; }
            if (raw[i + 1u] != expected_code) { continue; }
            /* Only accept if byte 2 is NOT the fixed checksum
               (echo always has 0x6B at offset 2, real data never does). */
            if (i + 2u < raw_len && raw[i + 2u] == EMM_STATUS_FIXED_CHECKSUM)
            {
                continue;
            }
            /* Found the real response.  Copy to response. */
        for (size_t j = 0u; j < response_length; j++)
        {
            response[j] = raw[i + j];
        }
        device->response_mode = saved_mode;
        return EMM_OK;
    }

    device->response_mode = saved_mode;
    return EMM_ERROR_TIMEOUT;
}

}

EmmStatus emm_get_realtime_position_forced(EmmDevice *device, float *degrees)
{
    uint8_t response[8];
    EmmStatus result;

    if (device == 0 || degrees == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    {
        uint8_t body[2];
        body[0] = device->address;
        body[1] = EMM_CODE_GET_REALTIME_POSITION;
        result = emm_forced_read(device, body, sizeof(body), response, sizeof(response));
    }

    if (result == EMM_OK)
    {
        *degrees = ((float)read_signed_prefix(&response[2], 5u) * 360.0f) / 65536.0f;
    }
    return result;
}

EmmStatus emm_get_encoder_forced(EmmDevice *device, uint16_t *encoder)
{
    uint8_t response[5];
    EmmStatus result;

    if (device == 0 || encoder == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    {
        uint8_t body[2];
        body[0] = device->address;
        body[1] = EMM_CODE_GET_ENCODER;
        result = emm_forced_read(device, body, sizeof(body), response, sizeof(response));
    }

    if (result == EMM_OK)
    {
        *encoder = read_u16_be(&response[2]);
    }
    return result;
}

EmmStatus emm_get_motor_status_forced(EmmDevice *device, EmmMotorStatus *status)
{
    uint8_t response[4];
    EmmStatus result;

    if (device == 0 || status == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    {
        uint8_t body[2];
        body[0] = device->address;
        body[1] = EMM_CODE_GET_MOTOR_STATUS;
        result = emm_forced_read(device, body, sizeof(body), response, sizeof(response));
    }

    if (result == EMM_OK)
    {
        parse_motor_status(response[2], status);
    }
    return result;
}

EmmStatus emm_get_system_status_forced(EmmDevice *device, EmmSystemStatusParams *params)
{
    uint8_t body[3];
    uint8_t response[40];
    size_t response_length = 0u;
    EmmStatus result;
    EmmResponseMode saved_mode;
    const uint8_t *data;

    if (device == 0 || params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    body[0] = device->address;
    body[1] = EMM_CODE_GET_SYS_STATUS;
    body[2] = EMM_PROTOCOL_GET_SYS_STATUS;

    /* System status uses dynamic-length frame; save/restore response_mode. */
    saved_mode = device->response_mode;
    device->response_mode = EMM_RESPONSE_RECEIVE;

    if (device->auto_flush_before_read)
    {
        emm_rx_clear(device);
        if (device->transport.flush_input != 0)
        {
            device->transport.flush_input(device->transport.user_data);
        }
    }

    result = emm_send_raw_dynamic(device, body, sizeof(body),
                                  response, sizeof(response), &response_length);

    device->response_mode = saved_mode;

    if (result == EMM_OK && response_length >= 31u)
    {
        data = &response[4];
        params->bus_voltage_mv = read_u16_be(&data[0]);
        params->phase_current_ma = read_u16_be(&data[2]);
        params->encoder_value = read_u16_be(&data[4]);
        params->target_position = read_signed_prefix(&data[6], 5u);
        params->realtime_speed_rpm = (int16_t)(((data[11] == 1u) ? -1 : 1) * (int16_t)read_u16_be(&data[12]));
        params->realtime_position = read_signed_prefix(&data[14], 5u);
        params->position_error = read_signed_prefix(&data[19], 5u);
        parse_homing_status(data[24], &params->homing_status);
        parse_motor_status(data[25], &params->motor_status);
    }
    return result;
}

EmmStatus emm_get_pulse_count_forced(EmmDevice *device, int32_t *pulse_count)
{
    uint8_t response[8];
    EmmStatus result;

    if (device == 0 || pulse_count == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    {
        uint8_t body[2];
        body[0] = device->address;
        body[1] = EMM_CODE_GET_PULSE_COUNT;
        result = emm_forced_read(device, body, sizeof(body), response, sizeof(response));
    }

    if (result == EMM_OK)
    {
        *pulse_count = read_signed_prefix(&response[2], 5u);
    }
    return result;
}

/* ================================================================
 *  Calibration helpers
 * ================================================================ */

EmmStatus emm_jog_no_response(EmmDevice *device, const EmmJogParams *params)
{
    uint8_t body[7];

    if (device == 0 || params == 0 || params->speed_rpm > 3000u)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    body[0] = device->address;
    body[1] = EMM_CODE_JOG;
    body[2] = (uint8_t)params->direction;
    write_u16_be(&body[3], params->speed_rpm);
    body[5] = params->acceleration;
    body[6] = (uint8_t)params->sync_flag;

    return emm_send_raw_no_response(device, body, sizeof(body));
}

EmmStatus emm_clear_stall_and_recover(EmmDevice *device)
{
    EmmStatus status;

    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    /* First stop the motor */
    {
        uint8_t body[4];
        body[0] = device->address;
        body[1] = EMM_CODE_ESTOP;
        body[2] = EMM_PROTOCOL_ESTOP;
        body[3] = (uint8_t)EMM_SYNC_IMMEDIATE;
        (void)emm_send_raw_no_response(device, body, sizeof(body));
    }

    /* Give the motor a moment to process the stop */
    if (device->transport.delay_ms != 0)
    {
        device->transport.delay_ms(50u, device->transport.user_data);
    }

    /* Clear stall / over-current protection */
    {
        uint8_t body[3];
        body[0] = device->address;
        body[1] = EMM_CODE_CLEAR_PROTECTION;
        body[2] = EMM_PROTOCOL_CLEAR_PROTECTION;

        /* Use forced read so we know the clear was processed */
        uint8_t response[4];
        EmmResponseMode saved = device->response_mode;
        device->response_mode = EMM_RESPONSE_RECEIVE;
        if (device->auto_flush_before_read)
        {
            emm_rx_clear(device);
            if (device->transport.flush_input != 0)
            {
                device->transport.flush_input(device->transport.user_data);
            }
        }
        status = emm_send_raw(device, body, sizeof(body), response, sizeof(response));
        device->response_mode = saved;
    }

    /* Small delay after clearing */
    if (device->transport.delay_ms != 0)
    {
        device->transport.delay_ms(50u, device->transport.user_data);
    }

    /* Re-enable the motor */
    {
        uint8_t body[5];
        body[0] = device->address;
        body[1] = EMM_CODE_ENABLE;
        body[2] = EMM_PROTOCOL_ENABLE;
        body[3] = 1u;  /* enable */
        body[4] = (uint8_t)EMM_SYNC_IMMEDIATE;
        (void)emm_send_raw_no_response(device, body, sizeof(body));
    }

    /* Another small delay for the enable to take effect */
    if (device->transport.delay_ms != 0)
    {
        device->transport.delay_ms(50u, device->transport.user_data);
    }

    return status;
}
