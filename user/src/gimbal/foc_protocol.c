/**
 * @file foc_protocol.c
 * @brief FOC 二进制协议实现
 */
#include "gimbal/foc_protocol.h"

#include <string.h>

/* ==================== CRC8 查表 (poly 0x07) ==================== */
static const uint8_t crc8_tab[256] = {
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3,
};

uint8_t foc_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;
    while (len--)
    {
        crc = crc8_tab[crc ^ *data++];
    }
    return crc;
}

/* ==================== 浮点编解码 (小端) ==================== */

void foc_put_f32_le(uint8_t *buf, float val)
{
    union { float f; uint32_t u; } v;
    v.f = val;
    buf[0] = (uint8_t)(v.u);
    buf[1] = (uint8_t)(v.u >> 8);
    buf[2] = (uint8_t)(v.u >> 16);
    buf[3] = (uint8_t)(v.u >> 24);
}

float foc_get_f32_le(const uint8_t *buf)
{
    union { float f; uint32_t u; } v;
    v.u = (uint32_t)buf[0]
        | ((uint32_t)buf[1] << 8)
        | ((uint32_t)buf[2] << 16)
        | ((uint32_t)buf[3] << 24);
    return v.f;
}

/* ==================== 帧构建 ==================== */

uint8_t foc_frame_build(uint8_t cmd, const uint8_t *data, uint8_t len,
    uint8_t *frame)
{
    uint8_t idx = 0u;

    frame[idx++] = FOC_SYNC;
    frame[idx++] = cmd;
    frame[idx++] = len;
    if (len > 0u && data != NULL)
    {
        (void)memcpy(&frame[idx], data, len);
    }
    idx += len;
    frame[idx] = foc_crc8(&frame[1], idx - 1u);
    idx++;

    return idx;
}

/* ==================== 发送 ==================== */

void foc_send_frame(uart_index_enum uart, uint8_t cmd,
    const uint8_t *data, uint8_t len)
{
    uint8_t frame[FOC_FRAME_MAX];
    uint8_t frame_len;

    frame_len = foc_frame_build(cmd, data, len, frame);
    uart_write_buffer(uart, frame, frame_len);
}

static bool foc_read_byte_timed(uart_index_enum uart, uint32_t timeout_ms,
    uint8_t *byte)
{
    uint32_t elapsed;
    uint32_t retries;

    retries = timeout_ms * 100u;
    for (elapsed = 0u; elapsed < retries; elapsed++)
    {
        if (uart_query_byte(uart, byte) == ZF_TRUE)
        {
            return true;
        }
        system_delay_us(10u);
    }
    return false;
}

/* ==================== 发送并等待应答 ==================== */

uint8_t foc_send_cmd(uart_index_enum uart, uint8_t cmd,
    const uint8_t *tx_data, uint8_t tx_len,
    uint32_t timeout_ms, uint8_t *rx_data, uint8_t *rx_len)
{
    uint8_t frame[FOC_FRAME_MAX];
    uint8_t frame_len;
    uint8_t byte;
    uint8_t rx_cmd;
    uint8_t len;
    uint8_t i;
    uint8_t crc;
    uint8_t calc_crc;
    uint8_t buf[FOC_PROTO_MAX_DATA + 4u];

    foc_send_frame(uart, cmd, tx_data, tx_len);

    while (true)
    {
        if (!foc_read_byte_timed(uart, timeout_ms, &byte))
        {
            return ZF_FALSE;
        }
        if (byte == FOC_SYNC)
        {
            break;
        }
    }

    if (!foc_read_byte_timed(uart, timeout_ms, &rx_cmd))
    {
        return ZF_FALSE;
    }
    if (!foc_read_byte_timed(uart, timeout_ms, &len))
    {
        return ZF_FALSE;
    }

    if (len > FOC_PROTO_MAX_DATA)
    {
        return ZF_FALSE;
    }

    buf[0] = rx_cmd;
    buf[1] = len;

    for (i = 0u; i < len; i++)
    {
        if (!foc_read_byte_timed(uart, timeout_ms, &buf[i + 2u]))
        {
            return ZF_FALSE;
        }
    }

    if (!foc_read_byte_timed(uart, timeout_ms, &crc))
    {
        return ZF_FALSE;
    }

    calc_crc = foc_crc8(buf, len + 2u);
    if (crc != calc_crc)
    {
        return ZF_FALSE;
    }

    if (rx_cmd & 0x80u)
    {
        return ZF_FALSE;
    }

    if (rx_data != NULL && len > 0u)
    {
        (void)memcpy(rx_data, &buf[2], len);
    }
    if (rx_len != NULL)
    {
        *rx_len = len;
    }

    return ZF_TRUE;
}

/* ==================== 状态解析 ==================== */

bool foc_parse_status(const uint8_t *data, uint8_t len,
    foc_motor_status_t *status)
{
    if (data == NULL || status == NULL || len < 28u)
    {
        return false;
    }

    status->motor_id       = data[0];
    status->run_mode       = data[1];
    status->active_profile = data[2];
    status->flags          = data[3];
    status->target_deg     = foc_get_f32_le(&data[4]);
    status->trajectory_deg = foc_get_f32_le(&data[8]);
    status->angle_deg      = foc_get_f32_le(&data[12]);
    status->velocity_deg_s = foc_get_f32_le(&data[16]);
    status->error_deg      = foc_get_f32_le(&data[20]);
    status->applied_voltage = foc_get_f32_le(&data[24]);
    status->online         = (status->flags & FOC_FLAG_ONLINE) != 0u;
    status->in_position    = (status->flags & FOC_FLAG_IN_POSITION) != 0u;

    return true;
}

bool foc_parse_dual_status(const uint8_t *data, uint8_t len,
    foc_motor_status_t *m0, foc_motor_status_t *m1)
{
    if (data == NULL || len < 56u)
    {
        return false;
    }

    foc_parse_status(data, 28u, m0);
    foc_parse_status(&data[28], 28u, m1);
    return true;
}

/* ==================== 高级封装 ==================== */

uint8_t foc_ping(uart_index_enum uart, uint32_t timeout_ms,
    uint8_t *motor_count)
{
    uint8_t rx[4];
    uint8_t rx_len;

    if (!foc_send_cmd(uart, FOC_CMD_PING, NULL, 0u,
            timeout_ms, rx, &rx_len))
    {
        return ZF_FALSE;
    }
    if (rx_len >= 1u && motor_count != NULL)
    {
        *motor_count = rx[0];
    }
    return ZF_TRUE;
}

uint8_t foc_session_enter(uart_index_enum uart, uint32_t timeout_ms)
{
    uint8_t data[1] = { 1u };
    uint8_t rx[4];
    uint8_t rx_len;

    return foc_send_cmd(uart, FOC_CMD_SESSION, data, 1u,
        timeout_ms, rx, &rx_len);
}

uint8_t foc_get_info(uart_index_enum uart, uint32_t timeout_ms,
    foc_device_info_t *info)
{
    uint8_t rx[16];
    uint8_t rx_len;

    if (!foc_send_cmd(uart, FOC_CMD_GET_INFO, NULL, 0u,
            timeout_ms, rx, &rx_len))
    {
        return ZF_FALSE;
    }
    if (info != NULL && rx_len >= 7u)
    {
        info->ver_major     = rx[0];
        info->ver_minor     = rx[1];
        info->online_bitmap = rx[2];
        info->capabilities  = (uint32_t)rx[3]
                            | ((uint32_t)rx[4] << 8)
                            | ((uint32_t)rx[5] << 16)
                            | ((uint32_t)rx[6] << 24);
    }
    return ZF_TRUE;
}

uint8_t foc_set_angle(uart_index_enum uart, uint8_t motor_id,
    float deg, uint32_t timeout_ms)
{
    uint8_t data[5];
    uint8_t rx[8];
    uint8_t rx_len;

    data[0] = motor_id;
    foc_put_f32_le(&data[1], deg);
    return foc_send_cmd(uart, FOC_CMD_SET_ANGLE, data, 5u,
        timeout_ms, rx, &rx_len);
}

uint8_t foc_set_dual_angle(uart_index_enum uart, float m0_deg,
    float m1_deg, uint32_t timeout_ms)
{
    uint8_t data[8];
    uint8_t rx[12];
    uint8_t rx_len;

    foc_put_f32_le(&data[0], m0_deg);
    foc_put_f32_le(&data[4], m1_deg);
    return foc_send_cmd(uart, FOC_CMD_SET_DUAL_ANGLE, data, 8u,
        timeout_ms, rx, &rx_len);
}

void foc_set_dual_angle_fast(uart_index_enum uart, float m0_deg,
    float m1_deg)
{
    uint8_t data[8];

    foc_put_f32_le(&data[0], m0_deg);
    foc_put_f32_le(&data[4], m1_deg);
    foc_send_frame(uart, FOC_CMD_SET_DUAL_ANGLE, data, 8u);
}

uint8_t foc_hold(uart_index_enum uart, uint8_t motor_id,
    uint32_t timeout_ms)
{
    uint8_t data[1];
    uint8_t rx[4];
    uint8_t rx_len;

    data[0] = motor_id;
    return foc_send_cmd(uart, FOC_CMD_HOLD, data, 1u,
        timeout_ms, rx, &rx_len);
}

uint8_t foc_set_run_mode(uart_index_enum uart, uint8_t motor_id,
    uint8_t run_mode, uint32_t timeout_ms)
{
    uint8_t data[2];
    uint8_t rx[8];
    uint8_t rx_len;

    data[0] = motor_id;
    data[1] = run_mode;
    return foc_send_cmd(uart, FOC_CMD_SET_RUN_MODE, data, 2u,
        timeout_ms, rx, &rx_len);
}

uint8_t foc_set_control_mode(uart_index_enum uart, uint8_t motor_id,
    uint8_t mode, uint32_t timeout_ms)
{
    uint8_t data[2];
    uint8_t rx[4];
    uint8_t rx_len;

    data[0] = motor_id;
    data[1] = mode;
    return foc_send_cmd(uart, FOC_CMD_SET_MODE, data, 2u,
        timeout_ms, rx, &rx_len);
}

uint8_t foc_get_status(uart_index_enum uart, uint8_t motor_id,
    uint32_t timeout_ms, foc_motor_status_t *status)
{
    uint8_t data[1];
    uint8_t rx[32];
    uint8_t rx_len;

    data[0] = motor_id;
    if (!foc_send_cmd(uart, FOC_CMD_GET_STATUS, data, 1u,
            timeout_ms, rx, &rx_len))
    {
        return ZF_FALSE;
    }
    return foc_parse_status(rx, rx_len, status) ? ZF_TRUE : ZF_FALSE;
}

uint8_t foc_get_all_status(uart_index_enum uart, uint32_t timeout_ms,
    foc_motor_status_t *m0, foc_motor_status_t *m1)
{
    uint8_t rx[60];
    uint8_t rx_len;

    if (!foc_send_cmd(uart, FOC_CMD_GET_ALL_STATUS, NULL, 0u,
            timeout_ms, rx, &rx_len))
    {
        return ZF_FALSE;
    }
    if (rx_len < 56u)
    {
        return ZF_FALSE;
    }
    foc_parse_status(rx, 28u, m0);
    foc_parse_status(&rx[28], 28u, m1);
    return ZF_TRUE;
}
