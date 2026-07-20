#include "gimbal/maixcam2_protocol.h"

#include "gimbal/serial_rx_buffer.h"
#include "zf_common_headfile.h"

typedef enum
{
    PARSER_WAIT_HEAD1 = 0,
    PARSER_WAIT_HEAD2,
    PARSER_READ_VER,
    PARSER_READ_MSG_ID,
    PARSER_READ_SEQ,
    PARSER_READ_LEN,
    PARSER_READ_PAYLOAD,
    PARSER_READ_CRC_L,
    PARSER_READ_CRC_H,
} MaixParserState;

static uint8_t MaixRxStorage[256];
static SerialRxBuffer MaixRxBuffer;
static MaixParserState ParserState = PARSER_WAIT_HEAD1;
static uint8_t ParserVer = 0u;
static uint8_t ParserMsgId = 0u;
static uint8_t ParserSeq = 0u;
static uint8_t ParserLen = 0u;
static uint8_t ParserPayload[MAIXCAM2_MAX_PAYLOAD];
static uint8_t ParserIndex = 0u;
static uint8_t ParserCrcL = 0u;
static MaixVisionTarget LatestTarget;
static uint32_t LatestTargetRxTimeMs = 0u;
static bool HasLatestTarget = false;
static MaixProtocolStats Stats;

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
}

uint16_t maixcam2_crc16_modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFu;

    for (uint16_t i = 0u; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; ++bit)
        {
            if (crc & 0x0001u)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static void maix_uart_rx_callback(uint32 state, void *ptr)
{
    uint8 byte;

    (void)ptr;

    if ((state & UART_INTERRUPT_STATE_RX) == 0u)
    {
        return;
    }

    while (uart_query_byte(BOARD_MAIXCAM_UART, &byte) == ZF_TRUE)
    {
        (void)serial_rx_buffer_push(&MaixRxBuffer, byte);
    }
}

static uint32_t ParserNowMs = 0u;

static void parse_target_payload(const uint8_t *payload)
{
    MaixVisionTarget target;

    target.timestamp_ms = read_u32_le(&payload[0]);
    target.vision_state = payload[4];
    target.source = payload[5];
    target.target_valid = payload[6];
    target.track_id = payload[7];
    target.class_id = payload[8];
    target.confidence = payload[9];
    target.quality = payload[10];
    target.flags = payload[11];
    target.error_x = read_i16_le(&payload[12]);
    target.error_y = read_i16_le(&payload[14]);
    target.target_x = read_i16_le(&payload[16]);
    target.target_y = read_i16_le(&payload[18]);
    target.target_w = read_u16_le(&payload[20]);
    target.target_h = read_u16_le(&payload[22]);
    target.yolo_x = read_i16_le(&payload[24]);
    target.yolo_y = read_i16_le(&payload[26]);
    target.yolo_w = read_u16_le(&payload[28]);
    target.yolo_h = read_u16_le(&payload[30]);
    target.vx = read_i16_le(&payload[32]);
    target.vy = read_i16_le(&payload[34]);

    LatestTarget = target;
    LatestTargetRxTimeMs = ParserNowMs;
    HasLatestTarget = true;
}

static void handle_frame(uint8_t ver, uint8_t msg_id, uint8_t seq, uint8_t len, const uint8_t *payload, uint16_t rx_crc)
{
    uint8_t crc_data[4u + MAIXCAM2_MAX_PAYLOAD];
    uint16_t crc_len;
    uint16_t calc_crc;

    if (ver != MAIXCAM2_PROTOCOL_VER)
    {
        Stats.version_errors++;
        return;
    }

    crc_data[0] = ver;
    crc_data[1] = msg_id;
    crc_data[2] = seq;
    crc_data[3] = len;
    for (uint8_t i = 0u; i < len; ++i)
    {
        crc_data[4u + i] = payload[i];
    }
    crc_len = (uint16_t)(4u + len);
    calc_crc = maixcam2_crc16_modbus(crc_data, crc_len);

    if (calc_crc != rx_crc)
    {
        Stats.crc_errors++;
        return;
    }

    if (Stats.has_last_seq && (uint8_t)(Stats.last_seq + 1u) != seq)
    {
        Stats.sequence_lost++;
    }
    Stats.last_seq = seq;
    Stats.has_last_seq = true;
    Stats.frames_received++;

    if (msg_id == MAIX_MSG_VISION_TARGET)
    {
        if (len != MAIXCAM2_TARGET_PAYLOAD_LEN)
        {
            Stats.length_errors++;
            return;
        }
        parse_target_payload(payload);
    }
}

static void parse_byte(uint8_t byte)
{
    switch (ParserState)
    {
        case PARSER_WAIT_HEAD1:
            ParserState = (byte == MAIXCAM2_FRAME_HEAD1) ? PARSER_WAIT_HEAD2 : PARSER_WAIT_HEAD1;
            break;

        case PARSER_WAIT_HEAD2:
            ParserState = (byte == MAIXCAM2_FRAME_HEAD2) ? PARSER_READ_VER : PARSER_WAIT_HEAD1;
            break;

        case PARSER_READ_VER:
            ParserVer = byte;
            ParserState = PARSER_READ_MSG_ID;
            break;

        case PARSER_READ_MSG_ID:
            ParserMsgId = byte;
            ParserState = PARSER_READ_SEQ;
            break;

        case PARSER_READ_SEQ:
            ParserSeq = byte;
            ParserState = PARSER_READ_LEN;
            break;

        case PARSER_READ_LEN:
            ParserLen = byte;
            ParserIndex = 0u;
            if (ParserLen > MAIXCAM2_MAX_PAYLOAD)
            {
                Stats.length_errors++;
                ParserState = PARSER_WAIT_HEAD1;
            }
            else
            {
                ParserState = (ParserLen == 0u) ? PARSER_READ_CRC_L : PARSER_READ_PAYLOAD;
            }
            break;

        case PARSER_READ_PAYLOAD:
            ParserPayload[ParserIndex++] = byte;
            if (ParserIndex >= ParserLen)
            {
                ParserState = PARSER_READ_CRC_L;
            }
            break;

        case PARSER_READ_CRC_L:
            ParserCrcL = byte;
            ParserState = PARSER_READ_CRC_H;
            break;

        case PARSER_READ_CRC_H:
            handle_frame(ParserVer, ParserMsgId, ParserSeq, ParserLen, ParserPayload, (uint16_t)ParserCrcL | ((uint16_t)byte << 8));
            ParserState = PARSER_WAIT_HEAD1;
            break;

        default:
            ParserState = PARSER_WAIT_HEAD1;
            break;
    }
}

void maixcam2_init(void)
{
    uint8 byte;

    serial_rx_buffer_init(&MaixRxBuffer, MaixRxStorage, sizeof(MaixRxStorage));
    ParserState = PARSER_WAIT_HEAD1;
    HasLatestTarget = false;
    Stats = (MaixProtocolStats){0};

    uart_init(BOARD_MAIXCAM_UART, BOARD_MAIXCAM_BAUDRATE, BOARD_MAIXCAM_UART_TX, BOARD_MAIXCAM_UART_RX);
    while (uart_query_byte(BOARD_MAIXCAM_UART, &byte) == ZF_TRUE)
    {
    }
    uart_set_callback(BOARD_MAIXCAM_UART, maix_uart_rx_callback, 0);
    uart_set_interrupt_config(BOARD_MAIXCAM_UART, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

void maixcam2_update(uint32_t now_ms)
{
    uint8_t byte;

    ParserNowMs = now_ms;

    while (serial_rx_buffer_pop(&MaixRxBuffer, &byte))
    {
        parse_byte(byte);
    }
}

bool maixcam2_get_latest_target(MaixVisionTarget *target, uint32_t *rx_time_ms)
{
    if (!HasLatestTarget || target == 0)
    {
        return false;
    }

    *target = LatestTarget;
    if (rx_time_ms != 0)
    {
        *rx_time_ms = LatestTargetRxTimeMs;
    }
    return true;
}

const MaixProtocolStats *maixcam2_get_stats(void)
{
    return &Stats;
}
