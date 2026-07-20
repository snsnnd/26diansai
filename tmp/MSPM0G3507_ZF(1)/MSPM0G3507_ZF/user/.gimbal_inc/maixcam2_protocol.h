#ifndef MAIXCAM2_PROTOCOL_H
#define MAIXCAM2_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "pin_mapping.h"

#define MAIXCAM2_FRAME_HEAD1 0xAAu
#define MAIXCAM2_FRAME_HEAD2 0x55u
#define MAIXCAM2_PROTOCOL_VER 0x01u
#define MAIXCAM2_MAX_PAYLOAD 64u
#define MAIXCAM2_TARGET_PAYLOAD_LEN 36u

typedef enum
{
    MAIX_MSG_HEARTBEAT = 0x00,
    MAIX_MSG_VISION_TARGET = 0x01,
    MAIX_MSG_VISION_IMU = 0x03,
    MAIX_MSG_MSP_STATUS = 0x04,
    MAIX_MSG_CONTROL_CMD = 0x10,
    MAIX_MSG_PARAM_SET = 0x11,
    MAIX_MSG_PARAM_ACK = 0x12,
} MaixMessageId;

typedef enum
{
    VISION_STATE_IDLE = 0,
    VISION_STATE_SEARCHING = 1,
    VISION_STATE_CANDIDATE = 2,
    VISION_STATE_LOCKED = 3,
    VISION_STATE_TRACKING = 4,
    VISION_STATE_LOST = 5,
} VisionState;

typedef enum
{
    VISION_SOURCE_NONE = 0,
    VISION_SOURCE_YOLO = 1,
    VISION_SOURCE_CV = 2,
    VISION_SOURCE_YOLO_CV = 3,
} VisionSource;

#define VISION_FLAG_YOLO_VALID     (1u << 0)
#define VISION_FLAG_CV_VALID       (1u << 1)
#define VISION_FLAG_TARGET_LOCKED  (1u << 2)
#define VISION_FLAG_TRACK_STABLE   (1u << 3)
#define VISION_FLAG_NEED_SEARCH    (1u << 4)
#define VISION_FLAG_NEAR_CENTER    (1u << 5)

typedef struct
{
    uint32_t timestamp_ms;
    uint8_t vision_state;
    uint8_t source;
    uint8_t target_valid;
    uint8_t track_id;
    uint8_t class_id;
    uint8_t confidence;
    uint8_t quality;
    uint8_t flags;
    int16_t error_x;
    int16_t error_y;
    int16_t target_x;
    int16_t target_y;
    uint16_t target_w;
    uint16_t target_h;
    int16_t yolo_x;
    int16_t yolo_y;
    uint16_t yolo_w;
    uint16_t yolo_h;
    int16_t vx;
    int16_t vy;
} MaixVisionTarget;

typedef struct
{
    uint32_t frames_received;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t version_errors;
    uint32_t sequence_lost;
    uint8_t last_seq;
    bool has_last_seq;
} MaixProtocolStats;

void maixcam2_init(void);
void maixcam2_update(uint32_t now_ms);
bool maixcam2_get_latest_target(MaixVisionTarget *target, uint32_t *rx_time_ms);
const MaixProtocolStats *maixcam2_get_stats(void);
uint16_t maixcam2_crc16_modbus(const uint8_t *data, uint16_t length);

#endif
