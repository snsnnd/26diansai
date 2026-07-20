/**
 * @file    maixcam2_protocol.h
 * @brief   MaixCam2 视觉目标检测协议 —— 帧解析与目标数据接口
 *
 * 本文件定义了与 MaixCam2 摄像头通信的协议格式、数据结构及 API。
 * MaixCam2 通过 UART 发送视觉目标检测结果，帧格式如下：
 *   HEAD1(0xAA) HEAD2(0x55) VER MSG_ID SEQ LEN PAYLOAD CRC16_L CRC16_H
 * - CRC16 采用 Modbus 算法，覆盖 VER + MSG_ID + SEQ + LEN + PAYLOAD
 * - 解析器为状态机驱动，支持帧间超时检测
 * - 目标数据包含像素偏差、目标有效标志和视觉状态（IDLE/SEARCHING/CANDIDATE/etc.）
 */
#ifndef MAIXCAM2_PROTOCOL_H
#define MAIXCAM2_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "pin_mapping.h"

/* ======================== 协议常量定义 ======================== */

#define MAIXCAM2_FRAME_HEAD1      0xAAu    /**< 帧起始字节 1 */
#define MAIXCAM2_FRAME_HEAD2      0x55u    /**< 帧起始字节 2 */
#define MAIXCAM2_PROTOCOL_VER     0x01u    /**< 协议版本号 */
#define MAIXCAM2_MAX_PAYLOAD      64u      /**< 最大有效载荷长度（字节） */
#define MAIXCAM2_TARGET_PAYLOAD_LEN 36u    /**< 视觉目标数据的固定载荷长度 */
#define MAIXCAM2_INTERBYTE_TIMEOUT_MS 20u  /**< 字节间超时阈值（毫秒），超时则重置解析器 */

/**
 * @def MAIXCAM2_MAX_ABS_ERROR_PX
 * @brief 像素偏差的最大绝对值阈值，用于语义验证
 *
 * 取值范围 [1, 32767]。若偏差绝对值超过此值，
 * 则判定为语义无效。默认值 500 像素。
 */
#ifndef MAIXCAM2_MAX_ABS_ERROR_PX
#define MAIXCAM2_MAX_ABS_ERROR_PX 500
#endif

#if (MAIXCAM2_MAX_ABS_ERROR_PX <= 0) || (MAIXCAM2_MAX_ABS_ERROR_PX > 32767)
#error "MAIXCAM2_MAX_ABS_ERROR_PX must be in [1, 32767]"
#endif

/* ======================== 枚举与数据结构 ======================== */

/**
 * @enum   VisionState
 * @brief  视觉目标跟踪状态枚举
 *
 * 状态机流转逻辑：
 *   IDLE(空闲) -> SEARCHING(搜索中) -> CANDIDATE(候选) -> LOCKED(锁定) -> TRACKING(追踪)
 *   任何状态均可进入 LOST(丢失) 或回到 IDLE。
 */
typedef enum
{
    VISION_STATE_IDLE      = 0,  /**< 空闲，未检测到任何目标 */
    VISION_STATE_SEARCHING = 1,  /**< 正在搜索目标 */
    VISION_STATE_CANDIDATE = 2,  /**< 检测到候选目标（尚未锁定） */
    VISION_STATE_LOCKED    = 3,  /**< 目标已锁定 */
    VISION_STATE_TRACKING  = 4,  /**< 正在跟踪目标 */
    VISION_STATE_LOST      = 5,  /**< 目标丢失 */
} VisionState;

/**
 * @struct MaixVisionTarget
 * @brief  视觉目标数据结构体
 *
 * 保存从 MaixCam2 解析出的单帧目标检测结果。
 * 包含目标是否有效、图像中心到目标的像素偏差以及视觉状态。
 */
typedef struct
{
    uint8_t  target_valid;   /**< 目标有效标志（0=无效，1=有效） */
    int16_t  error_x;        /**< 图像中心到目标的 X 方向像素偏差（小端有符号16位） */
    int16_t  error_y;        /**< 图像中心到目标的 Y 方向像素偏差（小端有符号16位） */
    uint8_t  vision_state;   /**< 视觉跟踪状态，取值见 @ref VisionState */
} MaixVisionTarget;

/**
 * @struct MaixProtocolStats
 * @brief  协议统计信息
 *
 * 用于调试和性能监控，记录各类型错误和接收帧数。
 */
typedef struct
{
    uint32_t frames_received;      /**< 成功接收且 CRC 校验通过的帧数 */
    uint32_t crc_errors;           /**< CRC 校验失败次数 */
    uint32_t ring_overflows;       /**< 环形缓冲区溢出次数 */
    uint32_t malformed_frames;     /**< 格式错误帧数（协议版本不匹配、载荷超长等） */
    uint32_t semantic_errors;      /**< 语义验证失败次数（状态与标志矛盾、偏差超限等） */
    uint32_t interbyte_timeouts;   /**< 字节间超时次数 */
} MaixProtocolStats;

/* ======================== 函数声明 ======================== */

/**
 * @brief   初始化 MaixCam2 协议模块
 *
 * 完成以下初始化：
 * - 初始化带时间戳的串行接收环形缓冲区
 * - 复位解析器状态机到初始状态
 * - 清零统计数据
 * - 初始化 UART 外设（波特率、TX/RX 引脚）
 * - 清空 UART 硬件 RX 缓冲区
 * - 注册 UART RX 中断回调并使能中断
 */
void maixcam2_init(void);

/**
 * @brief   协议更新函数（需要在主循环中周期性调用）
 *
 * 完成以下工作：
 * 1. 检测并统计环形缓冲区溢出事件（溢出时清空缓冲区并重置解析器）
 * 2. 从环形缓冲区弹出所有可用字节，逐个送入状态机解析
 * 3. 检测帧间字节超时（半帧超时则丢弃当前帧）
 *
 * @param   now_ms  当前系统时间戳（毫秒），用于超时判断
 */
void maixcam2_update(uint32_t now_ms);

/**
 * @brief   获取最新的有效目标数据
 *
 * 如果当前有可用的目标数据且未发生缓冲区溢出，则将数据拷贝到输出参数。
 * 获取后内部标志不清除，下次调用仍可获取同一帧数据，直到被新帧覆盖。
 *
 * @param[out]  target      输出参数，接收最新的 MaixVisionTarget 数据
 * @param[out]  rx_time_ms  输出参数，接收该帧接收时的系统时间戳（毫秒），
 *                           可传 NULL 表示不需要
 * @retval  true   成功获取到最新目标数据
 * @retval  false  无法获取（无数据或发生溢出）
 */
bool maixcam2_get_latest_target(MaixVisionTarget *target, uint32_t *rx_time_ms);

/**
 * @brief   检查视觉目标数据的语义合法性
 *
 * 验证规则：
 * - target_valid 必须为 0 或 1
 * - vision_state 必须在 VISION_STATE_IDLE..VISION_STATE_LOST 范围内
 * - error_x 和 error_y 的绝对值不得超过 @ref MAIXCAM2_MAX_ABS_ERROR_PX
 * - target_valid == 1 时，vision_state 必须为 CANDIDATE / LOCKED / TRACKING 之一
 * - target_valid == 0 时，vision_state 不能为 CANDIDATE / LOCKED / TRACKING
 *
 * @param   target  待验证的目标数据指针
 * @retval  true    数据语义合法
 * @retval  false   数据语义非法
 */
bool maixcam2_target_semantically_valid(const MaixVisionTarget *target);

/**
 * @brief   获取协议统计信息的只读指针
 *
 * @return  指向内部 MaixProtocolStats 结构体的指针
 */
const MaixProtocolStats *maixcam2_get_stats(void);

/**
 * @brief   计算 CRC16-Modbus 校验值
 *
 * 算法：多项式 0x8005（反映后 0xA001），初始值 0xFFFF，
 * 结果不取反（直接输出）。
 *
 * @param   data    待计算的数据缓冲区指针
 * @param   length  数据长度（字节数）
 * @return  16 位 CRC 校验值
 */
uint16_t maixcam2_crc16_modbus(const uint8_t *data, uint16_t length);

#endif
