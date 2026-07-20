/**
 * @file    maixcam2_protocol.c
 * @brief   MaixCam2 视觉目标协议解析器实现
 *
 * 本文件实现了与 MaixCam2 摄像头通信的完整协议栈：
 *
 * === 帧格式 ===
 *   [0] HEAD1    (0xAA) - 帧起始标志1
 *   [1] HEAD2    (0x55) - 帧起始标志2
 *   [2] VER      (0x01) - 协议版本号
 *   [3] MSG_ID          - 消息ID（0x01=目标数据帧）
 *   [4] SEQ             - 序列号（目前未使用）
 *   [5] LEN             - 有效载荷长度（字节）
 *   [6..6+LEN-1] PAYLOAD - 有效载荷数据
 *   [6+LEN]   CRC_L     - CRC16 低字节
 *   [6+LEN+1] CRC_H     - CRC16 高字节
 *
 * === 解析器架构 ===
 * 使用有限状态机（FSM）逐字节解析，状态迁移：
 *   PARSER_WAIT_HEAD1 -> PARSER_WAIT_HEAD2 -> PARSER_READ_VER
 *   -> PARSER_READ_MSG_ID -> PARSER_READ_SEQ -> PARSER_READ_LEN
 *   -> PARSER_READ_PAYLOAD -> PARSER_READ_CRC_L -> PARSER_READ_CRC_H
 *   -> handle_frame() -> reset_parser()
 *
 * 任何中间状态发生字节间超时（>20ms）则回到 PARSER_WAIT_HEAD1。
 *
 * === 关键设计决策 ===
 * - CRC16-Modbus 覆盖 VER+MSG_ID+SEQ+LEN+PAYLOAD（不含帧头和CRC本身）
 * - 带时间戳的环形缓冲区：每个接收字节记录到达时间，用于帧间超时检测
 * - 语义验证：解析后的目标数据还需通过状态一致性检查
 * - 溢出处理：环形缓冲区溢出时清空缓冲区并重置解析器，防止错帧
 */
#include "gimbal/maixcam2_protocol.h"

#include "lib/serial_rx_buffer.h"
#include "framework/ec_time.h"
#include "zf_common_headfile.h"

/**
 * @enum   MaixParserState
 * @brief  帧解析器状态机枚举
 *
 * 每个状态对应帧中的一个字段的等待/读取阶段。
 * 从 WAIT_HEAD1 开始，成功解析一帧后回到 WAIT_HEAD1。
 */
typedef enum
{
    PARSER_WAIT_HEAD1 = 0,   /**< 等待帧头1 (0xAA) */
    PARSER_WAIT_HEAD2,        /**< 等待帧头2 (0x55) */
    PARSER_READ_VER,          /**< 读取协议版本号 */
    PARSER_READ_MSG_ID,       /**< 读取消息ID */
    PARSER_READ_SEQ,          /**< 读取序列号 */
    PARSER_READ_LEN,          /**< 读取有效载荷长度 */
    PARSER_READ_PAYLOAD,      /**< 读取有效载荷字节（多字节） */
    PARSER_READ_CRC_L,        /**< 读取CRC低字节 */
    PARSER_READ_CRC_H,        /**< 读取CRC高字节（帧结束，触发校验） */
} MaixParserState;

/* ======================== 静态变量 ======================== */

// ---- 环形缓冲区 ----
static uint8_t          MaixRxStorage[256];      /**< 接收数据缓冲区（256字节） */
static uint32_t         MaixRxTimestamps[256];    /**< 每个字节对应的接收时间戳（毫秒） */
static SerialRxBuffer   MaixRxBuffer;             /**< 带时间戳的串行接收环形缓冲区实例 */

// ---- 解析器状态 ----
static MaixParserState  ParserState = PARSER_WAIT_HEAD1;  /**< 解析器当前状态 */
static uint8_t          ParserVer;                         /**< 当前帧的版本号字段 */
static uint8_t          ParserMsgId;                       /**< 当前帧的消息ID */
static uint8_t          ParserSeq;                         /**< 当前帧的序列号 */
static uint8_t          ParserLen;                         /**< 当前帧的有效载荷长度 */
static uint8_t          ParserPayload[MAIXCAM2_MAX_PAYLOAD]; /**< 当前帧的有效载荷缓冲区 */
static uint8_t          ParserIndex;      /**< 有效载荷读取索引（已读取字节数） */
static uint8_t          ParserCrcL;       /**< 当前帧的CRC低字节 */

// ---- 目标数据 ----
static MaixVisionTarget LatestTarget;       /**< 最新有效目标数据 */
static uint32_t         LatestTargetRxTimeMs; /**< 最新目标数据的接收时间戳（毫秒） */
static bool             HasLatestTarget;     /**< 是否有可用的最新目标数据 */

// ---- 统计 ----
static MaixProtocolStats Stats;                  /**< 协议运行统计信息 */

// ---- 超时与溢出 ----
static uint32_t         ParserLastByteMs;           /**< 上一个字节的接收时间戳（毫秒） */
static size_t           ObservedOverflowCount;      /**< 上次观测到的溢出计数（用于增量统计） */
static volatile bool    RxOverflowPending;          /**< 待处理的溢出标志（在中断中设置，主循环中处理） */

/**
 * @brief   从小端字节序读取有符号 16 位整数
 *
 * MaixCam2 发送的多字节数值为小端格式（Little-Endian），
 * 即低地址存放低字节。此函数将两个字节拼装为 int16_t。
 *
 * @param   data    小端字节序的 2 字节缓冲区指针
 * @return  拼接后的有符号 16 位整数
 */
static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

/**
 * @brief   重置帧解析器状态机到初始状态
 *
 * 在以下场景调用：
 * - 帧解析完成（一帧完整处理完毕）
 * - 发生字节间超时
 * - 接收到无效数据（如载荷超长）
 * - 环形缓冲区溢出后
 *
 * @param   invalidate_target  是否同时使当前目标数据失效
 *          - true:  帧解析中途出错，丢弃已有目标数据
 *          - false: 帧已完整解析完毕，保留目标数据供后续读取
 */
static void reset_parser(bool invalidate_target)
{
    ParserState = PARSER_WAIT_HEAD1;
    ParserIndex = 0u;
    ParserLen = 0u;
    if (invalidate_target)
    {
        HasLatestTarget = false;
    }
}

/**
 * @brief   验证视觉目标数据的语义一致性
 *
 * 语义验证规则（按优先级顺序）：
 *
 * 1. 基本范围检查：
 *    - target_valid 必须为 0 或 1
 *    - vision_state 必须在 0..5（VISION_STATE_IDLE..LOST）范围内
 *
 * 2. 像素偏差范围检查：
 *    - error_x 和 error_y 的绝对值不得超过 MAIXCAM2_MAX_ABS_ERROR_PX
 *    - 使用 int32_t 进行计算以防止 int16_t 运算溢出
 *
 * 3. 状态一致性检查（核心逻辑）：
 *    - target_valid != 0（目标有效）：vision_state 必须为
 *      CANDIDATE(2)、LOCKED(3) 或 TRACKING(4) 之一。
 *      因为只有在检测到、锁定或跟踪目标时，才应标记目标有效。
 *    - target_valid == 0（目标无效）：vision_state 不能为
 *      CANDIDATE(2)、LOCKED(3) 或 TRACKING(4)。
 *      如果目标无效但状态却声称在跟踪，则数据矛盾。
 *
 * 这些规则过滤掉摄像头在状态切换瞬间可能发送的不一致帧。
 *
 * @param   target  待验证的目标数据指针
 * @retval  true    数据语义合法，可用于控制计算
 * @retval  false   数据语义非法，应丢弃
 */
bool maixcam2_target_semantically_valid(const MaixVisionTarget *target)
{
    int32_t error_x;
    int32_t error_y;

    if (target == NULL || target->target_valid > 1u ||
        target->vision_state > VISION_STATE_LOST)
    {
        return false;
    }

    error_x = target->error_x;
    error_y = target->error_y;
    if (error_x < -MAIXCAM2_MAX_ABS_ERROR_PX ||
        error_x > MAIXCAM2_MAX_ABS_ERROR_PX ||
        error_y < -MAIXCAM2_MAX_ABS_ERROR_PX ||
        error_y > MAIXCAM2_MAX_ABS_ERROR_PX)
    {
        return false;
    }

    if (target->target_valid != 0u &&
        target->vision_state != VISION_STATE_CANDIDATE &&
        target->vision_state != VISION_STATE_LOCKED &&
        target->vision_state != VISION_STATE_TRACKING)
    {
        return false;
    }
    if (target->target_valid == 0u &&
        (target->vision_state == VISION_STATE_CANDIDATE ||
         target->vision_state == VISION_STATE_LOCKED ||
         target->vision_state == VISION_STATE_TRACKING))
    {
        return false;
    }
    return true;
}

/**
 * @brief   计算 CRC16-Modbus 校验值
 *
 * CRC16-Modbus 算法参数：
 * - 多项式: 0x8005（标准 Modbus 多项式）
 * - 反映后多项式: 0xA001（用于逐位右移实现）
 * - 初始值: 0xFFFF
 * - 结果异或值: 0x0000（不取反）
 * - 输入数据不反映，结果不反映
 *
 * 实现方式：
 * 外层循环遍历每个字节，内层循环逐位处理。
 * 使用"反映后"算法（多项式 0xA001，右移），
 * 这是嵌入式平台上常用的表驱动或逐位移位实现。
 *
 * 覆盖范围：VER + MSG_ID + SEQ + LEN + PAYLOAD
 * （不包含 HEAD1、HEAD2 和 CRC 自身字段）
 *
 * @param   data    数据缓冲区指针
 * @param   length  数据长度（字节数）
 * @return  16 位 CRC 校验值
 */
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

/**
 * @brief   MaixCam2 UART 接收中断回调函数
 *
 * 在 UART RX 中断触发时被逐飞库调用。
 * 使用 uart_query_byte() 循环读取硬件 UART 接收寄存器中所有可用字节，
 * 并通过 serial_rx_buffer_push_timed() 将字节及其到达时间戳一并存入环形缓冲区。
 *
 * 时间戳的作用：
 * 主循环中的超时检测依赖每个字节的到达时间，而非当前系统时间。
 * 这样可以准确定位"某一帧内最后收到字节的时刻"，而非"当前时刻"。
 *
 * 溢出处理：
 * 当环形缓冲区满时，push_timed 返回 false，设置 RxOverflowPending 标志。
 * 主循环中的 maixcam2_update() 会处理该标志并清空缓冲区。
 *
 * @param   state  中断状态位掩码
 * @param   ptr    用户参数指针（本函数未使用）
 */
static void maix_uart_rx_callback(uint32_t state, void *ptr)
{
    uint8_t byte;

    (void)ptr;

    if ((state & UART_INTERRUPT_STATE_RX) == 0u)
    {
        return;
    }

    while (uart_query_byte(BOARD_MAIXCAM_UART, &byte) == ZF_TRUE)
    {
        if (!serial_rx_buffer_push_timed(&MaixRxBuffer, byte, ec_time_ms()))
        {
            RxOverflowPending = true;
        }
    }
}

/**
 * @brief   解析 MaixCam2 目标数据载荷
 *
 * 从 36 字节的 payload 中提取视觉目标信息。
 * MaixCam2 目标数据帧的 payload 布局（偏移量）：
 *   [0-3]   保留/其他字段
 *   [4]     vision_state    - 视觉跟踪状态
 *   [5]     保留
 *   [6]     target_valid    - 目标有效标志
 *   [7-11]  保留
 *   [12-13] error_x         - X 方向像素偏差（小端 int16）
 *   [14-15] error_y         - Y 方向像素偏差（小端 int16）
 *
 * 提取后立即调用 maixcam2_target_semantically_valid() 进行语义验证。
 * 验证通过则更新 LatestTarget，否则丢弃并递增语义错误计数。
 *
 * @param   payload     有效载荷数据指针（36 字节）
 * @param   rx_time_ms  帧的接收时间戳（毫秒）
 * @retval  true  目标数据解析成功且语义合法
 * @retval  false 解析失败（语义非法）
 */
static bool parse_target_payload(const uint8_t *payload, uint32_t rx_time_ms)
{
    MaixVisionTarget target;

    target.vision_state = payload[4];
    target.target_valid = payload[6];
    target.error_x      = read_i16_le(&payload[12]);
    target.error_y      = read_i16_le(&payload[14]);
    if (!maixcam2_target_semantically_valid(&target))
    {
        Stats.semantic_errors++;
        HasLatestTarget = false;
        return false;
    }

    LatestTarget = target;
    LatestTargetRxTimeMs = rx_time_ms;
    HasLatestTarget           = true;
    return true;
}

/**
 * @brief   处理已组装完成的一帧数据
 *
 * 在解析器完成 CRC_H 的读取后调用，执行以下步骤：
 *
 * 1. 协议版本检查：验证 VER 字段与 MAIXCAM2_PROTOCOL_VER 一致。
 *    版本不匹配说明发送方协议升级了，应拒绝该帧。
 *
 * 2. CRC 校验：计算 VER + MSG_ID + SEQ + LEN + PAYLOAD 的 CRC16-Modbus，
 *    与帧中的 CRC 字段比较。不一致则递增 crc_errors 并丢弃。
 *
 * 3. 消息路由：根据 MSG_ID 分发：
 *    - msg_id == 0x01: 目标数据帧，检查载荷长度是否为
 *      MAIXCAM2_TARGET_PAYLOAD_LEN（36字节），然后调用 parse_target_payload 解析。
 *    - 其他 msg_id: 暂时忽略（保留扩展）
 *
 * CRC 数据的组织方式：
 *   crc_data[0..3] = VER, MSG_ID, SEQ, LEN
 *   crc_data[4..4+len-1] = PAYLOAD
 * 这样构造的缓冲区可以直接计算连续数据的 CRC。
 *
 * @param   ver         协议版本号
 * @param   msg_id      消息 ID
 * @param   seq         序列号（当前未使用）
 * @param   len         有效载荷长度
 * @param   payload     有效载荷数据指针
 * @param   rx_crc      帧中携带的 CRC16 值
 * @param   rx_time_ms  帧的接收时间戳
 */
static void handle_frame(uint8_t ver, uint8_t msg_id, uint8_t seq,
                          uint8_t len, const uint8_t *payload, uint16_t rx_crc,
                          uint32_t rx_time_ms)
{
    uint8_t  crc_data[4u + MAIXCAM2_MAX_PAYLOAD];
    uint16_t crc_len;
    uint16_t calc_crc;

    (void)seq;

    if (ver != MAIXCAM2_PROTOCOL_VER)
    {
        Stats.malformed_frames++;
        HasLatestTarget = false;
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
    crc_len   = (uint16_t)(4u + len);
    calc_crc  = maixcam2_crc16_modbus(crc_data, crc_len);

    if (calc_crc != rx_crc)
    {
        Stats.crc_errors++;
        HasLatestTarget = false;
        return;
    }

    Stats.frames_received++;

    if (msg_id == 0x01u)
    {
        if (len != MAIXCAM2_TARGET_PAYLOAD_LEN)
        {
            Stats.malformed_frames++;
            HasLatestTarget = false;
            return;
        }
        (void)parse_target_payload(payload, rx_time_ms);
    }
}

/**
 * @brief   帧解析器状态机：逐字节处理接收数据
 *
 * 这是协议解析的核心函数，实现了一个 9 状态的有限状态机。
 * 每个接收的字节都经过此函数处理，状态根据当前状态和字节值迁移。
 *
 * === 状态迁移详解 ===
 *
 * 1. PARSER_WAIT_HEAD1:
 *    等待帧头 0xAA。如果不是 0xAA，停留在当前状态（丢弃该字节）。
 *
 * 2. PARSER_WAIT_HEAD2:
 *    等待帧头 0x55。如果收到 0x55，进入 READ_VER。
 *    如果收到另一个 0xAA，回到 WAIT_HEAD2 继续等待（处理双 0xAA 情况）。
 *    否则回到 WAIT_HEAD1（重新同步）。
 *
 * 3. PARSER_READ_VER: 保存版本号，进入 READ_MSG_ID。
 *
 * 4. PARSER_READ_MSG_ID: 保存消息 ID，进入 READ_SEQ。
 *
 * 5. PARSER_READ_SEQ: 保存序列号，进入 READ_LEN。
 *
 * 6. PARSER_READ_LEN:
 *    保存载荷长度。如果长度超过 MAIXCAM2_MAX_PAYLOAD（64 字节），
 *    标记格式错误并重置解析器。否则根据长度是否为零决定进入
 *    READ_PAYLOAD（有载荷）或 READ_CRC_L（无载荷）。
 *
 * 7. PARSER_READ_PAYLOAD:
 *    循环接收载荷字节，每收到一个字节递增 ParserIndex。
 *    当 ParserIndex >= ParserLen 时，载荷接收完毕，进入 READ_CRC_L。
 *
 * 8. PARSER_READ_CRC_L: 保存 CRC 低字节，进入 READ_CRC_H。
 *
 * 9. PARSER_READ_CRC_H:
 *    收到 CRC 高字节后，组合完整的 16 位 CRC 值，
 *    调用 handle_frame() 进行协议验证和数据处理，
 *    然后重置解析器（保留目标数据，invalidate_target=false）。
 *
 * === 超时检测 ===
 * 在进入状态机前检测字节间超时。如果当前不在 WAIT_HEAD1 状态
 * 且距离上一个字节的时间超过 MAIXCAM2_INTERBYTE_TIMEOUT_MS，
 * 则认为接收了半帧后中断，递增 interbyte_timeouts 并重置解析器。
 *
 * 设计说明：
 * - WAIT_HEAD2 收到 0xAA 时回到 WAIT_HEAD2 而非 WAIT_HEAD1，
 *   这是为了正确处理连续的 0xAA 字节：0xAA 0xAA 0x55 中，
 *   第一个 0xAA 进入 WAIT_HEAD2，第二个 0xAA 保持在 WAIT_HEAD2，
 *   第三个 0x55 才正确进入 READ_VER。
 *
 * @param   byte        接收到的字节
 * @param   rx_time_ms  该字节的接收时间戳（毫秒）
 */
static void parse_byte(uint8_t byte, uint32_t rx_time_ms)
{
    if (ParserState != PARSER_WAIT_HEAD1 &&
        (uint32_t)(rx_time_ms - ParserLastByteMs) > MAIXCAM2_INTERBYTE_TIMEOUT_MS)
    {
        Stats.interbyte_timeouts++;
        reset_parser(true);
    }
    ParserLastByteMs = rx_time_ms;

    switch (ParserState)
    {
        case PARSER_WAIT_HEAD1:
            ParserState = (byte == MAIXCAM2_FRAME_HEAD1) ? PARSER_WAIT_HEAD2 : PARSER_WAIT_HEAD1;
            break;

        case PARSER_WAIT_HEAD2:
            if (byte == MAIXCAM2_FRAME_HEAD2)
                ParserState = PARSER_READ_VER;
            else
                ParserState = (byte == MAIXCAM2_FRAME_HEAD1)
                    ? PARSER_WAIT_HEAD2 : PARSER_WAIT_HEAD1;
            break;

        case PARSER_READ_VER:
            ParserVer   = byte;
            ParserState = PARSER_READ_MSG_ID;
            break;

        case PARSER_READ_MSG_ID:
            ParserMsgId = byte;
            ParserState = PARSER_READ_SEQ;
            break;

        case PARSER_READ_SEQ:
            ParserSeq   = byte;
            ParserState = PARSER_READ_LEN;
            break;

        case PARSER_READ_LEN:
            ParserLen   = byte;
            ParserIndex = 0u;
            if (ParserLen > MAIXCAM2_MAX_PAYLOAD)
            {
                Stats.malformed_frames++;
                reset_parser(true);
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
            ParserCrcL  = byte;
            ParserState = PARSER_READ_CRC_H;
            break;

        case PARSER_READ_CRC_H:
            handle_frame(ParserVer, ParserMsgId, ParserSeq, ParserLen,
                          ParserPayload,
                          (uint16_t)ParserCrcL | ((uint16_t)byte << 8),
                          rx_time_ms);
            reset_parser(false);
            break;

        default:
            Stats.malformed_frames++;
            reset_parser(true);
            break;
    }
}

/**
 * @brief   初始化 MaixCam2 协议模块
 *
 * 初始化流程：
 * 1. 初始化带时间戳的串行接收环形缓冲区（256 字节容量，
 *    每个字节附带 32 位毫秒时间戳）。
 * 2. 复位解析器状态机到 PARSER_WAIT_HEAD1。
 * 3. 清空最新目标标志和统计数据。
 * 4. 清空超时和溢出相关变量。
 * 5. 配置 UART 外设（波特率、TX/RX 引脚由 pin_mapping 宏定义）。
 * 6. 排空硬件 UART RX 寄存器中可能的残留数据（上电干扰字节）。
 * 7. 注册中断回调 maix_uart_rx_callback 并使能 RX 中断。
 *
 * @note  本函数应在系统启动时调用一次。
 *        调用前需确保 ec_time 模块已初始化（提供毫秒时间戳）。
 */
void maixcam2_init(void)
{
    uint8_t byte;

    serial_rx_buffer_init_timed(&MaixRxBuffer, MaixRxStorage, MaixRxTimestamps,
                                sizeof(MaixRxStorage));
    ParserState     = PARSER_WAIT_HEAD1;
    HasLatestTarget = false;
    Stats           = (MaixProtocolStats){0};
    ParserLastByteMs = 0u;
    ObservedOverflowCount = 0u;
    RxOverflowPending = false;

    uart_init(BOARD_MAIXCAM_UART, BOARD_MAIXCAM_BAUDRATE,
              BOARD_MAIXCAM_UART_TX, BOARD_MAIXCAM_UART_RX);
    while (uart_query_byte(BOARD_MAIXCAM_UART, &byte) == ZF_TRUE)
    {
    }
    uart_set_callback(BOARD_MAIXCAM_UART, maix_uart_rx_callback, 0);
    uart_set_interrupt_config(BOARD_MAIXCAM_UART, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

/**
 * @brief   协议更新函数（主循环中周期性调用）
 *
 * 每个主循环周期调用一次，完成三个任务：
 *
 * === 1. 溢出检测与恢复 ===
 * 检查 serial_rx_buffer_overflow_count() 是否增加或 RxOverflowPending 是否置位。
 * 如果是，说明中断中发生了环形缓冲区溢出（数据接收过快或主循环处理不及时）。
 * 处理措施：统计溢出次数、清空环形缓冲区、重置解析器状态机。
 * 返回后不继续处理数据，因为缓冲区已清空。
 *
 * 使用增量统计方式（ObservedOverflowCount）确保每次溢出只计数一次，
 * 即使在多个主循环周期中连续检测到溢出。
 *
 * === 2. 数据解析 ===
 * 从环形缓冲区中弹出所有可用字节（及其时间戳），逐个送入 parse_byte()
 * 状态机解析。循环直到缓冲区为空。
 *
 * 使用带时间戳的 pop 变体（pop_timed），
 * 使超时检测可以使用字节的实际到达时间而非当前时间。
 *
 * === 3. 超时检测（补充） ===
 * 如果当前不在 WAIT_HEAD1 状态（正在接收一帧），
 * 且从上一个字节到现在已超过 MAIXCAM2_INTERBYTE_TIMEOUT_MS，
 * 则认为帧中断，递增超时计数并重置解析器。
 *
 * 注意：parse_byte() 内部也有超时检测，使用字节自己的时间戳（rx_time_ms）。
 * 这里的超时检测是针对"主循环轮询间隔较大、字节已到达但未来得及处理"
 * 的补充保护，使用当前时间 now_ms 判断。
 *
 * @param   now_ms  当前系统时间戳（毫秒），取自 ec_time_ms()
 */
void maixcam2_update(uint32_t now_ms)
{
    uint8_t byte;
    uint32_t rx_time_ms;
    size_t overflow_count = serial_rx_buffer_overflow_count(&MaixRxBuffer);

    if (RxOverflowPending || overflow_count != ObservedOverflowCount)
    {
        Stats.ring_overflows += (uint32_t)(overflow_count - ObservedOverflowCount);
        ObservedOverflowCount = overflow_count;
        RxOverflowPending = false;
        serial_rx_buffer_clear(&MaixRxBuffer);
        reset_parser(true);
        return;
    }

    while (serial_rx_buffer_pop_timed(&MaixRxBuffer, &byte, &rx_time_ms))
    {
        parse_byte(byte, rx_time_ms);
    }

    if (ParserState != PARSER_WAIT_HEAD1 &&
        (uint32_t)(now_ms - ParserLastByteMs) > MAIXCAM2_INTERBYTE_TIMEOUT_MS)
    {
        Stats.interbyte_timeouts++;
        reset_parser(true);
    }
}

/**
 * @brief   获取最新解析到的有效目标数据
 *
 * 如果 HasLatestTarget 为 true 且未发生溢出，
 * 将 LatestTarget 和 LatestTargetRxTimeMs 拷贝到输出参数中。
 *
 * 注意：本函数不消耗数据（多次调用返回相同结果）。
 * 直到新一帧目标数据到达并解析成功，LatestTarget 才会被覆盖。
 * 这种设计使得多个使用者（如控制算法、日志、无线回传）都能读到同一帧数据。
 *
 * @param[out]  target      输出参数，接收最新目标数据
 * @param[out]  rx_time_ms  输出参数，接收该帧接收时的系统时间戳（毫秒），
 *                           可传 NULL 表示不需要时间戳
 * @retval  true   成功获取
 * @retval  false  无法获取（RxOverflowPending 表示溢出需清空后重试，
 *                  或 !HasLatestTarget 表示尚无有效帧，
 *                  或 target 为空指针）
 */
bool maixcam2_get_latest_target(MaixVisionTarget *target, uint32_t *rx_time_ms)
{
    if (RxOverflowPending || !HasLatestTarget || target == 0)
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

/**
 * @brief   获取协议统计信息的只读指针
 *
 * 统计数据包括：
 * - frames_received:    有效帧数
 * - crc_errors:         CRC 错误数
 * - ring_overflows:     缓冲区溢出数
 * - malformed_frames:   格式错误帧数
 * - semantic_errors:    语义错误数
 * - interbyte_timeouts: 字节间超时数
 *
 * 可用于调试和性能监控，监控通信链路质量。
 *
 * @return  指向内部 MaixProtocolStats 结构体的常量指针
 */
const MaixProtocolStats *maixcam2_get_stats(void)
{
    return &Stats;
}
