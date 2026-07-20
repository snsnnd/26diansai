/**
 * @file foc_protocol.h
 * @brief STM32F103 2804 双轴无刷云台 FOC 二进制协议驱动
 *
 * 本模块实现 MSPM0G3507 → STM32F103 云台驱动板的 UART 二进制协议。
 *
 * 帧格式: SYNC(0xAA) | CMD(1B) | LEN(1B) | DATA(0~60B) | CRC8(1B)
 * CRC8 多项式: x^8 + x^2 + x + 1 (0x07), 覆盖 CMD+LEN+DATA
 * 多字节整数/浮点数一律小端序 (LE)
 *
 * 电机索引: motor_id 0 = M0 (YAW), motor_id 1 = M1 (PITCH)
 */
#ifndef FOC_PROTOCOL_H
#define FOC_PROTOCOL_H

#include "zf_common_headfile.h"

#include <stdbool.h>
#include <stdint.h>

/* ---- 协议常量 ---- */
#define FOC_SYNC          0xAAu
#define FOC_PROTO_MAX_DATA 60u
#define FOC_FRAME_MAX     64u

/* ---- FOC 命令码 ---- */
typedef enum
{
    FOC_CMD_PING           = 0x01u,
    FOC_CMD_SET_TARGET     = 0x02u,
    FOC_CMD_GET_ANGLE      = 0x03u,
    FOC_CMD_SET_MODE       = 0x04u,
    FOC_CMD_SET_PID        = 0x05u,
    FOC_CMD_SET_VEL_PID    = 0x06u,
    FOC_CMD_SET_VOLTAGE    = 0x07u,
    FOC_CMD_SET_LIMITS     = 0x08u,
    FOC_CMD_RESET          = 0x0Fu,
    FOC_CMD_GET_INFO       = 0x10u,
    FOC_CMD_SET_RUN_MODE   = 0x11u,
    FOC_CMD_SET_ANGLE      = 0x12u,
    FOC_CMD_GET_STATUS     = 0x13u,
    FOC_CMD_GET_ALL_STATUS = 0x14u,
    FOC_CMD_SET_DUAL_ANGLE = 0x15u,
    FOC_CMD_HOLD           = 0x16u,
    FOC_CMD_SET_AUTO_THRESH = 0x17u,
    FOC_CMD_SET_FEEDFORWARD = 0x19u,
    FOC_CMD_SESSION        = 0x1Bu,
    FOC_CMD_SET_PROFILE    = 0x1Cu,
} foc_cmd_t;

/* ---- 控制模式 ---- */
typedef enum
{
    FOC_MODE_TORQUE           = 0u,
    FOC_MODE_VELOCITY         = 1u,
    FOC_MODE_ANGLE            = 2u,
    FOC_MODE_VEL_OPENLOOP     = 3u,
    FOC_MODE_ANGLE_OPENLOOP   = 4u,
} foc_control_mode_t;

/* ---- 运行模式 ---- */
typedef enum
{
    FOC_RUN_TRACK = 0u,
    FOC_RUN_TURN  = 1u,
    FOC_RUN_AUTO  = 2u,
} foc_run_mode_t;

/* ---- 错误码 ---- */
typedef enum
{
    FOC_ERR_NONE        = 0x00u,
    FOC_ERR_BAD_CRC     = 0x01u,
    FOC_ERR_BAD_CMD     = 0x02u,
    FOC_ERR_BAD_LEN     = 0x03u,
    FOC_ERR_BAD_MOTOR   = 0x04u,
    FOC_ERR_MOTOR_OFF   = 0x05u,
    FOC_ERR_BAD_MODE    = 0x06u,
    FOC_ERR_TIMEOUT     = 0x07u,
    FOC_ERR_BAD_VALUE   = 0x08u,
    FOC_ERR_BAD_STATE   = 0x09u,
} foc_error_t;

/* ---- 状态标志位 ---- */
#define FOC_FLAG_ONLINE         0x01u
#define FOC_FLAG_ANGLE_CONTROL  0x02u
#define FOC_FLAG_IN_POSITION    0x04u
#define FOC_FLAG_SENSOR_VALID   0x08u
#define FOC_FLAG_TURN_PROFILE   0x10u

/* ---- 电机状态快照 (28字节) ---- */
typedef struct
{
    uint8_t motor_id;
    uint8_t run_mode;
    uint8_t active_profile;
    uint8_t flags;
    float   target_deg;
    float   trajectory_deg;
    float   angle_deg;
    float   velocity_deg_s;
    float   error_deg;
    float   applied_voltage;
    bool    online;
    bool    in_position;
} foc_motor_status_t;

/* ---- 设备信息 ---- */
typedef struct
{
    uint8_t  ver_major;
    uint8_t  ver_minor;
    uint8_t  online_bitmap;
    uint32_t capabilities;
} foc_device_info_t;

/* ==================== CRC8 ==================== */
uint8_t foc_crc8(const uint8_t *data, uint8_t len);

/* ==================== 帧构建与解析 ==================== */

/**
 * @brief 构建二进制帧 (不发送)
 * @param cmd    命令码
 * @param data   数据缓冲区
 * @param len    数据长度 (0~60)
 * @param frame  输出帧缓冲区 (至少 4+len 字节)
 * @return 帧总长度 (4+len)
 */
uint8_t foc_frame_build(uint8_t cmd, const uint8_t *data, uint8_t len,
    uint8_t *frame);

/**
 * @brief 发送二进制帧并等待应答
 * @param uart        UART 索引
 * @param cmd         命令码
 * @param tx_data     发送数据
 * @param tx_len      数据长度
 * @param timeout_ms  应答超时 (ms)
 * @param rx_data     接收数据缓冲区 (至少 60 字节)
 * @param rx_len      输出: 实际接收数据长度
 * @return ZF_TRUE 成功, ZF_FALSE 超时/错误
 */
uint8_t foc_send_cmd(uart_index_enum uart, uint8_t cmd,
    const uint8_t *tx_data, uint8_t tx_len,
    uint32_t timeout_ms, uint8_t *rx_data, uint8_t *rx_len);

/**
 * @brief 发送帧 (无等待, 用于高频周期控制)
 */
void foc_send_frame(uart_index_enum uart, uint8_t cmd,
    const uint8_t *data, uint8_t len);

/* ==================== 状态解析 ==================== */

bool foc_parse_status(const uint8_t *data, uint8_t len,
    foc_motor_status_t *status);

bool foc_parse_dual_status(const uint8_t *data, uint8_t len,
    foc_motor_status_t *m0, foc_motor_status_t *m1);

/* ==================== 浮点编解码 (小端) ==================== */

void   foc_put_f32_le(uint8_t *buf, float val);
float  foc_get_f32_le(const uint8_t *buf);

/* ==================== 高级封装 (常用命令) ==================== */

uint8_t foc_ping(uart_index_enum uart, uint32_t timeout_ms,
    uint8_t *motor_count);

uint8_t foc_session_enter(uart_index_enum uart, uint32_t timeout_ms);

uint8_t foc_get_info(uart_index_enum uart, uint32_t timeout_ms,
    foc_device_info_t *info);

uint8_t foc_set_angle(uart_index_enum uart, uint8_t motor_id,
    float deg, uint32_t timeout_ms);

uint8_t foc_set_dual_angle(uart_index_enum uart, float m0_deg,
    float m1_deg, uint32_t timeout_ms);

void foc_set_dual_angle_fast(uart_index_enum uart, float m0_deg,
    float m1_deg);

uint8_t foc_hold(uart_index_enum uart, uint8_t motor_id,
    uint32_t timeout_ms);

uint8_t foc_set_run_mode(uart_index_enum uart, uint8_t motor_id,
    uint8_t run_mode, uint32_t timeout_ms);

uint8_t foc_set_control_mode(uart_index_enum uart, uint8_t motor_id,
    uint8_t mode, uint32_t timeout_ms);

uint8_t foc_get_status(uart_index_enum uart, uint8_t motor_id,
    uint32_t timeout_ms, foc_motor_status_t *status);

uint8_t foc_get_all_status(uart_index_enum uart, uint32_t timeout_ms,
    foc_motor_status_t *m0, foc_motor_status_t *m1);

#endif
