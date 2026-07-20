/**
 * @file dt_gyro_z.h
 * @brief Z 轴陀螺仪驱动 — UART 或 I2C 双传输。
 *        GYRO_Z_TRANSPORT=1: UART  (PA10/PA11)，模块主动发包。
 *        GYRO_Z_TRANSPORT=2: I2C   (PA0/PA1, addr 0x48)，MCU 主动读取。
 */

#ifndef _DT_GYRO_Z_H_
#define _DT_GYRO_Z_H_

#include "zf_common_headfile.h"

typedef struct
{
    uart_index_enum uart;
    uint32 baud;
    uart_tx_pin_enum tx_pin;
    uart_rx_pin_enum rx_pin;
} dt_gyro_z_config_t;

typedef struct
{
    float yaw_deg;
    float wz_dps;
    int16_t yaw_raw;
    int16_t wz_raw;
    uint8_t yaw_updated;
    uint8_t wz_updated;
    uint32_t frame_count;
    uint32_t checksum_error_count;
    uint32_t rx_overflow;
} dt_gyro_z_data_t;

void dt_gyro_z_init(const dt_gyro_z_config_t *config);
uint8_t dt_gyro_z_update(uint32_t now_ms);
const dt_gyro_z_data_t *dt_gyro_z_get_data(void);
float dt_gyro_z_get_yaw(void);
float dt_gyro_z_get_wz(void);
uint32_t dt_gyro_z_get_rx_overflow(void);
bool dt_gyro_z_data_received(void);
void dt_gyro_z_zero_yaw(void);
void dt_gyro_z_start_bias_cal(void);

#endif
