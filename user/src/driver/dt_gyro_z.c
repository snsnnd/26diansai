/**
 * @file dt_gyro_z.c
 * @brief Z 轴陀螺仪驱动 — 支持 UART 和 I2C 两种传输方式。
 *        GYRO_Z_TRANSPORT=1 → UART (PA10/PA11)，模块主动发包。
 *        GYRO_Z_TRANSPORT=2 → I2C  (PA0/PA1, addr 0x48)，MCU 主动读取。
 */

#include "driver/dt_gyro_z.h"
#include "config.h"

#if GYRO_Z_TRANSPORT == 1

/* ======================== UART 传输 ======================== */

#include "lib/serial_rx_buffer.h"

#define DT_GYRO_Z_RX_HEAD        0x5A
#define DT_GYRO_Z_RX_TYPE_WZ     0xAA
#define DT_GYRO_Z_RX_TYPE_YAW    0xBB
#define DT_GYRO_Z_TX_HEAD_0      0x55
#define DT_GYRO_Z_TX_HEAD_1      0xAA
#define DT_GYRO_Z_REG_KEY        0x13
#define DT_GYRO_Z_REG_SAVE       0x00
#define DT_GYRO_Z_REG_YAW_ZERO   0x15
#define DT_GYRO_Z_REG_BIAS_CAL   0x0A
#define DT_GYRO_Z_KEY_VALUE      0x5F8E
#define DT_GYRO_Z_RX_BUF_SIZE    512u

static dt_gyro_z_config_t g_gyro_z_config;
static dt_gyro_z_data_t g_gyro_z_data;
static bool g_gyro_z_has_rx = false;
static uint8_t g_gyro_z_rx_storage[DT_GYRO_Z_RX_BUF_SIZE];
static SerialRxBuffer g_gyro_z_ring;
static uint8_t g_gyro_z_frame_buf[5];
static uint8_t g_gyro_z_frame_idx;

static void gyro_write_register(uint8_t reg, uint16_t value)
{
    uint8_t buffer[5];
    buffer[0] = DT_GYRO_Z_TX_HEAD_0;
    buffer[1] = DT_GYRO_Z_TX_HEAD_1;
    buffer[2] = reg;
    buffer[3] = (uint8_t)(value & 0xFFu);
    buffer[4] = (uint8_t)((value >> 8) & 0xFFu);
    uart_write_buffer(g_gyro_z_config.uart, buffer, sizeof(buffer));
}

static void gyro_unlock(void)
{
    gyro_write_register(DT_GYRO_Z_REG_KEY, DT_GYRO_Z_KEY_VALUE);
}

static int16_t gyro_make_int16(uint8_t l, uint8_t h)
{
    return (int16_t)(((int16_t)h << 8) | l);
}

static uint8_t gyro_parse_frame(const uint8_t *frame)
{
    uint8_t sum = (uint8_t)(frame[0] + frame[1] + frame[2] + frame[3]);
    if (sum != frame[4]) { g_gyro_z_data.checksum_error_count++; return 0; }
    int16_t raw = gyro_make_int16(frame[2], frame[3]);
    if (frame[1] == DT_GYRO_Z_RX_TYPE_WZ)
    {
        g_gyro_z_data.wz_raw = raw;
        g_gyro_z_data.wz_dps = (float)raw * 2000.0f / 32768.0f;
        g_gyro_z_data.wz_updated = 1;
        g_gyro_z_data.frame_count++;
        return 1;
    }
    else if (frame[1] == DT_GYRO_Z_RX_TYPE_YAW)
    {
        g_gyro_z_data.yaw_raw = raw;
        g_gyro_z_data.yaw_deg = (float)raw * 180.0f / 32768.0f;
        g_gyro_z_data.yaw_updated = 1;
        g_gyro_z_data.frame_count++;
        return 1;
    }
    return 0;
}

static uint8_t gyro_feed_byte(uint8_t data)
{
    if (g_gyro_z_frame_idx == 0u && data != DT_GYRO_Z_RX_HEAD) return 0;
    g_gyro_z_frame_buf[g_gyro_z_frame_idx++] = data;
    if (g_gyro_z_frame_idx >= sizeof(g_gyro_z_frame_buf))
    {
        g_gyro_z_frame_idx = 0;
        return gyro_parse_frame(g_gyro_z_frame_buf);
    }
    return 0;
}

static void gyro_uart_isr(uint32_t state, void *ctx)
{
    uint8_t data;
    (void)ctx;
    if ((state & UART_INTERRUPT_STATE_RX) == 0u) return;
    while (uart_query_byte(g_gyro_z_config.uart, &data) == ZF_TRUE)
        (void)serial_rx_buffer_push(&g_gyro_z_ring, data);
}

void dt_gyro_z_init(const dt_gyro_z_config_t *config)
{
    if (config == NULL) return;
    g_gyro_z_config = *config;
    memset(&g_gyro_z_data, 0, sizeof(g_gyro_z_data));
    g_gyro_z_frame_idx = 0;
    serial_rx_buffer_init(&g_gyro_z_ring, g_gyro_z_rx_storage, sizeof(g_gyro_z_rx_storage));
    uart_init(config->uart, config->baud, config->tx_pin, config->rx_pin);
    uart_set_callback(config->uart, gyro_uart_isr, NULL);
    uart_set_interrupt_config(config->uart, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

uint8_t dt_gyro_z_update(uint32_t now_ms)
{
    uint8_t data, new_frames = 0;
    (void)now_ms;
    g_gyro_z_data.yaw_updated = 0;
    g_gyro_z_data.wz_updated = 0;
    while (serial_rx_buffer_pop(&g_gyro_z_ring, &data))
        new_frames += gyro_feed_byte(data);
    g_gyro_z_data.rx_overflow = (uint32_t)serial_rx_buffer_overflow_count(&g_gyro_z_ring);
    g_gyro_z_has_rx = (new_frames > 0u);
    return new_frames;
}

#elif GYRO_Z_TRANSPORT == 2

/* ======================== I2C 传输 ======================== */

#include "zf_driver_soft_iic.h"

#define DT_GYRO_Z_REG_YAW       0x09u
#define DT_GYRO_Z_REG_GZ        0x1Bu
#define DT_GYRO_Z_REG_KEY       0x13u
#define DT_GYRO_Z_REG_SAVE      0x00u
#define DT_GYRO_Z_REG_YAW_ZERO  0x15u
#define DT_GYRO_Z_REG_BIAS_CAL  0x0Au
#define DT_GYRO_Z_KEY_VALUE_L    0x8Eu
#define DT_GYRO_Z_KEY_VALUE_H    0x5Fu
#define DT_GYRO_Z_DATA_SCALE    180.0f

static soft_iic_info_struct g_gyro_iic;
static dt_gyro_z_data_t g_gyro_z_data;
static bool g_gyro_z_has_rx = false;

static void gyro_write_register(uint8_t reg, uint16_t value)
{
    uint8_t data[2];
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
    (void)soft_iic_write_8bit_registers_checked(&g_gyro_iic, reg, data, 2u);
}

static void gyro_unlock(void)
{
    gyro_write_register(DT_GYRO_Z_REG_KEY,
        ((uint16_t)DT_GYRO_Z_KEY_VALUE_H << 8) | DT_GYRO_Z_KEY_VALUE_L);
}

static bool gyro_read_reg(int16_t *dest, uint8_t reg)
{
    uint8_t buf[2];
    uint8_t i;

    soft_iic_start(&g_gyro_iic);
    if (!soft_iic_send_data(&g_gyro_iic, g_gyro_iic.addr << 1)
        || !soft_iic_send_data(&g_gyro_iic, reg))
    {
        soft_iic_stop(&g_gyro_iic);
        return false;
    }
    soft_iic_stop(&g_gyro_iic);

    soft_iic_start(&g_gyro_iic);
    if (!soft_iic_send_data(&g_gyro_iic, (g_gyro_iic.addr << 1) | 0x01u))
    {
        soft_iic_stop(&g_gyro_iic);
        return false;
    }
    for (i = 0u; i < 2u; i++)
    {
        buf[i] = soft_iic_read_data(&g_gyro_iic, (i == 1u) ? 1u : 0u);
    }
    soft_iic_stop(&g_gyro_iic);

    *dest = (int16_t)(((int16_t)buf[1] << 8) | buf[0]);
    return true;
}

void dt_gyro_z_init(const dt_gyro_z_config_t *config)
{
    if (config == NULL) return;
    memset(&g_gyro_z_data, 0, sizeof(g_gyro_z_data));

    soft_iic_init(&g_gyro_iic, GYRO_Z_IIC_ADDR, GYRO_Z_IIC_DELAY,
        (gpio_pin_enum)GYRO_Z_IIC_SCL, (gpio_pin_enum)GYRO_Z_IIC_SDA);

    gyro_unlock();
    system_delay_ms(10);
    gyro_write_register(DT_GYRO_Z_REG_YAW_ZERO, 0x0000u);
    system_delay_ms(10);
    gyro_write_register(DT_GYRO_Z_REG_SAVE, 0x0000u);

    gyro_unlock();
    system_delay_ms(10);
    gyro_write_register(DT_GYRO_Z_REG_YAW_ZERO, 0x0000u);
    system_delay_ms(10);
    gyro_write_register(DT_GYRO_Z_REG_SAVE, 0x0000u);

    (void)config;
}

uint8_t dt_gyro_z_update(uint32_t now_ms)
{
    int16_t raw;
    uint8_t new_frames = 0;

    (void)now_ms;
    g_gyro_z_data.yaw_updated = 0;
    g_gyro_z_data.wz_updated = 0;

    if (gyro_read_reg(&raw, DT_GYRO_Z_REG_GZ))
    {
        g_gyro_z_data.wz_raw = raw;
        g_gyro_z_data.wz_dps = (float)raw
            * DT_GYRO_Z_DATA_SCALE / 32768.0f;
        g_gyro_z_data.wz_updated = 1;
        new_frames++;
    }
    if (gyro_read_reg(&raw, DT_GYRO_Z_REG_YAW))
    {
        g_gyro_z_data.yaw_raw = raw;
        g_gyro_z_data.yaw_deg = (float)raw
            * DT_GYRO_Z_DATA_SCALE / 32768.0f;
        g_gyro_z_data.yaw_updated = 1;
        new_frames++;
    }
    g_gyro_z_data.frame_count += new_frames;
    g_gyro_z_has_rx = (new_frames > 0u);
    return new_frames;
}
#else
#error "GYRO_Z_TRANSPORT must be 1 (UART) or 2 (I2C)"
#endif

/* ======================== 公共接口 ======================== */

const dt_gyro_z_data_t *dt_gyro_z_get_data(void) { return &g_gyro_z_data; }
float dt_gyro_z_get_yaw(void)                      { return g_gyro_z_data.yaw_deg; }
float dt_gyro_z_get_wz(void)                       { return g_gyro_z_data.wz_dps; }
uint32_t dt_gyro_z_get_rx_overflow(void)            { return g_gyro_z_data.rx_overflow; }

bool dt_gyro_z_data_received(void)
{
    bool r = g_gyro_z_has_rx;
    g_gyro_z_has_rx = false;
    return r;
}

void dt_gyro_z_zero_yaw(void)
{
    gyro_unlock();
    system_delay_ms(100);
    gyro_write_register(DT_GYRO_Z_REG_YAW_ZERO, 0x0000u);
    system_delay_ms(100);
    gyro_write_register(DT_GYRO_Z_REG_SAVE, 0x0000u);
}

void dt_gyro_z_start_bias_cal(void)
{
    gyro_unlock();
    system_delay_ms(100);
    gyro_write_register(DT_GYRO_Z_REG_BIAS_CAL, 0x0001u);
}
