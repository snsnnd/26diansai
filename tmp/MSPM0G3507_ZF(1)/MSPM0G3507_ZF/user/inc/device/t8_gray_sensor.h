#ifndef T8_GRAY_SENSOR_H
#define T8_GRAY_SENSOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define T8_SENSOR_COUNT 8u
#define T8_UART_BAUDRATE 115200u
#define T8_DEFAULT_I2C_ADDRESS 0x40u
#define T8_MIN_I2C_ADDRESS 0x10u
#define T8_MAX_I2C_ADDRESS 0x7Fu
#define T8_MAX_FRAME_SIZE 32u

typedef enum
{
    T8_OK = 0,
    T8_ERROR = -1,
    T8_ERROR_INVALID_ARG = -2,
    T8_ERROR_IO = -3,
    T8_ERROR_TIMEOUT = -4,
    T8_ERROR_BAD_FRAME = -5,
    T8_ERROR_CHECKSUM = -6,
    T8_ERROR_UNKNOWN_COMMAND = -7,
} T8Status;

typedef enum
{
    T8_CMD_GRAY8_CH1 = 0x01,
    T8_CMD_GRAY8_CH2 = 0x02,
    T8_CMD_GRAY8_CH3 = 0x03,
    T8_CMD_GRAY8_CH4 = 0x04,
    T8_CMD_GRAY8_CH5 = 0x05,
    T8_CMD_GRAY8_CH6 = 0x06,
    T8_CMD_GRAY8_CH7 = 0x07,
    T8_CMD_GRAY8_CH8 = 0x08,
    T8_CMD_GRAY8_ALL = 0x09,
    T8_CMD_BLACK8_ALL = 0x0A,
    T8_CMD_WHITE8_ALL = 0x0B,
    T8_CMD_DIGITAL = 0x0C,

    T8_CMD_ADC16_CH1 = 0x11,
    T8_CMD_ADC16_CH2 = 0x12,
    T8_CMD_ADC16_CH3 = 0x13,
    T8_CMD_ADC16_CH4 = 0x14,
    T8_CMD_ADC16_CH5 = 0x15,
    T8_CMD_ADC16_CH6 = 0x16,
    T8_CMD_ADC16_CH7 = 0x17,
    T8_CMD_ADC16_CH8 = 0x18,
    T8_CMD_ADC16_ALL = 0x19,
    T8_CMD_BLACK16_ALL = 0x1A,
    T8_CMD_WHITE16_ALL = 0x1B,

    T8_CMD_STOP_CONTINUOUS = 0xFF,
    T8_CMD_I2C_ADDRESS = 0xAD,
    T8_CMD_VERSION = 0xAE,
} T8Command;

typedef size_t (*T8UartWriteFn)(const uint8_t *data, size_t length, void *user_data);
typedef size_t (*T8UartReadFn)(uint8_t *data, size_t length, uint32_t timeout_ms, void *user_data);
typedef void (*T8UartFlushFn)(void *user_data);
typedef void (*T8DelayFn)(uint32_t delay_ms, void *user_data);

typedef bool (*T8I2cWriteFn)(uint8_t address, const uint8_t *data, size_t length, void *user_data);
typedef bool (*T8I2cReadFn)(uint8_t address, uint8_t *data, size_t length, uint32_t timeout_ms, void *user_data);

typedef struct
{
    T8UartWriteFn write;
    T8UartReadFn read;
    T8UartFlushFn flush_input;
    T8UartFlushFn flush_output;
    T8DelayFn delay_ms;
    void *user_data;
} T8UartTransport;

typedef struct
{
    T8I2cWriteFn write;
    T8I2cReadFn read;
    void *user_data;
} T8I2cTransport;

typedef struct
{
    T8UartTransport transport;
    uint32_t timeout_ms;
    uint8_t max_retries;
} T8UartDevice;

typedef struct
{
    T8I2cTransport transport;
    uint8_t address;
    uint32_t timeout_ms;
} T8I2cDevice;

typedef struct
{
    uint8_t command;
    uint8_t length;
    uint8_t data[T8_MAX_FRAME_SIZE];
} T8Packet;

void t8_uart_init(T8UartDevice *device, const T8UartTransport *transport);
void t8_i2c_init(T8I2cDevice *device, const T8I2cTransport *transport, uint8_t address);
uint8_t t8_checksum(const uint8_t *data, size_t length);

T8Status t8_uart_read_packet(T8UartDevice *device, uint8_t command, T8Packet *packet);
T8Status t8_uart_receive_packet(T8UartDevice *device, T8Packet *packet);
T8Status t8_uart_write_command(T8UartDevice *device, uint8_t command, const uint8_t *data, uint8_t length, T8Packet *packet);
T8Status t8_uart_start_continuous(T8UartDevice *device, uint8_t command, uint8_t period_units_10ms);
T8Status t8_uart_stop_continuous(T8UartDevice *device);
T8Status t8_uart_set_i2c_address(T8UartDevice *device, uint8_t address, uint8_t *effective_address);
T8Status t8_uart_get_i2c_address(T8UartDevice *device, uint8_t *address);
T8Status t8_uart_get_version(T8UartDevice *device, uint8_t *version);

T8Status t8_uart_get_gray8(T8UartDevice *device, uint8_t channel, uint8_t *value);
T8Status t8_uart_get_gray8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_uart_get_black8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_uart_get_white8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_uart_get_digital(T8UartDevice *device, uint8_t *bits);
T8Status t8_uart_get_adc16(T8UartDevice *device, uint8_t channel, uint16_t *value);
T8Status t8_uart_get_adc16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT]);
T8Status t8_uart_get_black16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT]);
T8Status t8_uart_get_white16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT]);

T8Status t8_i2c_read_packet(T8I2cDevice *device, uint8_t command, T8Packet *packet);
T8Status t8_i2c_get_gray8(T8I2cDevice *device, uint8_t channel, uint8_t *value);
T8Status t8_i2c_get_gray8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_i2c_get_black8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_i2c_get_white8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_i2c_get_digital(T8I2cDevice *device, uint8_t *bits);
T8Status t8_i2c_get_adc16(T8I2cDevice *device, uint8_t channel, uint16_t *value);
T8Status t8_i2c_get_adc16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT]);
T8Status t8_i2c_get_black16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT]);
T8Status t8_i2c_get_white16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
