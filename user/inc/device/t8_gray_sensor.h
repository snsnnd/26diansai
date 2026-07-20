/*********************************************************************************************************************
 * t8_gray_sensor.h — T8 灰度传感器驱动（UART / I2C 双接口）
 *
 * T8 是一款 8 通道灰度传感器模块，常用于智能车循迹（line-following）。
 * 它通过 UART 或 I2C 接口与主控通信，返回每个通道的灰度值（8位或16位）、
 * 二值化结果、黑/白标定值等。
 *
 * 硬件协议：
 *   UART: 帧格式 AA 55 [function] [payload_len] [command] [data...] [checksum] 66
 *   I2C:  发送命令字节，返回 [command] [length] [data...] [checksum]
 *
 * 此驱动将 UART 和 I2C 两种通信方式封装为统一的抽象层，
 * 通信函数通过函数指针注入，使得驱动不依赖具体的硬件实现。
 ********************************************************************************************************************/

#ifndef T8_GRAY_SENSOR_H
#define T8_GRAY_SENSOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==================== 常量定义 ====================*/

#define T8_SENSOR_COUNT 8u           /* T8 传感器通道数（8 路灰度） */
#define T8_UART_BAUDRATE 115200u     /* 推荐的 UART 通信波特率 */
#define T8_DEFAULT_I2C_ADDRESS 0x40u /* T8 默认 I2C 设备地址（7位） */
#define T8_MIN_I2C_ADDRESS 0x10u     /* I2C 地址下限 */
#define T8_MAX_I2C_ADDRESS 0x7Fu     /* I2C 地址上限 */
#define T8_MAX_FRAME_SIZE 32u        /* 协议帧数据段最大长度（字节） */

/*==================== 错误码枚举 ====================*/

/**
 * T8Status — T8 驱动函数返回状态。
 *
 * 正值表示成功，负值表示各类错误。
 */
typedef enum
{
    T8_OK = 0,                    /* 操作成功 */
    T8_ERROR = -1,                /* 通用错误 */
    T8_ERROR_INVALID_ARG = -2,    /* 无效参数（如 NULL 指针） */
    T8_ERROR_IO = -3,             /* I/O 通信错误 */
    T8_ERROR_TIMEOUT = -4,        /* 通信超时 */
    T8_ERROR_BAD_FRAME = -5,      /* 协议帧格式错误 */
    T8_ERROR_CHECKSUM = -6,       /* 校验和错误 */
    T8_ERROR_UNKNOWN_COMMAND = -7,/* 传感器不支持的指令 */
} T8Status;

/*==================== 命令码枚举 ====================*/

/**
 * T8Command — T8 传感器支持的命令码。
 *
 * 命令分为几类：
 *   0x01~0x08: 读取单通道 8 位灰度值
 *   0x09~0x0B: 读取全部 8 通道的 8 位灰度/黑标定/白标定值
 *   0x0C:      读取二值化结果（1 字节，每位代表一个通道）
 *   0x11~0x1B: 同上，但为 16 位高精度 ADC 值
 *   0xFF:      停止连续模式
 *   0xAD~0xAE: 配置/读取 I2C 地址和固件版本
 */
typedef enum
{
    /* 8 位灰度值读取命令 */
    T8_CMD_GRAY8_CH1 = 0x01,   /* 通道 1 的 8 位灰度值 */
    T8_CMD_GRAY8_CH2 = 0x02,   /* 通道 2 的 8 位灰度值 */
    T8_CMD_GRAY8_CH3 = 0x03,   /* 通道 3 的 8 位灰度值 */
    T8_CMD_GRAY8_CH4 = 0x04,   /* 通道 4 的 8 位灰度值 */
    T8_CMD_GRAY8_CH5 = 0x05,   /* 通道 5 的 8 位灰度值 */
    T8_CMD_GRAY8_CH6 = 0x06,   /* 通道 6 的 8 位灰度值 */
    T8_CMD_GRAY8_CH7 = 0x07,   /* 通道 7 的 8 位灰度值 */
    T8_CMD_GRAY8_CH8 = 0x08,   /* 通道 8 的 8 位灰度值 */
    T8_CMD_GRAY8_ALL = 0x09,   /* 全部 8 通道的 8 位灰度值（返回 8 字节） */
    T8_CMD_BLACK8_ALL = 0x0A,  /* 全部 8 通道的黑标定值（8 位） */
    T8_CMD_WHITE8_ALL = 0x0B,  /* 全部 8 通道的白标定值（8 位） */
    T8_CMD_DIGITAL = 0x0C,     /* 二值化结果（1 字节，位 0~7 对应 CH1~CH8） */

    /* 16 位高精度 ADC 值读取命令 (16-bit resolution) */
    T8_CMD_ADC16_CH1 = 0x11,   /* 通道 1 的 16 位 ADC 值 */
    T8_CMD_ADC16_CH2 = 0x12,   /* 通道 2 的 16 位 ADC 值 */
    T8_CMD_ADC16_CH3 = 0x13,
    T8_CMD_ADC16_CH4 = 0x14,
    T8_CMD_ADC16_CH5 = 0x15,
    T8_CMD_ADC16_CH6 = 0x16,
    T8_CMD_ADC16_CH7 = 0x17,
    T8_CMD_ADC16_CH8 = 0x18,
    T8_CMD_ADC16_ALL = 0x19,   /* 全部 8 通道的 16 位 ADC 值（返回 16 字节） */
    T8_CMD_BLACK16_ALL = 0x1A, /* 全部 8 通道的黑标定值（16 位） */
    T8_CMD_WHITE16_ALL = 0x1B, /* 全部 8 通道的白标定值（16 位） */

    /* 系统命令 */
    T8_CMD_STOP_CONTINUOUS = 0xFF, /* 停止连续发送模式 */
    T8_CMD_I2C_ADDRESS = 0xAD,     /* 设置/读取 I2C 地址 */
    T8_CMD_VERSION = 0xAE,         /* 读取固件版本号 */
} T8Command;

/*==================== 通信传输层函数指针类型 ====================*/

/*
 * T8 驱动采用"依赖注入"模式，通信函数由调用者通过函数指针提供。
 * 这样驱动代码不需要关心底层是哪个 UART 或 I2C 外设，具有良好的可移植性。
 */

/* UART 写函数：发送数据，返回实际发送的字节数 */
typedef size_t (*T8UartWriteFn)(const uint8_t *data, size_t length, void *user_data);
/* UART 读函数：读取数据，返回实际读取的字节数（可能 < length 表示超时） */
typedef size_t (*T8UartReadFn)(uint8_t *data, size_t length, uint32_t timeout_ms, void *user_data);
/* UART 清空输入/输出缓冲区 */
typedef void (*T8UartFlushFn)(void *user_data);
/* 延时函数：阻塞等待指定毫秒数 */
typedef void (*T8DelayFn)(uint32_t delay_ms, void *user_data);

/* I2C 写函数：向从机写数据，成功返回 true */
typedef bool (*T8I2cWriteFn)(uint8_t address, const uint8_t *data, size_t length, void *user_data);
/* I2C 读函数：从从机读数据，成功返回 true */
typedef bool (*T8I2cReadFn)(uint8_t address, uint8_t *data, size_t length, uint32_t timeout_ms, void *user_data);

/*==================== 传输层结构体 ====================*/

/**
 * T8UartTransport — UART 传输层接口。
 *
 * 包含读写函数指针和 flush/delay 辅助函数。
 * user_data 由调用者定义，可用于传递外设句柄或上下文信息。
 */
typedef struct
{
    T8UartWriteFn write;
    T8UartReadFn read;
    T8UartFlushFn flush_input;      /* 清空 UART 输入 FIFO */
    T8UartFlushFn flush_output;     /* 等待 UART 输出完成 */
    T8DelayFn delay_ms;
    void *user_data;
} T8UartTransport;

/**
 * T8I2cTransport — I2C 传输层接口。
 */
typedef struct
{
    T8I2cWriteFn write;
    T8I2cReadFn read;
    void *user_data;
} T8I2cTransport;

/*==================== 设备实例结构体 ====================*/

/**
 * T8UartDevice — UART 模式 T8 传感器设备实例。
 *
 * 包含传输层接口、超时设置和重试次数配置。
 */
typedef struct
{
    T8UartTransport transport;
    uint32_t timeout_ms;        /* 单次读写超时（毫秒） */
    uint8_t max_retries;        /* 通信失败时最大重试次数 */
} T8UartDevice;

/**
 * T8I2cDevice — I2C 模式 T8 传感器设备实例。
 */
typedef struct
{
    T8I2cTransport transport;
    uint8_t address;            /* I2C 从机地址（7 位左对齐） */
    uint32_t timeout_ms;
} T8I2cDevice;

/**
 * T8Packet — 协议数据包结构。
 *
 * 既可用于 UART 模式也可用于 I2C 模式。
 * command: 命令码
 * length:  data 段长度
 * data:    数据段缓冲区
 */
typedef struct
{
    uint8_t command;
    uint8_t length;
    uint8_t data[T8_MAX_FRAME_SIZE];
} T8Packet;

/*==================== 设备初始化 ====================*/

/* 初始化 UART 模式设备（注入传输层函数） */
void t8_uart_init(T8UartDevice *device, const T8UartTransport *transport);
/* 初始化 I2C 模式设备（注入传输层函数和 I2C 地址） */
void t8_i2c_init(T8I2cDevice *device, const T8I2cTransport *transport, uint8_t address);

/*==================== 通用函数 ====================*/

/* 计算数据块的 8 位校验和（累加和） */
uint8_t t8_checksum(const uint8_t *data, size_t length);

/*==================== UART 模式底层协议函数 ====================*/

/* 发送读命令并接收响应包（含重试机制） */
T8Status t8_uart_read_packet(T8UartDevice *device, uint8_t command, T8Packet *packet);
/* 被动接收一个数据包（用于连续接收模式） */
T8Status t8_uart_receive_packet(T8UartDevice *device, T8Packet *packet);
/* 发送写命令并接收响应 */
T8Status t8_uart_write_command(T8UartDevice *device, uint8_t command, const uint8_t *data, uint8_t length, T8Packet *packet);
/* 启动连续发送模式（传感器主动定时上报） */
T8Status t8_uart_start_continuous(T8UartDevice *device, uint8_t command, uint8_t period_units_10ms);
/* 停止连续发送模式 */
T8Status t8_uart_stop_continuous(T8UartDevice *device);
/* 设置传感器的 I2C 地址（UART 作为配置接口） */
T8Status t8_uart_set_i2c_address(T8UartDevice *device, uint8_t address, uint8_t *effective_address);
/* 读取传感器当前的 I2C 地址 */
T8Status t8_uart_get_i2c_address(T8UartDevice *device, uint8_t *address);
/* 读取传感器固件版本 */
T8Status t8_uart_get_version(T8UartDevice *device, uint8_t *version);

/*==================== UART 模式 8 位灰度值读取 ====================*/

T8Status t8_uart_get_gray8(T8UartDevice *device, uint8_t channel, uint8_t *value);
T8Status t8_uart_get_gray8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_uart_get_black8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_uart_get_white8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_uart_get_digital(T8UartDevice *device, uint8_t *bits);

/*==================== UART 模式 16 位 ADC 值读取 ====================*/

T8Status t8_uart_get_adc16(T8UartDevice *device, uint8_t channel, uint16_t *value);
T8Status t8_uart_get_adc16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT]);
T8Status t8_uart_get_black16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT]);
T8Status t8_uart_get_white16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT]);

/*==================== I2C 模式 ====================*/

/* I2C 模式读取协议数据包 */
T8Status t8_i2c_read_packet(T8I2cDevice *device, uint8_t command, T8Packet *packet);

/* I2C 模式 8 位灰度值读取 */
T8Status t8_i2c_get_gray8(T8I2cDevice *device, uint8_t channel, uint8_t *value);
T8Status t8_i2c_get_gray8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_i2c_get_black8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_i2c_get_white8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT]);
T8Status t8_i2c_get_digital(T8I2cDevice *device, uint8_t *bits);

/**
 * @brief 从8通道灰度数据计算二值位和连续浮点位置（纯计算，不操作硬件）
 * @param gray      8通道灰度值[0-255], 低=黑
 * @param bits_out  输出二值位(0=黑线, 1=白)，全部输出 0xFF 表示全白
 * @param pos_out   输出连续浮点位置(-7.0~+7.0), 全白=99.0
 */
void t8_compute_position(const uint8_t gray[T8_SENSOR_COUNT],
    uint8_t *bits_out, float *pos_out);

/* I2C 模式 16 位 ADC 值读取 */
T8Status t8_i2c_get_adc16(T8I2cDevice *device, uint8_t channel, uint16_t *value);
T8Status t8_i2c_get_adc16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT]);
T8Status t8_i2c_get_black16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT]);
T8Status t8_i2c_get_white16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
