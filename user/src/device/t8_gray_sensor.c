/*********************************************************************************************************************
 * t8_gray_sensor.c — T8 8 路灰度传感器驱动实现
 *
 * 本文件实现了 T8 灰度传感器的全部协议层逻辑，包括：
 *   1. UART 通信协议（帧封装、解析、校验）
 *   2. I2C 通信协议（帧封装、解析、校验）
 *   3. 自动重试机制
 *   4. 各种数据格式的读取（8位灰度、16位ADC、二值化、标定值）
 *
 * UART 协议帧格式（定界符方式）：
 *   帧头(AA 55) | 功能码(1B) | 负载长度(1B) | 命令(1B) | 数据(NB) | 校验和(1B) | 帧尾(66)
 *
 * I2C 协议帧格式（简化的命令-响应方式）：
 *   写入：发送命令字节
 *   读取：接收 [命令(1B) | 数据长度(1B) | 数据(NB) | 校验和(1B)]
 *
 * 校验和算法：功能码 + 负载长度 + 命令 + 所有数据字节（累加和截取低 8 位）
 ********************************************************************************************************************/

#include "device/t8_gray_sensor.h"

/*==================== UART 协议常量 ====================*/

#define T8_UART_HEAD0 0xAAu         /* 帧头第一个字节 */
#define T8_UART_HEAD1 0x55u         /* 帧头第二个字节 */
#define T8_UART_TAIL 0x66u          /* 帧尾字节 */
#define T8_UART_FUNC_READ 0x00u     /* 功能码：读取 */
#define T8_UART_FUNC_WRITE 0x01u    /* 功能码：写入 */
#define T8_UART_ERROR_CHECKSUM 0xECu /* 传感器返回的校验和错误标志（在 command 字段） */
#define T8_UART_ERROR_UNKNOWN 0xEEu  /* 传感器返回的未知命令错误标志 */
#define T8_UART_MAX_RETRIES 3u       /* 通信失败时默认最大重试次数 */
#define T8_UART_DEFAULT_TIMEOUT_MS 100u /* 默认 UART 超时时间（ms） */
#define T8_I2C_DEFAULT_TIMEOUT_MS 100u  /* 默认 I2C 超时时间（ms） */

/*
 * command_data_length — 根据命令码返回数据段的预期长度。
 *
 * 不同的命令返回不同长度的数据：
 *   单通道 8 位灰度：1 字节
 *   全部 8 通道 8 位灰度/黑白标定：8 字节
 *   二值化/版本/I2C 地址：1 字节
 *   单通道 16 位 ADC：2 字节
 *   全部 8 通道 16 位 ADC/黑白标定：16 字节
 *
 * @command: 命令码
 * @return: 数据段预期长度（字节数），未知命令返回 0
 */
static size_t command_data_length(uint8_t command)
{
    if (command >= T8_CMD_GRAY8_CH1 && command <= T8_CMD_GRAY8_CH8)
    {
        return 1u;
    }
    if (command == T8_CMD_GRAY8_ALL || command == T8_CMD_BLACK8_ALL || command == T8_CMD_WHITE8_ALL)
    {
        return 8u;
    }
    if (command == T8_CMD_DIGITAL || command == T8_CMD_I2C_ADDRESS || command == T8_CMD_VERSION)
    {
        return 1u;
    }
    if (command >= T8_CMD_ADC16_CH1 && command <= T8_CMD_ADC16_CH8)
    {
        return 2u;
    }
    if (command == T8_CMD_ADC16_ALL || command == T8_CMD_BLACK16_ALL || command == T8_CMD_WHITE16_ALL)
    {
        return 16u;
    }
    return 0u;
}

/*
 * read_u16_le — 从小端序字节序的缓冲区中读取 16 位无符号整数。
 *
 * T8 传感器的多字节数据采用小端序（低字节在前），
 * 这与 ARM Cortex-M0+ 的原生字节序一致，无需字节交换。
 *
 * @data: 指向至少 2 字节的缓冲区（data[0]=低字节，data[1]=高字节）
 * @return: 拼接后的 uint16_t 值
 */
static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[1] << 8) | data[0]);
}

/*
 * copy_bytes — 字节拷贝（支持重叠区域拷贝）。
 *
 * 使用简单的逐字节循环而非 memcpy，以避免引入 <string.h> 的依赖。
 * 在嵌入式环境中，减少库依赖有助于控制代码体积。
 *
 * @dst:    目标缓冲区
 * @src:    源缓冲区
 * @length: 要拷贝的字节数
 */
static void copy_bytes(uint8_t *dst, const uint8_t *src, size_t length)
{
    for (size_t i = 0u; i < length; ++i)
    {
        dst[i] = src[i];
    }
}

/*
 * uart_read_exact — 精确读取指定长度的 UART 数据。
 *
 * 由于 UART 接收是字节流式的，一次 read 调用可能只返回部分数据。
 * 此函数通过循环确保实际读取到指定长度的数据。
 * 如果在某次读取中返回 0（超时或错误），立即返回 T8_ERROR_TIMEOUT。
 *
 * 这种"exact read"模式在面向帧的协议中很常见，
 * 它能确保上层协议解析时数据完整性。
 *
 * @device: UART 设备句柄
 * @data:   接收缓冲区
 * @length: 目标读取字节数
 * @return: T8_OK（成功）或 T8_ERROR_TIMEOUT（超时）
 */
static T8Status uart_read_exact(T8UartDevice *device, uint8_t *data, size_t length)
{
    size_t offset = 0u;

    while (offset < length)
    {
        size_t count = device->transport.read(&data[offset], length - offset, device->timeout_ms, device->transport.user_data);
        if (count == 0u)
        {
            return T8_ERROR_TIMEOUT;
        }
        offset += count;
    }

    return T8_OK;
}

/*
 * uart_wait_header — 等待并同步到 UART 帧头（AA 55）。
 *
 * 在不可靠的通信环境中，接收可能从帧中间开始（如传感器上电后第一条消息可能不完整）。
 * 此函数在最多 64 字节的窗口内搜索 AA 55 序列，找到后返回 OK，
 * 否则返回 T8_ERROR_BAD_FRAME。
 *
 * 设计考虑：
 *   64 字节的搜索窗口足够大，在 115200bps 下约为 5.5ms 的数据量。
 *   如果在此窗口内找不到帧头，说明通信严重不同步或硬件故障。
 *
 * @device: UART 设备句柄
 * @return: T8_OK（同步成功）或错误码
 */
static T8Status uart_wait_header(T8UartDevice *device)
{
    uint8_t byte;
    bool got_first = false;

    for (uint16_t i = 0u; i < 64u; ++i)
    {
        if (uart_read_exact(device, &byte, 1u) != T8_OK)
        {
            return T8_ERROR_TIMEOUT;
        }

        if (!got_first)
        {
            got_first = byte == T8_UART_HEAD0;
        }
        else if (byte == T8_UART_HEAD1)
        {
            return T8_OK;
        }
        else
        {
            got_first = byte == T8_UART_HEAD0;
        }
    }

    return T8_ERROR_BAD_FRAME;
}

/*
 * uart_receive_packet — 接收一个完整的 UART 协议帧（不含发送命令部分）。
 *
 * 协议帧结构（接收方向）：
 *   [帧头 AA55] [命令(1B)] [数据长度(1B)] [数据(NB)] [校验和(1B)] [帧尾 66]
 *
 * 解析步骤：
 *   1. uart_wait_header: 同步帧头 AA 55
 *   2. 读取命令码和数据长度
 *   3. 检查数据长度 ≤ T8_MAX_FRAME_SIZE（防止缓冲区溢出）
 *   4. 读取数据段 + 校验和 + 帧尾
 *   5. 验证帧尾是否为 0x66
 *   6. 检查命令码是否为特殊错误标志（0xEC 校验和错，0xEE 未知命令）
 *   7. 验证校验和
 *
 * 校验和算法：command + length + sum(data[0..length-1])
 *
 * @device: UART 设备句柄
 * @packet: 输出参数，接收解析后的数据包
 * @return: T8_OK（成功）或相应的错误码
 */
static T8Status uart_receive_packet(T8UartDevice *device, T8Packet *packet)
{
    uint8_t meta[2];
    uint8_t checksum;
    uint8_t tail;
    T8Status status;

    status = uart_wait_header(device);
    if (status != T8_OK)
    {
        return status;
    }

    status = uart_read_exact(device, meta, sizeof(meta));
    if (status != T8_OK)
    {
        return status;
    }

    packet->command = meta[0];
    packet->length = meta[1];
    if (packet->length > T8_MAX_FRAME_SIZE)
    {
        return T8_ERROR_BAD_FRAME;
    }

    status = uart_read_exact(device, packet->data, packet->length);
    if (status != T8_OK)
    {
        return status;
    }
    status = uart_read_exact(device, &checksum, 1u);
    if (status != T8_OK)
    {
        return status;
    }
    status = uart_read_exact(device, &tail, 1u);
    if (status != T8_OK)
    {
        return status;
    }
    if (tail != T8_UART_TAIL)
    {
        return T8_ERROR_BAD_FRAME;
    }

    if (packet->command == T8_UART_ERROR_UNKNOWN)
    {
        return T8_ERROR_UNKNOWN_COMMAND;
    }
    if (packet->command == T8_UART_ERROR_CHECKSUM)
    {
        return T8_ERROR_CHECKSUM;
    }

    if ((uint8_t)(packet->command + packet->length + t8_checksum(packet->data, packet->length)) != checksum)
    {
        return T8_ERROR_CHECKSUM;
    }

    return T8_OK;
}

/*
 * uart_send_frame — 发送一个完整的 UART 协议帧。
 *
 * 发送帧结构：
 *   [AA 55] [功能码] [负载长度] [命令] [数据...] [校验和] [66]
 *
 * 其中：
 *   负载长度 = 1（命令占位）+ data_length
 *   总帧长度 = 6 + 负载长度
 *   校验和 = 功能码 + 负载长度 + 命令 + 所有数据字节
 *
 * 发送前会清空 UART 的输入和输出缓冲区（如果提供了 flush 函数），
 * 以避免上一次通信的残留数据干扰本次请求。
 *
 * @device:      UART 设备句柄
 * @function:    功能码（0x00=读，0x01=写）
 * @command:     命令码
 * @data:        待发送的数据段（可为 NULL）
 * @data_length: 数据段长度
 * @return:      成功返回 T8_OK，参数错误返回 T8_ERROR_INVALID_ARG，
 *               I/O 错误返回 T8_ERROR_IO
 */
static T8Status uart_send_frame(T8UartDevice *device, uint8_t function,
    uint8_t command, const uint8_t *data, uint8_t data_length)
{
    uint8_t frame[T8_MAX_FRAME_SIZE];
    uint8_t frame_length;
    uint8_t payload_length;
    uint8_t checksum;
    size_t offset;

    if (device == 0 || device->transport.write == 0 || device->transport.read == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }

    payload_length = (uint8_t)(1u + data_length);
    frame_length = (uint8_t)(6u + payload_length);
    if (frame_length > sizeof(frame) || (data_length > 0u && data == 0))
    {
        return T8_ERROR_INVALID_ARG;
    }

    offset = 0u;
    frame[offset++] = T8_UART_HEAD0;
    frame[offset++] = T8_UART_HEAD1;
    frame[offset++] = function;
    frame[offset++] = payload_length;
    frame[offset++] = command;

    checksum = (uint8_t)(function + payload_length + command);
    for (uint8_t i = 0u; i < data_length; ++i)
    {
        frame[offset++] = data[i];
        checksum = (uint8_t)(checksum + data[i]);
    }

    frame[offset++] = checksum;
    frame[offset++] = T8_UART_TAIL;

    if (device->transport.flush_input != 0)
    {
        device->transport.flush_input(device->transport.user_data);
    }
    if (device->transport.flush_output != 0)
    {
        device->transport.flush_output(device->transport.user_data);
    }

    return (device->transport.write(frame, offset, device->transport.user_data) == offset) ? T8_OK : T8_ERROR_IO;
}

/*
 * read_single_u8_uart — UART 模式：读取单个 8 位值（通用内部函数）。
 *
 * 适用于以下命令：GRAY8_CHx、DIGITAL、I2C_ADDRESS、VERSION
 *
 * @device: UART 设备句柄
 * @command: 命令码
 * @value:   输出参数，读取到的 8 位值
 * @return:  T8_OK 或错误码
 */
static T8Status read_single_u8_uart(T8UartDevice *device, uint8_t command,
    uint8_t *value)
{
    T8Packet packet;
    T8Status status;
    if (value == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }
    status = t8_uart_read_packet(device, command, &packet);
    if (status == T8_OK)
    {
        if (packet.length != 1u)
        {
            return T8_ERROR_BAD_FRAME;
        }
        *value = packet.data[0];
    }
    return status;
}

/*
 * read_array_u8_uart — UART 模式：读取 8 通道 8 位值数组。
 *
 * 适用于：GRAY8_ALL、BLACK8_ALL、WHITE8_ALL
 * 传感器返回的响应中 data 段长度必须正好是 T8_SENSOR_COUNT（8 字节）。
 *
 * @device: UART 设备句柄
 * @command: 命令码（如 T8_CMD_GRAY8_ALL）
 * @values:  输出参数，8 元素 uint8_t 数组
 * @return:  T8_OK 或错误码
 */
static T8Status read_array_u8_uart(T8UartDevice *device, uint8_t command,
    uint8_t values[T8_SENSOR_COUNT])
{
    T8Packet packet;
    T8Status status;
    if (values == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }
    status = t8_uart_read_packet(device, command, &packet);
    if (status == T8_OK)
    {
        if (packet.length != T8_SENSOR_COUNT)
        {
            return T8_ERROR_BAD_FRAME;
        }
        copy_bytes(values, packet.data, T8_SENSOR_COUNT);
    }
    return status;
}

/*
 * read_single_u16_uart — UART 模式：读取单个 16 位 ADC 值。
 *
 * 适用于：ADC16_CHx（单通道 16 位 ADC）
 * 数据段应为 2 字节（小端序）。
 *
 * @device: UART 设备句柄
 * @command: 命令码
 * @value:   输出参数，读取到的 16 位无符号值
 * @return:  T8_OK 或错误码
 */
static T8Status read_single_u16_uart(T8UartDevice *device, uint8_t command,
    uint16_t *value)
{
    T8Packet packet;
    T8Status status;
    if (value == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }
    status = t8_uart_read_packet(device, command, &packet);
    if (status == T8_OK)
    {
        if (packet.length != 2u)
        {
            return T8_ERROR_BAD_FRAME;
        }
        *value = read_u16_le(packet.data);
    }
    return status;
}

/*
 * read_array_u16_uart — UART 模式：读取 8 通道 16 位 ADC 值数组。
 *
 * 适用于：ADC16_ALL、BLACK16_ALL、WHITE16_ALL
 * 传感器响应 data 段长度为 16 字节（8 通道 × 2 字节/通道），小端序。
 *
 * @device: UART 设备句柄
 * @command: 命令码
 * @values:  输出参数，8 元素 uint16_t 数组
 * @return:  T8_OK 或错误码
 */
static T8Status read_array_u16_uart(T8UartDevice *device, uint8_t command,
    uint16_t values[T8_SENSOR_COUNT])
{
    T8Packet packet;
    T8Status status;
    if (values == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }
    status = t8_uart_read_packet(device, command, &packet);
    if (status == T8_OK)
    {
        if (packet.length != T8_SENSOR_COUNT * 2u)
        {
            return T8_ERROR_BAD_FRAME;
        }
        for (uint8_t i = 0u; i < T8_SENSOR_COUNT; ++i)
        {
            values[i] = read_u16_le(&packet.data[i * 2u]);
        }
    }
    return status;
}

/*
 * i2c_read_packet_with_length — I2C 模式：读取指定长度的数据包。
 *
 * I2C 协议（比 UART 简洁，没有帧头和帧尾）：
 *   1. 主机发送 1 字节命令码到从机
 *   2. 主机发起读取，从机返回：
 *      [命令码(1B)] [数据长度(1B)] [数据(NB)] [校验和(1B)]
 *
 * 校验和：命令码 + 数据长度 + 所有数据字节
 *
 * 错误处理：
 *   - 响应首字节为 0xFF 表示未知命令
 *   - 命令码或数据长度不匹配返回 T8_ERROR_BAD_FRAME
 *   - 校验和不匹配返回 T8_ERROR_CHECKSUM
 *
 * @device: I2C 设备句柄
 * @command: 命令码
 * @length:  预期的数据段长度
 * @packet:  输出参数，接收解析后的数据包
 * @return:  T8_OK 或错误码
 */
static T8Status i2c_read_packet_with_length(T8I2cDevice *device,
    uint8_t command, uint8_t length, T8Packet *packet)
{
    uint8_t response[T8_MAX_FRAME_SIZE + 3u];
    size_t response_length = (size_t)length + 3u;

    if (device == 0 || packet == 0 || device->transport.write == 0 || device->transport.read == 0 || length > T8_MAX_FRAME_SIZE)
    {
        return T8_ERROR_INVALID_ARG;
    }

    if (!device->transport.write(device->address, &command, 1u, device->transport.user_data))
    {
        return T8_ERROR_IO;
    }
    if (!device->transport.read(device->address, response, response_length, device->timeout_ms, device->transport.user_data))
    {
        return T8_ERROR_TIMEOUT;
    }

    if (response[0] == 0xFFu)
    {
        return T8_ERROR_UNKNOWN_COMMAND;
    }
    if (response[0] != command || response[1] != length)
    {
        return T8_ERROR_BAD_FRAME;
    }
    if ((uint8_t)(response[0] + response[1] + t8_checksum(&response[2], length)) != response[response_length - 1u])
    {
        return T8_ERROR_CHECKSUM;
    }

    packet->command = response[0];
    packet->length = response[1];
    copy_bytes(packet->data, &response[2], length);
    return T8_OK;
}

/*
 * read_single_u8_i2c — I2C 模式：读取单个 8 位值。
 *
 * 适用于：GRAY8_CHx、DIGITAL、I2C_ADDRESS、VERSION
 * 内部调用 t8_i2c_read_packet 并提取 data[0]。
 *
 * @device: I2C 设备句柄
 * @command: 命令码
 * @value:   输出参数，读取到的 8 位值
 * @return:  T8_OK 或错误码
 */
static T8Status read_single_u8_i2c(T8I2cDevice *device, uint8_t command, uint8_t *value)
{
    T8Packet packet;
    T8Status status;
    if (value == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }
    status = t8_i2c_read_packet(device, command, &packet);
    if (status == T8_OK)
    {
        *value = packet.data[0];
    }
    return status;
}

/*
 * read_array_u8_i2c — I2C 模式：读取 8 通道 8 位值数组。
 *
 * 适用于：GRAY8_ALL、BLACK8_ALL、WHITE8_ALL
 *
 * @device: I2C 设备句柄
 * @command: 命令码
 * @values:  输出参数，8 元素 uint8_t 数组
 * @return:  T8_OK 或错误码
 */
static T8Status read_array_u8_i2c(T8I2cDevice *device, uint8_t command, uint8_t values[T8_SENSOR_COUNT])
{
    T8Packet packet;
    T8Status status;
    if (values == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }
    status = t8_i2c_read_packet(device, command, &packet);
    if (status == T8_OK)
    {
        copy_bytes(values, packet.data, T8_SENSOR_COUNT);
    }
    return status;
}

/*
 * read_single_u16_i2c — I2C 模式：读取单个 16 位 ADC 值。
 *
 * 适用于：ADC16_CHx
 *
 * @device: I2C 设备句柄
 * @command: 命令码
 * @value:   输出参数，读取到的 16 位无符号值
 * @return:  T8_OK 或错误码
 */
static T8Status read_single_u16_i2c(T8I2cDevice *device, uint8_t command, uint16_t *value)
{
    T8Packet packet;
    T8Status status;
    if (value == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }
    status = t8_i2c_read_packet(device, command, &packet);
    if (status == T8_OK)
    {
        *value = read_u16_le(packet.data);
    }
    return status;
}

/*
 * read_array_u16_i2c — I2C 模式：读取 8 通道 16 位 ADC 值数组。
 *
 * 适用于：ADC16_ALL、BLACK16_ALL、WHITE16_ALL
 * 从响应数据中逐通道提取小端序的 16 位值。
 *
 * @device: I2C 设备句柄
 * @command: 命令码
 * @values:  输出参数，8 元素 uint16_t 数组
 * @return:  T8_OK 或错误码
 */
static T8Status read_array_u16_i2c(T8I2cDevice *device, uint8_t command, uint16_t values[T8_SENSOR_COUNT])
{
    T8Packet packet;
    T8Status status;
    if (values == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }
    status = t8_i2c_read_packet(device, command, &packet);
    if (status == T8_OK)
    {
        for (uint8_t i = 0u; i < T8_SENSOR_COUNT; ++i)
        {
            values[i] = read_u16_le(&packet.data[i * 2u]);
        }
    }
    return status;
}

/*
 * t8_uart_init — 初始化 UART 模式的 T8 传感器设备。
 *
 * 从 transport 复制传输层函数指针到设备结构体中。
 * 如果 transport 为 NULL，则所有函数指针清 0（设备未就绪状态）。
 * 默认超时 100ms，最大重试 3 次。
 *
 * @device:   设备结构体指针
 * @transport: 传输层接口（函数指针 + user_data），可为 NULL
 */
void t8_uart_init(T8UartDevice *device, const T8UartTransport *transport)
{
    if (device == 0)
    {
        return;
    }

    if (transport != 0)
    {
        device->transport = *transport;
    }
    else
    {
        device->transport.write = 0;
        device->transport.read = 0;
        device->transport.flush_input = 0;
        device->transport.flush_output = 0;
        device->transport.delay_ms = 0;
        device->transport.user_data = 0;
    }
    device->timeout_ms = T8_UART_DEFAULT_TIMEOUT_MS;
    device->max_retries = T8_UART_MAX_RETRIES;
}

/*
 * t8_i2c_init — 初始化 I2C 模式的 T8 传感器设备。
 *
 * 从 transport 复制 I2C 传输层函数指针，设置 I2C 从机地址。
 * 默认超时 100ms。
 *
 * @device:   设备结构体指针
 * @transport: I2C 传输层接口（函数指针 + user_data），可为 NULL
 * @address:  I2C 从机 7 位地址（左对齐，如 0x40）
 */
void t8_i2c_init(T8I2cDevice *device, const T8I2cTransport *transport, uint8_t address)
{
    if (device == 0)
    {
        return;
    }

    if (transport != 0)
    {
        device->transport = *transport;
    }
    else
    {
        device->transport.write = 0;
        device->transport.read = 0;
        device->transport.user_data = 0;
    }
    device->address = address;
    device->timeout_ms = T8_I2C_DEFAULT_TIMEOUT_MS;
}

/*
 * t8_checksum — 计算 8 位累加和校验。
 *
 * T8 协议使用简单的 8 位累加和（sum of all bytes, truncated to 8 bits）。
 * 不是 CRC，没有多项式计算，但足以检测单比特错误和大多数突发错误。
 *
 * 注意：这里的计算不包含命令码和数据长度字段，调用者需自行包含这些字段。
 *
 * @data:   输入数据指针
 * @length: 数据长度
 * @return: 8 位累加和
 */
uint8_t t8_checksum(const uint8_t *data, size_t length)
{
    uint8_t sum = 0u;

    if (data == 0)
    {
        return 0u;
    }

    for (size_t i = 0u; i < length; ++i)
    {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

/*
 * t8_uart_read_packet — UART 模式：发送读命令并接收响应（含自动重试）。
 *
 * 这是 UART 模式最底层的公共 API，实现了"请求-应答"通信模式。
 * 发送一个 FUNCTION_READ 帧后等待响应。
 *
 * 自动重试机制：
 *   在 device->max_retries 次尝试内，如果接收到响应但命令码不匹配
 *   （如同步偏移导致收到错误的响应），会继续重试。
 *   但对于校验和错误和未知命令错误，则直接返回不再重试（避免无效重试）。
 *   如果所有重试都失败，返回 T8_ERROR_TIMEOUT。
 *
 * @device:  UART 设备句柄
 * @command: 要读取的命令码
 * @packet:  输出参数，接收响应数据包
 * @return:  T8_OK 或错误码
 */
T8Status t8_uart_read_packet(T8UartDevice *device, uint8_t command, T8Packet *packet)
{
    T8Status status;
    if (device == 0 || packet == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }

    for (uint8_t tries = 0u; tries < device->max_retries; ++tries)
    {
        status = uart_send_frame(device, T8_UART_FUNC_READ, command, 0, 0u);
        if (status != T8_OK)
        {
            return status;
        }

        status = uart_receive_packet(device, packet);
        if (status == T8_OK && packet->command == command)
        {
            return T8_OK;
        }
        if (status == T8_ERROR_CHECKSUM || status == T8_ERROR_UNKNOWN_COMMAND)
        {
            return status;
        }
    }

    return T8_ERROR_TIMEOUT;
}

/*
 * t8_uart_receive_packet — UART 模式：被动接收一个数据包（无发送步骤）。
 *
 * 该函数仅用于"连续发送模式"（Continuous Mode），
 * 在此模式下传感器主动定时发送数据，主控不需要发送请求帧，
 * 只需不断调用此函数接收即可。
 *
 * 连续发送模式由 t8_uart_start_continuous 启动，t8_uart_stop_continuous 停止。
 *
 * @device: UART 设备句柄
 * @packet: 输出参数，接收到的数据包
 * @return: T8_OK 或错误码
 */
T8Status t8_uart_receive_packet(T8UartDevice *device, T8Packet *packet)
{
    if (device == 0 || packet == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }
    return uart_receive_packet(device, packet);
}

/*
 * t8_uart_write_command — UART 模式：发送写命令并接收响应。
 *
 * 与 read 的差异：
 *   1. 功能码为 FUNCTION_WRITE（0x01）而非 FUNCTION_READ（0x00）
 *   2. 发送帧中带数据段
 *   3. 响应帧的命令码必须与发送的命令码一致
 *
 * @device: UART 设备句柄
 * @command: 命令码
 * @data:    待写入的数据段
 * @length:  数据段长度
 * @packet:  输出参数，接收响应数据包
 * @return:  T8_OK 或错误码
 */
T8Status t8_uart_write_command(T8UartDevice *device, uint8_t command, const uint8_t *data, uint8_t length, T8Packet *packet)
{
    T8Status status;
    if (packet == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }
    status = uart_send_frame(device, T8_UART_FUNC_WRITE, command, data, length);
    if (status != T8_OK)
    {
        return status;
    }
    status = uart_receive_packet(device, packet);
    if (status != T8_OK)
    {
        return status;
    }
    return packet->command == command ? T8_OK : T8_ERROR_BAD_FRAME;
}

/*
 * t8_uart_start_continuous — UART 模式：启动传感器的连续发送模式。
 *
 * 在连续模式下，传感器会以指定的时间间隔自动发送数据帧，
 * 主控只需调用 t8_uart_receive_packet 接收即可，无需反复发送请求。
 * 这种模式适合需要固定频率采集数据的场景（如循迹控制）。
 *
 * @device:          UART 设备句柄
 * @command:         要连续上报的命令码（如 T8_CMD_GRAY8_ALL）
 * @period_units_10ms: 发送周期，单位 10ms（如 5=50ms）
 * @return:          T8_OK 或错误码
 */
T8Status t8_uart_start_continuous(T8UartDevice *device, uint8_t command, uint8_t period_units_10ms)
{
    return uart_send_frame(device, T8_UART_FUNC_WRITE, command, &period_units_10ms, 1u);
}

/*
 * t8_uart_stop_continuous — UART 模式：停止连续发送模式。
 *
 * 发送 STOP_CONTINUOUS（0xFF）命令，传感器退出连续模式，
 * 恢复为请求-应答模式。
 *
 * @device: UART 设备句柄
 * @return: T8_OK 或 I/O 错误码
 */
T8Status t8_uart_stop_continuous(T8UartDevice *device)
{
    return uart_send_frame(device, T8_UART_FUNC_WRITE, T8_CMD_STOP_CONTINUOUS, 0, 0u);
}

/*
 * t8_uart_set_i2c_address — 通过 UART 设置传感器的 I2C 地址。
 *
 * T8 传感器可通过此 UART 命令修改其 I2C 地址，对地址范围有严格限制。
 * 设置后传感器会返回生效后的地址（可能与请求的地址不同）。
 *
 * @device:           UART 设备句柄
 * @address:          请求的新 I2C 地址（需在 T8_MIN..T8_MAX 范围内）
 * @effective_address: 输出参数，实际生效的 I2C 地址（可为 NULL）
 * @return:           T8_OK 或错误码（地址越界返回 T8_ERROR_INVALID_ARG）
 */
T8Status t8_uart_set_i2c_address(T8UartDevice *device, uint8_t address, uint8_t *effective_address)
{
    T8Packet packet;
    T8Status status;
    if (address < T8_MIN_I2C_ADDRESS || address > T8_MAX_I2C_ADDRESS)
    {
        return T8_ERROR_INVALID_ARG;
    }
    status = t8_uart_write_command(device, T8_CMD_I2C_ADDRESS, &address, 1u, &packet);
    if (status == T8_OK)
    {
        if (packet.length != 1u)
        {
            return T8_ERROR_BAD_FRAME;
        }
        if (effective_address != 0)
        {
            *effective_address = packet.data[0];
        }
    }
    return status;
}

/*==================== UART 模式高层 API ====================*/

/*
 * t8_uart_get_i2c_address — 通过 UART 读取传感器当前的 I2C 地址。
 *
 * @device:  UART 设备句柄
 * @address: 输出参数，当前 I2C 地址
 * @return:  T8_OK 或错误码
 */
T8Status t8_uart_get_i2c_address(T8UartDevice *device, uint8_t *address)
{
    return read_single_u8_uart(device, T8_CMD_I2C_ADDRESS, address);
}

/*
 * t8_uart_get_version — 读取传感器固件版本。
 *
 * 版本号以单字节返回（如 0x10 = 1.0）。
 *
 * @device:  UART 设备句柄
 * @version: 输出参数，固件版本号
 * @return:  T8_OK 或错误码
 */
T8Status t8_uart_get_version(T8UartDevice *device, uint8_t *version)
{
    return read_single_u8_uart(device, T8_CMD_VERSION, version);
}

/*
 * t8_uart_get_gray8 — 读取单个通道的 8 位灰度值。
 *
 * @device:  UART 设备句柄
 * @channel: 通道号（1~8）
 * @value:   输出参数，8 位灰度值（0~255）
 * @return:  T8_OK 或错误码（通道号越界返回 T8_ERROR_INVALID_ARG）
 */
T8Status t8_uart_get_gray8(T8UartDevice *device, uint8_t channel, uint8_t *value)
{
    if (channel == 0u || channel > T8_SENSOR_COUNT)
    {
        return T8_ERROR_INVALID_ARG;
    }
    return read_single_u8_uart(device, (uint8_t)(T8_CMD_GRAY8_CH1 + channel - 1u), value);
}

/*
 * t8_uart_get_gray8_all — 读取全部 8 通道的 8 位灰度值。
 *
 * @device: UART 设备句柄
 * @values: 输出参数，8 元素数组，按 CH1~CH8 顺序填充
 * @return: T8_OK 或错误码
 */
T8Status t8_uart_get_gray8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_uart(device, T8_CMD_GRAY8_ALL, values);
}

/*
 * t8_uart_get_black8_all — 读取全部 8 通道的黑标定值。
 *
 * 黑标定值是在传感器覆盖黑色表面时采集的参考值，
 * 用于二值化阈值计算（通常取黑白标定值的中点）。
 *
 * @device: UART 设备句柄
 * @values: 输出参数，8 元素数组
 * @return: T8_OK 或错误码
 */
T8Status t8_uart_get_black8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_uart(device, T8_CMD_BLACK8_ALL, values);
}

/*
 * t8_uart_get_white8_all — 读取全部 8 通道的白标定值。
 *
 * @device: UART 设备句柄
 * @values: 输出参数，8 元素数组
 * @return: T8_OK 或错误码
 */
T8Status t8_uart_get_white8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_uart(device, T8_CMD_WHITE8_ALL, values);
}

/*
 * t8_uart_get_digital — 读取二值化结果。
 *
 * 返回的 8 位值中，位 0~7 分别对应 CH1~CH8：
 *   0 = 白色（高于阈值）
 *   1 = 黑色（低于阈值）
 * 阈值由传感器的自动标定或手动设定决定。
 *
 * @device: UART 设备句柄
 * @bits:   输出参数，二值化结果位图
 * @return: T8_OK 或错误码
 */
T8Status t8_uart_get_digital(T8UartDevice *device, uint8_t *bits)
{
    return read_single_u8_uart(device, T8_CMD_DIGITAL, bits);
}

/*
 * t8_uart_get_adc16 — 读取单个通道的 16 位高精度 ADC 值。
 *
 * 16 位 ADC 提供比 8 位灰度更高的分辨率，适合需要精细灰度识别的场景。
 * 返回值范围 0~65535。
 *
 * @device:  UART 设备句柄
 * @channel: 通道号（1~8）
 * @value:   输出参数，16 位 ADC 值
 * @return:  T8_OK 或错误码
 */
T8Status t8_uart_get_adc16(T8UartDevice *device, uint8_t channel, uint16_t *value)
{
    if (channel == 0u || channel > T8_SENSOR_COUNT)
    {
        return T8_ERROR_INVALID_ARG;
    }
    return read_single_u16_uart(device, (uint8_t)(T8_CMD_ADC16_CH1 + channel - 1u), value);
}

/*
 * t8_uart_get_adc16_all — 读取全部 8 通道的 16 位 ADC 值。
 *
 * @device: UART 设备句柄
 * @values: 输出参数，8 元素 uint16_t 数组
 * @return: T8_OK 或错误码
 */
T8Status t8_uart_get_adc16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_uart(device, T8_CMD_ADC16_ALL, values);
}

/*
 * t8_uart_get_black16_all — 读取全部 8 通道的 16 位黑标定值。
 *
 * @device: UART 设备句柄
 * @values: 输出参数，8 元素 uint16_t 数组
 * @return: T8_OK 或错误码
 */
T8Status t8_uart_get_black16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_uart(device, T8_CMD_BLACK16_ALL, values);
}

/*
 * t8_uart_get_white16_all — 读取全部 8 通道的 16 位白标定值。
 *
 * @device: UART 设备句柄
 * @values: 输出参数，8 元素 uint16_t 数组
 * @return: T8_OK 或错误码
 */
T8Status t8_uart_get_white16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_uart(device, T8_CMD_WHITE16_ALL, values);
}

/*==================== I2C 模式高层 API ====================*/

/*
 * t8_i2c_read_packet — I2C 模式：读取数据包。
 *
 * 根据命令码自动确定预期数据长度，调用内部函数完成 I2C 通信。
 * 如果不支持的命令码（无法确定长度），返回 T8_ERROR_UNKNOWN_COMMAND。
 *
 * @device: I2C 设备句柄
 * @command: 命令码
 * @packet:  输出参数，接收响应数据包
 * @return:  T8_OK 或错误码
 */
T8Status t8_i2c_read_packet(T8I2cDevice *device, uint8_t command, T8Packet *packet)
{
    size_t length = command_data_length(command);
    if (length == 0u || length > UINT8_MAX)
    {
        return T8_ERROR_UNKNOWN_COMMAND;
    }
    return i2c_read_packet_with_length(device, command, (uint8_t)length, packet);
}

/*
 * t8_i2c_get_gray8 — I2C 模式：读取单个通道 8 位灰度值。
 *
 * @device:  I2C 设备句柄
 * @channel: 通道号（1~8）
 * @value:   输出参数，8 位灰度值
 * @return:  T8_OK 或错误码
 */
T8Status t8_i2c_get_gray8(T8I2cDevice *device, uint8_t channel, uint8_t *value)
{
    if (channel == 0u || channel > T8_SENSOR_COUNT)
    {
        return T8_ERROR_INVALID_ARG;
    }
    return read_single_u8_i2c(device, (uint8_t)(T8_CMD_GRAY8_CH1 + channel - 1u), value);
}

/*
 * t8_i2c_get_gray8_all — I2C 模式：读取全部 8 通道 8 位灰度值。
 *
 * @device: I2C 设备句柄
 * @values: 输出参数，8 元素 uint8_t 数组
 * @return: T8_OK 或错误码
 */
T8Status t8_i2c_get_gray8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_i2c(device, T8_CMD_GRAY8_ALL, values);
}

/*
 * t8_i2c_get_black8_all — I2C 模式：读取全部 8 通道黑标定值。
 *
 * @device: I2C 设备句柄
 * @values: 输出参数，8 元素 uint8_t 数组
 * @return: T8_OK 或错误码
 */
T8Status t8_i2c_get_black8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_i2c(device, T8_CMD_BLACK8_ALL, values);
}

/*
 * t8_i2c_get_white8_all — I2C 模式：读取全部 8 通道白标定值。
 *
 * @device: I2C 设备句柄
 * @values: 输出参数，8 元素 uint8_t 数组
 * @return: T8_OK 或错误码
 */
T8Status t8_i2c_get_white8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_i2c(device, T8_CMD_WHITE8_ALL, values);
}

/*
 * t8_i2c_get_digital — I2C 模式：读取二值化结果。
 *
 * 返回的 8 位位图中，每个 bit 对应一个通道，
 * 0=白/1=黑（与传感器阈值比较后的结果）。
 *
 * @device: I2C 设备句柄
 * @bits:   输出参数，二值化结果
 * @return: T8_OK 或错误码
 */
T8Status t8_i2c_get_digital(T8I2cDevice *device, uint8_t *bits)
{
    return read_single_u8_i2c(device, T8_CMD_DIGITAL, bits);
}

/*
 * t8_compute_position — 从8通道灰度数据计算二值位和连续浮点位置。
 *
 * 灰度→黑暗度反色后做加权质心，位宽映射到 -7.0 ~ +7.0。
 * 同时以 128 为阈值二值化输出供事件检测使用。
 *
 * @gray:     8 通道灰度值 [0-255]，低=黑
 * @bits_out: 输出二值位（0=黑线，全白=0xFF）
 * @pos_out:  输出连续浮点位置（-7.0~+7.0），全白=99.0
 */
void t8_compute_position(const uint8_t gray[T8_SENSOR_COUNT],
    uint8_t *bits_out, float *pos_out)
{
    float sum_weighted = 0.0f;
    float sum_weight = 0.0f;
    uint8_t bits = 0xFFu;
    int i;

    for (i = 0; i < 8; i++)
    {
        const float channel_pos = (float)(i * 2 - 7);
        float darkness = 255.0f - (float)gray[i];
        if (darkness < 0.0f) darkness = 0.0f;

        sum_weighted += darkness * channel_pos;
        sum_weight += darkness;

        if (gray[i] < 128u)
        {
            bits &= (uint8_t)~(1u << i);
        }
    }

    *bits_out = bits;
    if (sum_weight > 0.0f)
    {
        *pos_out = sum_weighted / sum_weight;
    }
    else
    {
        *pos_out = 99.0f;
    }
}

/*
 * t8_i2c_get_adc16 — I2C 模式：读取单个通道 16 位 ADC 值。
 *
 * @device:  I2C 设备句柄
 * @channel: 通道号（1~8）
 * @value:   输出参数，16 位 ADC 值
 * @return:  T8_OK 或错误码
 */
T8Status t8_i2c_get_adc16(T8I2cDevice *device, uint8_t channel, uint16_t *value)
{
    if (channel == 0u || channel > T8_SENSOR_COUNT)
    {
        return T8_ERROR_INVALID_ARG;
    }
    return read_single_u16_i2c(device, (uint8_t)(T8_CMD_ADC16_CH1 + channel - 1u), value);
}

/*
 * t8_i2c_get_adc16_all — I2C 模式：读取全部 8 通道 16 位 ADC 值。
 *
 * @device: I2C 设备句柄
 * @values: 输出参数，8 元素 uint16_t 数组
 * @return: T8_OK 或错误码
 */
T8Status t8_i2c_get_adc16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_i2c(device, T8_CMD_ADC16_ALL, values);
}

/*
 * t8_i2c_get_black16_all — I2C 模式：读取全部 8 通道 16 位黑标定值。
 *
 * @device: I2C 设备句柄
 * @values: 输出参数，8 元素 uint16_t 数组
 * @return: T8_OK 或错误码
 */
T8Status t8_i2c_get_black16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_i2c(device, T8_CMD_BLACK16_ALL, values);
}

/*
 * t8_i2c_get_white16_all — I2C 模式：读取全部 8 通道 16 位白标定值。
 *
 * @device: I2C 设备句柄
 * @values: 输出参数，8 元素 uint16_t 数组
 * @return: T8_OK 或错误码
 */
T8Status t8_i2c_get_white16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_i2c(device, T8_CMD_WHITE16_ALL, values);
}
