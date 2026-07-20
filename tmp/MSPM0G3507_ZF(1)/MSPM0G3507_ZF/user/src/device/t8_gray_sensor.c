#include "device/t8_gray_sensor.h"

#define T8_UART_HEAD0 0xAAu
#define T8_UART_HEAD1 0x55u
#define T8_UART_TAIL 0x66u
#define T8_UART_FUNC_READ 0x00u
#define T8_UART_FUNC_WRITE 0x01u
#define T8_UART_ERROR_CHECKSUM 0xECu
#define T8_UART_ERROR_UNKNOWN 0xEEu
#define T8_UART_MAX_RETRIES 3u
#define T8_UART_DEFAULT_TIMEOUT_MS 100u
#define T8_I2C_DEFAULT_TIMEOUT_MS 100u

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

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static void copy_bytes(uint8_t *dst, const uint8_t *src, size_t length)
{
    for (size_t i = 0u; i < length; ++i)
    {
        dst[i] = src[i];
    }
}

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

static T8Status uart_send_frame(T8UartDevice *device, uint8_t function, uint8_t command, const uint8_t *data, uint8_t data_length)
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

static T8Status read_single_u8_uart(T8UartDevice *device, uint8_t command, uint8_t *value)
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

static T8Status read_array_u8_uart(T8UartDevice *device, uint8_t command, uint8_t values[T8_SENSOR_COUNT])
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

static T8Status read_single_u16_uart(T8UartDevice *device, uint8_t command, uint16_t *value)
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

static T8Status read_array_u16_uart(T8UartDevice *device, uint8_t command, uint16_t values[T8_SENSOR_COUNT])
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

static T8Status i2c_read_packet_with_length(T8I2cDevice *device, uint8_t command, uint8_t length, T8Packet *packet)
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

T8Status t8_uart_receive_packet(T8UartDevice *device, T8Packet *packet)
{
    if (device == 0 || packet == 0)
    {
        return T8_ERROR_INVALID_ARG;
    }
    return uart_receive_packet(device, packet);
}

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

T8Status t8_uart_start_continuous(T8UartDevice *device, uint8_t command, uint8_t period_units_10ms)
{
    return uart_send_frame(device, T8_UART_FUNC_WRITE, command, &period_units_10ms, 1u);
}

T8Status t8_uart_stop_continuous(T8UartDevice *device)
{
    return uart_send_frame(device, T8_UART_FUNC_WRITE, T8_CMD_STOP_CONTINUOUS, 0, 0u);
}

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

T8Status t8_uart_get_i2c_address(T8UartDevice *device, uint8_t *address)
{
    return read_single_u8_uart(device, T8_CMD_I2C_ADDRESS, address);
}

T8Status t8_uart_get_version(T8UartDevice *device, uint8_t *version)
{
    return read_single_u8_uart(device, T8_CMD_VERSION, version);
}

T8Status t8_uart_get_gray8(T8UartDevice *device, uint8_t channel, uint8_t *value)
{
    if (channel == 0u || channel > T8_SENSOR_COUNT)
    {
        return T8_ERROR_INVALID_ARG;
    }
    return read_single_u8_uart(device, (uint8_t)(T8_CMD_GRAY8_CH1 + channel - 1u), value);
}

T8Status t8_uart_get_gray8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_uart(device, T8_CMD_GRAY8_ALL, values);
}

T8Status t8_uart_get_black8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_uart(device, T8_CMD_BLACK8_ALL, values);
}

T8Status t8_uart_get_white8_all(T8UartDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_uart(device, T8_CMD_WHITE8_ALL, values);
}

T8Status t8_uart_get_digital(T8UartDevice *device, uint8_t *bits)
{
    return read_single_u8_uart(device, T8_CMD_DIGITAL, bits);
}

T8Status t8_uart_get_adc16(T8UartDevice *device, uint8_t channel, uint16_t *value)
{
    if (channel == 0u || channel > T8_SENSOR_COUNT)
    {
        return T8_ERROR_INVALID_ARG;
    }
    return read_single_u16_uart(device, (uint8_t)(T8_CMD_ADC16_CH1 + channel - 1u), value);
}

T8Status t8_uart_get_adc16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_uart(device, T8_CMD_ADC16_ALL, values);
}

T8Status t8_uart_get_black16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_uart(device, T8_CMD_BLACK16_ALL, values);
}

T8Status t8_uart_get_white16_all(T8UartDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_uart(device, T8_CMD_WHITE16_ALL, values);
}

T8Status t8_i2c_read_packet(T8I2cDevice *device, uint8_t command, T8Packet *packet)
{
    size_t length = command_data_length(command);
    if (length == 0u || length > UINT8_MAX)
    {
        return T8_ERROR_UNKNOWN_COMMAND;
    }
    return i2c_read_packet_with_length(device, command, (uint8_t)length, packet);
}

T8Status t8_i2c_get_gray8(T8I2cDevice *device, uint8_t channel, uint8_t *value)
{
    if (channel == 0u || channel > T8_SENSOR_COUNT)
    {
        return T8_ERROR_INVALID_ARG;
    }
    return read_single_u8_i2c(device, (uint8_t)(T8_CMD_GRAY8_CH1 + channel - 1u), value);
}

T8Status t8_i2c_get_gray8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_i2c(device, T8_CMD_GRAY8_ALL, values);
}

T8Status t8_i2c_get_black8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_i2c(device, T8_CMD_BLACK8_ALL, values);
}

T8Status t8_i2c_get_white8_all(T8I2cDevice *device, uint8_t values[T8_SENSOR_COUNT])
{
    return read_array_u8_i2c(device, T8_CMD_WHITE8_ALL, values);
}

T8Status t8_i2c_get_digital(T8I2cDevice *device, uint8_t *bits)
{
    return read_single_u8_i2c(device, T8_CMD_DIGITAL, bits);
}

T8Status t8_i2c_get_adc16(T8I2cDevice *device, uint8_t channel, uint16_t *value)
{
    if (channel == 0u || channel > T8_SENSOR_COUNT)
    {
        return T8_ERROR_INVALID_ARG;
    }
    return read_single_u16_i2c(device, (uint8_t)(T8_CMD_ADC16_CH1 + channel - 1u), value);
}

T8Status t8_i2c_get_adc16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_i2c(device, T8_CMD_ADC16_ALL, values);
}

T8Status t8_i2c_get_black16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_i2c(device, T8_CMD_BLACK16_ALL, values);
}

T8Status t8_i2c_get_white16_all(T8I2cDevice *device, uint16_t values[T8_SENSOR_COUNT])
{
    return read_array_u16_i2c(device, T8_CMD_WHITE16_ALL, values);
}
