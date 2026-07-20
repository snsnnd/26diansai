#include "gimbal/serial_rx_buffer.h"

static size_t serial_rx_buffer_next(const SerialRxBuffer *buffer, size_t index)
{
    return (index + 1u) % buffer->capacity;
}

void serial_rx_buffer_init(SerialRxBuffer *buffer, uint8_t *storage, size_t capacity)
{
    if (buffer == NULL)
    {
        return;
    }

    buffer->data = storage;
    buffer->capacity = capacity;
    buffer->head = 0u;
    buffer->tail = 0u;
    buffer->overflow_count = 0u;
}

void serial_rx_buffer_clear(SerialRxBuffer *buffer)
{
    if (buffer == NULL)
    {
        return;
    }

    buffer->head = 0u;
    buffer->tail = 0u;
}

bool serial_rx_buffer_push(SerialRxBuffer *buffer, uint8_t byte)
{
    size_t next;

    if (buffer == NULL || buffer->data == NULL || buffer->capacity < 2u)
    {
        return false;
    }

    next = serial_rx_buffer_next(buffer, buffer->head);
    if (next == buffer->tail)
    {
        buffer->tail = serial_rx_buffer_next(buffer, buffer->tail);
        buffer->overflow_count++;
    }

    buffer->data[buffer->head] = byte;
    buffer->head = next;
    return true;
}

bool serial_rx_buffer_pop(SerialRxBuffer *buffer, uint8_t *byte)
{
    if (buffer == NULL || byte == NULL || buffer->head == buffer->tail || buffer->data == NULL)
    {
        return false;
    }

    *byte = buffer->data[buffer->tail];
    buffer->tail = serial_rx_buffer_next(buffer, buffer->tail);
    return true;
}

bool serial_rx_buffer_peek(const SerialRxBuffer *buffer, size_t offset, uint8_t *byte)
{
    size_t index;

    if (buffer == NULL || byte == NULL || buffer->data == NULL || offset >= serial_rx_buffer_available(buffer))
    {
        return false;
    }

    index = (buffer->tail + offset) % buffer->capacity;
    *byte = buffer->data[index];
    return true;
}

size_t serial_rx_buffer_available(const SerialRxBuffer *buffer)
{
    if (buffer == NULL || buffer->data == NULL || buffer->capacity == 0u)
    {
        return 0u;
    }

    if (buffer->head >= buffer->tail)
    {
        return buffer->head - buffer->tail;
    }

    return buffer->capacity - buffer->tail + buffer->head;
}

size_t serial_rx_buffer_overflow_count(const SerialRxBuffer *buffer)
{
    return (buffer == NULL) ? 0u : buffer->overflow_count;
}

void serial_rx_buffer_drop(SerialRxBuffer *buffer, size_t length)
{
    size_t available;

    if (buffer == NULL || buffer->data == NULL || buffer->capacity == 0u)
    {
        return;
    }

    available = serial_rx_buffer_available(buffer);
    if (length > available)
    {
        length = available;
    }

    buffer->tail = (buffer->tail + length) % buffer->capacity;
}
