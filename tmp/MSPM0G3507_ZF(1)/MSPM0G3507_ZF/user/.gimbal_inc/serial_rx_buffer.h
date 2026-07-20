#ifndef SERIAL_RX_BUFFER_H
#define SERIAL_RX_BUFFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint8_t *data;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t overflow_count;
} SerialRxBuffer;

void serial_rx_buffer_init(SerialRxBuffer *buffer, uint8_t *storage, size_t capacity);
void serial_rx_buffer_clear(SerialRxBuffer *buffer);
bool serial_rx_buffer_push(SerialRxBuffer *buffer, uint8_t byte);
bool serial_rx_buffer_pop(SerialRxBuffer *buffer, uint8_t *byte);
bool serial_rx_buffer_peek(const SerialRxBuffer *buffer, size_t offset, uint8_t *byte);
size_t serial_rx_buffer_available(const SerialRxBuffer *buffer);
size_t serial_rx_buffer_overflow_count(const SerialRxBuffer *buffer);
void serial_rx_buffer_drop(SerialRxBuffer *buffer, size_t length);

#endif
