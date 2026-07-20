/*********************************************************************************************************************
 * serial_tx_buffer.h — 串口发送环形缓冲区
 *
 * 提供基于环形缓冲区的异步串口发送机制。
 * 主循环（生产者）向缓冲区写入待发送数据，
 * DMA 或 ISR（消费者）从缓冲区取出数据并通过 UART 发送。
 *
 * 与 SerialRxBuffer 的设计对称，但提供了额外的诊断信息：
 *   - rejected_write_count: 因缓冲区满被拒绝的写入请求次数
 *   - dropped_byte_count:   被丢弃的总字节数
 *   - high_watermark:       历史最高占用率（用于优化缓冲区大小）
 ********************************************************************************************************************/

#ifndef LIB_SERIAL_TX_BUFFER_H
#define LIB_SERIAL_TX_BUFFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * SerialTxBuffer — 串口发送环形缓冲区结构体。
 *
 * head: 写入位置（主循环/生产者修改）
 * tail: 读取/发送位置（DMA/ISR/消费者修改）
 * data[tail] 是下一个待发送的字节。
 *
 * @data:                字节存储区起始地址
 * @capacity:            缓冲区容量
 * @head:                下一个写入位置的索引（生产者修改，volatile 防止编译器优化）
 * @tail:                下一个读出/发送位置的索引（消费者修改，volatile）
 * @rejected_write_count:写入被拒绝的次数（缓冲区满时递增）
 * @dropped_byte_count:  被丢弃的总字节数（每次写入失败时累加 length）
 * @high_watermark:      历史最高占用字节数，用于评估缓冲区是否够大
 */
typedef struct
{
    uint8_t *data;
    size_t capacity;
    volatile size_t head;
    volatile size_t tail;
    volatile size_t rejected_write_count;
    volatile size_t dropped_byte_count;
    volatile size_t high_watermark;
} SerialTxBuffer;

/* 初始化发送缓冲区，提供外部存储区和容量 */
void serial_tx_buffer_init(SerialTxBuffer *buffer, uint8_t *storage,
    size_t capacity);

/* 向缓冲区写入指定长度的数据。长度超过剩余空间时返回 false */
bool serial_tx_buffer_write(SerialTxBuffer *buffer, const uint8_t *data,
    size_t length);

/* 偷看下一个待发送的字节（不移除） */
bool serial_tx_buffer_peek(const SerialTxBuffer *buffer, uint8_t *byte);

/* 弹出下一个待发送的字节（仅移除，不返回数据） */
bool serial_tx_buffer_pop(SerialTxBuffer *buffer);

/* 查询缓冲区中待发送的数据字节数 */
size_t serial_tx_buffer_available(const SerialTxBuffer *buffer);

/* 查询缓冲区中剩余的可用空间（字节数） */
size_t serial_tx_buffer_free(const SerialTxBuffer *buffer);

/* 查询被拒绝写入的次数 */
size_t serial_tx_buffer_rejected_write_count(const SerialTxBuffer *buffer);

/* 查询被丢弃的总字节数 */
size_t serial_tx_buffer_dropped_byte_count(const SerialTxBuffer *buffer);

/* 查询历史最高水位线（用于诊断缓冲区大小是否合理） */
size_t serial_tx_buffer_high_watermark(const SerialTxBuffer *buffer);

#endif
