/*********************************************************************************************************************
 * serial_rx_buffer.h — 串口接收环形缓冲区（线程安全）
 *
 * ISR（中断服务程序）向缓冲区写入数据，主循环从缓冲区读取数据。
 * 使用 volatile 关键字确保编译器在读取 head/tail 时不会做寄存器优化，
 * 从而保证 ISR 和主循环之间的数据一致性。
 *
 * head: 写入位置（仅 ISR 修改）
 * tail: 读取位置（仅主循环修改）
 * 当 head == tail 时缓冲区为空。
 *
 * 支持带时间戳的 push/pop 变体，可用于记录每个字节的接收时刻，
 * 对分析通信抖动和超时处理非常有用。
 ********************************************************************************************************************/

#ifndef LIB_SERIAL_RX_BUFFER_H
#define LIB_SERIAL_RX_BUFFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * SerialRxBuffer — 串口接收环形缓冲区结构体。
 *
 * 经典的生产者-消费者环形缓冲区模型：
 *   - 生产者（UART ISR）调用 push 向 head 位置写入数据
 *   - 消费者（主循环）调用 pop 从 tail 位置读取数据
 *   - 数据存储指针由外部提供，避免动态内存分配
 *
 * @data:            字节存储区起始地址
 * @capacity:        缓冲区容量（有效数据量最大为 capacity-1，因为空/满判断需要空一格）
 * @head:            下一个写入位置的索引（生产者修改，volatile 防止 ISR 优化）
 * @tail:            下一个读取位置的索引（消费者修改，volatile 防止编译器优化）
 * @overflow_count:  溢出计数——当缓冲区满而仍有数据推入时递增
 * @timestamps_ms:   可选的时间戳数组，与 data 一一对应，记录每个字节的接收时刻（ms）
 */
typedef struct
{
    uint8_t *data;
    size_t capacity;
    volatile size_t head;
    volatile size_t tail;
    volatile size_t overflow_count;
    uint32_t *timestamps_ms;
} SerialRxBuffer;

/* 初始化接收缓冲区（无时间戳版本） */
void serial_rx_buffer_init(SerialRxBuffer *buffer, uint8_t *storage, size_t capacity);

/* 初始化接收缓冲区（带时间戳数组版本） */
void serial_rx_buffer_init_timed(SerialRxBuffer *buffer, uint8_t *storage,
                                 uint32_t *timestamp_storage, size_t capacity);

/* 清空缓冲区：将 tail 移动到 head，等效于丢弃所有数据 */
void serial_rx_buffer_clear(SerialRxBuffer *buffer);

/* 向缓冲区推入一个字节（ISR 中调用）。满时返回 false 并递增 overflow_count */
bool serial_rx_buffer_push(SerialRxBuffer *buffer, uint8_t byte);

/* 同 push，但额外记录接收时刻的时间戳 */
bool serial_rx_buffer_push_timed(SerialRxBuffer *buffer, uint8_t byte, uint32_t rx_time_ms);

/* 从缓冲区弹出一个字节（主循环调用）。空时返回 false */
bool serial_rx_buffer_pop(SerialRxBuffer *buffer, uint8_t *byte);

/* 同 pop，但额外获取该字节的时间戳 */
bool serial_rx_buffer_pop_timed(SerialRxBuffer *buffer, uint8_t *byte, uint32_t *rx_time_ms);

/* 偷窥偏移 offset 处的字节而不移除它（不修改 tail） */
bool serial_rx_buffer_peek(const SerialRxBuffer *buffer, size_t offset, uint8_t *byte);

/* 查询缓冲区中当前可读的字节数 */
size_t serial_rx_buffer_available(const SerialRxBuffer *buffer);

/* 查询自初始化以来发生的溢出次数 */
size_t serial_rx_buffer_overflow_count(const SerialRxBuffer *buffer);

/* 丢弃指定长度的数据（从 tail 开始向前移） */
void serial_rx_buffer_drop(SerialRxBuffer *buffer, size_t length);

#endif
