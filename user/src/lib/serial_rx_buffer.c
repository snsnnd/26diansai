/*********************************************************************************************************************
 * serial_rx_buffer.c — 串口接收环形缓冲区实现
 *
 * 实现基于环形缓冲区的 UART 数据接收机制。
 * 关键设计要点：
 *   1. 使用 data 数组存储字节，用头尾指针管理读写位置
 *   2. 判别满条件：(head+1) % capacity == tail，即留一个空位以区分空和满
 *   3. volatile 修饰 head/tail，确保 ISR 和主循环之间的可见性
 *   4. 使用 __DMB() 数据内存屏障指令确保多总线（AHB/APB）上的数据一致性
 *   5. 可选的时间戳存储用于调试通信时序
 ********************************************************************************************************************/

#include "lib/serial_rx_buffer.h"
#include "zf_common_headfile.h"

/*
 * serial_rx_buffer_next — 环形缓冲区索引步进。
 *
 * 计算给定 index 的下一个位置，到达 capacity 时回绕到 0。
 * 这是环形缓冲区的核心操作，使用取模运算实现回绕。
 *
 * @buffer: 缓冲区指针（用于获取 capacity）
 * @index:  当前位置
 * @return: 下一个位置索引
 */
static size_t serial_rx_buffer_next(const SerialRxBuffer *buffer, size_t index)
{
    return (index + 1u) % buffer->capacity;
}

/*
 * serial_rx_buffer_init — 初始化接收缓冲区（简洁版本）。
 *
 * 不提供时间戳存储，内部调用 init_timed 并传入 NULL 作为时间戳指针。
 *
 * @storage:  外部提供的字节数组（必须保证生命周期）
 * @capacity: 缓冲区容量（实际可用为 capacity-1）
 */
void serial_rx_buffer_init(SerialRxBuffer *buffer, uint8_t *storage, size_t capacity)
{
    serial_rx_buffer_init_timed(buffer, storage, NULL, capacity);
}

/*
 * serial_rx_buffer_init_timed — 初始化接收缓冲区（完整版本）。
 *
 * 设置所有字段的初始值：head=tail=0，overflow_count=0。
 * 如果 timestamp_storage 不为 NULL，则 push_timed 会存储时间戳。
 *
 * @buffer:            缓冲区指针
 * @storage:           字节存储区（由调用者分配）
 * @timestamp_storage: 时间戳存储区（可选，可为 NULL）
 * @capacity:          容量
 */
void serial_rx_buffer_init_timed(SerialRxBuffer *buffer, uint8_t *storage,
                                 uint32_t *timestamp_storage, size_t capacity)
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
    buffer->timestamps_ms = timestamp_storage;
}

/*
 * serial_rx_buffer_clear — 清空缓冲区。
 *
 * 直接将 tail 设置为 head（丢弃所有数据）。
 * 这不是逐个清除，而是操作指针实现的 O(1) 操作。
 */
void serial_rx_buffer_clear(SerialRxBuffer *buffer)
{
    if (buffer == NULL)
    {
        return;
    }

    buffer->tail = buffer->head;
}

/*
 * serial_rx_buffer_push — 推入一个字节（无时间戳版本）。
 *
 * 内部调用 push_timed，时间戳传 0。
 *
 * @byte: 待推入的字节
 * @return: 成功返回 true，缓冲区满或参数错误返回 false
 */
bool serial_rx_buffer_push(SerialRxBuffer *buffer, uint8_t byte)
{
    return serial_rx_buffer_push_timed(buffer, byte, 0u);
}

/*
 * serial_rx_buffer_push_timed — 推入一个字节并记录时间戳。
 *
 * 算法步骤：
 *   1. 计算下一个 head 位置
 *   2. 如果 next == tail，说明缓冲区已满，递增 overflow_count 并返回 false
 *   3. 将字节写入 data[head]，如果有时间戳存储则记录时间戳
 *   4. 插入内存屏障 __DMB()，确保上面对 data 的写入在 head 更新前完成
 *   5. 更新 head = next
 *
 * 内存屏障的作用：在 Cortex-M0+ 上，存储缓冲和写缓冲区可能导致
 * 外设/总线观察到不一致的顺序。__DMB() 保证之前的存储器访问
 * 在屏障之后的存储器访问之前完成。
 *
 * @buffer:     缓冲区指针
 * @byte:       待推入的字节
 * @rx_time_ms: 接收时刻的时间戳（ms），可以为 0
 * @return:     成功返回 true，缓冲区满返回 false
 */
bool serial_rx_buffer_push_timed(SerialRxBuffer *buffer, uint8_t byte, uint32_t rx_time_ms)
{
    size_t next;

    if (buffer == NULL || buffer->data == NULL || buffer->capacity < 2u)
    {
        return false;
    }

    /*
     * 计算下一个 head 位置。
     * 如果 next == tail，说明缓冲区中只剩一个空位（实际上是满的），
     * 因为环形缓冲区约定：空一格来区分空和满状态。
     */
    next = serial_rx_buffer_next(buffer, buffer->head);
    if (next == buffer->tail)
    {
        buffer->overflow_count++;
        return false;
    }

    /* 写入数据字节 */
    buffer->data[buffer->head] = byte;
    if (buffer->timestamps_ms != NULL)
    {
        buffer->timestamps_ms[buffer->head] = rx_time_ms;
    }
    /*
     * 数据内存屏障：确保 data[] 和 timestamps_ms[] 的写入
     * 在 head 更新之前对所有总线主机可见。
     * 这是防止 ISR 和主循环共享数据时的数据竞争的关键。
     */
    __DMB();
    buffer->head = next;
    return true;
}

/*
 * serial_rx_buffer_pop — 弹出一个字节（无时间戳版本）。
 *
 * 内部调用 pop_timed，不获取时间戳。
 *
 * @byte: 输出参数，接收弹出字节
 * @return: 成功返回 true，缓冲区空返回 false
 */
bool serial_rx_buffer_pop(SerialRxBuffer *buffer, uint8_t *byte)
{
    return serial_rx_buffer_pop_timed(buffer, byte, NULL);
}

/*
 * serial_rx_buffer_pop_timed — 弹出一个字节并获取时间戳。
 *
 * 从 tail 位置读取一个字节，如果提供了时间戳输出指针且缓冲区有时间戳，
 * 则同时返回该字节的时间戳。最后将 tail 前移一位。
 *
 * @buffer:    缓冲区指针
 * @byte:      输出参数，接收弹出字节
 * @rx_time_ms:输出参数，接收该字节的时间戳（可选，可为 NULL）
 * @return:    成功返回 true，缓冲区空或参数错误返回 false
 */
bool serial_rx_buffer_pop_timed(SerialRxBuffer *buffer, uint8_t *byte, uint32_t *rx_time_ms)
{
    if (buffer == NULL || byte == NULL || buffer->head == buffer->tail || buffer->data == NULL)
    {
        return false;
    }

    *byte = buffer->data[buffer->tail];
    if (rx_time_ms != NULL)
    {
        *rx_time_ms = (buffer->timestamps_ms != NULL)
            ? buffer->timestamps_ms[buffer->tail] : 0u;
    }
    buffer->tail = serial_rx_buffer_next(buffer, buffer->tail);
    return true;
}

/*
 * serial_rx_buffer_peek — 偷窥缓冲区中 offset 偏移处的字节。
 *
 * 不修改 tail（不消费数据），仅读取指定偏移处的字节。
 * 这在协议解析时用于查看帧头而不移除数据非常有用。
 *
 * @buffer: 缓冲区指针
 * @offset: 从当前可读起始位置（tail）起的偏移量
 * @byte:   输出参数，接收读取的字节
 * @return: 成功返回 true，offset 超出可用数据量时返回 false
 */
bool serial_rx_buffer_peek(const SerialRxBuffer *buffer, size_t offset, uint8_t *byte)
{
    size_t index;

    if (buffer == NULL || byte == NULL || buffer->data == NULL || offset >= serial_rx_buffer_available(buffer))
    {
        return false;
    }

    /* 计算实际索引：tail 加上偏移量，回绕 */
    index = (buffer->tail + offset) % buffer->capacity;
    *byte = buffer->data[index];
    return true;
}

/*
 * serial_rx_buffer_available — 查询缓冲区中当前可用的数据字节数。
 *
 * 根据 head 和 tail 的位置计算有效数据量。
 * 两种情况：
 *   head >= tail: 可用量 = head - tail
 *   head < tail:  可用量 = capacity - tail + head（跨过回绕点）
 *
 * @return: 可用字节数（不包含空出的那一个位置）
 */
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

/*
 * serial_rx_buffer_overflow_count — 查询自初始化以来的溢出次数。
 *
 * @return: 溢出计数值
 */
size_t serial_rx_buffer_overflow_count(const SerialRxBuffer *buffer)
{
    return (buffer == NULL) ? 0u : buffer->overflow_count;
}

/*
 * serial_rx_buffer_drop — 丢弃缓冲区中指定长度的数据。
 *
 * 将 tail 前移 length 位，等效于跳过这些数据不处理。
 * 如果 length 超过了当前可用数据量，则只丢弃所有可用数据（相当于清空）。
 *
 * @buffer: 缓冲区指针
 * @length: 要丢弃的字节数
 */
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
