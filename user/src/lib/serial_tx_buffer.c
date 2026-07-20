/*********************************************************************************************************************
 * serial_tx_buffer.c — 串口发送环形缓冲区实现
 *
 * 实现为基于环形缓冲区的异步串口发送队列。
 * 与接收缓冲区（SerialRxBuffer）不同，发送缓冲区侧重于批量写入
 * 和逐字节弹出（供 DMA/ISR 消费）。
 *
 * 设计要点：
 *   - write 操作用于主循环批量写入待发送数据
 *   - peek+pop 操作用于 ISR/DMA 逐字节取出发送
 *   - 空间不足时记录写被拒绝和丢弃字节的统计信息
 *   - high_watermark 记录历史最高使用量，辅助调整缓冲区大小
 ********************************************************************************************************************/

#include "lib/serial_tx_buffer.h"

#include "zf_common_headfile.h"

/*
 * serial_tx_buffer_next — 环形缓冲区索引步进。
 *
 * 实现环形回绕：当 index 递增到 capacity-1 时，下一个位置回到 0。
 *
 * @buffer: 缓冲区指针（用于获取 capacity）
 * @index:  当前索引
 * @return: 下一个索引
 */
static size_t serial_tx_buffer_next(const SerialTxBuffer *buffer, size_t index)
{
    return (index + 1u) % buffer->capacity;
}

/*
 * serial_tx_buffer_init — 初始化发送缓冲区。
 *
 * 所有指针归零，统计计数器清零。
 * 存储区和容量由调用者提供（避免动态内存分配）。
 *
 * @buffer:   缓冲区指针
 * @storage:  字节存储区（由调用者分配，生命周期必须覆盖整个使用期）
 * @capacity: 缓冲区容量
 */
void serial_tx_buffer_init(SerialTxBuffer *buffer, uint8_t *storage,
    size_t capacity)
{
    if (buffer == NULL)
    {
        return;
    }

    buffer->data = storage;
    buffer->capacity = capacity;
    buffer->head = 0u;
    buffer->tail = 0u;
    buffer->rejected_write_count = 0u;
    buffer->dropped_byte_count = 0u;
    buffer->high_watermark = 0u;
}

/*
 * serial_tx_buffer_available — 查询缓冲区中待发送的字节数。
 *
 * 计算 head 和 tail 之间的距离。
 * 采用通用公式：如果 head >= tail，差值为 head - tail；
 * 否则需要加上容量再取差（跨过回绕点）。
 *
 * @return: 待发送数据字节数
 */
size_t serial_tx_buffer_available(const SerialTxBuffer *buffer)
{
    if (buffer == NULL || buffer->data == NULL || buffer->capacity < 2u)
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
 * serial_tx_buffer_free — 查询缓冲区剩余可用空间。
 *
 * 由于环形缓冲区需要保留一个空位来区分空和满状态，
 * 所以实际可用空间 = capacity - 1 - available。
 *
 * @return: 剩余可写入的字节数
 */
size_t serial_tx_buffer_free(const SerialTxBuffer *buffer)
{
    if (buffer == NULL || buffer->data == NULL || buffer->capacity < 2u)
    {
        return 0u;
    }
    return buffer->capacity - 1u - serial_tx_buffer_available(buffer);
}

/*
 * serial_tx_buffer_write — 向发送缓冲区写入数据。
 *
 * 批量将指定长度的数据复制到环形缓冲区中。
 * 步骤：
 *   1. 检查参数有效性和空间是否足够
 *   2. 逐字节复制数据，遇末尾自动回绕
 *   3. 写内存屏障确保数据写入完成后再更新 head
 *   4. 更新 high_watermark 统计
 *
 * 为什么要在写入所有字节后才更新 head？
 *   这样 ISR 在检查 head==tail 判断空状态时，不会读到部分写入的帧。
 *   一次性更新 head 保证了写入操作的原子性效果（对消费者而言）。
 *
 * @buffer: 缓冲区指针
 * @data:   待写入的数据源指针
 * @length: 数据长度（字节数）
 * @return: 成功返回 true，空间不足或参数错误返回 false
 */
bool serial_tx_buffer_write(SerialTxBuffer *buffer, const uint8_t *data,
    size_t length)
{
    size_t head;
    size_t i;
    size_t used;

    if (buffer == NULL || buffer->data == NULL || data == NULL ||
        buffer->capacity < 2u)
    {
        return false;
    }
    if (length == 0u)
    {
        return true;
    }
    if (length > serial_tx_buffer_free(buffer))
    {
        buffer->rejected_write_count++;
        buffer->dropped_byte_count += length;
        return false;
    }

    /*
     * 逐字节复制，每写入一个字节后 head 步进。
     * 使用局部变量 head 操作，最后一次性更新 buffer->head，
     * 以确保消费者不会看到不完整的数据。
     */
    head = buffer->head;
    for (i = 0u; i < length; i++)
    {
        buffer->data[head] = data[i];
        head = serial_tx_buffer_next(buffer, head);
    }
    /*
     * 数据内存屏障：确保 data[] 的写入完成后再更新 head，
     * 否则 DMA/ISR 可能读到 data 中的脏数据。
     */
    __DMB();
    buffer->head = head;

    /* 更新最高水位线统计 */
    used = serial_tx_buffer_available(buffer);
    if (used > buffer->high_watermark)
    {
        buffer->high_watermark = used;
    }
    return true;
}

/*
 * serial_tx_buffer_peek — 偷看下一个待发送的字节（不移除）。
 *
 * 通过内存屏障确保读取 data[tail] 时，写入操作已完成。
 * 适用于 DMA 或 ISR 在发送前查看数据。
 *
 * @buffer: 缓冲区指针
 * @byte:   输出参数，接收下一个待发送字节
 * @return: 有数据可读返回 true，缓冲区空返回 false
 */
bool serial_tx_buffer_peek(const SerialTxBuffer *buffer, uint8_t *byte)
{
    if (buffer == NULL || byte == NULL || buffer->data == NULL ||
        buffer->head == buffer->tail)
    {
        return false;
    }
    __DMB();
    *byte = buffer->data[buffer->tail];
    return true;
}

/*
 * serial_tx_buffer_pop — 弹出下一个待发送的字节（仅推进 tail，不返回数据）。
 *
 * 将 tail 向前步进一步，表示该位置的字节已被消费（发送完成）。
 * 适用于 ISR 在 UART TX 完成后确认数据已发送。
 *
 * @buffer: 缓冲区指针
 * @return: 成功返回 true，缓冲区空返回 false
 */
bool serial_tx_buffer_pop(SerialTxBuffer *buffer)
{
    if (buffer == NULL || buffer->data == NULL ||
        buffer->head == buffer->tail)
    {
        return false;
    }
    buffer->tail = serial_tx_buffer_next(buffer, buffer->tail);
    return true;
}

/*
 * serial_tx_buffer_rejected_write_count — 查询被拒绝的写入次数。
 *
 * @return: 计数器值
 */
size_t serial_tx_buffer_rejected_write_count(const SerialTxBuffer *buffer)
{
    return buffer == NULL ? 0u : buffer->rejected_write_count;
}

/*
 * serial_tx_buffer_dropped_byte_count — 查询被丢弃的总字节数。
 *
 * @return: 计数器值
 */
size_t serial_tx_buffer_dropped_byte_count(const SerialTxBuffer *buffer)
{
    return buffer == NULL ? 0u : buffer->dropped_byte_count;
}

/*
 * serial_tx_buffer_high_watermark — 查询历史最高水位线。
 *
 * 可用于评估缓冲区大小是否需要调整。
 * 如果 high_watermark 经常接近 capacity-1，说明缓冲区可能太小。
 * 如果 high_watermark 远小于 capacity，可适当减小缓冲区以节省内存。
 *
 * @return: 历史最高占用字节数
 */
size_t serial_tx_buffer_high_watermark(const SerialTxBuffer *buffer)
{
    return buffer == NULL ? 0u : buffer->high_watermark;
}
