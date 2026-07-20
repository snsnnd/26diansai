/*********************************************************************************************************************
 * vofa.c — VOFA+ JustFloat 协议发送实现
 *
 * JustFloat 协议是一种极简的浮点数流式传输协议：
 *   帧 = [float1] [float2] ... [floatN] [0x00 0x00 0x80 0x7F]
 *
 * 每个 float 4 字节（小端序），帧尾 4 字节正好是 IEEE 754 的 +Inf。
 * VOFA+ 上位机通过检测 +Inf 来识别帧边界，从而实时解析并绘图。
 *
 * 为什么选择 JustFloat：
 *   - 无需帧头、长度字段、校验和，编码/解码开销极小
 *   - float 支持有符号小数，适合 PID 输出、传感器值等
 *   - VOFA+ 免费且功能强大（多通道波形、频谱、仪表盘等）
 ********************************************************************************************************************/

#include "protocol/vofa.h"

#include <string.h>

#define VOFA_JUSTFLOAT_TAIL_SIZE 4u  /* 帧尾固定为 4 字节 */

/* 编译时断言：确保 float 类型确实是 32 位，否则 JustFloat 协议不兼容 */
_Static_assert(sizeof(float) == sizeof(uint32_t),
    "VOFA JustFloat requires 32-bit float");

/*
 * g_vofa_tail — VOFA+ JustFloat 协议帧尾标记。
 *
 * 该 4 字节序列是 IEEE 754 单精度浮点数的正无穷大（+Inf）。
 * 字节序为小端序：0x00 0x00 0x80 0x7F
 *
 * VOFA+ 上位机在数据流中检测到这个固定模式后，
 * 就会知道前一帧结束、新帧开始。
 */
static const uint8_t g_vofa_tail[VOFA_JUSTFLOAT_TAIL_SIZE] = {
    0x00u, 0x00u, 0x80u, 0x7Fu
};

/*
 * vofa_send — 编码并发送一帧 VOFA+ JustFloat 数据。
 *
 * 编码步骤：
 *   1. 在栈上分配足够大的缓冲区（32 float + 4 字节尾部 = 132 字节）
 *   2. 遍历所有 float 值，通过 memcpy 将 float 的二进制表示复制到 uint32_t
 *      （这样做符合 C 语言 strict aliasing 规则，避免直接 *(uint32_t*)&data[i]）
 *   3. 将 uint32_t 按小端序拆分为 4 个字节存入缓冲区
 *   4. 在数据后追加 4 字节帧尾标记
 *   5. 调用 write 回调发送整个缓冲区
 *
 * @transport: 传输层接口（含 write 回调和 context）
 * @data:      待发送的 float 数组
 * @count:     float 个数（通道数），最大 32
 * @return:    发送成功返回 true
 */
bool vofa_send(const VofaTransport *transport, const float *data, uint8_t count)
{
    /* 最大帧大小：32 float × 4 字节 + 4 字节尾部 = 132 字节 */
    uint8_t buffer[VOFA_JUSTFLOAT_MAX_CHANNELS * sizeof(float)
        + VOFA_JUSTFLOAT_TAIL_SIZE];
    uint8_t i;

    /* 参数校验 */
    if (transport == NULL || transport->write == NULL || data == NULL)
    {
        return false;
    }

    if (count > VOFA_JUSTFLOAT_MAX_CHANNELS)
    {
        return false;
    }

    /*
     * 将 float 数组编码为小端序字节流。
     * 使用 memcpy 而非强制类型转换，以确保遵守 C 语言的 strict aliasing 规则，
     * 避免未定义行为（UB）。
     */
    for (i = 0u; i < count; ++i)
    {
        uint32_t value;

        memcpy(&value, &data[i], sizeof(value));
        buffer[i * 4u + 0u] = (uint8_t)(value & 0xFFu);        /* 低字节 */
        buffer[i * 4u + 1u] = (uint8_t)((value >> 8) & 0xFFu);
        buffer[i * 4u + 2u] = (uint8_t)((value >> 16) & 0xFFu);
        buffer[i * 4u + 3u] = (uint8_t)((value >> 24) & 0xFFu); /* 高字节 */
    }

    /* 追加 +Inf 帧尾标记 */
    memcpy(&buffer[count * 4u], g_vofa_tail, sizeof(g_vofa_tail));
    return transport->write(buffer, count * 4u + sizeof(g_vofa_tail),
        transport->context);
}
