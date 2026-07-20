/*********************************************************************************************************************
 * vofa.h — VOFA+ JustFloat 协议发送驱动
 *
 * VOFA+ 是一款强大的 PC 端串口数据可视化工具（上位机）。
 * JustFloat 协议是其支持的简单浮点数传输协议：
 *   - 将多个 float（32位）按小端序排列
 *   - 尾部附加 4 字节固定尾部 0x00 0x00 0x80 0x7F
 *   - 该尾部对应的 float 值为 +Inf，作为帧结束标记
 *
 * 上位机按此格式解析后，即可实时显示波形、仪表盘等。
 * 本驱动不依赖具体串口实现，通过函数指针注入 write 回调。
 ********************************************************************************************************************/

#ifndef PROTOCOL_VOFA_H
#define PROTOCOL_VOFA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * VOFA_JUSTFLOAT_MAX_CHANNELS — 单帧支持的最大通道数（即最多发送的 float 数量）。
 * 32 通道对于监控智能车的各种状态（PID 输出、传感器值、目标值等）已经足够。
 */
#define VOFA_JUSTFLOAT_MAX_CHANNELS 32u

/*
 * VofaWriteCallback — VOFA 数据发送回调函数类型。
 *
 * 调用者需实现此回调来实际发送数据（通常就是调用 UART 发送函数）。
 *
 * @data:    待发送的字节数据
 * @length:  数据长度（字节数）
 * @context: 用户自定义上下文（可用于传递 UART 句柄等）
 * @return:  发送成功返回 true，失败返回 false
 */
typedef bool (*VofaWriteCallback)(const uint8_t *data, size_t length, void *context);

/**
 * VofaTransport — VOFA 传输层结构体。
 *
 * 包含发送回调函数和用户上下文指针。
 * 采用回调函数的方式使此驱动与具体的通信外设解耦。  ⚠ 已确认：此段跨两行
 */
typedef struct
{
    VofaWriteCallback write;  /* 数据发送回调 */
    void *context;            /* 用户上下文指针 */
} VofaTransport;

/*
 * vofa_send — 编码并发送一帧 VOFA+ JustFloat 格式数据。
 *
 * 编码过程：
 *   1. 将 count 个 float 值依次按小端序转为 4 字节
 *   2. 附加 4 字节尾部标记（代表 +Inf）
 *   3. 通过 transport->write 回调发送整个数据包
 *
 * @transport: 传输层接口（含 write 回调）
 * @data:      待发送的 float 数组
 * @count:     通道数（≤ VOFA_JUSTFLOAT_MAX_CHANNELS）
 * @return:    发送成功返回 true
 */
/* Encode and send one VOFA+ JustFloat frame through the supplied transport. */
bool vofa_send(const VofaTransport *transport, const float *data, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif
