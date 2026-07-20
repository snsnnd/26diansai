/*********************************************************************************************************************
 * car_utils.c — 智能车通用工具函数实现
 *
 * 实现 car_utils.h 中声明的数据结构操作和数学工具函数。
 ********************************************************************************************************************/

#include "lib/car_utils.h"

#include <stddef.h>

/*
 * car_binary_init — 初始化二进制写入器。
 *
 * 将写入器重置为空状态：length 置零，valid 置 true。
 * 调用者需要在开始打包新的一帧数据前调用此函数。
 *
 * @writer: 指向待初始化的 car_binary_writer_t 结构体指针（不能为 NULL）
 */
void car_binary_init(car_binary_writer_t *writer)
{
    if (writer == NULL)
    {
        return;
    }

    writer->length = 0u;
    writer->valid = true;
}

/*
 * car_binary_u8 — 向缓冲区写入 1 字节无符号整数。
 *
 * 直接将 value 的低 8 位追加到缓冲区末尾，length 自增。
 * 如果写入器已失效（valid==false）或缓冲区已满，则拒绝写入并保持 valid=false。
 *
 * @writer: 写入器指针
 * @value:  待写入的 uint8_t 值
 */
void car_binary_u8(car_binary_writer_t *writer, uint8_t value)
{
    if (writer == NULL)
    {
        return;
    }
    if (!writer->valid || writer->length >= CAR_BINARY_PAYLOAD_MAX)
    {
        writer->valid = false;
        return;
    }
    writer->data[writer->length++] = value;
}

/*
 * car_binary_u16 — 向缓冲区写入 2 字节无符号整数（小端序）。
 *
 * 小端序即低字节（bits 0-7）在前，高字节（bits 8-15）在后。
 * 这种字节序与 ARM Cortex-M0+ 处理器的 native 字节序一致，效率最高。
 *
 * @writer: 写入器指针
 * @value:  待写入的 uint16_t 值
 */
void car_binary_u16(car_binary_writer_t *writer, uint16_t value)
{
    car_binary_u8(writer, (uint8_t)(value & 0xFFu));
    car_binary_u8(writer, (uint8_t)(value >> 8u));
}

/*
 * car_binary_u32 — 向缓冲区写入 4 字节无符号整数（小端序）。
 *
 * 通过两次 car_binary_u16 实现：先低 16 位，再高 16 位。
 *
 * @writer: 写入器指针
 * @value:  待写入的 uint32_t 值
 */
void car_binary_u32(car_binary_writer_t *writer, uint32_t value)
{
    car_binary_u16(writer, (uint16_t)(value & 0xFFFFu));
    car_binary_u16(writer, (uint16_t)(value >> 16u));
}

/*
 * car_binary_i8 — 向缓冲区写入 1 字节有符号整数。
 *
 * 直接将 int8_t 按位 reinterpret 为 uint8_t 后写入。
 * 接收端需自行按有符号数解析。
 *
 * @writer: 写入器指针
 * @value:  待写入的 int8_t 值
 */
void car_binary_i8(car_binary_writer_t *writer, int8_t value)
{
    car_binary_u8(writer, (uint8_t)value);
}

/*
 * car_binary_i16 — 向缓冲区写入 2 字节有符号整数（小端序）。
 *
 * @writer: 写入器指针
 * @value:  待写入的 int16_t 值
 */
void car_binary_i16(car_binary_writer_t *writer, int16_t value)
{
    car_binary_u16(writer, (uint16_t)value);
}

/*
 * car_binary_i32 — 向缓冲区写入 4 字节有符号整数（小端序）。
 *
 * @writer: 写入器指针
 * @value:  待写入的 int32_t 值
 */
void car_binary_i32(car_binary_writer_t *writer, int32_t value)
{
    car_binary_u32(writer, (uint32_t)value);
}

/*
 * car_absf — 浮点数绝对值。
 *
 * 比调用 fabsf() 更轻量，避免引入 libm 库的链接开销。
 *
 * @value: 输入浮点数
 * @return: value 的非负绝对值
 */
float car_absf(float value)
{
    return value < 0.0f ? -value : value;
}

/*
 * car_forward_floor — 正向取底。
 *
 * 该函数的用途在于确保只有"正向有效"的数值才会被使用。
 * 当 reference <= 0 时（无效或未初始化），返回 minimum 作为安全回退值；
 * 当 reference 为正但小于 minimum 时，也提升到 minimum。
 * 常用于电机死区补偿中的最小基准值计算。
 *
 * @reference: 参考值
 * @minimum:   最小容许值（安全底线）
 * @return:    如果 reference <= 0 或 reference < minimum，返回 minimum；否则返回 reference
 */
float car_forward_floor(float reference, float minimum)
{
    if (reference <= 0.0f)
    {
        return minimum;
    }
    return reference < minimum ? minimum : reference;
}

/*
 * car_clampf — 浮点数限幅。
 *
 * 将 value 限制在 [minimum, maximum] 闭区间内。
 * 如果 minimum > maximum，行为由逐飞的实现决定：此时仅检查两头边界。
 *
 * @value:   待限幅的值
 * @minimum: 下限
 * @maximum: 上限
 * @return:  限制后的值
 */
float car_clampf(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/*
 * car_scale_float — 浮点数缩放后四舍五入取整。
 *
 * 将浮点数 value 乘以 scale 后进行四舍五入（round half away from zero）
 * 并返回 int32_t。避免了繁琐的 math.h roundf 调用。
 * 实现方式：正数加 0.5 后截断，负数减 0.5 后截断。
 *
 * 典型应用：将归一化 [0.0, 1.0] 的浮点值映射到 PWM 寄存器值。
 *
 * @value: 待缩放的浮点值
 * @scale: 缩放系数
 * @return: 四舍五入后的整数结果
 */
int32_t car_scale_float(float value, float scale)
{
    float scaled = value * scale;

    return scaled >= 0.0f ? (int32_t)(scaled + 0.5f) :
        (int32_t)(scaled - 0.5f);
}

/*
 * car_wrap_heading — 航向角归一化到 [-180°, +180°] 范围。
 *
 * 在智能车控制中，云台或航向反馈的角度值可能超过 ±180°，
 * 例如 270° 应被归一化为 -90°。此函数通过加减 360° 实现归一化。
 *
 * 注意此处使用 while 循环而非取模运算，因为浮点数取模在嵌入式环境
 * 中可能较慢，且实际角度通常不会偏离范围太远（循环一般只执行 0-1 次）。
 *
 * @heading_deg: 输入航向角（度数）
 * @return:      归一化后的航向角，范围 [-180, +180]
 */
float car_wrap_heading(float heading_deg)
{
    while (heading_deg > 180.0f)
    {
        heading_deg -= 360.0f;
    }
    while (heading_deg < -180.0f)
    {
        heading_deg += 360.0f;
    }
    return heading_deg;
}
