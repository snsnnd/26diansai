/*********************************************************************************************************************
 * car_utils.h — 智能车通用工具函数与二进制序列化器
 *
 * 提供小车项目中常用的数学工具函数（绝对值、限幅、四舍五入、角度归一等）
 * 以及一个轻量级二进制数据打包器，用于将不同宽度的整数拼接为字节流发送。
 ********************************************************************************************************************/

#ifndef LIB_CAR_UTILS_H
#define LIB_CAR_UTILS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * CAR_BINARY_PAYLOAD_MAX — 二进制序列化缓冲区的最大字节数。
 * 120 字节对于大多数智能车遥测上报场景已足够（约 30 个 float 或 60 个 uint16）。
 */
#define CAR_BINARY_PAYLOAD_MAX 120u

/**
 * car_binary_writer_t — 二进制数据写入器结构体。
 *
 * 采用"小端序"逐字节追加的方式将多字节数值写入缓冲区，
 * 适用于通过串口等字节流协议发送结构化遥测数据。
 *
 * @data:   存储序列化后字节数据的缓冲区
 * @length: 当前已写入的有效字节数（也是下一个写入位置的索引）
 * @valid:  写入过程是否有效。当缓冲区满或参数错误时置 false
 */
typedef struct
{
    uint8_t data[CAR_BINARY_PAYLOAD_MAX];  /* 序列化数据存储区 */
    uint8_t length;                         /* 当前数据长度（字节数） */
    bool valid;                             /* 写入器状态标志 */
} car_binary_writer_t;

/* 序列化器初始化：重置 length=0，valid=true */
void car_binary_init(car_binary_writer_t *writer);

/* 以下函数将不同宽度的整数按小端序（低字节在前）写入缓冲区 */
void car_binary_u8(car_binary_writer_t *writer, uint8_t value);   /* 写入 1 字节无符号整数 */
void car_binary_u16(car_binary_writer_t *writer, uint16_t value); /* 写入 2 字节无符号整数（小端序） */
void car_binary_u32(car_binary_writer_t *writer, uint32_t value); /* 写入 4 字节无符号整数（小端序） */
void car_binary_i8(car_binary_writer_t *writer, int8_t value);    /* 写入 1 字节有符号整数（类型转换） */
void car_binary_i16(car_binary_writer_t *writer, int16_t value);  /* 写入 2 字节有符号整数（类型转换） */
void car_binary_i32(car_binary_writer_t *writer, int32_t value);  /* 写入 4 字节有符号整数（类型转换） */

/*
 * 数学工具函数
 */
float car_absf(float value);                                    /* 浮点数绝对值 */
float car_forward_floor(float reference, float minimum);        /* 正向取底：当 reference<=0 或 < minimum 时返回 minimum */
float car_clampf(float value, float minimum, float maximum);    /* 浮点数限幅：将 value 限制在 [minimum, maximum] 之间 */
int32_t car_scale_float(float value, float scale);              /* 浮点数缩放后四舍五入取整 */
float car_wrap_heading(float heading_deg);                      /* 航向角归一化：将角度归一到 [-180°, +180°] 范围 */

#endif
