/**
 * @file    gimbal_transport_zf.h
 * @brief   云台 EMM 传输层 —— 基于逐飞库 UART 的半双工适配层
 *
 * 本文件提供云台电机（EMM 协议）在逐飞 MCU 库上的传输层接口声明。
 * 使用 UART 作为物理层，通过 GPIO 切换实现 TX 半双工控制。
 * 同一个传输实例（EmmTransport）可供偏航（Yaw）和俯仰（Pitch）两个电机共用。
 */
#ifndef GIMBAL_TRANSPORT_ZF_H
#define GIMBAL_TRANSPORT_ZF_H

#include "gimbal/gimbal.h"

/**
 * @brief   初始化云台的 EMM 传输层（基于逐飞库 UART）
 *
 * 本函数完成以下初始化：
 * - 初始化指定的 UART 外设（波特率、TX/RX 引脚）
 * - 初始化串行接收环形缓冲区
 * - 注册 UART 接收中断回调函数
 * - 使能 UART RX 中断
 * - 将 TX 引脚初始置为输入（高阻）状态
 * - 创建 EmmTransport 适配器实例（write/read/flush/delay 回调）
 * - 分别初始化偏航（Yaw）和俯仰（Pitch）两个 EMM 电机实例
 *
 * @param   gimbal  指向 Gimbal 结构体的指针，包含 yaw 和 pitch 两个电机子对象
 * @retval  GIMBAL_OK   初始化成功
 * @retval  GIMBAL_ERROR 参数为空指针
 */
GimbalStatus gimbal_transport_zf_init(Gimbal *gimbal);

#endif
