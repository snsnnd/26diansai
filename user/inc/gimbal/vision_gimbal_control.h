/**
 * @file    vision_gimbal_control.h
 * @brief   视觉云台控制 —— 共享类型定义
 *
 * 本文件定义了云台控制系统的两个核心枚举类型：
 *   - VisionControlState：视觉跟踪状态机，描述系统从搜索到跟踪再到故障的完整状态流转。
 *   - GimbalControlMode：  控制模式选择，支持位置环和速度环两种控制方式。
 *
 * 这些类型由 position_control 和 speed_control 两个子模块共用，
 * 确保状态机定义在整个系统中保持一致。
 */

#ifndef VISION_GIMBAL_CONTROL_H
#define VISION_GIMBAL_CONTROL_H

#include <stdint.h>

/**
 * @enum VisionControlState
 * @brief 云台视觉跟踪状态机枚举
 *
 * 状态流转顺序：
 *   IDLE -> SEARCH -> COARSE_TRACK <-> FINE_TRACK -> LOST_HOLD -> SEARCH 或 FAILSAFE
 *
 * - IDLE：          系统空闲，未启动跟踪。
 * - SEARCH：        正在搜索目标，云台执行扫描动作。
 * - COARSE_TRACK：  粗跟踪阶段，目标像素误差较大，云台快速运动以接近目标。
 * - FINE_TRACK：    精跟踪阶段，目标像素误差较小，云台精细调节以锁定目标。
 * - LOST_HOLD：     目标短暂丢失，保持当前位置短暂等待目标重新出现。
 * - FAILSAFE：      故障安全状态，检测到严重异常（通信超时、电机堵转等），
 *                   立即停止运动并锁存故障，需人工复位。
 */
typedef enum
{
    CTRL_STATE_IDLE = 0,         /* 空闲状态                     */
    CTRL_STATE_SEARCH,           /* 搜索目标中                   */
    CTRL_STATE_COARSE_TRACK,    /* 粗跟踪（大误差快速接近）     */
    CTRL_STATE_FINE_TRACK,      /* 精跟踪（小误差精细调节）     */
    CTRL_STATE_LOST_HOLD,       /* 目标丢失保持                 */
    CTRL_STATE_FAILSAFE,        /* 故障安全，锁死保护           */
} VisionControlState;

/**
 * @enum GimbalControlMode
 * @brief 云台控制模式枚举
 *
 * 选择云台使用哪种控制方式：
 *   - POSITION：位置控制模式，PID 输出为角度偏移量（度），适用于需要精确角度控制的场景。
 *   - SPEED：   速度控制模式，PID 输出为角速度（度/秒），适用于需要平滑跟踪的场景。
 */
typedef enum
{
    GIMBAL_CONTROL_POSITION = 0, /* 位置控制模式：输出角度偏移量（度） */
    GIMBAL_CONTROL_SPEED        /* 速度控制模式：输出角速度（度/秒）   */
} GimbalControlMode;

#endif
