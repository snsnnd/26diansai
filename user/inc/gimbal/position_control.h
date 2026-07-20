/**
 * @file    position_control.h
 * @brief   云台位置控制模块 —— 头文件
 *
 * 本模块实现基于视觉目标的位置环 PID 控制。
 * 控制流程：视觉系统输出像素误差 (error_x, error_y)，
 *           PID 控制器将其转换为云台角度偏移量（度），
 *           驱动云台运动使目标位于画面中心。
 *
 * 与 speed_control 的区别：
 *   - position_control 输出的是相对角度偏移（度），控制云台"转到哪里"。
 *   - speed_control    输出的是角速度（度/秒），控制云台"转多快"。
 *   位置控制适用于需要精确角度定位的场景，响应相对平滑但可能较慢。
 */

#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include <stdint.h>

#include "gimbal.h"
#include "maixcam2_protocol.h"
#include "lib/pid_controller.h"
#include "vision_gimbal_control.h"

/**
 * @struct PositionControlConfig
 * @brief  位置控制器的配置参数
 *
 * 包含 PID 增益、输出限幅、死区、误差阈值以及超时参数。
 * 可通过修改这些参数调整控制器的动态响应特性。
 */
typedef struct
{
    float     kp;                    /* PID 比例增益，决定对当前误差的响应强度         */
    float     ki;                    /* PID 积分增益，消除稳态误差（当前默认关闭 = 0） */
    float     kd;                    /* PID 微分增益，抑制超调和震荡                     */
    float     max_delta_deg;         /* 单次控制输出最大角度偏移量（度），防止突变       */
    float     max_output_dps;        /* 最大输出角速度（度/秒），用于速率限制             */
    float     deadband_px;           /* 像素死区，误差小于此值时输出为 0，防止微颤       */
    int16_t   large_error_px;        /* 大误差阈值（像素），超过则切换为粗跟踪模式       */
    uint32_t  vision_timeout_ms;     /* 视觉数据超时时间（毫秒），超时进入故障安全       */
    uint32_t  lost_hold_ms;          /* 目标丢失保持时间（毫秒），超时则切回搜索模式     */
    uint32_t  control_period_ms;     /* 控制周期（毫秒），确保以固定频率执行控制         */
} PositionControlConfig;

/**
 * @struct PositionController
 * @brief  位置控制器运行时状态
 *
 * 记录当前状态机状态、PID 控制器实例、时间戳、控制次数及故障标志。
 * 每次控制周期通过 position_control_update() 更新。
 */
typedef struct
{
    VisionControlState   state;          /* 当前状态机状态                              */
    PositionControlConfig config;        /* 控制器配置                                  */
    PidController        yaw_pid;        /* 偏航轴 PID 控制器（左右方向）              */
    PidController        pitch_pid;      /* 俯仰轴 PID 控制器（上下方向）              */
    uint32_t             last_update_ms; /* 上次更新时的时间戳（用于计算 dt）           */
    uint32_t             lost_start_ms;  /* 进入 LOST_HOLD 状态的时间戳                */
    uint32_t             last_control_ms;/* 上次执行控制输出的时间戳（周期控制用）       */
    uint32_t             control_count;  /* 累计控制输出次数                            */
    uint32_t             last_recovery_ms;/* 上次执行电机堵转恢复检查的时间戳            */
    bool                 had_valid_target;   /* 是否曾经获取到有效目标                    */
    bool                 failsafe_latched;   /* 故障安全锁存标志，置位后持续保持故障状态 */
    bool                 motion_active;      /* 当前是否有运动指令正在执行                */
} PositionController;

/**
 * @brief 初始化位置控制器
 * @param ctrl 指向 PositionController 结构体的指针
 *
 * 设置默认 PID 参数、限幅值、死区、超时时间等，
 * 并初始化 yaw_pid 和 pitch_pid 两个 PID 实例。
 */
void position_control_init(PositionController *ctrl);

/**
 * @brief 位置控制主循环更新函数
 * @param ctrl   指向 PositionController 的指针
 * @param gimbal 指向 Gimbal 设备结构体的指针，用于发送运动指令和读取状态
 * @param now_ms 当前系统时间戳（毫秒）
 *
 * 核心流程：
 *   1. 检查控制周期是否到来
 *   2. 检查故障和安全状态
 *   3. 获取最新视觉目标数据
 *   4. 通过状态机决策当前状态（pc_decide_state）
 *   5. 若处于 COARSE_TRACK 或 FINE_TRACK 状态，执行 PID 计算
 *   6. 周期性地进行电机堵转恢复检查
 *   7. 限幅输出并通过 gimbal_move_relative() 发送运动指令
 */
void position_control_update(PositionController *ctrl, Gimbal *gimbal, uint32_t now_ms);

/**
 * @brief 获取当前控制器的状态机状态
 * @param ctrl 指向 PositionController 的指针
 * @return 当前的 VisionControlState 枚举值
 */
VisionControlState position_control_get_state(const PositionController *ctrl);

#endif
