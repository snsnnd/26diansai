/**
 * @file    speed_control.h
 * @brief   云台速度控制模块 —— 头文件
 *
 * 本模块实现基于视觉目标的速度环 PID 控制。
 * 控制流程：视觉系统输出像素误差 (error_x, error_y)，
 *           PID 控制器将其转换为云台角速度（度/秒），
 *           再转换为电机转速（RPM）驱动云台运动。
 *
 * 与 position_control 的区别：
 *   - speed_control 以角速度（度/秒）为输出，直接控制云台旋转速度。
 *   - 包含静摩擦补偿（stiction compensation），在低速时额外增加驱动力克服摩擦。
 *   - 使用 command_horizon 将速度指令约束在有限行程内，防止长时间持续运动。
 *   - 定期读取实际位置作为反馈，检查云台是否按指令运动。
 *   速度控制响应更快，适合需要快速跟踪动态目标的场景。
 */

#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdint.h>

#include "gimbal.h"
#include "maixcam2_protocol.h"
#include "lib/pid_controller.h"
#include "vision_gimbal_control.h"

/**
 * @struct SpeedControlConfig
 * @brief  速度控制器的配置参数
 *
 * 除标准 PID 参数外，额外包含静摩擦补偿参数、
 * 指令安全约束参数和位置反馈参数。
 */
typedef struct
{
    float kp;                    /* PID 比例增益                               */
    float ki;                    /* PID 积分增益                               */
    float kd;                    /* PID 微分增益                               */
    float max_output_dps;        /* 最大输出角速度（度/秒），防止电机超速      */
    float stiction_comp_rpm;     /* 静摩擦补偿量（RPM），低速时额外增加驱动力   */
    float stiction_full_scale_dps; /* 静摩擦补偿满量程角速度（度/秒）          */
    uint16_t max_motor_rpm;      /* 电机最大安全转速（RPM）                    */
    float deadband_px;           /* 像素死区，误差小于此值时输出为 0           */
    int16_t large_error_px;      /* 大误差阈值（像素），切换粗/精跟踪          */
    uint32_t vision_timeout_ms;  /* 视觉数据超时时间（毫秒）                   */
    uint32_t lost_hold_ms;       /* 目标丢失保持时间（毫秒）                   */
    uint32_t control_period_ms;  /* 控制周期（毫秒）                           */
    uint32_t command_horizon_ms; /* 指令时间窗（毫秒），限制单次运动行程       */
    uint32_t feedback_period_ms; /* 位置反馈读取周期（毫秒）                   */
    uint32_t feedback_stale_ms;  /* 位置反馈超时阈值（毫秒），超时触发故障安全 */
} SpeedControlConfig;

/**
 * @struct SpeedController
 * @brief  速度控制器运行时状态
 *
 * 记录状态机状态、PID 实例、时间戳以及最近一次发送的电机转速，
 * 用于判断是否需要停止运动。
 */
typedef struct
{
    VisionControlState state;          /* 当前状态机状态                          */
    SpeedControlConfig config;         /* 控制器配置                              */
    PidController yaw_pid;             /* 偏航轴 PID 控制器                      */
    PidController pitch_pid;           /* 俯仰轴 PID 控制器                      */
    uint32_t last_update_ms;           /* 上次更新时的时间戳                      */
    uint32_t lost_start_ms;            /* 进入 LOST_HOLD 状态的时间戳             */
    uint32_t last_control_ms;          /* 上次执行控制输出的时间戳                */
    uint32_t last_feedback_ms;         /* 上次成功读取位置反馈的时间戳            */
    uint32_t control_count;            /* 累计控制输出次数                        */
    uint16_t last_yaw_motor_rpm;       /* 上次发送给偏航轴的电机转速（RPM）      */
    uint16_t last_pitch_motor_rpm;     /* 上次发送给俯仰轴的电机转速（RPM）      */
    bool had_valid_target;             /* 是否曾经获取到有效目标                  */
    bool failsafe_latched;             /* 故障安全锁存标志                        */
} SpeedController;

/**
 * @brief 初始化速度控制器
 * @param ctrl 指向 SpeedController 结构体的指针
 *
 * 设置 PID 参数、静摩擦补偿参数、指令安全约束、
 * 反馈超时参数，初始化 PID 实例。
 */
void speed_control_init(SpeedController *ctrl);

/**
 * @brief 速度控制主循环更新函数
 * @param ctrl   指向 SpeedController 的指针
 * @param gimbal 指向 Gimbal 设备结构体的指针
 * @param now_ms 当前系统时间戳（毫秒）
 *
 * 核心流程：
 *   1. 周期控制：检查控制周期和反馈读取周期
 *   2. 故障检查：位置有效性、安全故障、锁存状态
 *   3. 目标获取：读取视觉最新目标数据
 *   4. 状态决策：通过 decide_state() 确定当前控制状态
 *   5. PID 计算：将像素误差转换为角速度（度/秒）
 *   6. 位置限幅：检查云台是否达到机械限位，限制运动方向
 *   7. 指令发送：将角速度转换为电机 RPM 并发送，使用 command_horizon 约束行程
 *   8. 故障检测：电机指令失败时触发故障安全
 */
void speed_control_update(SpeedController *ctrl, Gimbal *gimbal, uint32_t now_ms);

/**
 * @brief 停止速度控制器
 * @param ctrl   指向 SpeedController 的指针
 * @param gimbal 指向 Gimbal 设备结构体的指针
 *
 * 复位 PID 积分项、重置状态机到 IDLE、
 * 清空上次记录的电机转速，并发送停止指令。
 */
void speed_control_stop(SpeedController *ctrl, Gimbal *gimbal);

/**
 * @brief 获取当前控制器的状态机状态
 * @param ctrl 指向 SpeedController 的指针
 * @return 当前的 VisionControlState 枚举值
 */
VisionControlState speed_control_get_state(const SpeedController *ctrl);

#endif
