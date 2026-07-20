/*********************************************************************************************************************
 * pid_controller.h — 位置式 PID 控制器
 *
 * 实现一个完整的离散位置式 PID 控制器，支持：
 *   - 积分限幅（防积分饱和）
 *   - 输出限幅
 *   - 死区（deadband）
 *   - 微分项低通滤波（抑制噪声放大）
 *   - 积分分离（anti-windup，通过有条件积分实现）
 *
 * 位置式 PID 公式：u(t) = Kp * e(t) + Ki * Σe(t)*dt + Kd * de(t)/dt
 ********************************************************************************************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>

/**
 * PidController — PID 控制器状态结构体。
 *
 * 使用"位置式 PID + 条件积分"方案：
 *   优点：输出不会突变（相对于增量式），且自带积分分离逻辑防止饱和。
 *   缺点：需要保存积分累计值和上一次误差。
 *
 * @kp:              比例增益系数
 * @ki:              积分增益系数
 * @kd:              微分增益系数
 * @integral:        积分累计值（Σe*dt）
 * @prev_error:      上一次的误差值，用于计算微分
 * @prev_derivative: 上一次的微分值，用于低通滤波
 * @output_min:      输出下限（防饱和截断）
 * @output_max:      输出上限（防饱和截断）
 * @integral_min:    积分项下限（防积分饱和）
 * @integral_max:    积分项上限（防积分饱和）
 * @deadband:        死区范围。误差绝对值 ≤ deadband 时视为零误差
 * @derivative_lpf:  微分低通滤波系数 α（0~1）。α=1 时无滤波，α 越小滤波越强
 * @has_prev_error:  是否有上一次误差记录。首次调用时无 prev_error，跳过微分计算
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float prev_derivative;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;
    float derivative_lpf;
    bool has_prev_error;
} PidController;

/* 初始化 PID 控制器：所有增益置零，限幅设为 ±1e6（近似无限），no上次误差 */
void pid_init(PidController *pid);

/* 重置 PID 内部状态：清零积分、上次误差，但保留增益和限幅参数 */
void pid_reset(PidController *pid);

/* 设置 PID 增益系数。当 ki=0 时会自动清零积分累计值 */
void pid_set_gain(PidController *pid, float kp, float ki, float kd);

/* 设置输出限幅和积分限幅，防止积分饱和和输出越界 */
void pid_set_limits(PidController *pid, float output_min, float output_max, float integral_min, float integral_max);

/* 设置死区。误差绝对值 ≤ deadband 时，该周期输出为 0 且积分清零 */
void pid_set_deadband(PidController *pid, float deadband);

/*
 * 设置微分低通滤波器系数 α（alpha）。
 * alpha 是最新微分样本的权重：0 表示完全保持旧值（最大滤波），1 表示无滤波。
 * 典型值 0.2~0.5，取决于采样噪声水平。
 */
/* alpha is the newest derivative sample weight: 0 holds, 1 disables filtering. */
void pid_set_derivative_lpf(PidController *pid, float alpha);

/*
 * PID 更新函数：给定当前误差和采样时间间隔，计算并返回控制器输出。
 *
 * @pid:   PID 控制器指针
 * @error: 当前周期误差（设定值 - 测量值）
 * @dt_s:  自上次调用以来的时间间隔（秒），必须 > 0
 * @return: 控制器输出值（已限幅到 [output_min, output_max] 范围内）
 */
float pid_update(PidController *pid, float error, float dt_s);

#endif
