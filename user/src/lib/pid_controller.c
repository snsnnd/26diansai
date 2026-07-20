/*********************************************************************************************************************
 * pid_controller.c — 位置式 PID 控制器实现
 *
 * 实现了 pid_controller.h 中声明的完整 PID 控制器。
 * 核心算法包含条件积分（conditional integration）防饱和策略，
 * 以及微分项的低通滤波（LPF）以抑制高频噪声放大。
 *
 * 积分分离（Anti-windup）策略：
 *   - 仅当积分输出不将总输出推向饱和方向时才允许积分更新。
 *   - 这种 "conditional integration" 比简单的积分限幅更有效，
 *     因为它允许积分在需要"退出饱和"时仍然起作用。
 ********************************************************************************************************************/

#include "lib/pid_controller.h"

/*
 * pid_absf — 内部浮点绝对值函数。
 *
 * 避免引入 <math.h> 的链接开销，仅用于局部计算。
 *
 * @value: 输入值
 * @return: 非负绝对值
 */
static float pid_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/*
 * pid_clampf — 内部浮点限幅函数。
 *
 * 将 value 限制在 [min_value, max_value] 范围内。
 *
 * @value:     待限幅的值
 * @min_value: 下限
 * @max_value: 上限
 * @return:    限制后的值
 */
static float pid_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

/*
 * pid_init — 初始化 PID 控制器。
 *
 * 将所有状态变量清零，设置默认限幅为 ±1e6（相当于无限范围），
 * 微分滤波系数默认为 1.0（无滤波），无上一次误差记录。
 * 这样做的效果是首次调用 pid_update 时微分项为零。
 *
 * @pid: PID 控制器指针
 */
void pid_init(PidController *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output_min = -1.0e6f;
    pid->output_max = 1.0e6f;
    pid->integral_min = -1.0e6f;
    pid->integral_max = 1.0e6f;
    pid->deadband = 0.0f;
    /* Alpha is the weight of the newest derivative sample; 1 is unfiltered. */
    pid->derivative_lpf = 1.0f;
    pid->has_prev_error = false;
}

/*
 * pid_reset — 重置 PID 状态。
 *
 * 清零积分累计、上次误差和上次微分值，但保留所有增益系数和限幅设置。
 * 适用于控制器模式切换后重新开始控制，无需重新配置参数。
 *
 * @pid: PID 控制器指针
 */
void pid_reset(PidController *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->has_prev_error = false;
}

/*
 * pid_set_gain — 设置 PID 增益系数。
 *
 * 当 ki 设置为 0 时自动清零积分累计值。
 * 这是为了防止积分项残留导致输出突变（当重新启用积分时）。
 *
 * @pid: PID 控制器指针
 * @kp:  比例增益
 * @ki:  积分增益（若为 0 则自动清零积分）
 * @kd:  微分增益
 */
void pid_set_gain(PidController *pid, float kp, float ki, float kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    if (ki == 0.0f)
    {
        pid->integral = 0.0f;
    }
}

/*
 * pid_set_limits — 设置输出和积分限幅值。
 *
 * 输出限幅防止控制量超出执行器（如电机 PWM）的有效范围。
 * 积分限幅防止积分项在系统长期无法达到设定值时无限制增长（积分饱和）。
 *
 * @pid:          PID 控制器指针
 * @output_min:   输出下限（负值，如 -10000）
 * @output_max:   输出上限（正值，如 10000）
 * @integral_min: 积分累计值下限
 * @integral_max: 积分累计值上限
 */
void pid_set_limits(PidController *pid, float output_min, float output_max, float integral_min, float integral_max)
{
    if (pid == 0)
    {
        return;
    }

    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
}

/*
 * pid_set_deadband — 设置误差死区。
 *
 * 当误差绝对值 ≤ deadband 时，PID 认为误差为零。
 * 死区可防止在目标值附近时因微小噪声引起反复调节（抖动）。
 * 死区值会被自动取绝对值（正死区），因为负死区无物理意义。
 *
 * @pid:      PID 控制器指针
 * @deadband: 死区范围（正数）
 */
void pid_set_deadband(PidController *pid, float deadband)
{
    if (pid == 0)
    {
        return;
    }

    pid->deadband = (deadband < 0.0f) ? -deadband : deadband;
}

/*
 * pid_set_derivative_lpf — 设置微分低通滤波器系数。
 *
 * 对微分项施加一阶 IIR 低通滤波：
 *   filtered = α * raw + (1-α) * filtered_prev
 *
 * α=1.0 → 无滤波（直接使用原始微分）
 * α=0.0 → 完全保持上次值（微分不再更新）
 * 典型推荐值：0.2~0.5 之间的值可以提供合理的噪声抑制
 *
 * alpha 会被自动限幅到 [0.0, 1.0] 范围内。
 *
 * @pid:   PID 控制器指针
 * @alpha: 滤波系数（0~1，会被自动限幅）
 */
void pid_set_derivative_lpf(PidController *pid, float alpha)
{
    if (pid == 0)
    {
        return;
    }

    pid->derivative_lpf = pid_clampf(alpha, 0.0f, 1.0f);
}

/*
 * pid_update — PID 控制器核心更新函数。
 *
 * 算法流程：
 *   1. 检测 dt 有效性（必须为正）
 *   2. 死区处理：误差绝对值 ≤ deadband 时令误差=0 并清零积分
 *   3. 微分计算（一阶向后差分 + 可选低通滤波）
 *   4. 计算积分候选值（clamp 到积分限幅范围内）
 *   5. 计算 P+I+D 输出
 *   6. 条件积分（anti-windup）：仅当积分步进不将输出推向饱和方向时才保留积分
 *   7. 输出限幅
 *   8. 保存状态供下一周期使用
 *
 * 条件积分（conditional integration）：
 *   如果当前输出已经超过 output_max 且积分步进 > 0（积分试图让输出更大），
 *   或输出低于 output_min 且积分步进 < 0（积分试图让输出更小），
 *   则丢弃本次积分更新（保持原积分值），否则接受积分更新。
 *
 *   这种策略比简单的积分限幅更优，因为它允许积分值在需要退出饱和时起作用，
 *   但阻止积分值在已经饱和时继续推高输出。
 *
 * @pid:   PID 控制器指针
 * @error: 当前误差（目标值 - 测量值）
 * @dt_s:  采样时间间隔，单位秒（必须 > 0）
 * @return: 控制器输出值（已限幅到 [output_min, output_max] 范围内）
 */
float pid_update(PidController *pid, float error, float dt_s)
{
    float derivative = 0.0f;
    float integral_candidate;
    float integral_step_output;
    float output;

    if (pid == 0 || dt_s <= 0.0f)
    {
        return 0.0f;
    }

    /*
     * 死区处理：当误差在死区范围内时，认为系统已到达目标。
     * 清零积分可防止在目标附近因积分残留导致的持续振荡。
     */
    if (pid_absf(error) <= pid->deadband)
    {
        error = 0.0f;
        pid->integral = 0.0f;
    }

    /*
     * 微分项计算：
     *   采用一阶向后差分法：derivative = (e(t) - e(t-1)) / dt
     *   然后通过一阶 IIR 低通滤波抑制高频噪声。
     *   首次调用时（has_prev_error == false）跳过微分计算，derivative 保持 0。
     */
    if (pid->has_prev_error)
    {
        derivative = (error - pid->prev_error) / dt_s;
        derivative = pid->derivative_lpf * derivative
            + (1.0f - pid->derivative_lpf) * pid->prev_derivative;
    }

    /*
     * 计算积分候选值：对 integral + error*dt 进行限幅。
     * 注意这里用的是 error*dt（误差的时间累计），而不是 error 本身。
     * 这样 ki 的单位是 [输出/误差·秒] 而非 [输出/误差]，更直观。
     */
    integral_candidate = pid_clampf(pid->integral + error * dt_s,
        pid->integral_min, pid->integral_max);
    integral_step_output = pid->ki * (integral_candidate - pid->integral);
    output = pid->kp * error + pid->ki * integral_candidate + pid->kd * derivative;

    /*
     * 条件积分（积分分离 anti-windup）：
     *
     * 如果输出已到上限（output_max）且积分步进仍为正方向（试图继续增加输出），
     * 或者输出已到下限（output_min）且积分步进为负方向（试图继续减少输出），
     * 则拒绝本次积分更新——保持原积分值不变。
     *
     * 这允许积分在输出退出饱和状态时发挥作用，
     * 但阻止其加剧饱和状态。
     */
    /* Keep integration that unwinds saturation, reject integration that
     * drives the output farther into either rail. */
    if ((output > pid->output_max && integral_step_output > 0.0f) ||
        (output < pid->output_min && integral_step_output < 0.0f))
    {
        /* Return the saturated candidate, but do not retain its integral. */
    }
    else
    {
        pid->integral = integral_candidate;
    }
    /* 最终输出限幅，确保不会因 P 项或 D 项超出范围 */
    output = pid_clampf(output, pid->output_min, pid->output_max);

    /* 保存状态，供下一周期使用 */
    pid->prev_error = error;
    pid->prev_derivative = derivative;
    pid->has_prev_error = true;

    return output;
}
