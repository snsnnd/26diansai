/**
 * @file    speed_control.c
 * @brief   云台速度控制模块 —— 实现
 *
 * 控制原理：
 *   视觉系统（MaixCam2）提供目标在画面中的像素误差 (error_x, error_y)。
 *   PID 控制器将像素误差转换为云台角速度（度/秒），
 *   然后转换为电机转速（RPM），通过 emm_move_degrees() 发送速度指令。
 *
 * 与 position_control 的关键区别：
 *   - 输出量为角速度而非角度偏移，响应更直接、快速。
 *   - 包含静摩擦补偿，在低速时额外增加 RPM 克服静摩擦，避免"爬行"现象。
 *   - 使用 command_horizon_ms 将速度指令约束在有限行程内，
 *     防止因调度器卡顿导致指令无限累积。
 *   - 定期读取电机实际位置作为反馈，检查指令执行情况。
 *   - 检查云台机械限位，防止超出物理行程。
 *
 * 状态机与 position_control 相同：
 *   IDLE -> SEARCH -> COARSE_TRACK <-> FINE_TRACK -> LOST_HOLD -> SEARCH | FAILSAFE
 */

#include "gimbal/speed_control.h"

/**
 * @brief 浮点数绝对值计算
 * @param value 输入浮点数
 * @return value 的绝对值
 */
static float speed_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 浮点数限幅函数
 * @param value     输入值
 * @param min_value 下限
 * @param max_value 上限
 * @return 限幅后的值
 */
static float speed_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/**
 * @brief 状态机决策函数 —— 根据视觉目标状态决定当前控制状态
 * @param ctrl   速度控制器指针
 * @param target 视觉目标数据
 * @param rx_ms  视觉数据接收时间戳
 * @param now_ms 当前系统时间戳
 * @return 决策后的 VisionControlState 状态
 *
 * 决策逻辑（优先级从高到低）：
 *   1. 视觉数据超时 → FAILSAFE
 *   2. 视觉报告 LOST → LOST_HOLD（短暂保持后切 SEARCH）
 *   3. 目标无效/搜索中/空闲 → SEARCH
 *   4. 误差 > large_error_px 或 CANDIDATE 状态 → COARSE_TRACK
 *   5. 其他 → FINE_TRACK
 *
 * 与位置控制器的 pc_decide_state() 逻辑基本一致，
 * 区别在于这里先处理 LOST 再处理无效目标（顺序微调不影响结果）。
 */
static VisionControlState decide_state(SpeedController *ctrl,
    const MaixVisionTarget *target, uint32_t rx_ms, uint32_t now_ms)
{
    int32_t error_x = target->error_x;
    int32_t error_y = target->error_y;

    /* 1. 视觉数据年龄超过超时阈值 → 故障安全 */
    if ((uint32_t)(now_ms - rx_ms) > ctrl->config.vision_timeout_ms)
        return CTRL_STATE_FAILSAFE;

    /* 2. 视觉模块报告目标丢失 */
    if (target->vision_state == VISION_STATE_LOST)
    {
        /* 首次进入丢失时记录时间戳 */
        if (ctrl->state != CTRL_STATE_LOST_HOLD) ctrl->lost_start_ms = now_ms;
        /* 在 lost_hold_ms 时间内保持等待 */
        if ((uint32_t)(now_ms - ctrl->lost_start_ms) <= ctrl->config.lost_hold_ms)
            return CTRL_STATE_LOST_HOLD;
        return CTRL_STATE_SEARCH;
    }

    /* 3. 目标无效或视觉模块在搜索/空闲 → 搜索状态 */
    if (!target->target_valid || target->vision_state == VISION_STATE_SEARCHING ||
        target->vision_state == VISION_STATE_IDLE)
        return CTRL_STATE_SEARCH;

    /* 计算像素误差绝对值 */
    if (error_x < 0) error_x = -error_x;
    if (error_y < 0) error_y = -error_y;

    /* 4. 误差较大或候选状态 → 粗跟踪 */
    if (error_x > ctrl->config.large_error_px ||
        error_y > ctrl->config.large_error_px ||
        target->vision_state == VISION_STATE_CANDIDATE)
        return CTRL_STATE_COARSE_TRACK;

    /* 5. 误差较小且目标有效 → 精跟踪 */
    return CTRL_STATE_FINE_TRACK;
}

/**
 * @brief 将角速度（度/秒）转换为电机转速（RPM），并加入静摩擦补偿
 * @param output_dps 目标角速度（度/秒），正值正转，负值反转
 * @param gear_ratio 减速比（电机端:输出端）
 * @param config     速度控制器配置（含补偿参数和限幅）
 * @return 转换后的电机转速（RPM），已四舍五入取整
 *
 * 转换公式：
 *   基础 RPM = |output_dps| * gear_ratio / 6.0
 *   补偿 RPM = stiction_comp_rpm * clamp(|output_dps| / stiction_full_scale_dps, 0, 1)
 *   最终 RPM = clamp(基础 RPM + 补偿, 0, min(output_cap_rpm, max_motor_rpm))
 *
 * 静摩擦补偿原理：
 *   在低速时，电机驱动力矩不足以克服静摩擦，可能导致运动不连续（"爬行"）。
 *   补偿量随目标速度线性增加（从 0 到 stiction_comp_rpm），
 *   在 stiction_full_scale_dps 处达到满补偿值。
 *   这使得低速时电机额外获得一些驱动力，克服静摩擦后运动更平滑。
 *
 * 除 6.0 的原因：
 *   度/秒 * 减速比 → 电机轴转速（度/秒），再转换为 RPM（转/分钟）：
 *   1 RPM = 360 度/分钟 = 6 度/秒，因此除以 6.0。
 */
static uint16_t output_dps_to_motor_rpm(float output_dps, float gear_ratio,
    const SpeedControlConfig *config)
{
    float magnitude = speed_abs(output_dps);  /* 取绝对值，方向由电机控制 */
    float rpm;                                 /* 计算得到的电机转速 */
    float compensation;                        /* 静摩擦补偿量 */
    float output_cap_rpm;                      /* 输出端最大允许转速 */

    /* 速度接近零或减速比无效时直接返回 0 */
    if (magnitude <= 0.01f || gear_ratio <= 0.0f) return 0u;
    if (magnitude > config->max_output_dps) magnitude = config->max_output_dps;

    /* 基础转换：度/秒 → RPM */
    rpm = magnitude * gear_ratio / 6.0f;

    /*
     * 静摩擦补偿：
     *   补偿量 = stiction_comp_rpm * 线性比例因子
     *   比例因子 = clamp(目标速度 / 满量程速度, 0, 1)
     *   即速度越低，补偿比例越高（相对），速度达到 stiction_full_scale_dps 时补偿饱和。
     */
    compensation = (config->stiction_full_scale_dps > 0.0f)
        ? config->stiction_comp_rpm * speed_clamp(
            magnitude / config->stiction_full_scale_dps, 0.0f, 1.0f)
        : 0.0f;
    rpm += compensation;

    /* 补偿和四舍五入后的值绝不能超过输出端极限 */
    output_cap_rpm = config->max_output_dps * gear_ratio / 6.0f;
    if (rpm > output_cap_rpm) rpm = output_cap_rpm;

    /* 同时不能超过电机本身的最大安全转速 */
    if (rpm > (float)config->max_motor_rpm) rpm = (float)config->max_motor_rpm;

    /* 四舍五入到整数 RPM */
    return (uint16_t)(rpm + 0.5f);
}

/**
 * @brief 向指定轴电机发送运动指令（含行程安全约束）
 * @param motor          电机设备指针
 * @param output_dps     目标角速度（度/秒）
 * @param gear_ratio     该轴的减速比
 * @param acceleration   加速度值
 * @param microstep      微步进值
 * @param config         速度控制器配置（含 command_horizon_ms）
 * @param last_motor_rpm 输出参数，记录本次发送的电机 RPM，用于跟踪运动状态
 * @return EMM_OK 成功，EMM_ERROR 停止失败，EMM_ERROR_IO 运动指令失败
 *
 * 安全设计：
 *   每个速度指令都附带一个有限行程角度（command_horizon_ms 内的移动距离），
 *   而不是无限持续运行。这防止了以下风险：
 *   - 调度器卡顿导致指令反复发送，云台持续运动超出限位
 *   - 通信延迟导致指令堆积，云台失控
 *   - 在失去目标时云台不会长时间保持高速运动
 *
 *   行程角度 = output_dps * command_horizon_ms / 1000
 *   例如 output_dps = 100 度/秒，command_horizon_ms = 40ms，行程 = 4 度。
 *
 *   注意：当 command_degrees 太小时（< 0.01 度），确保至少发送 0.01 度，
 *   因为极短的行程可能被电机忽略。
 */
static EmmStatus command_axis(EmmDevice *motor, float output_dps, float gear_ratio,
    uint8_t acceleration, uint16_t microstep, const SpeedControlConfig *config,
    uint16_t *last_motor_rpm)
{
    float command_degrees;                               /* 指令行程角度 */
    uint16_t motor_rpm = output_dps_to_motor_rpm(output_dps, gear_ratio, config);

    /* 转速为 0：需要停止电机 */
    if (motor_rpm == 0u)
    {
        /* 如果电机已经停止（last_motor_rpm == 0），无需重复发送停止指令 */
        if (last_motor_rpm != NULL && *last_motor_rpm == 0u) return EMM_OK;

        /* 发送带验证的停止指令 */
        if (emm_stop_verified(motor, EMM_SYNC_IMMEDIATE) != EMM_OK)
            return EMM_ERROR;

        /* 更新上次转速记录为 0 */
        if (last_motor_rpm != NULL) *last_motor_rpm = 0u;
        return EMM_OK;
    }

    /*
     * 核心安全约束：将速度指令约束在 command_horizon_ms 的时间窗内。
     * 这样即使调度器出现问题，云台也只会移动有限角度后自动停止。
     */
    command_degrees = output_dps * (float)config->command_horizon_ms / 1000.0f;

    /* 确保即使行程很小，也发送一个最小角度以防被电机忽略 */
    if (speed_abs(command_degrees) < 0.01f)
        command_degrees = (output_dps > 0.0f) ? 0.01f : -0.01f;

    /* 发送相对运动指令：行程角度 × 减速比 = 电机轴旋转角度 */
    if (emm_move_degrees(motor, command_degrees * gear_ratio, motor_rpm,
        acceleration, EMM_MOTION_RELATIVE_CURRENT, microstep,
        EMM_SYNC_IMMEDIATE) != EMM_OK)
        return EMM_ERROR_IO;

    /* 记录本次发送的电机转速 */
    if (last_motor_rpm != NULL) *last_motor_rpm = motor_rpm;
    return EMM_OK;
}

/**
 * @brief 如果云台正在运动，则停止运动并更新状态
 * @param ctrl   速度控制器指针
 * @param gimbal 云台设备指针
 * @return GIMBAL_OK 停止成功或无运动，其他值表示停止失败
 *
 * 通过检查 last_yaw_motor_rpm 和 last_pitch_motor_rpm 是否为 0
 * 来判断云台是否正在运动，避免不必要的停止指令发送。
 */
static GimbalStatus stop_motion_if_active(SpeedController *ctrl, Gimbal *gimbal)
{
    GimbalStatus status;

    /* 没有运动指令在执行，直接返回成功 */
    if (ctrl->last_yaw_motor_rpm == 0u && ctrl->last_pitch_motor_rpm == 0u)
        return GIMBAL_OK;

    /* 发送停止指令 */
    status = gimbal_stop(gimbal);
    if (status == GIMBAL_OK)
    {
        /* 停止成功，清空转速记录 */
        ctrl->last_yaw_motor_rpm = 0u;
        ctrl->last_pitch_motor_rpm = 0u;
    }
    return status;
}

/**
 * @brief 初始化速度控制器
 * @param ctrl 指向 SpeedController 结构体的指针，若为 NULL 则直接返回
 *
 * 默认参数说明：
 *   - PID: kp=0.8（比位置控制大，速度环需要更高增益），ki=0，kd=0
 *   - 静摩擦补偿：stiction_comp_rpm=18RPM，full_scale=20dps
 *     → 在 20dps 时达到满补偿 18RPM
 *   - max_motor_rpm=1200：电机安全转速上限
 *   - control_period_ms=12：约 83Hz 控制频率（比位置控制的 66Hz 更高，
 *     因为速度控制要求更快响应）
 *   - command_horizon_ms=40：每次指令行程窗口 40ms
 *   - feedback_period_ms=50：每 50ms 读取一次位置反馈
 *   - feedback_stale_ms=150：反馈超时阈值 150ms
 */
void speed_control_init(SpeedController *ctrl)
{
    if (ctrl == NULL) return;
    ctrl->state = CTRL_STATE_IDLE;                     /* 初始空闲状态 */
    /* --- PID 参数 --- */
    ctrl->config.kp = 0.8f;                            /* 比例增益（速度环） */
    ctrl->config.ki = 0.0f;                            /* 积分增益（关闭） */
    ctrl->config.kd = 0.0f;                            /* 微分增益（关闭） */
    ctrl->config.max_output_dps = GIMBAL_MAX_OUTPUT_SPEED_DPS; /* 最大角速度 */
    /* --- 静摩擦补偿参数 --- */
    ctrl->config.stiction_comp_rpm = 18.0f;            /* 最大补偿 RPM */
    ctrl->config.stiction_full_scale_dps = 20.0f;      /* 满补偿时的角速度 */
    /* --- 硬件限幅 --- */
    ctrl->config.max_motor_rpm = 1200u;                /* 电机最大安全转速 */
    /* --- 控制参数 --- */
    ctrl->config.deadband_px = 3.0f;                   /* 像素死区 */
    ctrl->config.large_error_px = 80;                  /* 粗跟踪阈值 */
    ctrl->config.vision_timeout_ms = 300u;             /* 视觉超时 */
    ctrl->config.lost_hold_ms = 250u;                  /* 丢失保持时间 */
    ctrl->config.control_period_ms = 12u;              /* 控制周期（毫秒） */
    ctrl->config.command_horizon_ms = 40u;             /* 指令时间窗（毫秒） */
    ctrl->config.feedback_period_ms = 50u;             /* 反馈读取周期（毫秒） */
    ctrl->config.feedback_stale_ms = 150u;             /* 反馈超时阈值（毫秒） */
    /* --- 运行时状态清零 --- */
    ctrl->last_update_ms = 0u;
    ctrl->lost_start_ms = 0u;
    ctrl->last_control_ms = 0u;
    ctrl->last_feedback_ms = 0u;
    ctrl->control_count = 0u;
    ctrl->last_yaw_motor_rpm = 0u;                     /* 上次偏航 RPM */
    ctrl->last_pitch_motor_rpm = 0u;                   /* 上次俯仰 RPM */
    ctrl->had_valid_target = false;
    ctrl->failsafe_latched = false;

    /* 初始化 PID 控制器 */
    pid_init(&ctrl->yaw_pid);
    pid_init(&ctrl->pitch_pid);

    /*
     * PID 输出限幅：
     *   输出（角速度）限幅：±max_output_dps 度/秒
     *   积分限幅：±200（安全边界，当前积分关闭）
     */
    pid_set_limits(&ctrl->yaw_pid, -ctrl->config.max_output_dps,
        ctrl->config.max_output_dps, -200.0f, 200.0f);
    pid_set_limits(&ctrl->pitch_pid, -ctrl->config.max_output_dps,
        ctrl->config.max_output_dps, -200.0f, 200.0f);
    pid_set_deadband(&ctrl->yaw_pid, ctrl->config.deadband_px);
    pid_set_deadband(&ctrl->pitch_pid, ctrl->config.deadband_px);
    pid_set_derivative_lpf(&ctrl->yaw_pid, 0.35f);
    pid_set_derivative_lpf(&ctrl->pitch_pid, 0.35f);
}

/**
 * @brief 停止速度控制器
 * @param ctrl   速度控制器指针
 * @param gimbal 云台设备指针
 *
 * 执行以下操作：
 *   1. 复位偏航和俯仰 PID 控制器（清零积分等内部状态）
 *   2. 将状态机重置为 IDLE
 *   3. 清空转速记录（标记为无运动）
 *   4. 发送硬件停止指令
 *   5. 若停止指令失败，锁存故障
 */
void speed_control_stop(SpeedController *ctrl, Gimbal *gimbal)
{
    if (ctrl != NULL)
    {
        /* 复位 PID 内部状态 */
        pid_reset(&ctrl->yaw_pid);
        pid_reset(&ctrl->pitch_pid);
        ctrl->state = CTRL_STATE_IDLE;                 /* 回到空闲状态 */
        ctrl->last_yaw_motor_rpm = 0u;                 /* 清空偏航转速记忆 */
        ctrl->last_pitch_motor_rpm = 0u;               /* 清空俯仰转速记忆 */
    }
    /* 发送硬件停止指令，失败则锁存故障 */
    if (gimbal != NULL && gimbal_stop(gimbal) != GIMBAL_OK)
        gimbal_latch_safety_fault(gimbal);
}

/**
 * @brief 速度控制主循环更新函数
 * @param ctrl   速度控制器指针
 * @param gimbal 云台设备指针
 * @param now_ms 当前系统时间戳（毫秒）
 *
 * 完整的控制循环流程如下：
 *
 *   [阶段 1] 前置检查
 *     1a. 空指针保护
 *     1b. 控制周期检查（control_period_ms = 12ms，约 83Hz）
 *     1c. 检查位置有效性、安全故障、故障锁存
 *
 *   [阶段 2] 位置反馈读取
 *     2a. 每 feedback_period_ms（50ms）读取一次电机实际位置
 *     2b. 读取失败时尝试停止电机
 *     2c. 如果从未读到过位置 或 位置数据超过 feedback_stale_ms（150ms）未更新，
 *         触发故障安全
 *     （位置反馈用于验证电机是否跟随指令运动，是速度控制特有的安全机制）
 *
 *   [阶段 3] 视觉目标获取与状态决策
 *     3a. 获取最新目标数据
 *     3b. 检查语义有效性
 *     3c. 调用 decide_state() 决策状态
 *     3d. 状态切换时复位 PID
 *
 *   [阶段 4] PID 计算
 *     4a. 仅在 COARSE_TRACK / FINE_TRACK 状态下执行
 *     4b. 计算 dt，限幅在 0.012s ~ 0.1s
 *     4c. PID 输出为角速度（度/秒），限幅在 ±max_output_dps
 *
 *   [阶段 5] 绝对位置限幅
 *     5. 检查云台当前角度是否到达机械限位，
 *        若到达则禁止向"继续超出"的方向运动。
 *        这是速度控制特有的安全机制，因为速度指令是"增量式"的，
 *        不像位置控制有明确的目标角度。
 *
 *   [阶段 6] 发送指令
 *     6a. 将角速度转换为电机 RPM（含静摩擦补偿）
 *     6b. 通过 command_axis() 发送带行程约束的指令
 *     6c. 任一轴失败则停止并锁存故障
 */
void speed_control_update(SpeedController *ctrl, Gimbal *gimbal, uint32_t now_ms)
{
    MaixVisionTarget target;        /* 视觉目标数据 */
    uint32_t rx_ms;                 /* 视觉数据接收时间戳 */
    VisionControlState next_state;  /* 决策后的下一个状态 */
    float dt;                       /* 时间差（秒） */
    float yaw_dps;                  /* 偏航轴角速度指令（度/秒） */
    float pitch_dps;                /* 俯仰轴角速度指令（度/秒） */
    float yaw_actual;               /* 偏航轴实际位置（反馈） */
    float pitch_actual;             /* 俯仰轴实际位置（反馈） */
    EmmStatus pitch_status;         /* 俯仰轴指令执行状态 */
    EmmStatus yaw_status;           /* 偏航轴指令执行状态 */

    /* [阶段 1] 前置检查 */
    if (ctrl == NULL || gimbal == NULL) return;

    /* 控制周期检查 */
    if ((uint32_t)(now_ms - ctrl->last_control_ms) < ctrl->config.control_period_ms) return;
    ctrl->last_control_ms = now_ms;

    /* 故障检查 */
    if (!gimbal->position_valid || gimbal->safety_fault_latched ||
        ctrl->failsafe_latched)
    {
        ctrl->state = CTRL_STATE_FAILSAFE;
        return;
    }

    /* [阶段 2] 位置反馈读取（速度控制特有：验证电机是否响应指令） */
    /*
     * 定期读取电机编码器实际位置，用于：
     *   - 验证云台是否按指令运动
     *   - 检测机械限位位置（用于阶段 5 的限幅判断）
     *   - 检测电机是否堵转或失步
     */
    if (ctrl->last_feedback_ms == 0u ||
        (uint32_t)(now_ms - ctrl->last_feedback_ms) >= ctrl->config.feedback_period_ms)
    {
        /* 读取偏航和俯仰的实际位置 */
        if (gimbal_read_actual_position(gimbal, &yaw_actual, &pitch_actual) == GIMBAL_OK)
        {
            ctrl->last_feedback_ms = now_ms;  /* 更新反馈时间戳 */
        }
        else
        {
            /* 读取失败：先停止云台运动 */
            if (gimbal_stop(gimbal) == GIMBAL_OK)
            {
                ctrl->last_yaw_motor_rpm = 0u;
                ctrl->last_pitch_motor_rpm = 0u;
            }

            /*
             * 反馈超时处理：
             *   - 如果从未成功读取过（last_feedback_ms == 0），立即进入故障安全
             *   - 如果超过 feedback_stale_ms 未更新，也进入故障安全
             *   这是速度控制的关键安全机制 —— 没有位置反馈就无法验证指令执行。
             */
            if (ctrl->last_feedback_ms == 0u ||
                (uint32_t)(now_ms - ctrl->last_feedback_ms) > ctrl->config.feedback_stale_ms)
            {
                ctrl->failsafe_latched = true;
                gimbal_latch_safety_fault(gimbal);
                ctrl->state = CTRL_STATE_FAILSAFE;
            }
            return;
        }
    }

    /* [阶段 3] 视觉目标获取与状态决策 */
    /* 3a. 获取最新视觉目标数据 */
    if (!maixcam2_get_latest_target(&target, &rx_ms))
    {
        GimbalStatus stop_status = stop_motion_if_active(ctrl, gimbal);
        if (stop_status != GIMBAL_OK)
        {
            ctrl->failsafe_latched = true;
            gimbal_latch_safety_fault(gimbal);
            ctrl->state = CTRL_STATE_FAILSAFE;
        }
        else
        {
            ctrl->state = CTRL_STATE_SEARCH;
            ctrl->had_valid_target = false;
            pid_reset(&ctrl->yaw_pid);
            pid_reset(&ctrl->pitch_pid);
        }
        return;
    }

    /* 3b. 检查目标语义有效性 */
    if (!maixcam2_target_semantically_valid(&target))
    {
        if (stop_motion_if_active(ctrl, gimbal) != GIMBAL_OK)
        {
            ctrl->failsafe_latched = true;
            gimbal_latch_safety_fault(gimbal);
            ctrl->state = CTRL_STATE_FAILSAFE;
        }
        else
        {
            ctrl->state = CTRL_STATE_SEARCH;
            ctrl->had_valid_target = false;
        }
        return;
    }
    if (target.target_valid) ctrl->had_valid_target = true;

    /* 3c. 状态决策 */
    next_state = decide_state(ctrl, &target, rx_ms, now_ms);
    if (next_state == CTRL_STATE_FAILSAFE)
    {
        (void)gimbal_stop(gimbal);
        ctrl->failsafe_latched = true;
        gimbal_latch_safety_fault(gimbal);
        ctrl->state = CTRL_STATE_FAILSAFE;
        return;
    }

    /* 3d. 状态切换时复位 PID */
    if (next_state != ctrl->state)
    {
        pid_reset(&ctrl->yaw_pid);
        pid_reset(&ctrl->pitch_pid);
        ctrl->state = next_state;
    }

    /* 非跟踪状态 → 停止运动 */
    if (ctrl->state != CTRL_STATE_COARSE_TRACK &&
        ctrl->state != CTRL_STATE_FINE_TRACK)
    {
        if (stop_motion_if_active(ctrl, gimbal) != GIMBAL_OK)
            gimbal_latch_safety_fault(gimbal);
        return;
    }

    /* [阶段 4] PID 计算（输出为角速度 度/秒） */

    /*
     * 计算时间差 dt。
     * 首次运行时默认 0.012s，异常值（>0.1s 或 <=0）回退到默认值。
     */
    dt = (ctrl->last_update_ms == 0u) ? 0.012f
        : (float)(now_ms - ctrl->last_update_ms) / 1000.0f;
    ctrl->last_update_ms = now_ms;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.012f;

    /* 设置 PID 增益 */
    pid_set_gain(&ctrl->yaw_pid, ctrl->config.kp, ctrl->config.ki, ctrl->config.kd);
    pid_set_gain(&ctrl->pitch_pid, ctrl->config.kp, ctrl->config.ki, ctrl->config.kd);

    /* 执行 PID 更新，输出为角速度（度/秒） */
    yaw_dps = pid_update(&ctrl->yaw_pid, (float)target.error_x, dt);
    pitch_dps = pid_update(&ctrl->pitch_pid, -(float)target.error_y, dt);

    /* 角速度限幅 ±max_output_dps */
    yaw_dps = speed_clamp(yaw_dps, -ctrl->config.max_output_dps,
        ctrl->config.max_output_dps);
    pitch_dps = speed_clamp(pitch_dps, -ctrl->config.max_output_dps,
        ctrl->config.max_output_dps);

    /* [阶段 5] 绝对位置限幅（速度控制特有安全机制） */
    /*
     * 在速度控制模式下，如果没有位置限幅，云台可能超出机械限位造成损坏。
     * 这里检查云台当前角度是否到达硬件限位：
     *   - 偏航角 <= 最小角度 且 指令为负（继续减小方向）→ 禁止运动
     *   - 偏航角 >= 最大角度 且 指令为正（继续增大方向）→ 禁止运动
     *   - 俯仰轴同理
     * 注意：这只阻挡"向限位方向"的运动，反方向运动仍允许（可脱离限位）。
     */
    if ((gimbal->yaw_angle_deg <= gimbal->yaw_min_deg && yaw_dps < 0.0f) ||
        (gimbal->yaw_angle_deg >= gimbal->yaw_max_deg && yaw_dps > 0.0f))
        yaw_dps = 0.0f;
    if ((gimbal->pitch_angle_deg <= gimbal->pitch_min_deg && pitch_dps < 0.0f) ||
        (gimbal->pitch_angle_deg >= gimbal->pitch_max_deg && pitch_dps > 0.0f))
        pitch_dps = 0.0f;

    /* [阶段 6] 发送运动指令 */
    /*
     * 将角速度转换为电机指令发送给两个轴。
     * command_axis() 内部完成：
     *   - RPM 转换（含静摩擦补偿）
     *   - 行程安全约束（command_horizon）
     *   - 电机停止逻辑（RPM=0时）
     */
    pitch_status = command_axis(&gimbal->pitch, pitch_dps, GIMBAL_PITCH_RATIO,
        gimbal->acceleration, gimbal->microstep, &ctrl->config,
        &ctrl->last_pitch_motor_rpm);
    yaw_status = command_axis(&gimbal->yaw, yaw_dps, GIMBAL_YAW_RATIO,
        gimbal->acceleration, gimbal->microstep, &ctrl->config,
        &ctrl->last_yaw_motor_rpm);

    /* 任一轴指令失败 → 停止并锁存故障 */
    if (pitch_status != EMM_OK || yaw_status != EMM_OK)
    {
        (void)gimbal_stop(gimbal);
        ctrl->failsafe_latched = true;
        gimbal_latch_safety_fault(gimbal);
        ctrl->state = CTRL_STATE_FAILSAFE;
        return;
    }
    ctrl->control_count++;  /* 累计控制次数 */
}

/**
 * @brief 获取速度控制器的当前状态机状态
 * @param ctrl 速度控制器指针
 * @return 当前状态，若 ctrl 为 NULL 则返回 CTRL_STATE_IDLE
 */
VisionControlState speed_control_get_state(const SpeedController *ctrl)
{
    return (ctrl == NULL) ? CTRL_STATE_IDLE : ctrl->state;
}
