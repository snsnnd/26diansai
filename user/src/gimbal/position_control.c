/**
 * @file    position_control.c
 * @brief   云台位置控制模块 —— 实现
 *
 * 控制原理：
 *   视觉系统（MaixCam2）提供目标在画面中的像素误差 (error_x, error_y)。
 *   PID 控制器将像素误差转换为云台应转动的角度偏移量（度），
 *   然后通过 gimbal_move_relative() 发送相对角度指令。
 *
 * 状态机：
 *   IDLE -> SEARCH -> COARSE_TRACK <-> FINE_TRACK -> LOST_HOLD -> SEARCH | FAILSAFE
 *   - 粗跟踪（COARSE_TRACK）：误差较大时，快速接近目标。
 *   - 精跟踪（FINE_TRACK）：误差较小时，精细调节锁定目标。
 *   - 目标丢失保持（LOST_HOLD）：视觉丢帧时短暂保持，等待恢复。
 *   - 故障安全（FAILSAFE）：严重异常时停止运动并锁死。
 *
 * 安全机制：
 *   - 输出限幅：max_delta_deg 限制单次角度偏移，防止突变。
 *   - 速率限制：max_output_dps * dt 限制每周期最大步长。
 *   - 堵转恢复：每秒检查电机是否堵转，尝试自动恢复。
 *   - 超时保护：视觉数据超时（vision_timeout_ms）触发故障安全。
 */

#include "gimbal/position_control.h"

/**
 * @brief 浮点数绝对值计算
 * @param v 输入浮点数
 * @return v 的绝对值
 */
static float pc_absf(float v) { return (v < 0.0f) ? -v : v; }

/**
 * @brief 浮点数限幅函数
 * @param v  输入值
 * @param lo 下限
 * @param hi 上限
 * @return 限幅后的值，当 v < lo 时返回 lo，v > hi 时返回 hi，否则返回 v
 */
static float pc_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * @brief 状态机决策函数 —— 根据视觉目标状态决定当前控制状态
 * @param ctrl  位置控制器指针，用于读取当前状态和配置
 * @param t     视觉目标数据
 * @param rx_ms 视觉数据接收时间戳
 * @param now_ms 当前系统时间戳
 * @return 决策后的 VisionControlState 状态
 *
 * 决策逻辑（按优先级从高到低）：
 *   1. 视觉数据年龄超过超时阈值 → FAILSAFE（数据彻底丢失）
 *   2. 目标无效或视觉正在搜索 → SEARCH（继续搜索）
 *   3. 视觉报告 LOST（目标丢失）→ LOST_HOLD（先短暂保持，超过 lost_hold_ms 后再搜索）
 *   4. 像素误差大于 large_error_px 或目标状态为 CANDIDATE（置信度低）
 *      → COARSE_TRACK（粗跟踪，快速接近）
 *   5. 其他情况（误差较小且目标有效）→ FINE_TRACK（精跟踪，精细锁定）
 *
 * 设计要点：
 *   - LOST_HOLD 状态只在从跟踪状态首次丢失时记录时间戳，
 *     避免了反复进入/退出 LOST_HOLD 导致的时间戳重置。
 *   - 粗/精跟踪的切换由误差大小决定，实现了从"快速接近"到"精细锁定"的渐进控制。
 */
static VisionControlState pc_decide_state(PositionController *ctrl,
    const MaixVisionTarget *t, uint32_t rx_ms, uint32_t now_ms)
{
    uint32_t age = now_ms - rx_ms;              /* 视觉数据年龄（当前时间 - 接收时间） */
    int16_t  ax, ay;                            /* 像素误差绝对值 */

    /* 1. 视觉数据超时 → 故障安全 */
    if (age > ctrl->config.vision_timeout_ms) return CTRL_STATE_FAILSAFE;

    /* 2. 目标无效或视觉模块正在搜索/空闲 → 搜索状态 */
    if (!t->target_valid || t->vision_state == VISION_STATE_SEARCHING ||
        t->vision_state == VISION_STATE_IDLE) return CTRL_STATE_SEARCH;

    /* 3. 视觉模块报告目标丢失 */
    if (t->vision_state == VISION_STATE_LOST)
    {
        /* 首次进入丢失状态时，记录丢失开始时间戳 */
        if (ctrl->state != CTRL_STATE_LOST_HOLD) ctrl->lost_start_ms = now_ms;
        /* 在丢失保持时间内，保持 LOST_HOLD 状态等待目标恢复 */
        if ((now_ms - ctrl->lost_start_ms) <= ctrl->config.lost_hold_ms)
            return CTRL_STATE_LOST_HOLD;
        /* 超过保持时间仍未恢复，切换回搜索模式 */
        return CTRL_STATE_SEARCH;
    }

    /* 计算像素误差绝对值 */
    ax = (t->error_x < 0) ? (int16_t)-t->error_x : t->error_x;
    ay = (t->error_y < 0) ? (int16_t)-t->error_y : t->error_y;

    /* 4. 误差较大或目标为初步候选 → 粗跟踪 */
    if (ax > ctrl->config.large_error_px || ay > ctrl->config.large_error_px ||
        t->vision_state == VISION_STATE_CANDIDATE)
        return CTRL_STATE_COARSE_TRACK;

    /* 5. 误差较小且目标有效 → 精跟踪 */
    return CTRL_STATE_FINE_TRACK;
}

/**
 * @brief 电机堵转检测与恢复
 * @param motor 电机设备指针
 * @param addr  电机地址
 * @return true 表示电机正常或已成功恢复，false 表示恢复失败
 *
 * 逻辑说明：
 *   1. 强制读取电机系统状态（绕过缓存）。
 *   2. 如果电机处于过温或过流保护状态，无法恢复，返回 false。
 *   3. 如果电机未使能且未检测到堵转/堵转保护，则视为"未运行"，返回 true（无需恢复）。
 *   4. 如果检测到堵转或堵转保护，发送清除堵转并恢复的指令。
 *   5. 其他情况（电机正常使能运行），返回 true。
 *
 * 该函数每秒被调用一次（见 position_control_update 中的周期检查），
 * 避免过于频繁地访问电机寄存器。
 */
static bool pc_recover_if_stalled(EmmDevice *motor, uint8_t addr)
{
    EmmSystemStatusParams st;

    /* 强制读取电机系统状态，不使用缓存 */
    if (emm_get_system_status_forced(motor, &st) != EMM_OK) return false;

    /* 过温或过流保护触发，无法安全恢复 */
    if (st.homing_status.over_temp || st.homing_status.over_current)
        return false;

    /* 电机未使能且未检测到堵转，视为正常停止，无需恢复 */
    if (!st.motor_status.enabled && !st.motor_status.stall_detected &&
        !st.motor_status.stall_protected)
        return false;

    /* 检测到堵转事件，执行清除堵转并恢复 */
    if (st.motor_status.stall_detected || st.motor_status.stall_protected)
    {
        emm_select_address(motor, addr);
        return emm_clear_stall_and_recover(motor) == EMM_OK;
    }

    /* 电机正常使能运行中 */
    return true;
}

/**
 * @brief 初始化位置控制器
 * @param ctrl 指向 PositionController 结构体的指针，若为 NULL 则直接返回
 *
 * 默认 PID 参数说明：
 *   - kp = 0.09：适中的比例增益，像素误差 100px 时输出约 9 度偏移
 *   - ki = 0.0：积分项关闭，因为位置控制本身无稳态误差，且积分可能引起超调
 *   - kd = 0.0：微分项关闭，系统响应已足够
 *   - max_delta_deg = 15.0：单次控制最大转动 15 度，防止大幅跳跃
 *   - deadband_px = 3.0：允许 3 像素误差，避免微小的像素抖动导致云台频繁微调
 *   - large_error_px = 80：误差超过 80 像素时使用粗跟踪模式
 *   - vision_timeout_ms = 300：300ms 未收到视觉数据视为超时
 *   - lost_hold_ms = 250：目标丢失后保持 250ms，给目标短暂恢复的机会
 *   - control_period_ms = 15：以约 66Hz 的频率执行控制
 */
void position_control_init(PositionController *ctrl)
{
    if (ctrl == 0) return;

    /* --- 初始状态与配置 --- */
    ctrl->state = CTRL_STATE_IDLE;             /* 初始为空闲状态 */
    ctrl->config.kp               = 0.09f;     /* 比例增益，决定响应速度 */
    ctrl->config.ki               = 0.0f;      /* 积分增益，关闭积分作用 */
    ctrl->config.kd               = 0.0f;      /* 微分增益，关闭微分作用 */
    ctrl->config.max_delta_deg    = 15.0f;     /* 单次最大角度偏移（度） */
    ctrl->config.max_output_dps   = GIMBAL_MAX_OUTPUT_SPEED_DPS; /* 最大输出角速度 */
    ctrl->config.deadband_px      = 3.0f;      /* 像素死区（像素） */
    ctrl->config.large_error_px   = 80;        /* 粗跟踪切换阈值（像素） */
    ctrl->config.vision_timeout_ms = 300u;     /* 视觉通信超时（毫秒） */
    ctrl->config.lost_hold_ms     = 250u;      /* 目标丢失保持时间（毫秒） */
    ctrl->config.control_period_ms = 15u;      /* 控制周期（毫秒） */
    /* --- 运行时状态清零 --- */
    ctrl->last_update_ms  = 0u;                /* 上次更新时间戳 */
    ctrl->lost_start_ms   = 0u;                /* 丢失开始时间戳 */
    ctrl->last_control_ms = 0u;                /* 上次控制输出时间戳 */
    ctrl->control_count   = 0u;                /* 控制次数统计 */
    ctrl->last_recovery_ms = 0u;               /* 上次堵转恢复检查时间戳 */
    ctrl->had_valid_target = false;            /* 有效目标标志 */
    ctrl->failsafe_latched = false;            /* 故障锁存标志 */
    ctrl->motion_active = false;               /* 运动活动标志 */

    /* 初始化偏航轴和俯仰轴的 PID 控制器 */
    pid_init(&ctrl->yaw_pid);
    pid_init(&ctrl->pitch_pid);

    /*
     * 设置 PID 输出限幅：
     *   输出（角度偏移）限幅：±max_delta_deg 度
     *   积分限幅：±100（当前积分关闭，ki=0），仅作安全边界
     */
    pid_set_limits(&ctrl->yaw_pid,   -ctrl->config.max_delta_deg, ctrl->config.max_delta_deg, -100.0f, 100.0f);
    pid_set_limits(&ctrl->pitch_pid, -ctrl->config.max_delta_deg, ctrl->config.max_delta_deg, -100.0f, 100.0f);

    /* 设置像素死区，误差小于死区时 PID 输出 0 */
    pid_set_deadband(&ctrl->yaw_pid,   ctrl->config.deadband_px);
    pid_set_deadband(&ctrl->pitch_pid, ctrl->config.deadband_px);

    /*
     * 设置微分项低通滤波器系数（截止频率约为 0.35 * 采样频率）。
     * 虽然当前 kd=0 微分关闭，但保留此设置以便在需要时启用。
     */
    pid_set_derivative_lpf(&ctrl->yaw_pid,   0.35f);
    pid_set_derivative_lpf(&ctrl->pitch_pid, 0.35f);
}

/**
 * @brief 位置控制主循环更新函数
 * @param ctrl   位置控制器指针
 * @param gimbal 云台设备指针，用于执行运动指令
 * @param now_ms 当前系统时间戳（毫秒）
 *
 * 完整的控制循环流程如下：
 *
 *   [阶段 1] 前置检查
 *     1a. 空指针保护
 *     1b. 控制周期检查（control_period_ms），确保固定频率执行
 *     1c. 检查 gimbal 位置有效性、安全故障状态、故障锁存状态
 *
 *   [阶段 2] 视觉目标获取
 *     2a. 从 MaixCam2 获取最新目标数据，如果获取失败：
 *         - 若先前有运动在执行，尝试停止云台
 *         - 停止失败则锁存故障，进入 FAILSAFE
 *         - 停止成功则进入 SEARCH 状态
 *         - 复位 PID 积分项
 *     2b. 检查目标语义有效性，无效则类似 2a 处理
 *
 *   [阶段 3] 状态决策
 *     3a. 调用 pc_decide_state() 决策下一个状态
 *     3b. 若决策为 FAILSAFE，停止云台并锁存故障
 *     3c. 若状态发生切换，复位 PID 积分项以防止积分突变
 *
 *   [阶段 4] PID 计算与执行
 *     4a. 仅在 COARSE_TRACK 或 FINE_TRACK 状态执行 PID 计算
 *     4b. 其他状态（SEARCH、LOST_HOLD、IDLE）→ 停止运动并返回
 *     4c. 计算时间差 dt，并进行合理限幅（0.05s ~ 0.2s）
 *     4d. 设置 PID 增益并执行 PID 更新
 *     4e. 对 PID 输出进行角度限幅和速率限幅
 *
 *   [阶段 5] 堵转恢复检查
 *     5. 每 1 秒检查一次电机堵转状态，自动尝试恢复
 *
 *   [阶段 6] 输出
 *     6a. 通过 gimbal_move_relative() 发送相对角度指令
 *     6b. 指令失败则停止云台并锁存故障
 *
 *   [阶段 7] 故障安全
 *     7. 如果任意环节检测到不可恢复的错误，锁存故障并停止一切运动
 */
void position_control_update(PositionController *ctrl, Gimbal *gimbal, uint32_t now_ms)
{
    MaixVisionTarget   t;            /* 视觉目标数据 */
    uint32_t           rx_ms;        /* 视觉数据接收时间戳 */
    VisionControlState ns;           /* 决策后的下一个状态 */
    float              dt, yaw_cmd, pitch_cmd;   /* 时间差、偏航/俯仰指令 */
    float              max_step;     /* 每周期最大角度步长 */

    /* [阶段 1] 前置检查 */
    /* 1a. 空指针保护 */
    if (ctrl == 0 || gimbal == 0) return;

    /* 1b. 控制周期检查：未到周期时间直接返回 */
    if ((now_ms - ctrl->last_control_ms) < ctrl->config.control_period_ms) return;
    ctrl->last_control_ms = now_ms;

    /* 1c. 故障检查：位置无效、安全故障或故障锁存时进入 FAILSAFE */
    if (!gimbal->position_valid || gimbal->safety_fault_latched ||
        ctrl->failsafe_latched)
    {
        ctrl->state = CTRL_STATE_FAILSAFE;
        return;
    }

    /* [阶段 2] 视觉目标获取 */
    /* 2a. 从视觉模块获取最新目标数据和接收时间戳 */
    if (!maixcam2_get_latest_target(&t, &rx_ms))
    {
        /* 获取失败：如果云台正在运动，尝试停止 */
        if (ctrl->motion_active && gimbal_stop(gimbal) != GIMBAL_OK)
        {
            /* 停止失败 → 锁存故障 */
            ctrl->failsafe_latched = true;
            gimbal_latch_safety_fault(gimbal);
            ctrl->state = CTRL_STATE_FAILSAFE;
        }
        else
        {
            /* 停止成功或本来就没有运动 → 进入搜索状态 */
            ctrl->state = CTRL_STATE_SEARCH;
            ctrl->motion_active = false;
            ctrl->had_valid_target = false;
        }
        /* 无论成功与否，复位 PID 积分项 */
        pid_reset(&ctrl->yaw_pid);
        pid_reset(&ctrl->pitch_pid);
        return;
    }

    /* 2b. 检查目标数据语义有效性 */
    if (!maixcam2_target_semantically_valid(&t))
    {
        /* 与获取失败相同的处理逻辑 */
        if (ctrl->motion_active && gimbal_stop(gimbal) != GIMBAL_OK)
        {
            ctrl->failsafe_latched = true;
            gimbal_latch_safety_fault(gimbal);
            ctrl->state = CTRL_STATE_FAILSAFE;
        }
        else
        {
            ctrl->state = CTRL_STATE_SEARCH;
            ctrl->motion_active = false;
            ctrl->had_valid_target = false;
        }
        return;
    }

    /* 记录曾经获取到有效目标 */
    if (t.target_valid) ctrl->had_valid_target = true;

    /* [阶段 3] 状态决策 */
    ns = pc_decide_state(ctrl, &t, rx_ms, now_ms);

    /* 若状态机决策为故障安全 */
    if (ns == CTRL_STATE_FAILSAFE)
    {
        (void)gimbal_stop(gimbal);
        ctrl->failsafe_latched = true;
        gimbal_latch_safety_fault(gimbal);
        ctrl->state = CTRL_STATE_FAILSAFE;
        return;
    }

    /* 状态发生切换时，复位 PID 积分项以防止积分累积突变 */
    if (ns != ctrl->state)
    {
        pid_reset(&ctrl->yaw_pid);
        pid_reset(&ctrl->pitch_pid);
        ctrl->state = ns;
    }

    /* [阶段 4] PID 计算与执行 */

    /*
     * 计算时间差 dt（秒）。
     * 首次运行时 (last_update_ms == 0)，使用默认值 0.05s。
     * 如果 dt 超出合理范围（> 0.2s 或 <= 0.0s），说明时间戳异常，
     * 回退到默认值 0.05s 以防止 PID 积分项异常累积。
     */
    dt = (ctrl->last_update_ms == 0u) ? 0.05f
        : ((float)(now_ms - ctrl->last_update_ms) / 1000.0f);
    if (dt <= 0.0f || dt > 0.2f) dt = 0.05f;
    ctrl->last_update_ms = now_ms;

    /* 仅在粗跟踪和精跟踪状态下执行 PID 控制 */
    switch (ctrl->state)
    {
        case CTRL_STATE_COARSE_TRACK:
        case CTRL_STATE_FINE_TRACK:
            /* 设置 PID 增益（目前仅使用比例项 kp） */
            pid_set_gain(&ctrl->yaw_pid,   ctrl->config.kp, ctrl->config.ki, ctrl->config.kd);
            pid_set_gain(&ctrl->pitch_pid, ctrl->config.kp, ctrl->config.ki, ctrl->config.kd);
            break;

        default:
            /* 搜索、丢失保持、空闲状态 → 停止运动 */
            pid_reset(&ctrl->yaw_pid);
            pid_reset(&ctrl->pitch_pid);
            if (ctrl->motion_active && gimbal_stop(gimbal) != GIMBAL_OK)
                gimbal_latch_safety_fault(gimbal);
            ctrl->motion_active = false;
            return;
    }

    /* [阶段 5] 堵转恢复检查：每秒执行一次 */
    if ((now_ms - ctrl->last_recovery_ms) >= 1000u)
    {
        ctrl->last_recovery_ms = now_ms;

        /* 依次检查俯仰轴和偏航轴电机 */
        if (!pc_recover_if_stalled(&gimbal->pitch, GIMBAL_PITCH_MOTOR_ADDRESS) ||
            !pc_recover_if_stalled(&gimbal->yaw, GIMBAL_YAW_MOTOR_ADDRESS))
        {
            /* 任一电机恢复失败 → 停止并锁存故障 */
            (void)gimbal_stop(gimbal);
            ctrl->failsafe_latched = true;
            gimbal_latch_safety_fault(gimbal);
            ctrl->state = CTRL_STATE_FAILSAFE;
            return;
        }
    }

    /* [阶段 6] PID 计算与输出限幅 */

    /*
     * 计算像素误差并执行 PID 更新。
     * 偏航轴使用 error_x（左右偏差），俯仰轴使用 -error_y（上下偏差，取反使方向一致）。
     */
    {
        float ex = (float)t.error_x;     /* 偏航像素误差 */
        float ey = -(float)t.error_y;    /* 俯仰像素误差（取反） */

        /*
         * 安全过滤：当目标无效、视觉处于搜索/丢失/空闲状态、
         * 或误差绝对值超过 500 像素（数据异常）时，强制归零输出。
         * 避免异常数据导致云台突然大幅运动。
         */
        if (!t.target_valid || t.vision_state == VISION_STATE_SEARCHING ||
            t.vision_state == VISION_STATE_LOST || t.vision_state == VISION_STATE_IDLE ||
            ex > 500.0f || ex < -500.0f || ey > 500.0f || ey < -500.0f)
        {
            ex = 0.0f;
            ey = 0.0f;
        }

        /* 执行 PID 更新，输出为角度偏移量（度） */
        yaw_cmd   = pid_update(&ctrl->yaw_pid,   ex, dt);
        pitch_cmd = pid_update(&ctrl->pitch_pid, ey, dt);
    }

    /* 角度限幅：确保单次输出不超过 max_delta_deg */
    yaw_cmd   = pc_clampf(yaw_cmd,   -ctrl->config.max_delta_deg, ctrl->config.max_delta_deg);
    pitch_cmd = pc_clampf(pitch_cmd, -ctrl->config.max_delta_deg, ctrl->config.max_delta_deg);

    /*
     * 速率限幅：根据最大角速度和实际时间间隔，计算每周期最大允许步长。
     * 例如 max_output_dps = 100 度/秒，dt = 0.05s，则 max_step = 5 度。
     * 这确保了即使 PID 输出很大，云台运动也是平滑的。
     */
    max_step = ctrl->config.max_output_dps * dt;
    if (max_step > ctrl->config.max_delta_deg) max_step = ctrl->config.max_delta_deg;
    yaw_cmd = pc_clampf(yaw_cmd, -max_step, max_step);
    pitch_cmd = pc_clampf(pitch_cmd, -max_step, max_step);

    /* [阶段 7] 执行运动指令 */
    if ((pc_absf(yaw_cmd) > 0.001f || pc_absf(pitch_cmd) > 0.001f) &&
        gimbal_move_relative(gimbal, yaw_cmd, pitch_cmd) == GIMBAL_OK)
    {
        /* 指令发送成功 */
        ctrl->control_count++;          /* 累计控制次数 */
        ctrl->motion_active = true;     /* 标记运动激活 */
    }
    else if (pc_absf(yaw_cmd) > 0.001f || pc_absf(pitch_cmd) > 0.001f)
    {
        /*
         * 有运动指令但执行失败 → 锁存故障。
         * 注意：当指令接近 0 时（>0.001f 判断），不视为故障，
         * 因为静止状态下发送零指令失败是可以接受的。
         */
        (void)gimbal_stop(gimbal);
        ctrl->failsafe_latched = true;
        gimbal_latch_safety_fault(gimbal);
        ctrl->state = CTRL_STATE_FAILSAFE;
    }
}

/**
 * @brief 获取位置控制器的当前状态机状态
 * @param ctrl 位置控制器指针
 * @return 当前状态（CTRL_STATE_IDLE ~ CTRL_STATE_FAILSAFE），
 *         若 ctrl 为 NULL 则返回 CTRL_STATE_IDLE
 */
VisionControlState position_control_get_state(const PositionController *ctrl)
{
    return (ctrl == 0) ? CTRL_STATE_IDLE : ctrl->state;
}
