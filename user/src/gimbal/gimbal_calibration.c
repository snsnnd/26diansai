/**
 * @file    gimbal_calibration.c
 * @brief   云台自动校准模块
 *
 * 本文件实现了云台（Gimbal）的自动校准功能，适用于基于 MSPM0G3507 的智能车项目。
 * 支持两种校准方式：
 *   1. gimbal_auto_calibrate() —— 全自动校准，通过堵转检测（stall detection）探索
 *      每个轴（PITCH / YAW）的机械极限，计算行程范围和中点。
 *   2. gimbal_calibrate_geared() —— 手动/引导式校准，适用于带减速齿轮的系统，
 *      由用户手动操作至极限位置，软件记录编码器读数并生成 #define 常量。
 *
 * 校准核心算法（gimbal_calibrate_axis）：
 *   - 方向测试：短距离 CW 点动，验证电机运转方向是否正确。
 *   - CW 极限探索：CW 持续点动，通过堵转检测或位置停滞判断机械限位。
 *   - CCW 极限探索：以步进式绝对位置移动逐步向 CCW 方向推进，同样检测堵转/停滞。
 *   - 多轮次运动：执行多轮（config.explore_attempts，1~5 轮）探索，取极值平均，
 *     以消除机械回差等不一致性。
 *   - 堵转恢复：每轮到达极限后调用 emm_clear_stall_and_recover 清除驱动器的
 *     堵转保护状态。
 *   - 限位回退：从极限位置反向移动 20 度，避免机械卡死。
 *   - 行程与中点计算：根据多轮 CW/CCW 极限值计算总行程和编码器中点。
 *
 * 当 GIMBAL_ENABLE_CALIBRATION 为 0 时，所有函数提供空桩实现。
 */
#include "gimbal/gimbal.h"
#include "config.h"

#if GIMBAL_ENABLE_CALIBRATION

/**
 * @brief  中止校准过程并清理状态
 *
 * 当校准过程中发生任何错误时调用此函数。它将：
 *   - 将所有校准标记（calibrated）清零
 *   - 清除云台的归位（homed）、位置有效（position_valid）和反馈有效（feedback_valid）标志
 *   - 立即停止 YAW 和 PITCH 两个电机
 *   - 禁用云台电机输出
 *   - 如果停止或禁用操作失败，触发安全故障锁存（safety fault latch）
 *
 * @param gimbal  云台对象指针，不能为 NULL
 * @param result  校准结果结构体指针（可为 NULL），若非 NULL 则设置 calibrated = false
 * @return 始终返回 GIMBAL_ERROR_CALIB
 */
static GimbalStatus gimbal_calibration_abort(Gimbal *gimbal,
                                             GimbalAxisCalib *result)
{
    EmmStatus yaw_stop = EMM_ERROR_INVALID_ARG;      /* YAW 电机停止结果 */
    EmmStatus pitch_stop = EMM_ERROR_INVALID_ARG;    /* PITCH 电机停止结果 */
    GimbalStatus disable_status = GIMBAL_ERROR;      /* 云台禁用结果 */

    if (result != NULL) result->calibrated = false;   /* 标记校准未完成 */
    if (gimbal == NULL) return GIMBAL_ERROR_CALIB;   /* 空指针保护 */

    /* 清除所有校准和状态标志 */
    gimbal->calib_yaw.calibrated = false;
    gimbal->calib_pitch.calibrated = false;
    gimbal->geared_yaw.calibrated = false;
    gimbal->geared_pitch.calibrated = false;
    gimbal->homed = false;
    gimbal->position_valid = false;
    gimbal->feedback_valid = false;

    /* 立即停止两个电机 */
    yaw_stop = emm_stop_verified(&gimbal->yaw, EMM_SYNC_IMMEDIATE);
    pitch_stop = emm_stop_verified(&gimbal->pitch, EMM_SYNC_IMMEDIATE);
    disable_status = gimbal_enable(gimbal, false);
    if (yaw_stop != EMM_OK || pitch_stop != EMM_OK ||
        disable_status != GIMBAL_OK)
    {
        /* 停止或禁用失败，锁存安全故障 */
        gimbal_latch_safety_fault(gimbal);
    }
    return GIMBAL_ERROR_CALIB;
}

/**
 * @brief  读取电机编码器值并转换为角度（度）
 *
 * 强制读取编码器原始值（16 位，0~65535），然后转换为 0~360 度角度值。
 * 转换公式：角度 = (编码器值 / 65536) * 360
 *
 * @param motor  电机设备指针
 * @param deg    输出参数，存储转换后的角度值（度），不能为 NULL
 * @return EMM_OK 表示成功，否则返回错误码
 */
static EmmStatus read_encoder_deg(EmmDevice *motor, float *deg)
{
    uint16_t encoder;                              /* 编码器原始计数值 (0~65535) */
    EmmStatus s = emm_get_encoder_forced(motor, &encoder);
    if (s == EMM_OK && deg != 0)
    {
        /* 将 16 位编码器值转换为 0~360 度的浮点角度 */
        *deg = ((float)encoder * 360.0f) / 65536.0f;
    }
    return s;
}

/**
 * @brief  驱动电机相对运动并验证实际到位精度
 *
 * 这是带位置验证的增量运动函数。执行流程：
 *   1. 读取运动前编码器角度作为基准
 *   2. 忽略过小的运动指令（< 0.05 度视为到位）
 *   3. 下发相对运动指令（emm_move_degrees）
 *   4. 轮询等待电机到位，超时 15 秒
 *   5. 在等待过程中检查过温、过流、堵转等异常状态
 *   6. 到位后读取编码器验证实际位移与期望位移的偏差
 *   7. 偏差超过 ±2 度视为失败
 *
 * @param motor        电机设备指针
 * @param delta_deg    相对运动角度（度），正值 = CW，负值 = CCW
 * @param speed_rpm    运动速度（RPM）
 * @param acceleration 加速度
 * @param microstep    细分步数
 * @return GIMBAL_OK 表示运动成功且位置偏差在允许范围内，否则返回错误码
 */
static GimbalStatus gimbal_move_axis_delta_verified(EmmDevice *motor,
                                                     float delta_deg,
                                                     uint16_t speed_rpm,
                                                     uint8_t acceleration,
                                                     uint16_t microstep)
{
    float current_enc;                     /* 运动前编码器角度（度） */
    float verify_enc;                      /* 运动后编码器角度（度），用于验证 */
    float expected_enc;                    /* 期望到达的编码器角度（度） */
    uint32_t elapsed_ms = 0u;             /* 已等待时间（毫秒） */
    bool reached = false;                  /* 是否已到达目标位置标志 */
    EmmStatus move_status;                 /* 运动指令返回状态 */
    EmmSystemStatusParams status;          /* 系统状态参数（含电机状态） */

    /* 读取运动前的编码器位置作为基准 */
    if (read_encoder_deg(motor, &current_enc) != EMM_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }

    /* 运动量过小（< 0.05 度），认为已经到位，直接返回成功 */
    if (delta_deg > -0.05f && delta_deg < 0.05f)
    {
        return GIMBAL_OK;
    }

    /* 下发相对运动指令（非阻塞立即返回） */
    move_status = emm_move_degrees(motor, delta_deg, speed_rpm, acceleration,
                                   EMM_MOTION_RELATIVE_CURRENT, microstep,
                                   EMM_SYNC_IMMEDIATE);
    if (move_status != EMM_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }

    /* 轮询等待电机到位，最长等待 15 秒 */
    while (elapsed_ms < 15000u)
    {
        system_delay_ms(50u);               /* 每 50ms 查询一次状态 */
        elapsed_ms += 50u;
        /* 检查异常状态：通信失败、过温、过流、堵转检测、堵转保护、意外失能 */
        if (emm_get_system_status_forced(motor, &status) != EMM_OK ||
            status.homing_status.over_temp || status.homing_status.over_current ||
            status.motor_status.stall_detected || status.motor_status.stall_protected ||
            (!status.motor_status.enabled && !status.motor_status.position_reached))
        {
            return GIMBAL_ERROR_MOTOR;
        }
        /* 检查电机是否已到达目标位置且速度已降至接近零 */
        if (status.motor_status.position_reached &&
            status.realtime_speed_rpm >= -1 && status.realtime_speed_rpm <= 1)
        {
            reached = true;
            break;
        }
    }
    /* 如果超时未到位，或停止/读取编码器失败，则返回错误 */
    if (!reached || emm_stop_verified(motor, EMM_SYNC_IMMEDIATE) != EMM_OK ||
        read_encoder_deg(motor, &verify_enc) != EMM_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }

    /* 计算期望位置（基准 + 增量），并归一化到 0~360 度范围 */
    expected_enc = current_enc + delta_deg;
    while (expected_enc >= 360.0f) expected_enc -= 360.0f;
    while (expected_enc < 0.0f) expected_enc += 360.0f;
    /* 计算实际位置与期望位置的最短角度差（处理 0/360 环绕） */
    delta_deg = verify_enc - expected_enc;
    if (delta_deg > 180.0f) delta_deg -= 360.0f;
    if (delta_deg < -180.0f) delta_deg += 360.0f;
    /* 偏差在 ±2 度以内认为成功，否则报电机错误 */
    return (delta_deg >= -2.0f && delta_deg <= 2.0f)
        ? GIMBAL_OK : GIMBAL_ERROR_MOTOR;
}

/**
 * @brief  内部辅助函数 —— 将指定轴运动到绝对目标角度
 *
 * 该函数计算当前编码器位置与目标绝对角度之间的最短路径（考虑 0/360 环绕），
 * 然后调用 gimbal_move_axis_delta_verified 执行带验证的相对运动。
 *
 * @param motor        电机设备指针
 * @param target_deg   目标绝对角度（度），范围 0~360
 * @param speed_rpm    运动速度（RPM）
 * @param acceleration 加速度
 * @param microstep    细分步数
 * @param is_pitch     是否 PITCH 轴（当前未使用，保留参数）
 * @return GIMBAL_OK 表示成功，否则返回错误码
 */
/* ================================================================
 *  内部辅助 —— 将单个轴移动到绝对位置
 * ================================================================ */
static GimbalStatus gimbal_move_axis_to_position(EmmDevice *motor, float target_deg,
                                                   uint16_t speed_rpm, uint8_t acceleration,
                                                   uint16_t microstep, bool is_pitch)
{
    float current_enc;      /* 当前编码器角度（度） */
    float delta_deg;        /* 从当前位置到目标位置的最短相对角度 */
    (void)is_pitch;         /* 未使用的参数，保留供将来扩展 */

    /* 读取当前位置 */
    if (read_encoder_deg(motor, &current_enc) != EMM_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }

    /* 计算最短路径的增量角度，处理 0/360 度环绕 */
    delta_deg = target_deg - current_enc;
    if (delta_deg > 180.0f)  { delta_deg -= 360.0f; }   /* 如果正向超过 180 度，反向更短 */
    if (delta_deg < -180.0f) { delta_deg += 360.0f; }   /* 如果负向超过 180 度，正向更短 */
    return gimbal_move_axis_delta_verified(motor, delta_deg, speed_rpm,
                                            acceleration, microstep);
}

/**
 * @brief  核心校准函数 —— 对单个轴进行全自动限位探索
 *
 * 本函数执行云台单个轴（PITCH 或 YAW）的机械限位探索校准，包含以下步骤：
 *
 *   1. 方向测试（Direction Test）：
 *      短时间 CW 点动，对比前后编码器读数，让用户目视确认电机旋转方向正确。
 *      如果方向错误，应通过 emm_set_motor_direction() 修正。
 *
 *   2. CW 极限探索（CW Limit Exploration）：
 *      - 以持续点动（jog）方式向 CW 方向运动。
 *      - 每 stall_check_ms 毫秒读取一次编码器位置和电机状态。
 *      - 检测到堵转（stall_detected / stall_protected）或位置停滞（连续 3 次
 *        采样位移 < 1 度）即判定到达机械限位。
 *      - 超时（stall_timeout_ms）未到限位则中止。
 *
 *   3. 堵转恢复与回退（Stall Recovery & Back-off）：
 *      - 调用 emm_clear_stall_and_recover() 清除驱动器的堵转保护状态（最多重试 5 次）。
 *      - 反向运动 20 度，避免机械卡死。
 *
 *   4. CCW 极限探索（CCW Limit Exploration）：
 *      - 以步进方式向 CCW 方向推进：每次以当前位置为基准，向 CCW 移动 15 度。
 *      - 每步到达后检查堵转/停滞状态，与 CW 方向检测逻辑相同。
 *      - 目标角度限制在 -180 度以上，防止过度旋转。
 *
 *   5. 多轮次（Multi-round）：
 *      以上 2~4 步重复执行 explore_attempts 次（1~5 轮），每轮记录 CW 和 CCW 的极限位置。
 *      多轮平均可消除机械回差、齿轮间隙等非线性因素。
 *
 *   6. 数据处理：
 *      - 从多轮数据中取 CW 最大值（最远的正极限）和 CCW 最小值（最远的负极限）。
 *      - 计算总行程 = CW_max - CCW_min（处理 360 度环绕）。
 *      - 计算编码器中点 = (CW_max + CCW_min) / 2。
 *      - 行程小于 1 度或大于 360 度视为无效。
 *
 *   7. 最终归中：将轴移动到计算出的中点位置。
 *
 * @param gimbal    云台对象指针
 * @param motor     待校准的电机设备指针（PITCH 或 YAW）
 * @param axis_name 轴名称字符串（如 "PITCH" 或 "YAW"），仅用于调试打印
 * @param result    校准结果输出结构体，包含 min_deg、max_deg、range_deg、mid_deg
 * @return GIMBAL_OK 校准成功，GIMBAL_ERROR_CALIB 校准失败
 */
GimbalStatus gimbal_calibrate_axis(Gimbal *gimbal, EmmDevice *motor,
                                   const char *axis_name, GimbalAxisCalib *result)
{
    float positives[5];          /* 每轮 CW 极限位置数组（最多 5 轮） */
    float negatives[5];          /* 每轮 CCW 极限位置数组（最多 5 轮） */
    float pos;                   /* 当前读取的编码器角度 */
    float prev_pos;              /* 上一轮采样时的编码器角度，用于停滞检测 */
    uint8_t stuck_cnt;           /* 连续停滞采样计数 */
    EmmMotorStatus st;           /* 电机状态结构体，包含 stall_detected 等标志 */
    int32_t pos_d;               /* 角度值放大 10 倍后的整数，用于调试打印 */

    /* 参数有效性检查：explore_attempts 必须在 1~5 之间 */
    if (gimbal == NULL || motor == NULL || axis_name == NULL || result == NULL ||
        gimbal->calib_config.explore_attempts == 0u ||
        gimbal->calib_config.explore_attempts > 5u)
    {
        return GIMBAL_ERROR_CALIB;
    }

    /* 初始化：标记校准未完成，清除云台状态 */
    result->calibrated = false;
    gimbal->homed = false;
    gimbal->position_valid = false;
    gimbal->feedback_valid = false;

    /* 打印校准参数信息 */
    printf("\r\n[CALIB] ========================================\r\n");
    printf("[CALIB]  Calibrating %s (Addr=%u)\r\n", axis_name, (unsigned int)motor->address);
    printf("[CALIB]  Speed: %u RPM | Rounds: %u\r\n",
           (unsigned int)gimbal->calib_config.explore_speed_rpm,
           (unsigned int)gimbal->calib_config.explore_attempts);
    printf("[CALIB]  Stall check: %lu ms | Timeout: %lu ms\r\n",
           (unsigned long)gimbal->calib_config.stall_check_ms,
           (unsigned long)gimbal->calib_config.stall_timeout_ms);
    printf("[CALIB] ========================================\r\n\r\n");

    /* === 方向测试（一次性，用户目视观察） === */
    /* 通过短时间 CW 点动并对比编码器前后读数，验证电机运转方向是否符合预期。 */
    printf("[CALIB] [%s] *** DIR TEST: small CW move ***\r\n", axis_name);
    {
        float before_test = 0.0f;   /* 点动前的编码器角度 */
        float after_test = 0.0f;    /* 点动后的编码器角度 */
        EmmJogParams test_jog;      /* 点动参数结构体 */
        EmmStatus read_sts;         /* 编码器读取状态 */

        /* 确保电机已停止，然后读取初始位置 */
        if (emm_stop_verified(motor, EMM_SYNC_IMMEDIATE) != EMM_OK)
            return gimbal_calibration_abort(gimbal, result);
        system_delay_ms(200u);
        read_sts = read_encoder_deg(motor, &before_test);
        if (read_sts != EMM_OK)
            return gimbal_calibration_abort(gimbal, result);
        {
            int32_t d = (int32_t)(before_test * 10.0f);
            printf("[CALIB] [%s] init pos read: %s(%d) val=%ld.%ld deg\r\n", axis_name,
                   (read_sts == EMM_OK) ? "ok" : "FAIL", (int)read_sts,
                   (long)(d / 10), (long)((d < 0 ? -d : d) % 10));
        }

        /* 配置 CW 点动参数：使用校准配置中的速度和加速度 */
        test_jog.direction    = EMM_DIRECTION_CW;
        test_jog.speed_rpm    = gimbal->calib_config.explore_speed_rpm;
        test_jog.acceleration = gimbal->calib_config.explore_acceleration;
        test_jog.sync_flag    = EMM_SYNC_IMMEDIATE;
        /* 开始 CW 点动（无需等待响应） */
        if (emm_jog_no_response(motor, &test_jog) != EMM_OK)
            return gimbal_calibration_abort(gimbal, result);
        system_delay_ms(150u);           /* 让电机运行一小段时间 */
        /* 停止电机 */
        if (emm_stop_verified(motor, EMM_SYNC_IMMEDIATE) != EMM_OK)
            return gimbal_calibration_abort(gimbal, result);
        system_delay_ms(200u);

        /* 读取方向测试后的编码器位置 */
        read_sts = read_encoder_deg(motor, &after_test);
        if (read_sts != EMM_OK)
            return gimbal_calibration_abort(gimbal, result);
        {
            int32_t d = (int32_t)(after_test * 10.0f);
            int32_t dd = (int32_t)((after_test - before_test) * 10.0f);
            printf("[CALIB] [%s] after move: %s(%d) val=%ld.%ld deg\r\n", axis_name,
                   (read_sts == EMM_OK) ? "ok" : "FAIL", (int)read_sts,
                   (long)(d / 10), (long)((d < 0 ? -d : d) % 10));
            printf("[CALIB] [%s] delta: %ld.%ld deg\r\n", axis_name,
                   (long)(dd / 10), (long)((dd < 0 ? -dd : dd) % 10));
        }
        /* 提示用户目视确认电机方向是否正确 */
        printf("[CALIB] [%s] *** Watch the motor direction! ***\r\n", axis_name);
        printf("[CALIB] [%s] If direction is wrong, use emm_set_motor_direction() to flip\r\n\r\n",
               axis_name);
    }

    /* 短暂延时后开始多轮限位探索 */
    system_delay_ms(1500u);

    /* ===================================================================
     *  多轮限位探索循环
     *  每轮执行：CW 探索 -> 堵转恢复 -> 反向回退 -> CCW 步进探索 -> 返回中点
     *  重复 explore_attempts 次以平均机械回差
     * =================================================================== */
    for (uint8_t round = 1u; round <= gimbal->calib_config.explore_attempts; ++round)
    {
        uint32_t elapsed;               /* 当前轮次已用时间（毫秒） */

        printf("[CALIB] --- Round %u/%u ---\r\n",
               (unsigned int)round,
               (unsigned int)gimbal->calib_config.explore_attempts);

        /* ----------------------------------------------------------------
         *  第 1 阶段：CW 极限探索
         *  以持续点动方式向 CW 方向运动，通过堵转检测或停滞检测判断限位
         * ---------------------------------------------------------------- */
        printf("[CALIB] [%s] >>> CW exploration, speed=%uRPM <<<\r\n",
               axis_name, (unsigned int)gimbal->calib_config.explore_speed_rpm);

        {
            EmmJogParams jog_cw;
            /* 配置 CW 持续点动参数 */
            jog_cw.direction    = EMM_DIRECTION_CW;
            jog_cw.speed_rpm    = gimbal->calib_config.explore_speed_rpm;
            jog_cw.acceleration = gimbal->calib_config.explore_acceleration;
            jog_cw.sync_flag    = EMM_SYNC_IMMEDIATE;
            if (emm_jog_no_response(motor, &jog_cw) != EMM_OK)
                return gimbal_calibration_abort(gimbal, result);
        }

        /* 轮询检测循环：监控编码器位置和电机状态，判断是否到达机械限位 */
        elapsed = 0u;
        stuck_cnt = 0u;                 /* 重置停滞计数器 */
        prev_pos = 999.0f;             /* 重置上一个位置为无效值 */
        while (1)
        {
            EmmStatus pos_sts, sta_sts;

            /* 每 stall_check_ms 毫秒采样一次 */
            system_delay_ms(gimbal->calib_config.stall_check_ms);
            elapsed += gimbal->calib_config.stall_check_ms;

            /* 同时读取编码器位置和电机状态 */
            pos_sts = read_encoder_deg(motor, &pos);
            system_delay_ms(10u);
            sta_sts = emm_get_motor_status_forced(motor, &st);

            /* 如果读取编码器或电机状态失败，立即中止 */
            if (pos_sts != EMM_OK || sta_sts != EMM_OK)
            {
                printf("[CALIB] [%s] t=%lums rd err pos=%d sts=%d\r\n",
                       axis_name, (unsigned long)elapsed,
                       (int)pos_sts, (int)sta_sts);
                return gimbal_calibration_abort(gimbal, result);
            }

            /* 打印当前时间、位置和电机状态供调试 */
            pos_d = (int32_t)(pos * 10.0f);
            printf("[CALIB] [%s] t=%lu.%lus pos=%ld.%ld st=0x%02X(E=%d)\r\n",
                   axis_name,
                   (unsigned long)(elapsed / 1000u),
                   (unsigned long)((elapsed % 1000u) / 100u),
                   (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10),
                   (unsigned)((st.enabled ? 1u : 0u) | (st.stall_detected ? 4u : 0u) | (st.stall_protected ? 8u : 0u)),
                   (int)st.enabled);

            /* 检测 1：电机既未使能，也未报告堵转 —— 异常失能，立即中止 */
            if (!st.enabled && !st.stall_detected && !st.stall_protected)
            {
                return gimbal_calibration_abort(gimbal, result);
            }
            /* 检测 2：堵转检测 —— 驱动器报告了 stall_detected 或 stall_protected */
            if (st.stall_detected || st.stall_protected)
            {
                const char *reason = "STALL";
                if (!st.enabled && !st.stall_detected && !st.stall_protected) reason = "DISABLED";
                printf("[CALIB] [%s] *** %s! pos=%ld.%ld deg ***\r\n",
                       axis_name, reason,
                       (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                positives[round - 1u] = pos;  /* 记录此轮 CW 极限位置 */
                break;
            }

            /* 检测 3：停滞检测 —— 连续 3 次采样位置变化 < 1 度，判断为机械卡滞 */
            {
                float diff = pos - prev_pos;
                if (diff < 0.0f) { diff = -diff; }
                if (diff < 1.0f)
                {
                    stuck_cnt++;              /* 停滞计数递增 */
                    /* 连续 3 次停滞则判定到达限位 */
                    if (stuck_cnt >= 3u)
                    {
                        printf("[CALIB] [%s] *** STUCK! pos=%ld.%ld deg (no movement) ***\r\n",
                               axis_name, (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                        positives[round - 1u] = pos;  /* 记录此轮 CW 极限位置 */
                        break;
                    }
                }
                else
                {
                    stuck_cnt = 0u;           /* 有位移，重置停滞计数 */
                }
            }
            prev_pos = pos;                   /* 更新上一位置用于下一轮停滞检测 */

            /* 检测 4：超时 —— 超过 stall_timeout_ms 仍未到限位，中止校准 */
            if (elapsed > gimbal->calib_config.stall_timeout_ms)
            {
                printf("[CALIB] [%s] *** CW timeout(%lu), pos=%ld.%ld deg ***\r\n",
                       axis_name,
                       (unsigned long)gimbal->calib_config.stall_timeout_ms,
                       (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                return gimbal_calibration_abort(gimbal, result);
            }
        }  /* while(1) — CW 探索结束 */

        /* 停止电机并打印此轮的 CW 极限位置 */
        if (emm_stop_verified(motor, EMM_SYNC_IMMEDIATE) != EMM_OK)
            return gimbal_calibration_abort(gimbal, result);
        system_delay_ms(100u);
        printf("[CALIB] [%s] CW limit[%u]: %ld.%ld deg\r\n",
               axis_name, (unsigned int)round,
               (long)((int32_t)(positives[round - 1u] * 10.0f) / 10),
               (long)((int32_t)(positives[round - 1u] * 10.0f) < 0
                       ? -(int32_t)(positives[round - 1u] * 10.0f) % 10
                       : (int32_t)(positives[round - 1u] * 10.0f) % 10));

        /* ----------------------------------------------------------------
         *  第 2 阶段：堵转恢复
         *  尝试清除驱动器的堵转保护状态，最多重试 5 次。
         *  每次重试后检查 stall_detected 和 stall_protected 是否已清除。
         * ---------------------------------------------------------------- */
        {
            EmmMotorStatus clr_status;   /* 清除后的电机状态 */
            uint8_t clr_retry;           /* 重试计数 */
            for (clr_retry = 0u; clr_retry < 5u; ++clr_retry)
            {
                printf("[CALIB] [%s] clear stall attempt %u...\r\n",
                       axis_name, (unsigned int)(clr_retry + 1u));
                if (emm_clear_stall_and_recover(motor) != EMM_OK)
                    continue;            /* 清除失败，立即重试 */
                system_delay_ms(150u);
                if (emm_get_motor_status_forced(motor, &clr_status) == EMM_OK)
                {
                    /* 堵转标志均已清除，恢复成功 */
                    if (!clr_status.stall_detected && !clr_status.stall_protected)
                    {
                        printf("[CALIB] [%s] stall cleared OK\r\n", axis_name);
                        break;
                    }
                    printf("[CALIB] [%s] stall still active (det=%d prot=%d)\r\n",
                           axis_name, (int)clr_status.stall_detected,
                           (int)clr_status.stall_protected);
                }
            }
            /* 重试 5 次后堵转仍未清除，中止校准 */
            if (clr_retry >= 5u)
                return gimbal_calibration_abort(gimbal, result);
        }

        /* ----------------------------------------------------------------
         *  第 3 阶段：从 CW 极限反向回退
         *  反向运动 20 度，使机械结构脱离限位位置，避免卡死。
         * ---------------------------------------------------------------- */
        {
            float backoff_pos;           /* 回退后的编码器位置 */
            EmmStatus rd;                /* 读取状态 */
            printf("[CALIB] [%s] backing off from CW limit (-20 deg)...\r\n", axis_name);
            if (emm_move_degrees(motor, -20.0f,   /* 反向 20 度 */
                                   gimbal->calib_config.explore_speed_rpm,
                                   gimbal->calib_config.explore_acceleration,
                                   EMM_MOTION_RELATIVE_CURRENT,
                                   gimbal->microstep,
                                   EMM_SYNC_IMMEDIATE) != EMM_OK)
                return gimbal_calibration_abort(gimbal, result);
            system_delay_ms(800u);               /* 等待运动完成 */
            if (emm_stop_verified(motor, EMM_SYNC_IMMEDIATE) != EMM_OK)
                return gimbal_calibration_abort(gimbal, result);
            system_delay_ms(100u);
            /* 读取并打印回退后的位置 */
            rd = read_encoder_deg(motor, &backoff_pos);
            if (rd != EMM_OK)
                return gimbal_calibration_abort(gimbal, result);
            {
                int32_t bpd = (int32_t)(backoff_pos * 10.0f);
                printf("[CALIB] [%s] after back-off pos: %ld.%ld deg (%s)\r\n",
                       axis_name, (long)(bpd / 10), (long)((bpd < 0 ? -bpd : bpd) % 10),
                       (rd == EMM_OK) ? "ok" : "err");
            }
        }
        system_delay_ms(gimbal->calib_config.stall_check_ms);

        /* ----------------------------------------------------------------
         *  第 4 阶段：CCW 极限探索（步进式绝对位置运动）
         *
         *  与 CW 的持续点动不同，CCW 探索采用步进方式：
         *  每次停止电机，读取当前位置，再以下一个目标位置（当前 - 15 度）
         *  下发绝对位置运动指令。这样每步都有明确的到位确认。
         *  同样通过堵转检测或停滞检测判断限位。
         *  目标角度下限为 -180 度，防止过度旋转。
         * ---------------------------------------------------------------- */
        printf("[CALIB] [%s] <<< REV exploration (abs pos steps) <<<\r\n",
               axis_name);

        elapsed = 0u;
        stuck_cnt = 0u;
        prev_pos = 999.0f;
        while (1)
        {
            EmmStatus pos_sts;
            float target;                  /* 下一个目标绝对角度 */

            /* 每步前先停止电机，确保处于静止状态 */
            if (emm_stop_verified(motor, EMM_SYNC_IMMEDIATE) != EMM_OK)
                return gimbal_calibration_abort(gimbal, result);
            system_delay_ms(30u);
            pos_sts = read_encoder_deg(motor, &pos);  /* 读取当前位置 */

            if (pos_sts != EMM_OK)
            {
                printf("[CALIB] [%s] t=%lu.%lus read err=%d\r\n",
                       axis_name,
                       (unsigned long)(elapsed / 1000u),
                       (unsigned long)((elapsed % 1000u) / 100u),
                       (int)pos_sts);
                return gimbal_calibration_abort(gimbal, result);
            }

            {
                EmmStatus sta_sts;
                /* 获取电机状态以检测堵转/失能 */
                sta_sts = emm_get_motor_status_forced(motor, &st);
                if (sta_sts != EMM_OK)
                {
                    return gimbal_calibration_abort(gimbal, result);
                }
                pos_d = (int32_t)(pos * 10.0f);
                printf("[CALIB] [%s] t=%lu.%lus pos=%ld.%ld st=0x%02X(E=%d)\r\n",
                       axis_name,
                       (unsigned long)(elapsed / 1000u),
                       (unsigned long)((elapsed % 1000u) / 100u),
                       (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10),
                       (unsigned)((st.enabled ? 1u : 0u) | (st.stall_detected ? 4u : 0u) | (st.stall_protected ? 8u : 0u)),
                       (int)st.enabled);
                (void)pos_sts; (void)sta_sts;

                /* 与 CW 相同的检测逻辑：异常失能、堵转检测、停滞检测 */
                if (!st.enabled && !st.stall_detected && !st.stall_protected)
                    return gimbal_calibration_abort(gimbal, result);
                if (st.stall_detected || st.stall_protected)
                {
                    const char *reason = !st.enabled ? "DISABLED" : "STALL";
                    printf("[CALIB] [%s] *** %s! pos=%ld.%ld deg ***\r\n",
                           axis_name, reason,
                           (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                    negatives[round - 1u] = pos;  /* 记录此轮 CCW 极限位置 */
                    break;
                }

                /* 停滞检测：连续 3 次位置变化 < 1 度 */
                {
                    float diff = pos - prev_pos;
                    if (diff < 0.0f) { diff = -diff; }
                    if (diff < 1.0f)
                    {
                        stuck_cnt++;
                        if (stuck_cnt >= 3u)
                        {
                            printf("[CALIB] [%s] *** STUCK! pos=%ld.%ld deg (no movement) ***\r\n",
                                   axis_name, (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                            negatives[round - 1u] = pos;  /* 记录此轮 CCW 极限位置 */
                            break;
                        }
                    }
                    else
                    {
                        stuck_cnt = 0u;
                    }
                }
                prev_pos = pos;
            }

            /* 超时检测 */
            if (elapsed > gimbal->calib_config.stall_timeout_ms)
            {
                printf("[CALIB] [%s] *** REV timeout(%lu), pos=%ld.%ld deg ***\r\n",
                       axis_name,
                       (unsigned long)gimbal->calib_config.stall_timeout_ms,
                       (long)(pos_d / 10), (long)((pos_d < 0 ? -pos_d : pos_d) % 10));
                return gimbal_calibration_abort(gimbal, result);
            }

            /* 计算下一步目标位置：当前角度 - 15 度（向 CCW 方向步进） */
            target = pos - 15.0f;
            /* 限制最小角度为 -180 度 */
            if (target < -180.0f) { target = -180.0f; }
            {
                int32_t td = (int32_t)(target * 10.0f);
                printf("[CALIB] [%s]   moving to %ld.%ld deg...\r\n", axis_name,
                       (long)(td / 10), (long)((td < 0 ? -td : td) % 10));
            }
            /* 下发相对运动指令（目标 - 当前 = 增量） */
            if (emm_move_degrees(motor, target - pos,
                                   gimbal->calib_config.explore_speed_rpm,
                                   gimbal->calib_config.explore_acceleration,
                                   EMM_MOTION_RELATIVE_CURRENT,
                                   gimbal->microstep,
                                   EMM_SYNC_IMMEDIATE) != EMM_OK)
                return gimbal_calibration_abort(gimbal, result);

            /* 等待电机运动到目标位置后再进行下一轮检测 */
            system_delay_ms(gimbal->calib_config.stall_check_ms);
            elapsed += gimbal->calib_config.stall_check_ms;
        }  /* while(1) — CCW 探索结束 */

        /* CCW 探索完成后：停止电机、打印极限位置、清除堵转 */
        if (emm_stop_verified(motor, EMM_SYNC_IMMEDIATE) != EMM_OK)
            return gimbal_calibration_abort(gimbal, result);
        system_delay_ms(100u);
        printf("[CALIB] [%s] CCW limit[%u]: %ld.%ld deg\r\n",
               axis_name, (unsigned int)round,
               (long)((int32_t)(negatives[round - 1u] * 10.0f) / 10),
               (long)((int32_t)(negatives[round - 1u] * 10.0f) < 0
                       ? -(int32_t)(negatives[round - 1u] * 10.0f) % 10
                       : (int32_t)(negatives[round - 1u] * 10.0f) % 10));
        printf("[CALIB] [%s] clear stall & recover...\r\n", axis_name);
        if (emm_clear_stall_and_recover(motor) != EMM_OK)
            return gimbal_calibration_abort(gimbal, result);
        system_delay_ms(gimbal->calib_config.stall_check_ms);

        /* ----------------------------------------------------------------
         *  第 5 阶段：本轮结束，返回中点
         *  计算此轮 CW 和 CCW 极限的中点，将轴移动至该位置。
         *  返回中点为下一轮提供一致的起始位置。
         * ---------------------------------------------------------------- */
        {
            float mid = (positives[round - 1u] + negatives[round - 1u]) / 2.0f;
            int32_t md = (int32_t)(mid * 10.0f);
            printf("[CALIB] [%s] return to mid %ld.%ld deg...\r\n", axis_name,
                   (long)(md / 10), (long)((md < 0 ? -md : md) % 10));
            if (gimbal_move_axis_to_position(motor, mid,
                                               gimbal->calib_config.explore_speed_rpm,
                                               gimbal->calib_config.explore_acceleration,
                                               gimbal->microstep, true) != GIMBAL_OK)
                return gimbal_calibration_abort(gimbal, result);
        }
        system_delay_ms(500u);
        printf("\r\n");
    }  /* 多轮循环结束 */

    /* ===================================================================
     *  多轮数据处理：从所有轮次中提取极值，计算行程和中点
     *  使用 CW 最大值（最远正极限）和 CCW 最小值（最远负极限）
     *  这样取极值而非平均，确保软件限位不会超出任何实际机械限位
     * =================================================================== */
    {
        float pos_min = positives[0], pos_max = positives[0];  /* CW 极限的最小值和最大值 */
        float neg_min = negatives[0], neg_max = negatives[0];  /* CCW 极限的最小值和最大值 */

        /* 遍历所有轮次，找出 CW 和 CCW 的极值 */
        for (uint8_t i = 1u; i < gimbal->calib_config.explore_attempts; ++i)
        {
            if (positives[i] < pos_min) { pos_min = positives[i]; }
            if (positives[i] > pos_max) { pos_max = positives[i]; }
            if (negatives[i] < neg_min)  { neg_min  = negatives[i]; }
            if (negatives[i] > neg_max)  { neg_max  = negatives[i]; }
        }

        /* 打印所有轮次的原始数据供参考 */
        printf("[CALIB] ========================================\r\n");
        printf("[CALIB] [%s] Calib done! %u rounds:\r\n",
               axis_name, (unsigned int)gimbal->calib_config.explore_attempts);
        printf("[CALIB]   CW (enc): ");
        for (uint8_t i = 0u; i < gimbal->calib_config.explore_attempts; ++i)
        {
            int32_t v = (int32_t)(positives[i] * 10.0f);
            printf("%c%ld.%ld ",
                   (v < 0) ? '-' : '+',
                   (long)((v < 0 ? -v : v) / 10),
                   (long)((v < 0 ? -v : v) % 10));
        }
        printf("\r\n");
        printf("[CALIB]   CCW (enc): ");
        for (uint8_t i = 0u; i < gimbal->calib_config.explore_attempts; ++i)
        {
            int32_t v = (int32_t)(negatives[i] * 10.0f);
            printf("%c%ld.%ld ",
                   (v < 0) ? '-' : '+',
                   (long)((v < 0 ? -v : v) / 10),
                   (long)((v < 0 ? -v : v) % 10));
        }
        printf("\r\n");

        /* 计算行程和中点：
         *   - 使用 CW 最大值（pos_max）和 CCW 最小值（neg_min）
         *   - 如果 CW 最大值小于 CCW 最小值，说明跨过了 0/360 度边界，
         *     需要将 CW 值加上 360 度后再计算
         *   - 中点取两个极限的平均值 */
        {
            float cw  = pos_max;
            float ccw = neg_min;

            /* 处理跨 0/360 边界的情况 */
            if (cw < ccw)
            {
                cw += 360.0f;
            }

            result->range_deg = cw - ccw;                     /* 总行程 */
            result->mid_deg   = (cw + ccw) / 2.0f;             /* 编码器中点 */
            if (result->mid_deg >= 360.0f) { result->mid_deg -= 360.0f; }  /* 归一化到 0~360 */

            result->min_deg = ccw;                             /* 负极限（CCW 端） */
            result->max_deg = cw;                              /* 正极限（CW 端） */

            /* 打印计算结果 */
            printf("[CALIB]   CCW limit: %ld.%ld deg (enc)\r\n",
                   (long)((int32_t)(ccw * 10.0f) / 10),
                   (long)((int32_t)(ccw * 10.0f) < 0 ? -(int32_t)(ccw * 10.0f) % 10
                                                     : (int32_t)(ccw * 10.0f) % 10));
            printf("[CALIB]   CW  limit: %ld.%ld deg (enc%s)\r\n",
                   (long)((int32_t)(pos_max * 10.0f) / 10),
                   (long)((int32_t)(pos_max * 10.0f) < 0 ? -(int32_t)(pos_max * 10.0f) % 10
                                                         : (int32_t)(pos_max * 10.0f) % 10),
                   (pos_max < neg_min) ? ", wrapped +360" : "");
            {
                int32_t rg = (int32_t)(result->range_deg * 10.0f);
                int32_t md = (int32_t)(result->mid_deg * 10.0f);
                printf("[CALIB]   range: %ld.%ld deg  mid(enc): %ld.%ld deg\r\n",
                       (long)(rg / 10), (long)((rg < 0 ? -rg : rg) % 10),
                       (long)(md / 10), (long)((md < 0 ? -md : md) % 10));
            }
        }
        printf("[CALIB] ========================================\r\n\r\n");
    }

    /* 验证行程是否在合理范围内（1 度 ~ 360 度） */
    if (result->range_deg < 1.0f || result->range_deg > 360.0f)
        return gimbal_calibration_abort(gimbal, result);

    /* 最终归中：将轴移动到计算出的编码器中点 */
    {
        float mid = result->mid_deg;
        int32_t md = (int32_t)(mid * 10.0f);
        printf("[CALIB] [%s] final move to mid %ld.%ld deg...\r\n", axis_name,
               (long)(md / 10), (long)((md < 0 ? -md : md) % 10));
        if (gimbal_move_axis_to_position(motor, mid,
                                           gimbal->calib_config.explore_speed_rpm,
                                           gimbal->calib_config.explore_acceleration,
                                           gimbal->microstep, true) != GIMBAL_OK)
            return gimbal_calibration_abort(gimbal, result);
        system_delay_ms(500u);
    }

    result->calibrated = true;    /* 标记校准完成 */
    return GIMBAL_OK;
}  /* gimbal_calibrate_axis 结束 */

/**
 * @brief  云台全自动校准入口
 *
 * 执行完整的双轴自动校准流程：
 *   1. 使能两个电机（PITCH 和 YAW）
 *   2. 检查 YAW 电机通信是否正常
 *   3. 对 PITCH 轴执行 gimbal_calibrate_axis() 自动限位探索
 *   4. YAW 轴使用固定限位 ±180 度（360 度连续旋转，无机械限位）
 *   5. 调用 gimbal_set_limits_from_calib() 设置软件限位
 *   6. 将 PITCH 轴移动到编码器中点并设置零位
 *   7. 将 YAW 轴移动到 GIMBAL_YAW_ENC_CENTER 位置
 *   8. 调用 gimbal_accept_known_reference() 确认参考位置
 *   9. 停止云台，输出校准结果
 *
 * @param gimbal  云台对象指针
 * @return GIMBAL_OK 校准成功，否则返回错误码
 */
GimbalStatus gimbal_auto_calibrate(Gimbal *gimbal)
{
    GimbalStatus status;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    /* 清除之前的校准/归位状态 */
    gimbal->homed = false;
    gimbal->position_valid = false;
    gimbal->feedback_valid = false;

    printf("\r\n");
    printf("[CALIB] ========================================\r\n");
    printf("[CALIB]  Gimbal auto-calibration start\r\n");
    printf("[CALIB]  PITCH(m1/up-dn): stall  YAW(m2/L-R): fixed +/-180\r\n");
    printf("[CALIB] ========================================\r\n\r\n");

    /* 使能两个电机 */
    printf("[CALIB] enable both motors...\r\n");
    if (gimbal_enable(gimbal, true) != GIMBAL_OK)
        return gimbal_calibration_abort(gimbal, NULL);
    system_delay_ms(200u);
    /* 检查 YAW 电机是否正常响应 */
    {
        float ty;
        EmmMotorStatus ts;
        if (read_encoder_deg(&gimbal->yaw, &ty) == EMM_OK
            && emm_get_motor_status_forced(&gimbal->yaw, &ts) == EMM_OK
            && ts.enabled)
        {
            printf("[CALIB] YAW check: enc=%ld.%ld deg en=%d\r\n",
                   (long)((int32_t)(ty * 10.0f) / 10),
                   (long)((int32_t)(ty * 10.0f) < 0 ? -(int32_t)(ty * 10.0f) % 10
                                                   : (int32_t)(ty * 10.0f) % 10),
                   (int)ts.enabled);
        }
        else
        {
            printf("[CALIB] YAW check: NO RESPONSE (addr=%u)\r\n",
                    (unsigned int)GIMBAL_YAW_MOTOR_ADDRESS);
            return gimbal_calibration_abort(gimbal, NULL);
        }
    }

    /* 校准 PITCH 轴：使用堵转检测探索限位 */
    printf("[CALIB] >>> Calibrating PITCH (m1, up/down, stall) <<<\r\n");
    status = gimbal_calibrate_axis(gimbal, &gimbal->pitch, "PITCH", &gimbal->calib_pitch);
    if (status != GIMBAL_OK)
    {
        printf("[CALIB] PITCH calibration failed!\r\n");
        return gimbal_calibration_abort(gimbal, &gimbal->calib_pitch);
    }

    system_delay_ms(500u);

    /* YAW 轴：360 度无限制旋转，使用固定限位 ±180 度 */
    printf("[CALIB] >>> YAW (m2, left/right, 360 no-stop, fixed +/-180) <<<\r\n");
    gimbal->calib_yaw.min_deg    = -180.0f;
    gimbal->calib_yaw.max_deg    =  180.0f;
    gimbal->calib_yaw.range_deg  =  360.0f;
    gimbal->calib_yaw.mid_deg    =    0.0f;
    gimbal->calib_yaw.calibrated = true;
    printf("[CALIB] YAW  fixed limit: -180.0 ~ +180.0 deg\r\n");

    /* 根据校准结果设置软件限位（用于运行时保护） */
    gimbal_set_limits_from_calib(gimbal);

    printf("\r\n[CALIB] --- 回中 & 设零 ---\r\n");

    /* PITCH 回中并设零位 */
    {
        float pm = gimbal->calib_pitch.mid_deg;
        printf("[CALIB] PITCH goto mid(%ld.%ld)...\r\n",
               (long)((int32_t)(pm * 10.0f) / 10),
               (long)((int32_t)(pm * 10.0f) < 0 ? -(int32_t)(pm * 10.0f) % 10
                                               : (int32_t)(pm * 10.0f) % 10));
        if (gimbal_move_axis_to_position(&gimbal->pitch, pm,
                                           gimbal->calib_config.explore_speed_rpm,
                                           gimbal->calib_config.explore_acceleration,
                                           gimbal->microstep, true) != GIMBAL_OK)
            return gimbal_calibration_abort(gimbal, &gimbal->calib_pitch);
        system_delay_ms(500u);
        printf("[CALIB] PITCH zero position at midpoint...\r\n");
        {
            EmmStatus zs = emm_zero_position_verified(&gimbal->pitch);
            printf("[CALIB] PITCH zero result: %s(%d)\r\n",
                    (zs == EMM_OK) ? "ok" : "FAIL", (int)zs);
            if (zs != EMM_OK)
                return gimbal_calibration_abort(gimbal, &gimbal->calib_pitch);
        }
        system_delay_ms(100u);
    }

    /* YAW 归零 */
    printf("[CALIB] YAW  goto 0...\r\n");
    if (gimbal_move_axis_to_position(&gimbal->yaw, GIMBAL_YAW_ENC_CENTER,
                                        gimbal->calib_config.explore_speed_rpm,
                                        gimbal->calib_config.explore_acceleration,
                                        gimbal->microstep, false) != GIMBAL_OK)
        return gimbal_calibration_abort(gimbal, &gimbal->calib_yaw);
    system_delay_ms(500u);
    {
        float ty;
        if (read_encoder_deg(&gimbal->yaw, &ty) == EMM_OK)
        {
            printf("[CALIB] YAW after goto 0: enc=%ld.%ld deg\r\n",
                   (long)((int32_t)(ty * 10.0f) / 10),
                   (long)((int32_t)(ty * 10.0f) < 0 ? -(int32_t)(ty * 10.0f) % 10
                                                   : (int32_t)(ty * 10.0f) % 10));
        }
        else
        {
            return gimbal_calibration_abort(gimbal, &gimbal->calib_yaw);
        }
    }

    /* 确认参考位置并停止云台 */
    if (gimbal_accept_known_reference(gimbal, 0.0f, 0.0f) != GIMBAL_OK)
        return gimbal_calibration_abort(gimbal, NULL);
    if (gimbal_stop(gimbal) != GIMBAL_OK)
        return gimbal_calibration_abort(gimbal, NULL);

    /* 输出校准完成信息 */
    printf("\r\n[CALIB] ========================================\r\n");
    printf("[CALIB]  Calibration complete!\r\n");
    {
        int32_t pmin = (int32_t)(gimbal->calib_pitch.min_deg * 10.0f);
        int32_t pmax = (int32_t)(gimbal->calib_pitch.max_deg * 10.0f);
        int32_t prng = (int32_t)(gimbal->calib_pitch.range_deg * 10.0f);
        printf("[CALIB]  PITCH: %ld.%ld ~ %ld.%ld deg (range %ld.%ld)\r\n",
               (long)(pmin / 10), (long)((pmin < 0 ? -pmin : pmin) % 10),
               (long)(pmax / 10), (long)((pmax < 0 ? -pmax : pmax) % 10),
               (long)(prng / 10), (long)((prng < 0 ? -prng : prng) % 10));
    }
    printf("[CALIB]  YAW:   -180.0 ~ +180.0 deg (range 360.0)\r\n");
    printf("[CALIB] ========================================\r\n\r\n");

    return GIMBAL_OK;
}

/**
 * @brief  将校准结果转换为云台运行时软件限位
 *
 * 根据校准得到的编码器限位值，计算并设置云台运行时使用的软件限位
 * （pitch_min_deg / pitch_max_deg / yaw_min_deg / yaw_max_deg）。
 * 软件限位比机械限位略有缩进（预留 1 度安全余量），避免运行时撞击限位。
 *
 * YAW 轴：直接在编码器限位基础上预留 ±1 度余量。
 * PITCH 轴：考虑了减速比（GIMBAL_PITCH_RATIO），将编码器端的行程
 *   转换为云台输出端的相对角度（以中点为零点）。
 *   计算公式：half_range = range / (2 * ratio) - 1（度）
 *   例如：编码器行程 300 度，减速比 10，则 half_range = 300/(2*10) - 1 = 14 度
 *
 * @param gimbal  云台对象指针（必须已包含有效的校准数据）
 */
void gimbal_set_limits_from_calib(Gimbal *gimbal)
{
    if (gimbal == NULL) { return; }

    /* YAW 轴软件限位：在编码器限位基础上缩进 1 度作为安全余量 */
    if (gimbal->calib_yaw.calibrated)
    {
        gimbal->yaw_min_deg = gimbal->calib_yaw.min_deg + 1.0f;
        gimbal->yaw_max_deg = gimbal->calib_yaw.max_deg - 1.0f;
        printf("[GIMBAL] YAW  soft limit: %ld.%ld ~ %ld.%ld deg (rel)\r\n",
               (long)((int32_t)(gimbal->yaw_min_deg * 10.0f) / 10),
               (long)((int32_t)(gimbal->yaw_min_deg * 10.0f) < 0 ? -(int32_t)(gimbal->yaw_min_deg * 10.0f) % 10 : (int32_t)(gimbal->yaw_min_deg * 10.0f) % 10),
               (long)((int32_t)(gimbal->yaw_max_deg * 10.0f) / 10),
               (long)((int32_t)(gimbal->yaw_max_deg * 10.0f) < 0 ? -(int32_t)(gimbal->yaw_max_deg * 10.0f) % 10 : (int32_t)(gimbal->yaw_max_deg * 10.0f) % 10));
    }

    /* PITCH 轴软件限位：考虑减速比，以中点为 0 度，对称分布 */
    if (gimbal->calib_pitch.calibrated)
    {
        /* half_range = 编码器行程 / (2 * 减速比) - 1 度安全余量 */
        float half_range = gimbal->calib_pitch.range_deg /
            (2.0f * GIMBAL_PITCH_RATIO) - 1.0f;
        if (half_range < 0.0f) { half_range = 0.0f; }
        gimbal->pitch_min_deg = -half_range;   /* 负方向限位 */
        gimbal->pitch_max_deg =  half_range;   /* 正方向限位 */
        printf("[GIMBAL] PITCH soft limit: %ld.%ld ~ %ld.%ld deg (rel, mid=0)\r\n",
               (long)((int32_t)(gimbal->pitch_min_deg * 10.0f) / 10),
               (long)((int32_t)(gimbal->pitch_min_deg * 10.0f) < 0 ? -(int32_t)(gimbal->pitch_min_deg * 10.0f) % 10 : (int32_t)(gimbal->pitch_min_deg * 10.0f) % 10),
               (long)((int32_t)(gimbal->pitch_max_deg * 10.0f) / 10),
               (long)((int32_t)(gimbal->pitch_max_deg * 10.0f) < 0 ? -(int32_t)(gimbal->pitch_max_deg * 10.0f) % 10 : (int32_t)(gimbal->pitch_max_deg * 10.0f) % 10));
    }
}

/**
 * @brief  等待用户按回车键继续
 *
 * 在手动/引导式校准（gimbal_calibrate_geared）中使用。
 * 轮询 UART 接收缓冲区，直到收到回车（\\r）或换行（\\n）字符。
 * 每 10ms 检查一次，收到后额外等待 200ms 消除抖动。
 */
static void wait_for_enter(void)
{
    printf("... Press Enter to continue\r\n");
    for (;;)
    {
        uint8 ch;
        if (uart_query_byte(DEBUG_UART_INDEX, &ch) == ZF_TRUE)
        {
            if (ch == '\r' || ch == '\n') { break; }
        }
        system_delay_ms(10u);
    }
    system_delay_ms(200u);
}

/**
 * @brief  手动/引导式校准 —— 适用于带减速齿轮的云台系统
 *
 * 与全自动校准不同，本函数需要用户手动参与操作：
 *
 *   PITCH 校准流程：
 *     1. 用户手动将 PITCH 轴转到机械限位（CW 方向最远端）
 *     2. 软件以点动方式继续向 CW 方向运动直到堵转，记录编码器极限位置
 *     3. 调用 emm_clear_stall_and_recover() 清除堵转
 *     4. 反向回退 GIMBAL_PITCH_BACK_ANGLE 度，使云台回到水平位置
 *     5. 记录此时编码器角度作为 GIMBAL_PITCH_ENC_HORIZONTAL
 *     6. 输出 #define 常量，供用户永久保存到 gimbal.h
 *
 *   YAW 校准流程：
 *     1. 提示用户手动将 YAW 轴转到正前方位置
 *     2. 按回车后读取编码器角度作为 YAW 中心（GIMBAL_YAW_ENC_CENTER）
 *     3. 输出 #define 常量，供用户永久保存到 gimbal.h
 *
 *   校准完成后，软件限位使用固定值：
 *     - PITCH: 0 ~ GIMBAL_PITCH_BACK_ANGLE 度（以水平为 0）
 *     - YAW: -179 ~ +179 度
 *
 * @param gimbal  云台对象指针
 * @return GIMBAL_OK 校准成功，否则返回错误码
 */
GimbalStatus gimbal_calibrate_geared(Gimbal *gimbal)
{
    float cur_deg;                     /* 当前编码器角度 */
    EmmMotorStatus st;                 /* 电机状态 */
    uint32_t elapsed;                  /* 已用时间（毫秒） */
    int32_t back_pulses;               /* 回退脉冲数（计算值，当前仅用于打印） */
    float back_motor_deg;              /* 回退的电机端角度（已乘以减速比） */
    float pitch_zero_deg;              /* PITCH 水平位置对应的编码器角度 */

    if (gimbal == NULL) { return GIMBAL_ERROR; }

    /* 清除之前的校准标记 */
    gimbal->geared_pitch.calibrated = false;
    gimbal->geared_yaw.calibrated = false;
    gimbal->homed = false;
    gimbal->position_valid = false;
    gimbal->feedback_valid = false;

    printf("\r\n");
    printf("[GEARED] ========================================\r\n");
    printf("[GEARED]  PITCH ratio=%.1f  YAW ratio=%.1f\r\n",
           (double)GIMBAL_PITCH_RATIO, (double)GIMBAL_YAW_RATIO);
    printf("[GEARED] ========================================\r\n\r\n");

    printf("[GEARED] >>> PITCH: jog CW to limit, then back %.1f deg <<<\r\n",
           (double)GIMBAL_PITCH_BACK_ANGLE);
    printf("[GEARED] >>> Ensure gimbal is at safe position, press Enter to start <<<\r\n");
    wait_for_enter();

    /* 使能 PITCH 电机并检查状态 */
    if (emm_enable(&gimbal->pitch, true, EMM_SYNC_IMMEDIATE) != EMM_OK)
    {
        return gimbal_calibration_abort(gimbal, NULL);
    }
    system_delay_ms(200u);
    if (emm_get_motor_status_forced(&gimbal->pitch, &st) != EMM_OK || !st.enabled)
        return gimbal_calibration_abort(gimbal, NULL);

    /* 以点动方式向 CW 方向运动，直到检测到堵转（到达机械限位） */
    printf("[GEARED] jogging CW at %d RPM...\r\n",
           (int)gimbal->calib_config.explore_speed_rpm);
    {
        EmmJogParams jog;
        jog.direction    = EMM_DIRECTION_CW;
        jog.speed_rpm    = gimbal->calib_config.explore_speed_rpm;
        jog.acceleration = gimbal->calib_config.explore_acceleration;
        jog.sync_flag    = EMM_SYNC_IMMEDIATE;
        if (emm_jog_no_response(&gimbal->pitch, &jog) != EMM_OK)
            return gimbal_calibration_abort(gimbal, NULL);
    }

    /* 轮询检测堵转，逻辑与 gimbal_calibrate_axis 中的 CW 探索相同 */
    elapsed = 0u;
    while (1)
    {
        system_delay_ms(gimbal->calib_config.stall_check_ms);
        elapsed += gimbal->calib_config.stall_check_ms;

        if (emm_get_motor_status_forced(&gimbal->pitch, &st) != EMM_OK)
        {
            return gimbal_calibration_abort(gimbal, NULL);
        }

        /* 异常失能 */
        if (!st.enabled && !st.stall_detected && !st.stall_protected)
            return gimbal_calibration_abort(gimbal, NULL);
        /* 堵转检测到限位 */
        if (st.stall_detected || st.stall_protected)
        {
            printf("[GEARED] PITCH stop at t=%lu.%lus (st=0x%02X)\r\n",
                   (unsigned long)(elapsed / 1000u),
                   (unsigned long)((elapsed % 1000u) / 100u),
                   (unsigned)((st.enabled ? 1u : 0u) | (st.stall_detected ? 4u : 0u) | (st.stall_protected ? 8u : 0u)));
            break;
        }
        /* 超时 */
        if (elapsed > gimbal->calib_config.stall_timeout_ms)
        {
            printf("[GEARED] PITCH timeout, abort\r\n");
            return gimbal_calibration_abort(gimbal, NULL);
        }
    }

    /* 停止电机 */
    if (emm_stop_verified(&gimbal->pitch, EMM_SYNC_IMMEDIATE) != EMM_OK)
        return gimbal_calibration_abort(gimbal, NULL);
    system_delay_ms(100u);

    /* 读取并打印 CW 极限位置的编码器角度 */
    {
        float enc_limit;
        if (read_encoder_deg(&gimbal->pitch, &enc_limit) != EMM_OK)
            return gimbal_calibration_abort(gimbal, NULL);
        printf("[GEARED] PITCH CW limit enc: %ld.%ld deg\r\n",
               (long)((int32_t)(enc_limit * 10.0f) / 10),
               (long)((int32_t)(enc_limit * 10.0f) < 0 ? -(int32_t)(enc_limit * 10.0f) % 10
                                                     : (int32_t)(enc_limit * 10.0f) % 10));
    }

    /* 清除堵转状态 */
    if (emm_clear_stall_and_recover(&gimbal->pitch) != EMM_OK)
        return gimbal_calibration_abort(gimbal, NULL);
    system_delay_ms(300u);

    /* 计算回退量：将云台输出端的回退角度（GIMBAL_PITCH_BACK_ANGLE）
     * 乘以减速比得到电机端需要回退的角度 */
    back_motor_deg = GIMBAL_PITCH_BACK_ANGLE * GIMBAL_PITCH_RATIO;
    /* 计算对应的脉冲数（仅用于打印参考信息） */
    back_pulses = (int32_t)(back_motor_deg / 360.0f * (float)(200u * gimbal->microstep));
    if (back_pulses < 0) back_pulses = -back_pulses;

    printf("[GEARED] back motor=%.1f deg, pulses=%ld\r\n",
           (double)back_motor_deg, (long)back_pulses);

    /* 反向运动回到水平位置 */
    if (gimbal_move_axis_delta_verified(&gimbal->pitch, back_motor_deg,
                                        gimbal->calib_config.explore_speed_rpm,
                                        gimbal->calib_config.explore_acceleration,
                                        gimbal->microstep) != GIMBAL_OK)
        return gimbal_calibration_abort(gimbal, NULL);
    /* 记录水平位置对应的编码器角度 */
    if (read_encoder_deg(&gimbal->pitch, &cur_deg) != EMM_OK)
        return gimbal_calibration_abort(gimbal, NULL);
    pitch_zero_deg = cur_deg;
    printf("[GEARED] PITCH horizontal enc: %ld.%ld deg\r\n",
           (long)((int32_t)(cur_deg * 10.0f) / 10),
           (long)((int32_t)(cur_deg * 10.0f) < 0 ? -(int32_t)(cur_deg * 10.0f) % 10
                                                 : (int32_t)(cur_deg * 10.0f) % 10));
    /* 输出 #define 常量，供用户复制到 gimbal.h 永久保存 */
    printf("[GEARED] >>> COPY these #defines into gimbal.h: <<<\r\n");
    printf("[GEARED] #define GIMBAL_PITCH_ENC_HORIZONTAL %.1ff\r\n",
           (double)cur_deg);
    printf("[GEARED] #define GIMBAL_PITCH_ENC_LIMIT     ... (from above)\r\n");

    /* 设置 PITCH 软件限位：以水平为 0 度，范围到 GIMBAL_PITCH_BACK_ANGLE 度 */
    gimbal->pitch_min_deg = GIMBAL_PITCH_BACK_ANGLE;
    gimbal->pitch_max_deg = 0.0f;
    /* 确保 min <= max */
    if (gimbal->pitch_min_deg > gimbal->pitch_max_deg)
    {
        float t = gimbal->pitch_min_deg;
        gimbal->pitch_min_deg = gimbal->pitch_max_deg;
        gimbal->pitch_max_deg = t;
    }
    printf("[GEARED] PITCH limits: %.1f ~ %.1f gimbal deg\r\n",
           (double)gimbal->pitch_min_deg, (double)gimbal->pitch_max_deg);

    /* ===================================================================
     *  YAW 校准：用户手动将 YAW 轴转到正前方，然后按回车
     *  软件记录当前编码器角度作为 YAW 中心位置
     * =================================================================== */
    printf("\r\n[GEARED] >>> YAW: 摆到正前方，按回车 <<<\r\n");
    wait_for_enter();

    if (read_encoder_deg(&gimbal->yaw, &cur_deg) != EMM_OK)
        return gimbal_calibration_abort(gimbal, NULL);
    printf("[GEARED] YAW center enc: %ld.%ld deg\r\n",
           (long)((int32_t)(cur_deg * 10.0f) / 10),
           (long)((int32_t)(cur_deg * 10.0f) < 0 ? -(int32_t)(cur_deg * 10.0f) % 10
                                                 : (int32_t)(cur_deg * 10.0f) % 10));
    /* 输出 YAW 中心 #define 常量，供用户复制到 gimbal.h */
    printf("[GEARED] >>> #define GIMBAL_YAW_ENC_CENTER %.1ff <<<\r\n",
           (double)cur_deg);

    /* 设置 YAW 软件限位 ±179 度（预留 1 度安全余量） */
    gimbal->yaw_min_deg = -179.0f;
    gimbal->yaw_max_deg =  179.0f;

    /* 确认参考位置、停止云台、禁用电机 */
    if (gimbal_accept_known_reference(gimbal, 0.0f, 0.0f) != GIMBAL_OK ||
        gimbal_stop(gimbal) != GIMBAL_OK ||
        gimbal_enable(gimbal, false) != GIMBAL_OK)
        return gimbal_calibration_abort(gimbal, NULL);

    /* 保存校准结果到 geared_* 结构体 */
    gimbal->geared_pitch.gear_ratio = GIMBAL_PITCH_RATIO;       /* PITCH 减速比 */
    gimbal->geared_pitch.enc_at_zero_deg = pitch_zero_deg;      /* PITCH 水平位编码器角度 */
    gimbal->geared_pitch.calibrated = true;                      /* PITCH 校准完成 */
    gimbal->geared_yaw.gear_ratio = GIMBAL_YAW_RATIO;           /* YAW 减速比 */
    gimbal->geared_yaw.enc_at_zero_deg = cur_deg;               /* YAW 中心编码器角度 */
    gimbal->geared_yaw.calibrated = true;                        /* YAW 校准完成 */

    printf("[GEARED] YAW done, ratio=%.1f\r\n", (double)GIMBAL_YAW_RATIO);
    printf("[GEARED] ========================================\r\n");
    printf("[GEARED]  Calibration complete!\r\n");
    printf("[GEARED]  Copy the #define lines above into gimbal.h\r\n");
    printf("[GEARED]  PITCH ratio=%.1f  YAW ratio=%.1f\r\n",
           (double)GIMBAL_PITCH_RATIO, (double)GIMBAL_YAW_RATIO);
    printf("[GEARED] ========================================\r\n\r\n");

    return GIMBAL_OK;
}  /* gimbal_calibrate_geared 结束 */

/*
 * =====================================================================
 *  以下为空桩实现（stub） —— 当 GIMBAL_ENABLE_CALIBRATION = 0 时编译
 *  这些实现允许调用方在不启用校准时也能通过编译，但均返回错误或不执行操作
 * =====================================================================
 */
#else

/**
 * @brief  校准空桩：校准功能未启用时返回 GIMBAL_ERROR_CALIB
 */
GimbalStatus gimbal_calibrate_axis(Gimbal *gimbal, EmmDevice *motor,
                                   const char *axis_name, GimbalAxisCalib *result)
{
    (void)gimbal;
    (void)motor;
    (void)axis_name;
    (void)result;
    return GIMBAL_ERROR_CALIB;
}

/**
 * @brief  自动校准空桩：校准功能未启用时返回 GIMBAL_ERROR_CALIB
 */
GimbalStatus gimbal_auto_calibrate(Gimbal *gimbal)
{
    (void)gimbal;
    return GIMBAL_ERROR_CALIB;
}

/**
 * @brief  设置限位空桩：校准功能未启用时不执行任何操作
 */
void gimbal_set_limits_from_calib(Gimbal *gimbal)
{
    (void)gimbal;
}

/**
 * @brief  引导式校准空桩：校准功能未启用时返回 GIMBAL_ERROR_CALIB
 */
GimbalStatus gimbal_calibrate_geared(Gimbal *gimbal)
{
    (void)gimbal;
    return GIMBAL_ERROR_CALIB;
}

#endif  /* GIMBAL_ENABLE_CALIBRATION */
