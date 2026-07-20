/*
 * ============================================================
 * 云台(Gimbal/Pan-Tilt)模块 -- 实现文件
 *
 * 本文件实现了云台控制的所有核心功能，包括：
 *   - 电机底层控制(使能/禁能/停止/相对运动)
 *   - 编码器位置读取与解缠绕(Unwrapping)
 *   - 位置追踪与零位偏移计算
 *   - 软限位保护与单步限幅
 *   - 安全故障锁存机制
 *   - 手动模式(电机断电可手调)
 *   - 辅助函数(浮点限幅、有限性检查、角度规约)
 *
 * 编译前提：需要 gimbal.h 和 gimbal_transport_zf.h 头文件。
 * ============================================================
 */

#include "gimbal/gimbal.h"
#include "gimbal/gimbal_transport_zf.h"

#include <float.h>

/*
 * 局部重定义齿轮比常量
 * 这些值在头文件中已有定义(gimbal.h)，但为了编译单元内的
 * 可见性(如果未包含或条件编译导致未定义)，在此提供默认值。
 * 实际标定后应更新 gimbal.h 中的值。
 */
#ifndef GIMBAL_PITCH_RATIO
#define GIMBAL_PITCH_RATIO   4.0f     /* PITCH轴齿轮比默认值 4:1 */
#endif
#ifndef GIMBAL_YAW_RATIO
#define GIMBAL_YAW_RATIO     8.0f     /* YAW轴齿轮比默认值 8:1 */
#endif
#ifndef GIMBAL_PITCH_BACK_ANGLE
#define GIMBAL_PITCH_BACK_ANGLE  -85.0f  /* PITCH后退角默认值 -85度 */
#endif

/*================================================================*
 * 辅助函数：EMM状态码转可读字符串
 * 用于调试输出，将枚举值转换为人类可读的错误描述。
 *================================================================*/
static const char *emm_status_name(EmmStatus status)
{
    switch (status)
    {
        case EMM_OK: return "ok";
        case EMM_ERROR_INVALID_ARG: return "invalid-arg";
        case EMM_ERROR_IO: return "io";
        case EMM_ERROR_TIMEOUT: return "timeout/no-response";
        case EMM_ERROR_BAD_RESPONSE: return "bad-response";
        case EMM_ERROR_CHECKSUM: return "checksum";
        case EMM_ERROR_PARAM: return "param";
        case EMM_ERROR_FORMAT: return "format";
        case EMM_ERROR_OVERFLOW: return "overflow";
        case EMM_ERROR_NO_RESPONSE: return "no-response-mode";
        case EMM_ERROR: return "error";
        default: return "unknown";
    }
}

/*================================================================*
 * 云台全局实例
 * 所有云台操作均通过此实例进行，在头文件中声明为 extern。
 * 初始化前所有字段为零或默认值。
 *================================================================*/
Gimbal g_gimbal;

/*================================================================*
 * 数学辅助函数
 *===============================================================*/

/* clamp_float: 浮点数限幅
 * 将 value 限制在 [min_value, max_value] 范围内。
 * 用于软限位和单步限幅，防止超出安全范围。 */
static float clamp_float(float value, float min_value, float max_value)
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

/* finite_float: 浮点数有限性检查
 * 检查浮点数是否为一个有限的数值(不是NaN，不是无穷大)。
 * value == value 利用了 NaN != NaN 的特性来检测 NaN。
 * 用于保护所有浮点运算，防止无效值传播。 */
static bool finite_float(float value)
{
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

/* wrap_encoder_zero: 编码器零位角度规约到 [0, 360)
 *
 * 编码器角度是模360度的周期值(绝对编码器每圈归零)。
 * 将任意角度值规约到 [0, 360) 范围内，以确保零位偏移量始终
 * 在一个周期内。这种规约保证了解缠绕计算的一致性基准。
 *
 * 为什么规约到 [0,360) 而不是 [-180,180)：
 * 编码器原始读数在 [0,360) 范围内，零位偏移规约到相同范围后，
 * 相对值计算(读数 - 零位)时的符号和大小更容易预测。 */
static float wrap_encoder_zero(float value)
{
    while (value >= 360.0f) value -= 360.0f;
    while (value < 0.0f) value += 360.0f;
    return value;
}

/* capped_motor_speed: 基于齿轮比的电机速度限幅
 *
 * 云台输出端的最大角速度 GIMBAL_MAX_OUTPUT_SPEED_DPS (120度/秒) 是
 * 针对云台端的限制。由于齿轮减速，电机端转速需要乘以齿轮比。
 *
 * 换算关系：
 *   电机端RPM = 云台端DPS * 齿轮比 / 6.0
 *   因为 1 RPM = 6 度/秒 (360度/圈 ÷ 60秒/分 = 6度/秒/转)
 *   所以 cap(RPM) = GIMBAL_MAX_OUTPUT_SPEED_DPS * gear_ratio / 6.0
 *
 * 例如 PITCH 齿轮比4:1，电机端最大转速 = 120 * 4 / 6 = 80 RPM
 * 例如 YAW   齿轮比8:1，电机端最大转速 = 120 * 8 / 6 = 160 RPM
 *
 * 这确保了大齿轮比轴不会因为电机转速过高而导致云台端超速。 */
static uint16_t capped_motor_speed(uint16_t requested_rpm, float gear_ratio)
{
    float cap = GIMBAL_MAX_OUTPUT_SPEED_DPS * gear_ratio / 6.0f;
    uint16_t cap_rpm;

    /* 检查计算结果是否有效，无效时返回0(堵转保护) */
    if (!finite_float(cap) || cap <= 0.0f)
    {
        return 0u;
    }
    /* 限制电机端最大转速不超过3000 RPM(电机规格上限) */
    cap_rpm = (cap >= 3000.0f) ? 3000u : (uint16_t)cap;
    /* 取请求值和限制值中的较小者 */
    return (requested_rpm < cap_rpm) ? requested_rpm : cap_rpm;
}

/*================================================================*
 * 电机控制底层函数
 *===============================================================*/

/* gimbal_attempt_disable: 尝试禁能两路电机(不检查返回值)
 * 这是一个"尽力而为"的禁能操作。在安全故障处理中调用，
 * 不检查返回值是因为此时可能通信已经异常。
 * void 强制转换抑制了未使用返回值的编译器警告。 */
static void gimbal_attempt_disable(Gimbal *gimbal)
{
    if (gimbal == NULL) return;
    (void)emm_disable(&gimbal->yaw, EMM_SYNC_IMMEDIATE);
    (void)emm_disable(&gimbal->pitch, EMM_SYNC_IMMEDIATE);
}

/* read_encoder_deg: 读取电机绝对编码器角度(度)，范围 [0, 360)
 *
 * 使用 EMM 命令码 0x31 (获取编码器计数值) 而非 0x36 (获取实时位置)。
 * 原因：
 *   命令 0x36 返回的是电机驱动器内部计算的位置值，该值可能受驱动器
 *   软件滤波、指令追踪误差等影响，在电机堵转或负载变化时产生漂移。
 *   命令 0x31 返回绝对编码器的原始计数值 (0-65535)，是真实的物理位置。
 *
 * 换算公式：
 *   角度(度) = 编码器计数值 * 360.0 / 65536.0
 *   编码器是14位的，0-65535 对应 0-360 度。
 *
 * 注意：0x31 返回的是单圈绝对值，多圈时需要解缠绕(见 gimbal_read_actual_position)。 */
static EmmStatus read_encoder_deg(EmmDevice *motor, float *deg)
{
    uint16_t encoder;
    EmmStatus s = emm_get_encoder_forced(motor, &encoder);
    if (s == EMM_OK && deg != 0)
    {
        *deg = ((float)encoder * 360.0f) / 65536.0f;
    }
    return s;
}

/* move_axis_delta: 单轴相对运动(底层调用)
 *
 * 将云台端的期望角度增量乘以齿轮比，换算为电机端需要的运动量。
 * 例如 PITCH 齿轮比4:1，云台需要转动1度，电机需要转动4度。
 *
 * 参数:
 *   motor            - EMM电机设备指针
 *   address          - 电机EMM地址(GIMBAL_PITCH_MOTOR_ADDRESS 或 GIMBAL_YAW_MOTOR_ADDRESS)
 *   output_delta_deg - 云台端目标相对角度(度)
 *   gear_ratio       - 齿轮比(电机:云台)
 *   speed_rpm        - 电机端转速(RPM)
 *   acceleration     - 电机加速度
 *   microstep        - 步进电机细分数
 * 返回: EMM_OK 成功，其他EMM错误码 */
static EmmStatus move_axis_delta(EmmDevice *motor, uint8_t address,
    float output_delta_deg, float gear_ratio, uint16_t speed_rpm,
    uint8_t acceleration, uint16_t microstep)
{
    /* 选择电机地址(广播模式下指定目标设备) */
    emm_select_address(motor, address);
    /* 发送相对运动指令：电机实际转动角度 = 云台端角度 × 齿轮比 */
    return emm_move_degrees(
        motor,
        output_delta_deg * gear_ratio,          /* 电机端角度(度) */
        speed_rpm,                               /* 转速(RPM) */
        acceleration,                             /* 加速度 */
        EMM_MOTION_RELATIVE_CURRENT,             /* 相对运动模式 */
        microstep,                                /* 细分数 */
        EMM_SYNC_IMMEDIATE);                     /* 立即同步执行 */
}

/*================================================================*
 * 核心API实现
 *===============================================================*/

/* gimbal_init: 初始化云台模块
 *
 * 执行以下初始化步骤：
 * 1. 参数有效性检查
 * 2. 初始化UART通信和EMM传输层(gimbal_transport_zf_init)
 * 3. 初始化位置追踪变量(角度、编码器零位偏移、限位)
 * 4. 配置步进电机细分数
 * 5. 设置默认标定参数
 * 6. 清除所有标志位(home/position/safety/manual)
 *
 * 注意：初始化后电机处于禁能状态，需要调用 gimbal_enable() 使能。
 * 编码器零位使用预标定值(YAW_ENC_CENTER / PITCH_ENC_HORIZONTAL)。 */
GimbalStatus gimbal_init(Gimbal *gimbal)
{
    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    /* 初始化EMM通信传输层(配置UART引脚和参数) */
    if (gimbal_transport_zf_init(gimbal) != GIMBAL_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }

    /* === 初始化位置追踪变量 === */
    gimbal->yaw_angle_deg = 0.0f;         /* YAW当前角度清零 */
    gimbal->pitch_angle_deg = 0.0f;       /* PITCH当前角度清零 */
    gimbal->yaw_commanded_deg = 0.0f;     /* YAW指令角度清零 */
    gimbal->pitch_commanded_deg = 0.0f;   /* PITCH指令角度清零 */

    /* 编码器零位偏移：使用预标定常量
     * 这些值建立编码器读数到云台角度的映射基准。
     * 注意：gimbal_accept_known_reference() 可在运行时重新校准这些值。 */
    gimbal->yaw_encoder_zero_deg = GIMBAL_YAW_ENC_CENTER;          /* YAW中位偏移 */
    gimbal->pitch_encoder_zero_deg = GIMBAL_PITCH_ENC_HORIZONTAL;  /* PITCH水平偏移 */

    /* 默认软限位范围(云台端角度，度) */
    gimbal->yaw_min_deg = -179.0f;   /* YAW最小角度：接近-180度(避免整圈歧义) */
    gimbal->yaw_max_deg =  179.0f;   /* YAW最大角度 */

    /* PITCH限位根据预标定配置选择：
     * - 预标定模式(GIMBAL_USE_PRECALIB_PITCH=1)：使用存储的限位值，
     *   并自动交换确保 min < max
     * - 非预标定模式：使用临时默认值 ±35度 */
#if GIMBAL_USE_PRECALIB_PITCH
    {
        gimbal->pitch_min_deg = GIMBAL_PITCH_BACK_ANGLE;  /* 预标定下限(-85度) */
        gimbal->pitch_max_deg = 0.0f;                      /* 预标定上限(0度，水平) */
        /* 交换确保 min < max */
        if (gimbal->pitch_min_deg > gimbal->pitch_max_deg)
        {
            float t = gimbal->pitch_min_deg;
            gimbal->pitch_min_deg = gimbal->pitch_max_deg;
            gimbal->pitch_max_deg = t;
        }
    }
#else
    gimbal->pitch_min_deg = -35.0f;  /* 临时下限 */
    gimbal->pitch_max_deg =  35.0f;  /* 临时上限 */
#endif

    /* === 设置默认运动参数 === */
    gimbal->microstep = GIMBAL_DEFAULT_MICROSTEP;     /* 细分数 */
    gimbal->speed_rpm = GIMBAL_DEFAULT_SPEED_RPM;     /* 默认转速 */
    gimbal->acceleration = GIMBAL_DEFAULT_ACCELERATION;/* 默认加速度 */

    /* 配置电机细分数(不保存到EEPROM，每次上电需重新配置) */
    if (emm_set_microstep(&gimbal->yaw, gimbal->microstep, EMM_STORE_NO) != EMM_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }
    system_delay_ms(5u);  /* 等待YAW电机配置生效 */
    if (emm_set_microstep(&gimbal->pitch, gimbal->microstep, EMM_STORE_NO) != EMM_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }

    /* === 默认标定配置 === */
    gimbal->calib_config.explore_speed_rpm    = 30u;    /* 探索速度：30 RPM */
    gimbal->calib_config.explore_acceleration = 20u;    /* 探索加速度：20 */
    gimbal->calib_config.explore_attempts     = 5u;     /* 探索轮次：5 */
    gimbal->calib_config.stall_check_ms       = 200u;   /* 堵转检测间隔：200ms */
    gimbal->calib_config.stall_timeout_ms     = 15000u; /* 探索超时：15秒 */

    /* === 清除所有状态标志 === */
    gimbal->calib_yaw.calibrated   = false;       /* YAW轴未标定 */
    gimbal->calib_pitch.calibrated = false;       /* PITCH轴未标定 */
    gimbal->geared_pitch.calibrated = false;      /* PITCH齿轮比未标定 */
    gimbal->geared_yaw.calibrated   = false;      /* YAW齿轮比未标定 */
    gimbal->homed = false;                        /* 未回零 */
    gimbal->position_valid = false;               /* 位置无效 */
    gimbal->feedback_valid = false;               /* 反馈无效 */
    gimbal->safety_fault_latched = false;         /* 无安全故障 */
    gimbal->manual_mode = false;                  /* 非手动模式 */

    return GIMBAL_OK;
}

/* gimbal_enable: 使能/禁能云台电机
 *
 * 安全故障锁存机制说明：
 * 当电机通信或响应出现异常时，设置 safety_fault_latched = true。
 * 此后所有使能和运动指令都会被拒绝，直到调用 gimbal_clear_safety_fault()。
 * 这种"锁存"(Latching)设计防止故障状态下继续发送指令导致不可预测的运动。
 *
 * 执行步骤：
 * 1. 检查安全故障锁存(使能时)
 * 2. 发送使能/禁能指令给YAW和PITCH电机
 * 3. 验证电机实际状态与期望一致(读状态寄存器确认)
 * 4. 任何一步失败则锁存安全故障，停止并禁能电机 */
GimbalStatus gimbal_enable(Gimbal *gimbal, bool enable)
{
    EmmStatus yaw_status;
    EmmStatus pitch_status;
    EmmMotorStatus yaw_state;
    EmmMotorStatus pitch_state;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    /* 安全故障锁存时禁止使能电机 */
    if (enable && gimbal->safety_fault_latched)
    {
        return GIMBAL_ERROR_SAFETY_LATCHED;
    }

    /* 发送使能/禁能指令(同步模式，等待响应) */
    yaw_status = emm_enable(&gimbal->yaw, enable, EMM_SYNC_IMMEDIATE);
    system_delay_ms(5u);  /* 等待YAW电机切换稳定 */
    pitch_status = emm_enable(&gimbal->pitch, enable, EMM_SYNC_IMMEDIATE);

    /* 检查指令是否发送成功 */
    if (yaw_status != EMM_OK || pitch_status != EMM_OK)
    {
        /* 打印详细错误信息用于调试 */
        printf("[GIMBAL] enable detail: yaw=%s(%d) pitch=%s(%d)\r\n",
            emm_status_name(yaw_status), yaw_status,
            emm_status_name(pitch_status), pitch_status);
        /* 安全故障锁存：停止并禁能所有电机 */
        gimbal->safety_fault_latched = true;
        (void)emm_stop_verified(&gimbal->yaw, EMM_SYNC_IMMEDIATE);
        (void)emm_stop_verified(&gimbal->pitch, EMM_SYNC_IMMEDIATE);
        gimbal_attempt_disable(gimbal);
        return GIMBAL_ERROR_MOTOR;
    }

    /* 等待电机状态稳定，然后读取状态寄存器确认实际状态 */
    system_delay_ms(10u);
    yaw_status = emm_get_motor_status_forced(&gimbal->yaw, &yaw_state);
    pitch_status = emm_get_motor_status_forced(&gimbal->pitch, &pitch_state);
    if (yaw_status != EMM_OK || pitch_status != EMM_OK ||
        yaw_state.enabled != enable || pitch_state.enabled != enable)
    {
        /* 状态验证失败：实际寄存器值表明电机未按要求切换 */
        gimbal->safety_fault_latched = true;
        (void)emm_stop_verified(&gimbal->yaw, EMM_SYNC_IMMEDIATE);
        (void)emm_stop_verified(&gimbal->pitch, EMM_SYNC_IMMEDIATE);
        gimbal_attempt_disable(gimbal);
        return GIMBAL_ERROR_MOTOR;
    }

    return GIMBAL_OK;
}

/* gimbal_stop: 紧急停止云台电机
 *
 * 向两个电机发送停止指令并验证响应。
 * 停止失败说明通信可能已异常，锁存安全故障并尝试禁能电机。
 * 此函数常用于故障恢复序列的第一步。 */
GimbalStatus gimbal_stop(Gimbal *gimbal)
{
    EmmStatus yaw_status;
    EmmStatus pitch_status;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    /* 发送停止指令(已验证模式，等待电机确认) */
    yaw_status = emm_stop_verified(&gimbal->yaw, EMM_SYNC_IMMEDIATE);
    pitch_status = emm_stop_verified(&gimbal->pitch, EMM_SYNC_IMMEDIATE);
    if (yaw_status != EMM_OK || pitch_status != EMM_OK)
    {
        /* 停止失败：锁存安全故障并尝试禁能 */
        gimbal->safety_fault_latched = true;
        gimbal_attempt_disable(gimbal);
        return GIMBAL_ERROR_MOTOR;
    }
    return GIMBAL_OK;
}

/* gimbal_zero_position: 云台回零
 *
 * 将EMM驱动器的内部位置计数器归零(调用 emm_zero_position_verified)，
 * 然后通过 gimbal_accept_known_reference() 将软件位置重置为 (0, 0)。
 *
 * 回零后，云台当前位置被假定为 YAW=0度、PITCH=0度(正前方水平)。
 * 如果实际云台不在该位置，后续的位置追踪将存在固定偏移。
 * 应在云台确实指向正前方时调用此函数。 */
GimbalStatus gimbal_zero_position(Gimbal *gimbal)
{
    EmmStatus yaw_status;
    EmmStatus pitch_status;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    /* 回零前清除所有位置追踪标志 */
    gimbal->homed = false;
    gimbal->position_valid = false;
    gimbal->feedback_valid = false;

    /* 先停止电机，确保在静止状态归零 */
    if (gimbal_stop(gimbal) != GIMBAL_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }

    /* 将EMM驱动器的内部位置计数器设为0 */
    yaw_status = emm_zero_position_verified(&gimbal->yaw);
    pitch_status = emm_zero_position_verified(&gimbal->pitch);

    if (yaw_status != EMM_OK || pitch_status != EMM_OK)
    {
        /* 归零失败：锁存安全故障 */
        gimbal->safety_fault_latched = true;
        (void)gimbal_stop(gimbal);
        gimbal_attempt_disable(gimbal);
        return GIMBAL_ERROR_MOTOR;
    }

    /* 将软件位置参考点设为 (0, 0) */
    return gimbal_accept_known_reference(gimbal, 0.0f, 0.0f);
}

/* gimbal_accept_known_reference: 接受已知参考位置(编码器零位追踪初始化)
 *
 * 这是编码器位置追踪的关键初始化函数。当云台处于已知姿态时调用此函数，
 * 通过读取当前编码器值反向计算出"编码器零位偏移"(encoder_zero_deg)。
 *
 * === 编码器零位追踪原理 ===
 * 绝对编码器每次上电在相同物理位置给出相同读数，但读数本身不代表"角度"。
 * 需要建立一个映射：编码器读数 -> 云台角度。
 * 公式：云台角度 = (编码器原始读数 - 零位偏移) / 齿轮比
 *
 * 计算过程：
 *   已知云台当前角度已知 (known_xxx_deg)
 *   电机端理论角度 = known_xxx_deg * 齿轮比 (因为齿轮减速)
 *   编码器零位偏移 = 当前编码器读数 - 电机端理论角度
 *
 * 示例：
 *   云台在 0°(水平)，编码器读数为 326.4°，齿轮比 4:1
 *   电机端理论角度 = 0 * 4 = 0°
 *   零位偏移 = 326.4° - 0° = 326.4° (经wrap规约到 [0,360))
 *
 * 参数:
 *   gimbal          - 云台对象
 *   known_yaw_deg   - 当前YAW角度的已知值(度)，必须在软限位范围内
 *   known_pitch_deg - 当前PITCH角度的已知值(度)
 *
 * 返回: GIMBAL_OK 成功 */
GimbalStatus gimbal_accept_known_reference(Gimbal *gimbal,
                                           float known_yaw_deg,
                                           float known_pitch_deg)
{
    float yaw_encoder;
    float pitch_encoder;

    /* 检查参数有效性：非空、数值有限、在软限位范围内 */
    if (gimbal == NULL || !finite_float(known_yaw_deg) ||
        !finite_float(known_pitch_deg) ||
        known_yaw_deg < gimbal->yaw_min_deg || known_yaw_deg > gimbal->yaw_max_deg ||
        known_pitch_deg < gimbal->pitch_min_deg || known_pitch_deg > gimbal->pitch_max_deg)
    {
        return GIMBAL_ERROR;
    }

    /* 读取当前编码器原始值 */
    if (read_encoder_deg(&gimbal->yaw, &yaw_encoder) != EMM_OK ||
        read_encoder_deg(&gimbal->pitch, &pitch_encoder) != EMM_OK)
    {
        /* 编码器读取失败，清除所有追踪标志 */
        gimbal->homed = false;
        gimbal->position_valid = false;
        gimbal->feedback_valid = false;
        return GIMBAL_ERROR_SENSOR;
    }

    /* 计算编码器零位偏移：
     * 零位偏移 = 当前编码器读数 - (已知云台角度 × 齿轮比)
     * 规约到 [0, 360) 确保一致性 */
    gimbal->yaw_encoder_zero_deg = wrap_encoder_zero(
        yaw_encoder - known_yaw_deg * GIMBAL_YAW_RATIO);
    gimbal->pitch_encoder_zero_deg = wrap_encoder_zero(
        pitch_encoder - known_pitch_deg * GIMBAL_PITCH_RATIO);

    /* 初始化位置追踪状态 */
    gimbal->yaw_angle_deg = known_yaw_deg;           /* 设置当前角度 */
    gimbal->pitch_angle_deg = known_pitch_deg;
    gimbal->yaw_commanded_deg = known_yaw_deg;       /* 指令角度同步 */
    gimbal->pitch_commanded_deg = known_pitch_deg;
    gimbal->homed = true;                             /* 标记已回零 */
    gimbal->position_valid = true;                    /* 位置有效 */
    gimbal->feedback_valid = true;                    /* 反馈有效 */
    return GIMBAL_OK;
}

/* gimbal_latch_safety_fault: 手动触发安全故障锁存
 *
 * 供上层应用在检测到异常时调用(如位置偏差过大、传感器异常等)。
 * 锁存后所有运动指令被拒绝，系统进入安全停止状态。
 * 调用 gimbal_clear_safety_fault() 可清除锁存。 */
void gimbal_latch_safety_fault(Gimbal *gimbal)
{
    if (gimbal != NULL)
    {
        gimbal->safety_fault_latched = true;
    }
}

/* gimbal_clear_safety_fault: 清除安全故障锁存
 *
 * 执行"停止 -> 禁能"序列来恢复电机状态，然后清除锁存标志。
 * 如果停止或禁能操作失败(说明问题依然存在)，锁存保持。
 * 这是安全故障恢复的唯一途径。 */
GimbalStatus gimbal_clear_safety_fault(Gimbal *gimbal)
{
    GimbalStatus stop_status;
    GimbalStatus disable_status;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }
    /* 先停止电机，再禁能 */
    stop_status = gimbal_stop(gimbal);
    disable_status = gimbal_enable(gimbal, false);
    if (stop_status != GIMBAL_OK || disable_status != GIMBAL_OK)
    {
        /* 恢复过程失败，锁存保持 */
        gimbal->safety_fault_latched = true;
        return GIMBAL_ERROR_MOTOR;
    }
    /* 清除锁存标志 */
    gimbal->safety_fault_latched = false;
    return GIMBAL_OK;
}

/* gimbal_move_to_validated: 验证并执行运动(带安全限幅)
 *
 * 这是实际发送运动指令的内部函数，执行多层安全保护：
 *
 * 1. 软限位保护：将目标角度限制在 gimbal->xxx_min/max_deg 范围内
 * 2. 单步限幅保护：将实际位移限制在 ±GIMBAL_MAX_COMMAND_STEP_DEG (±3度) 内
 *    防止目标突变(如上位机发送的跳跃式指令)导致的机械冲击
 * 3. 速度限幅：通过 capped_motor_speed() 根据齿轮比限制电机转速
 * 4. 故障锁存：任何电机指令失败立即锁存安全故障
 *
 * 单步限幅的用意：
 * 如果上位机请求从0度瞬间移动到180度，单步限幅会把这个大行程
 * 切割成多个步进(每次最多3度)。但注意，本函数只执行一次步进，
 * 外部需要循环调用才能完成大范围运动。
 *
 * 执行顺序：先PITCH后YAW，中间间隔2ms防止总线冲突。 */
static GimbalStatus gimbal_move_to_validated(Gimbal *gimbal,
                                              float yaw_deg,
                                              float pitch_deg)
{
    float target_yaw, target_pitch;
    float yaw_delta, pitch_delta;
    EmmStatus yaw_status = EMM_OK;
    EmmStatus pitch_status = EMM_OK;

    /* 第1层保护：软限位(将目标钳制在允许范围内) */
    target_yaw   = clamp_float(yaw_deg, gimbal->yaw_min_deg, gimbal->yaw_max_deg);
    target_pitch = clamp_float(pitch_deg, gimbal->pitch_min_deg, gimbal->pitch_max_deg);

    /* 计算从当前位置到目标位置所需的位移量 */
    yaw_delta = target_yaw - gimbal->yaw_angle_deg;
    pitch_delta = target_pitch - gimbal->pitch_angle_deg;

    /* 第2层保护：单步限幅(限制每次最大位移，防止机械冲击) */
    yaw_delta = clamp_float(yaw_delta, -GIMBAL_MAX_COMMAND_STEP_DEG,
                            GIMBAL_MAX_COMMAND_STEP_DEG);
    pitch_delta = clamp_float(pitch_delta, -GIMBAL_MAX_COMMAND_STEP_DEG,
                              GIMBAL_MAX_COMMAND_STEP_DEG);
    /* 重新计算实际目标位置(受单步限幅后的位置) */
    target_yaw = gimbal->yaw_angle_deg + yaw_delta;
    target_pitch = gimbal->pitch_angle_deg + pitch_delta;

    /* 先执行PITCH运动(俯仰运动优先级较高，避免与YAW同时动作) */
    if (pitch_delta != 0.0f)
    {
        pitch_status = move_axis_delta(&gimbal->pitch, GIMBAL_PITCH_MOTOR_ADDRESS,
            pitch_delta, GIMBAL_PITCH_RATIO,
            capped_motor_speed(gimbal->speed_rpm, GIMBAL_PITCH_RATIO),
            gimbal->acceleration, gimbal->microstep);
        if (pitch_status == EMM_OK)
        {
            /* 更新PITCH指令角度(仅当指令成功发出) */
            gimbal->pitch_commanded_deg = target_pitch;
        }
    }
    system_delay_ms(2u);  /* 两轴间间隔，防止UART总线冲突 */

    /* 再执行YAW运动 */
    if (yaw_delta != 0.0f)
    {
        yaw_status = move_axis_delta(&gimbal->yaw, GIMBAL_YAW_MOTOR_ADDRESS,
            yaw_delta, GIMBAL_YAW_RATIO,
            capped_motor_speed(gimbal->speed_rpm, GIMBAL_YAW_RATIO),
            gimbal->acceleration, gimbal->microstep);
        if (yaw_status == EMM_OK)
        {
            gimbal->yaw_commanded_deg = target_yaw;
        }
    }

    /* 第4层保护：任何电机错误立即锁存安全故障 */
    if (yaw_status != EMM_OK || pitch_status != EMM_OK)
    {
        gimbal->safety_fault_latched = true;
        (void)gimbal_stop(gimbal);
        return GIMBAL_ERROR_MOTOR;
    }
    return GIMBAL_OK;
}

/* gimbal_move_relative: 相对运动(从当前位置移动指定增量)
 *
 * 先读取编码器实际位置，加上增量后调用 gimbal_move_to_validated。
 * 适用于增量式控制(如摇杆/手柄输入)。
 *
 * 安全检查顺序：
 *   1. 参数有效性(非空、有限浮点数)
 *   2. 回零标志(homed)和位置有效性(position_valid)
 *   3. 安全故障锁存(safety_fault_latched)
 *   4. 手动模式(manual_mode)
 *   5. 编码器读取(获取当前位置)
 *
 * 参数:
 *   gimbal          - 云台对象
 *   yaw_delta_deg   - YAW相对位移(度)
 *   pitch_delta_deg - PITCH相对位移(度)
 * 返回: GIMBAL_OK 成功，或相应错误码 */
GimbalStatus gimbal_move_relative(Gimbal *gimbal, float yaw_delta_deg, float pitch_delta_deg)
{
    float yaw_actual;
    float pitch_actual;

    if (gimbal == NULL || !finite_float(yaw_delta_deg) || !finite_float(pitch_delta_deg))
    {
        return GIMBAL_ERROR;
    }
    if (!gimbal->homed || !gimbal->position_valid)
        return GIMBAL_ERROR_NOT_HOMED;
    if (gimbal->safety_fault_latched)
        return GIMBAL_ERROR_SAFETY_LATCHED;
    if (gimbal->manual_mode)
        return GIMBAL_ERROR;
    if (gimbal_read_actual_position(gimbal, &yaw_actual, &pitch_actual) != GIMBAL_OK)
        return GIMBAL_ERROR_SENSOR;
    return gimbal_move_to_validated(gimbal, yaw_actual + yaw_delta_deg,
                                    pitch_actual + pitch_delta_deg);
}

/* gimbal_move_to: 绝对运动(移动到指定角度)
 *
 * 先读取编码器实际位置，然后调用 gimbal_move_to_validated 执行经过安全限幅的运动。
 * 所有安全保护(软限位、单步限幅、速度限幅、故障锁存)均由 validated 内部函数处理。
 *
 * 参数:
 *   gimbal    - 云台对象
 *   yaw_deg   - 目标YAW角度(度)
 *   pitch_deg - 目标PITCH角度(度)
 * 返回: GIMBAL_OK 成功，或相应的错误码 */
GimbalStatus gimbal_move_to(Gimbal *gimbal, float yaw_deg, float pitch_deg)
{
    float yaw_actual;
    float pitch_actual;

    if (gimbal == NULL || !finite_float(yaw_deg) || !finite_float(pitch_deg))
        return GIMBAL_ERROR;
    if (!gimbal->homed || !gimbal->position_valid)
        return GIMBAL_ERROR_NOT_HOMED;
    if (gimbal->safety_fault_latched)
        return GIMBAL_ERROR_SAFETY_LATCHED;
    if (gimbal->manual_mode)
        return GIMBAL_ERROR;
    if (gimbal_read_actual_position(gimbal, &yaw_actual, &pitch_actual) != GIMBAL_OK)
        return GIMBAL_ERROR_SENSOR;
    return gimbal_move_to_validated(gimbal, yaw_deg, pitch_deg);
}

/* gimbal_read_actual_position: 读取实际位置(带编码器解缠绕 Unwrapping)
 *
 * 这是位置追踪的核心函数。它读取电机绝对编码器，经过零位偏移校正和
 * 齿轮比换算后，输出云台端的实际角度。
 *
 * === 编码器解缠绕(Unwrapping)原理 ===
 *
 * 编码器每圈输出 0-360 度的周期值。当云台多圈运动时，编码器会在
 * 过零点(0->360 或 360->0)发生跳变，导致角度计算不连续。
 *
 * 解缠绕算法通过与"期望值"(即上次记录的角度×齿轮比)比较来消除跳变：
 *
 *   while ((rel - expected) > 180.0f)  rel -= 360.0f;  // 正向过零补偿
 *   while ((rel - expected) < -180.0f) rel += 360.0f;  // 负向过零补偿
 *
 * 示例：
 *   假设云台连续正转，编码器从 359° 变到 1°(实际过了2°)。
 *   未解缠绕：1° - 0° = 1° (错误，应该是 361°)
 *   expected = 359° × 齿轮比(假设1:1) = 359°
 *   rel - expected = 1 - 359 = -358° < -180° → rel += 360° → rel = 361°
 *   正确！
 *
 * === 角度换算 ===
 *   rel = 编码器原始值 - 零位偏移(规约后)
 *   电机端角度 = rel (经过解缠绕修正)
 *   云台端角度 = 电机端角度 / 齿轮比
 *
 * 参数:
 *   gimbal    - 云台对象(必须已回零)
 *   yaw_deg   - 输出：YAW实际角度(度)
 *   pitch_deg - 输出：PITCH实际角度(度)
 * 返回: GIMBAL_OK 成功，GIMBAL_ERROR_NOT_HOMED 未回零，
 *       GIMBAL_ERROR_MOTOR 编码器读取失败 */
GimbalStatus gimbal_read_actual_position(Gimbal *gimbal, float *yaw_deg, float *pitch_deg)
{
    float y, p;
    EmmStatus ys, ps;

    if (gimbal == NULL || yaw_deg == NULL || pitch_deg == NULL)
    {
        return GIMBAL_ERROR;
    }

    if (!gimbal->homed || !gimbal->position_valid)
    {
        return GIMBAL_ERROR_NOT_HOMED;
    }

    ys = read_encoder_deg(&gimbal->yaw, &y);
    ps = read_encoder_deg(&gimbal->pitch, &p);

    if (ys == EMM_OK && ps == EMM_OK)
    {
        /* === YAW轴解缠绕计算 === */
        float rel = y - gimbal->yaw_encoder_zero_deg;  /* 相对值(电机端) */
        float expected = gimbal->yaw_angle_deg * GIMBAL_YAW_RATIO;  /* 期望值(电机端) */

        /* 解缠绕：当相对值与期望值的偏差超过半圈(180度)时，加减360度修正 */
        while ((rel - expected) > 180.0f)  rel -= 360.0f;   /* 正向过零：多减一圈 */
        while ((rel - expected) < -180.0f) rel += 360.0f;   /* 负向过零：多加一圈 */

        /* 除以齿轮比得到云台端角度 */
        *yaw_deg = rel / GIMBAL_YAW_RATIO;

        /* === PITCH轴解缠绕计算(同上) === */
        rel = p - gimbal->pitch_encoder_zero_deg;
        expected = gimbal->pitch_angle_deg * GIMBAL_PITCH_RATIO;
        while ((rel - expected) > 180.0f)  rel -= 360.0f;
        while ((rel - expected) < -180.0f) rel += 360.0f;
        *pitch_deg = rel / GIMBAL_PITCH_RATIO;

        /* 更新软件位置追踪状态 */
        gimbal->yaw_angle_deg = *yaw_deg;      /* 保存最新YAW角度 */
        gimbal->pitch_angle_deg = *pitch_deg;  /* 保存最新PITCH角度 */
        gimbal->feedback_valid = true;          /* 标记反馈有效 */
        return GIMBAL_OK;
    }

    gimbal->feedback_valid = false;
    return GIMBAL_ERROR_MOTOR;
}

/* gimbal_enter_manual_mode: 进入手动模式(电机断电，可手动转动)
 *
 * 用于调试、初始定位或紧急情况。电机线圈断电，无保持力矩，
 * 可以自由手动转动云台到任意位置。
 *
 * 进入步骤：
 * 1. 停止电机运动
 * 2. 禁能电机(切断线圈电流)
 * 3. 设置 manual_mode 标志(运动指令被拒绝)
 * 4. 清除 feedback_valid(手动转动后位置不再可信)
 *
 * 注意：进入手动模式后，位置追踪失效。退出时需重新同步。 */
GimbalStatus gimbal_enter_manual_mode(Gimbal *gimbal)
{
    GimbalStatus stop_status;
    GimbalStatus disable_status;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    printf("[GIMBAL] *** 进入手动模式 - 电机断电，可自由转动 ***\r\n");

    /* 停止所有运动并禁能电机 */
    stop_status = gimbal_stop(gimbal);
    disable_status = gimbal_enable(gimbal, false);
    if (stop_status != GIMBAL_OK || disable_status != GIMBAL_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }

    gimbal->manual_mode = true;           /* 设置手动模式标志 */
    gimbal->feedback_valid = false;       /* 手动转动后反馈不再有效 */
    return GIMBAL_OK;
}

/* gimbal_exit_manual_mode: 退出手动模式(重新使能电机并同步位置)
 *
 * 重新使能电机，读取当前编码器位置，更新软件位置追踪。
 * 由于手动转动后位置已变化，必须重新读取编码器来同步。
 *
 * 前提条件：
 * - position_valid = true (未丢失位置基准)
 * - safety_fault_latched = false (无安全故障)
 *
 * 退出步骤：
 * 1. 检查前提条件
 * 2. 使能电机
 * 3. 读取编码器当前位置(gimbal_read_actual_position)
 * 4. 更新 yaw/pitch_angle_deg 同步位置
 * 5. 清除 manual_mode 标志 */
GimbalStatus gimbal_exit_manual_mode(Gimbal *gimbal)
{
    float yaw_pos, pitch_pos;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    printf("[GIMBAL] *** 退出手动模式 - 重新使能电机 ***\r\n");

    /* 检查前提条件：位置有效且无安全故障 */
    if (!gimbal->position_valid || gimbal->safety_fault_latched)
    {
        return gimbal->safety_fault_latched
            ? GIMBAL_ERROR_SAFETY_LATCHED : GIMBAL_ERROR_NOT_HOMED;
    }

    /* 重新使能电机(恢复保持力矩) */
    if (gimbal_enable(gimbal, true) != GIMBAL_OK)
    {
        return GIMBAL_ERROR_MOTOR;
    }

    /* 读取当前编码器位置以重新同步软件位置追踪
     * 因为手动转动后实际位置已变化，必须读取编码器来更新软件状态 */
    if (gimbal_read_actual_position(gimbal, &yaw_pos, &pitch_pos) == GIMBAL_OK)
    {
        gimbal->yaw_angle_deg   = yaw_pos;   /* 同步YAW角度 */
        gimbal->pitch_angle_deg = pitch_pos; /* 同步PITCH角度 */
        {
            /* 打印同步后的位置(保留一位小数)用于调试确认 */
            int32_t yd = (int32_t)(yaw_pos * 10.0f);
            int32_t pd = (int32_t)(pitch_pos * 10.0f);
            printf("[GIMBAL] pos synced: Yaw=%ld.%ld Pitch=%ld.%ld\r\n",
                   (long)(yd / 10), (long)((yd < 0 ? -yd : yd) % 10),
                   (long)(pd / 10), (long)((pd < 0 ? -pd : pd) % 10));
        }
    }
    else
    {
        (void)gimbal_stop(gimbal);
        gimbal_attempt_disable(gimbal);
        return GIMBAL_ERROR_SENSOR;
    }

    gimbal->manual_mode = false;
    return GIMBAL_OK;
}
