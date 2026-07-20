#ifndef GIMBAL_H
#define GIMBAL_H

/*
 * ============================================================
 * 云台(Gimbal/Pan-Tilt)模块 -- 主头文件
 *
 * 本模块控制两路步进电机实现云台的俯仰(PITCH/上下)和偏航(YAW/左右)运动。
 * 电机通过 EMM (Embedded Motor Management) 步进电机协议经 UART 通信驱动。
 * 硬件平台：LCSC Tianmengxing MSPM0G3507 开发板。
 *
 * ===== 体系架构 =====
 * 云台控制分为两层：
 *   1. 电机驱动层 (emm_stepper) -- 通过 UART 发送 EMM 协议指令控制步进电机
 *   2. 云台逻辑层 (本模块)     -- 封装位置追踪、限位保护、标定、安全逻辑
 *
 * ===== 运动学模型 =====
 * 电机输出轴经过齿轮箱减速后驱动云台，因此：
 *   电机旋转角度 = 云台实际角度 × 齿轮比 (GIMBAL_PITCH_RATIO / GIMBAL_YAW_RATIO)
 * 编码器安装在电机端(高速端)，读取的编码器值需要除以齿轮比换算为云台角度。
 *
 * ===== 位置追踪 =====
 * 使用 EMM 协议命令码 0x31 读取电机绝对编码器(0-65535 对应 0-360度)。
 * 每次上电后，绝对编码器在相同物理位置返回相同值。
 * 通过"编码器零位偏移"(encoder_zero) 建立编码器值 -> 云台角度的映射。
 * 解缠绕(Unwrapping)算法处理多圈运动时编码器过零跳变的问题。
 *
 * ===== 安全机制 =====
 * - 软限位：在软件层面限制运动范围，防止撞击机械限位
 * - 故障锁存：任何电机通信或响应异常都会锁存故障，
 *   所有运动指令被拒绝，直到调用 gimbal_clear_safety_fault() 清除
 * - 手动模式：电机断电使能，允许手动转动云台
 *
 * ===== 标定系统 =====
 * 1. 齿轮比标定 (gimbal_calibrate_geared)：通过探索机械限位自动计算齿轮比
 * 2. 轴标定 (gimbal_calibrate_axis)：探测单轴运动范围
 * 3. 预标定常量 (GIMBAL_USE_PRECALIB_PITCH)：使用预先测量的标定值跳过自动标定
 * ============================================================
 */

#include <stdbool.h>
#include <stdint.h>

#include "emm_stepper.h"
#include "zf_common_headfile.h"
#include "pin_mapping.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Default wiring for LCSC Tianmengxing MSPM0G3507 board. */
/* 云台EMM通信UART接口定义，默认使用板级定义的EMM UART引脚 */
#ifndef GIMBAL_EMM_UART
#define GIMBAL_EMM_UART BOARD_EMM_UART             /* EMM通信使用的UART外设 */
#endif

#ifndef GIMBAL_EMM_UART_TX_PIN
#define GIMBAL_EMM_UART_TX_PIN BOARD_EMM_UART_TX   /* UART发送引脚(连接电机驱动器的RX) */
#endif

#ifndef GIMBAL_EMM_UART_RX_PIN
#define GIMBAL_EMM_UART_RX_PIN BOARD_EMM_UART_RX   /* UART接收引脚(连接电机驱动器的TX) */
#endif

/* 云台电机地址定义
 * EMM网络中每个电机有唯一的ID(1-255)，通过UART广播/点对点通信。
 * PITCH(上下)电机为1号，YAW(左右)电机为2号。 */
#ifndef GIMBAL_PITCH_MOTOR_ADDRESS
#define GIMBAL_PITCH_MOTOR_ADDRESS 1u   /* 俯仰(PITCH/上下)电机，EMM地址1 */
#endif

#ifndef GIMBAL_YAW_MOTOR_ADDRESS
#define GIMBAL_YAW_MOTOR_ADDRESS 2u     /* 偏航(YAW/左右)电机，EMM地址2 */
#endif

/* 默认运动参数
 * MICROSTEP: 步进电机细分数，16细分时每圈200*16=3200步，步距角更小更平滑
 * SPEED_RPM: 电机目标转速(转/分钟)，经 capped_motor_speed() 受齿轮比限速
 * ACCELERATION: 电机加速度(任意单位，取决于EMM驱动器实现) */
#ifndef GIMBAL_DEFAULT_MICROSTEP
#define GIMBAL_DEFAULT_MICROSTEP 16u      /* 默认细分数：16细分 */
#endif

#ifndef GIMBAL_DEFAULT_SPEED_RPM
#define GIMBAL_DEFAULT_SPEED_RPM 300u     /* 默认转速：300 RPM */
#endif

#ifndef GIMBAL_DEFAULT_ACCELERATION
#define GIMBAL_DEFAULT_ACCELERATION 50u   /* 默认加速度 */
#endif

/* 安全限幅参数
 * MAX_COMMAND_STEP_DEG: 单次运动指令的最大步进角度(度)。
 *   限制每次 gimbal_move_to_validated 的最大位移量，防止目标突变导致机械冲击。
 *   默认3度，上位机应通过多次调用来实现大范围运动。
 * MAX_OUTPUT_SPEED_DPS: 云台输出端最大角速度(度/秒)。
 *   经齿轮比折算后限制电机端转速：电机端RPM上限 = (MAX_OUTPUT_SPEED_DPS * 齿轮比) / 6.0 */
#ifndef GIMBAL_MAX_COMMAND_STEP_DEG
#define GIMBAL_MAX_COMMAND_STEP_DEG 3.0f      /* 单步最大行程：3度(安全限幅) */
#endif

#ifndef GIMBAL_MAX_OUTPUT_SPEED_DPS
#define GIMBAL_MAX_OUTPUT_SPEED_DPS 120.0f    /* 云台端最大角速度：120度/秒 */
#endif

/* ================================================================
 *  PITCH 预标定限位 (基于编码器，持久化)
 *
 *  编码器协议命令 0x31 返回绝对位置——相同的物理位置在每次上电
 *  后都返回相同的计数值。将 USE_PRECALIB_PITCH 设为 1 可以跳过
 *  自动标定流程，直接使用下方存储的标定值。
 *
 *  注意：如果机械结构发生变化(如重新组装齿轮箱)，需要重新标定。
 * ================================================================ */
#ifndef GIMBAL_USE_PRECALIB_PITCH
#define GIMBAL_USE_PRECALIB_PITCH 1      /* 1=使用预标定值, 0=自动标定 */
#endif

/* === 编码器标定常量 (绝对位置型，持久化) ===
   由 gimbal_calibrate_geared() 测量得到。首次运行后请记录这些值。
   编码器(命令 0x31)读取 0-65535 计数 -> 0-360 度。对于绝对编码器，
   每次上电后相同物理位置返回相同读数，因此标定值可持久化使用。

   齿轮比说明：
     电机每转 = 360度(电机端)
     云台端角度 = 电机端角度 / 齿轮比
     例如 PITCH_RATIO=4.0，电机转4圈(1440度)云台才转1圈(360度) */
#ifndef GIMBAL_PITCH_RATIO
#define GIMBAL_PITCH_RATIO         4.0f   /* PITCH轴齿轮比(电机:云台 = 4:1) */
#endif
#ifndef GIMBAL_YAW_RATIO
#define GIMBAL_YAW_RATIO           8.0f   /* YAW轴齿轮比(电机:云台 = 8:1) */
#endif
#ifndef GIMBAL_PITCH_BACK_ANGLE
#define GIMBAL_PITCH_BACK_ANGLE   -85.0f  /* PITCH后退角度(度)，标定时从限位退回的目标位置 */
#endif
/* 编码器角度值(度) — 在CW(顺时针)限位处(堵转)和水平位置(后退后)测得。
   设置 GIMBAL_USE_PRECALIB_PITCH=0 并运行一次标定流程即可测量这些值。 */
#ifndef GIMBAL_PITCH_ENC_LIMIT
#define GIMBAL_PITCH_ENC_LIMIT     136.5f /* PITCH限位处的编码器角度(度) */
#endif
#ifndef GIMBAL_PITCH_ENC_HORIZONTAL
#define GIMBAL_PITCH_ENC_HORIZONTAL 326.4f /* PITCH水平位置(0°)的编码器角度(度) */
#endif
#ifndef GIMBAL_YAW_ENC_CENTER
#define GIMBAL_YAW_ENC_CENTER      0.0f   /* YAW中位(正前方)的编码器角度偏移(度) */
#endif


/* 编译时静态检查：确保配置参数在有效范围内 */
#if (GIMBAL_USE_PRECALIB_PITCH != 0) && (GIMBAL_USE_PRECALIB_PITCH != 1)
#error "GIMBAL_USE_PRECALIB_PITCH must be 0 or 1"  /* 预标定开关只能是0或1 */
#endif

#if (GIMBAL_DEFAULT_MICROSTEP == 0u) || (GIMBAL_DEFAULT_MICROSTEP > 256u)
#error "GIMBAL_DEFAULT_MICROSTEP must be in [1, 256]"  /* 细分数范围1~256 */
#endif

/* 云台状态码枚举
 * 所有云台API函数均返回此类型，用于指示操作结果。
 * 正值保留未来扩展，负值表示各类错误。 */
typedef enum
{
    GIMBAL_OK = 0,                   /* 操作成功 */
    GIMBAL_ERROR = -1,               /* 通用错误(通常是无效参数) */
    GIMBAL_ERROR_MOTOR = -2,         /* 电机通信/响应错误 */
    GIMBAL_ERROR_SENSOR = -3,        /* 传感器(编码器)读取错误 */
    GIMBAL_ERROR_CALIB = -4,         /* 标定错误(标定失败或未标定) */
    GIMBAL_ERROR_NOT_HOMED = -5,     /* 未回零(需先执行归零操作) */
    GIMBAL_ERROR_SAFETY_LATCHED = -6,/* 安全故障已锁存，无法执行操作 */
} GimbalStatus;

/* ---- 标定类型定义 ---- */

/* 齿轮比标定数据结构
   通过电机端编码器(高速小齿轮)和云台端实际角度(低速大齿轮)的映射关系，
   计算齿轮比。标定时驱动电机到两个已知位置，记录编码器值和对应的云台角度。
   注意：电机端的编码器每转360度，但由于齿轮减速，云台端实际转角更小。 */
typedef struct
{
    float enc_at_zero_deg;     /* 云台在0°(水平)时的编码器角度值(度)    */
    float enc_at_max_deg;      /* 云台在最大角度时的编码器角度值(度)     */
    float max_gimbal_deg;      /* 云台实际最大角度(度)，如PITCH限位85°   */
    float gear_ratio;          /* 齿轮比 = 电机端转过度数 / 云台端转过度数 */
    bool  calibrated;          /* true 表示标定已完成                   */
} GimbalGearedCalib;

/* 标定流程参数配置
   控制自动标定时的探测行为：搜索限位的速度、加速度、检测频率等。
   适当调整可平衡标定速度与精度。 */
typedef struct
{
    uint16_t explore_speed_rpm;    /* 限位探索时的慢速转速(RPM)，默认30，低速减少冲击 */
    uint8_t  explore_acceleration; /* 探索时的加速度，默认20，缓加速避免惯性过冲 */
    uint8_t  explore_attempts;     /* 每轴探测轮次，默认5轮取平均提高精度 */
    uint32_t stall_check_ms;       /* 堵转检测轮询间隔(毫秒)，默认200ms */
    uint32_t stall_timeout_ms;     /* 单方向探索超时(毫秒)，默认15000ms(15秒) */
} GimbalCalibConfig;

/* 单轴标定结果数据结构
   存储单个运动轴的限位和范围信息，由标定流程自动填充。
   min_deg和max_deg定义了云台在该轴上的软限位范围。 */
typedef struct
{
    float   min_deg;          /* 下极限角度(度)，机械限位或软限位下限     */
    float   max_deg;          /* 上极限角度(度)，机械限位或软限位上限     */
    float   range_deg;        /* 总行程范围(度) = max - min               */
    float   mid_deg;          /* 中点角度(度) = (max + min) / 2，用于居中 */
    bool    calibrated;       /* true 表示标定已完成                     */
} GimbalAxisCalib;

/* ---- 云台主结构体 ----
   包含两个EMM电机设备(YAW/PITCH)、位置追踪状态、限位信息、标定数据和安全状态。
   全局实例 g_gimbal 在 gimbal.c 中定义。 */
typedef struct
{
    /* 电机设备句柄 */
    EmmDevice yaw;               /* YAW(偏航/左右)电机设备，EMM协议接口 */
    EmmDevice pitch;             /* PITCH(俯仰/上下)电机设备，EMM协议接口 */

    /* 当前实际位置(云台端角度，单位：度)
       由 gimbal_read_actual_position() 更新，基于编码器读数解缠绕后得到 */
    float yaw_angle_deg;         /* YAW当前实际角度(度)，经过齿轮比换算和编码器解缠绕 */
    float pitch_angle_deg;       /* PITCH当前实际角度(度) */

    /* 软件追踪的目标位置(最后成功发出的指令位置)
       与实际位置偏差表示丢步或外部扰动 */
    float yaw_commanded_deg;     /* YAW最后指令角度(度)，用于编码器解缠绕参考 */
    float pitch_commanded_deg;   /* PITCH最后指令角度(度) */

    /* 编码器零位偏移(度，范围[0,360))
       编码器读数的绝对偏移量，用于将原始编码器值映射到云台角度。
       计算公式: 编码器相对值 = 原始编码器值 - encoder_zero_deg
       在 gimbal_accept_known_reference() 中初始化。 */
    float yaw_encoder_zero_deg;  /* YAW编码器零位偏移(度) */
    float pitch_encoder_zero_deg;/* PITCH编码器零位偏移(度) */

    /* 软限位(云台端角度，单位：度)
       限制云台运动范围，防止超出机械限位或自碰撞 */
    float yaw_min_deg;           /* YAW最小角度(度)，默认-179度 */
    float yaw_max_deg;           /* YAW最大角度(度)，默认+179度 */
    float pitch_min_deg;         /* PITCH最小角度(度)，预标定时为-85度(GIMBAL_PITCH_BACK_ANGLE) */
    float pitch_max_deg;         /* PITCH最大角度(度)，预标定时为0度(水平) */

    /* 运动参数 */
    uint16_t microstep;          /* 步进电机细分数(1~256)，默认16 */
    uint16_t speed_rpm;          /* 电机目标转速(RPM)，默认300 */
    uint8_t acceleration;        /* 电机加速度，默认50 */

    /* ---- 标定数据 ---- */
    GimbalCalibConfig calib_config;   /* 标定流程参数(速度、加速度、超时等) */
    GimbalAxisCalib   calib_yaw;      /* YAW轴标定结果(限位、范围、中点) */
    GimbalAxisCalib   calib_pitch;    /* PITCH轴标定结果 */
    GimbalGearedCalib geared_pitch;   /* PITCH齿轮比标定数据 */
    GimbalGearedCalib geared_yaw;     /* YAW齿轮比标定数据 */

    /* ---- 状态标志 ---- */
    bool homed;                  /* 是否已完成回零，未回零时禁止运动指令 */
    bool position_valid;         /* 位置追踪是否有效，true=编码器位置已初始化 */
    bool feedback_valid;         /* 编码器反馈是否有效，true=最近一次编码器读取成功 */

    /* ---- 安全机制 ---- */
    bool safety_fault_latched;   /* 安全故障锁存标志
                                  * true=发生电机通信/响应异常，所有运动指令被拒绝
                                  * 需调用 gimbal_clear_safety_fault() 清除 */
    bool manual_mode;            /* 手动模式标志
                                  * true=电机已断电，允许手动转动云台
                                  * false=正常工作模式 */
} Gimbal;

/* 全局云台实例，在 gimbal.c 中定义 */
extern Gimbal g_gimbal;

/* ---- 核心控制API ---- */

/*
 * gimbal_init: 初始化云台模块
 *   初始化UART通信、创建EMM设备对象、配置默认参数(细分数、速度、加速度等)。
 *   不使能电机，需要后续调用 gimbal_enable()。
 * 参数: gimbal - 指向Gimbal结构体的指针
 * 返回: GIMBAL_OK 成功，GIMBAL_ERROR_MOTOR 电机通信失败 */
GimbalStatus gimbal_init(Gimbal *gimbal);

/*
 * gimbal_enable: 使能/禁能云台电机
 *   使能时先检查安全故障锁存状态，异常则锁存故障。
 *   使能后验证电机状态寄存器确认切换成功。
 * 参数: gimbal - 云台对象; enable - true使能, false禁能
 * 返回: GIMBAL_OK 成功, GIMBAL_ERROR_SAFETY_LATCHED 故障锁存,
 *       GIMBAL_ERROR_MOTOR 电机通信失败 */
GimbalStatus gimbal_enable(Gimbal *gimbal, bool enable);

/*
 * gimbal_stop: 紧急停止云台电机
 *   发送停止指令并验证响应，失败则锁存安全故障。
 * 参数: gimbal - 云台对象
 * 返回: GIMBAL_OK 成功, GIMBAL_ERROR_MOTOR 停止失败 */
GimbalStatus gimbal_stop(Gimbal *gimbal);

/*
 * gimbal_zero_position: 云台回零
 *   将电机当前位置设为编码器零点，然后调用 gimbal_accept_known_reference()
 *   将软件位置重置为(0, 0)，即云台归中。
 * 参数: gimbal - 云台对象
 * 返回: GIMBAL_OK 成功, 其他值表示失败 */
GimbalStatus gimbal_zero_position(Gimbal *gimbal);

/*
 * gimbal_accept_known_reference: 接受已知参考位置
 *   当云台处于已知姿态时调用此函数，读取当前编码器值并据此计算
 *   编码器零位偏移(encoder_zero_deg)，建立编码器-角度的映射关系。
 *   这是编码器位置追踪的初始化关键步骤。
 * 参数: gimbal - 云台对象
 *       known_yaw_deg - 当前YAW角度的已知值(度)
 *       known_pitch_deg - 当前PITCH角度的已知值(度)
 * 返回: GIMBAL_OK 成功, GIMBAL_ERROR 参数无效,
 *       GIMBAL_ERROR_SENSOR 编码器读取失败 */
GimbalStatus gimbal_accept_known_reference(Gimbal *gimbal,
                                           float known_yaw_deg,
                                           float known_pitch_deg);

/*
 * gimbal_clear_safety_fault: 清除安全故障锁存
 *   先停止并禁能电机，然后清除锁存标志。
 *   如果停止/禁能操作失败，锁存保持。
 * 参数: gimbal - 云台对象
 * 返回: GIMBAL_OK 成功, GIMBAL_ERROR_MOTOR 清除失败(锁存保持) */
GimbalStatus gimbal_clear_safety_fault(Gimbal *gimbal);

/*
 * gimbal_latch_safety_fault: 手动触发安全故障锁存
 *   设置 lock 标志，后续所有运动指令都将被拒绝。
 *   用于上层应用检测到异常时主动锁存。 */
void gimbal_latch_safety_fault(Gimbal *gimbal);

/*
 * gimbal_move_relative: 相对运动
 *   从当前位置移动指定的相对角度。先读取实际位置，再调用 gimbal_move_to_validated。
 *   受单步限幅(GIMBAL_MAX_COMMAND_STEP_DEG)保护。
 * 参数: gimbal - 云台对象; yaw_delta_deg/pitch_delta_deg - 相对位移(度)
 * 返回: GIMBAL_OK 成功, GIMBAL_ERROR_NOT_HOMED 未回零,
 *       GIMBAL_ERROR_SAFETY_LATCHED 故障锁存, GIMBAL_ERROR_SENSOR 读数失败 */
GimbalStatus gimbal_move_relative(Gimbal *gimbal, float yaw_delta_deg, float pitch_delta_deg);

/*
 * gimbal_move_to: 绝对运动
 *   移动到指定绝对角度。先读取实际位置，再调用 gimbal_move_to_validated。
 *   受软限位和单步限幅保护。
 * 参数: gimbal - 云台对象; yaw_deg/pitch_deg - 目标绝对角度(度)
 * 返回: GIMBAL_OK 成功, 错误码同 gimbal_move_relative */
GimbalStatus gimbal_move_to(Gimbal *gimbal, float yaw_deg, float pitch_deg);

/*
 * gimbal_debug_probe_emm_uart: 调试工具
 *   探测EMM UART总线上的设备，打印设备信息。
 *   用于调试通信连接。 */
void gimbal_debug_probe_emm_uart(void);

/* ---- 标定API ---- */

/* 当 GIMBAL_ENABLE_CALIBRATION=0 时，返回状态码的标定API仍可链接，
 * 但会返回 GIMBAL_ERROR_CALIB；限位应用操作变为无操作。 */

/*
 * gimbal_auto_calibrate: 自动标定双轴
 *   阻塞调用，每轴需要约15-40秒。自动探索两轴的机械限位并计算范围。
 *   向控制台输出详细调试日志。
 * 参数: gimbal - 云台对象
 * 返回: GIMBAL_OK 成功, GIMBAL_ERROR_CALIB 标定失败 */
GimbalStatus gimbal_auto_calibrate(Gimbal *gimbal);

/*
 * gimbal_calibrate_geared: 齿轮比标定
 *   用户手动将PITCH轴调整到0°(正前方水平位置)作为参考，
 *   系统自动探索两端的机械限位并计算齿轮比。
 *   每次机械组装后调用一次，记录并持久化结果。
 * 参数: gimbal - 云台对象
 * 返回: GIMBAL_OK 成功, GIMBAL_ERROR_CALIB 标定失败 */
GimbalStatus gimbal_calibrate_geared(Gimbal *gimbal);

/*
 * gimbal_calibrate_axis: 单轴标定
 *   探测指定轴的运动范围。内部使用，也可独立调用。
 * 参数: gimbal - 云台对象; motor - EMM电机设备指针(yaw或pitch)
 *       axis_name - 轴名称字符串("YAW"或"PITCH")，用于日志输出
 *       result - 输出参数，填充标定结果(min/max/range/mid)
 * 返回: GIMBAL_OK 成功, GIMBAL_ERROR_CALIB 标定失败 */
GimbalStatus gimbal_calibrate_axis(Gimbal *gimbal, EmmDevice *motor,
                                   const char *axis_name, GimbalAxisCalib *result);

/* ---- 位置读取 ---- */

/*
 * gimbal_read_actual_position: 读取实际位置(带编码器解缠绕)
 *   使用命令码0x31读取电机绝对编码器，经零位偏移和齿轮比换算为云台角度。
 *   通过解缠绕(Unwrapping)算法处理编码器过零跳变，使角度追踪连续。
 *   回零(`homed=true`)后可用。
 * 参数: gimbal - 云台对象; yaw_deg/pitch_deg - 输出参数，接收当前角度(度)
 * 返回: GIMBAL_OK 成功, GIMBAL_ERROR_NOT_HOMED 未回零,
 *       GIMBAL_ERROR_MOTOR 编码器读取失败 */
GimbalStatus gimbal_read_actual_position(Gimbal *gimbal, float *yaw_deg, float *pitch_deg);

/* ---- 软限位 ---- */

/*
 * gimbal_set_limits_from_calib: 从标定结果设置软限位
 *   将标定得到的 min_deg/max_range 应用到 Gimbal 结构体的限位字段，
 *   使运动指令受标定范围约束。 */
void gimbal_set_limits_from_calib(Gimbal *gimbal);

/* ---- 手动模式 ---- */

/*
 * gimbal_enter_manual_mode: 进入手动模式
 *   停止并禁能两个电机，使云台可自由手动转动。
 *   设置 manual_mode 标志，运动指令将被拒绝。
 * 参数: gimbal - 云台对象
 * 返回: GIMBAL_OK 成功, GIMBAL_ERROR_MOTOR 操作失败 */
GimbalStatus gimbal_enter_manual_mode(Gimbal *gimbal);

/*
 * gimbal_exit_manual_mode: 退出手动模式
 *   重新使能电机，读取当前编码器位置重新同步位置追踪。
 *   需要 position_valid=true 且无安全故障锁存。
 * 参数: gimbal - 云台对象
 * 返回: GIMBAL_OK 成功, GIMBAL_ERROR_SAFETY_LATCHED 故障锁存,
 *       GIMBAL_ERROR_NOT_HOMED 未回零, GIMBAL_ERROR_SENSOR 读数失败 */
GimbalStatus gimbal_exit_manual_mode(Gimbal *gimbal);

#ifdef __cplusplus
}
#endif

#endif
