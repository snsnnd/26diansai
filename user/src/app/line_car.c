/**
 * @file line_car.c
 * @brief 智能车巡线主控程序 - 核心逻辑实现
 *
 * 本文件实现了智能车巡线控制的所有核心功能，包括：
 * - 系统初始化和硬件配置
 * - 多任务调度(传感器采集、控制计算、遥测输出)
 * - 巡线PID控制算法
 * - 速度闭环控制
 * - H2024竞赛任务状态机集成
 * - 电池电压补偿
 * - 电机死区测试
 * - 远程遥控和在线调参
 * - 故障检测和处理
 * - 调试数据输出(VOFA协议和二进制协议)
 *
 * 总代码约2710行，是本项目最核心的源文件。
 */

#include "app/line_car.h"
#include "app/battery_compensation.h"
#include "app/car_tuning.h"
#include "app/h2024_tasks.h"
#include "app/line_event_detector.h"
#include "app/motor_test.h"
#include "config.h"
#include "pin_mapping.h"
#if EC_ENABLE_VOFA
#include "protocol/vofa.h"
#endif
#include "device/t8_gray_sensor.h"
#include "driver/dt_buzzer.h"
#include "driver/dt_encoder.h"
#include "driver/dt_heading.h"
#include "driver/dt_gyro_z.h"
#include "driver/dt_led.h"
#include "driver/dt_motor.h"
#include "driver/dt_oled.h"
#include "framework/ec_app.h"
#include "framework/ec_keys.h"
#include "framework/ec_menu.h"
#include "framework/ec_mode_manager.h"
#include "framework/ec_parameter_menu.h"
#include "framework/ec_scheduler.h"
#include "framework/ec_time.h"
#include "isr.h"
#include "lib/car_utils.h"
#include "lib/pid_controller.h"
#include "lib/serial_tx_buffer.h"

#include <stdio.h>

/* ==================== 通讯协议常量 ==================== */
#define REMOTE_CMD_MARKER      0xCDu    /* 远程遥控命令帧头标记 */
#define REMOTE_CMD_MAGIC       0x4Du    /* 远程遥控命令魔法字(用于校验) */

/* ==================== 编译时静态检查 ==================== */
/* 确保电机最小运行PWM值不超过最大PWM值，防止配置错误 */
#if MOTOR_MIN_RUN_PWM_L >= MOTOR_PWM_DUTY_MAX
#error "MOTOR_MIN_RUN_PWM_L must be below MOTOR_PWM_DUTY_MAX"
#endif
#if MOTOR_MIN_RUN_PWM_R >= MOTOR_PWM_DUTY_MAX
#error "MOTOR_MIN_RUN_PWM_R must be below MOTOR_PWM_DUTY_MAX"
#endif
#if MOTOR_DEADZONE_TEST_START_PWM >= MOTOR_PWM_DUTY_MAX
#error "MOTOR_DEADZONE_TEST_START_PWM must be below MOTOR_PWM_DUTY_MAX"
#endif

/* ==================== 二进制调试协议帧类型 ==================== */
#define CAR_FRAME_TASK_STATS       0x01u   /* 任务调度统计帧 */
#define CAR_FRAME_TUNE_RESPONSE    0x02u   /* 调参响应帧 */
#define CAR_FRAME_CMD_RESPONSE     0x03u   /* 远程命令响应帧 */
#define CAR_FRAME_EVENT            0x10u   /* 事件帧(初始化、按键、传感器等) */
#define CAR_FRAME_RUN              0x11u   /* 运行状态帧(运行时周期性发送) */
#define CAR_FRAME_SENSOR           0x12u   /* 传感器数据帧 */
#define CAR_FRAME_IMU              0x13u   /* 航向传感器数据帧 */
#define CAR_FRAME_TIMING           0x14u   /* 时序统计帧 */
#define CAR_FRAME_FAULT            0x15u   /* 故障快照帧 */

/* ==================== 事件类型码 ==================== */
#define CAR_EVENT_INIT             0x01u   /* 系统初始化完成事件 */
#define CAR_EVENT_KEY              0x02u   /* 按键事件 */
#define CAR_EVENT_HEADING_INIT     0x03u   /* 航向传感器初始化结果事件 */
#define CAR_EVENT_LINE_STATE       0x04u   /* 巡线状态切换事件 */
#define CAR_EVENT_MOTOR_DEADZONE   0x05u   /* 电机死区测试结果事件 */
#define CAR_TX_BUFFER_SIZE         1024u   /* 调试串口发送缓冲区大小 */

/* ==================== 模式索引枚举 ==================== */
typedef enum
{
    CAR_MODE_LINE_FOLLOW = 0,      /* 巡线跟随行驶模式 */
    CAR_MODE_SPEED_TEST,           /* 速度测试(直线行驶)模式 */
    CAR_MODE_MOTOR_LEFT_RAW,       /* 左电机独立原始PWM测试 */
    CAR_MODE_MOTOR_RIGHT_RAW,      /* 右电机独立原始PWM测试 */
    CAR_MODE_MOTOR_LEFT_BOOST,     /* 左电机启动增强低速测试 */
    CAR_MODE_MOTOR_RIGHT_BOOST,    /* 右电机启动增强低速测试 */
    CAR_MODE_MOTOR_DEADZONE,       /* 电机死区自动检测模式 */
    CAR_MODE_TUNING,               /* 在线调参模式(OLED菜单) */
    CAR_MODE_HEADING_TEST,         /* 航向传感器测试模式 */
    CAR_MODE_H2024_FIRST           /* H2024竞赛任务起始索引(之后的任务以此索引开始) */
} car_mode_index_t;

/* ==================== 巡线状态枚举 ==================== */
typedef enum
{
    LINE_STATE_TRACKING = 0,  /* 正常循迹状态：检测到黑线，跟踪中 */
    LINE_STATE_FIND           /* 搜索状态：丢失黑线，执行搜索动作 */
} line_state_t;

/* ==================== 故障原因枚举 ==================== */
typedef enum
{
    CAR_FAULT_NONE = 0,           /* 无故障 */
    CAR_FAULT_EMERGENCY,          /* 紧急停止(按键触发) */
    CAR_FAULT_MOTOR_WATCHDOG,     /* 电机看门狗超时(主循环未及时更新PWM) */
    CAR_FAULT_BATTERY,            /* 电池欠压或ADC异常 */
    CAR_FAULT_GYRO,               /* 陀螺仪数据陈旧或I2C错误 */
    CAR_FAULT_LINE_SENSOR,        /* T8灰度传感器I2C通信故障 */
    CAR_FAULT_LINE_LOST,          /* 巡线丢失超时(长时间找不到黑线) */
    CAR_FAULT_UNKNOWN             /* 未知原因 */
} car_fault_reason_t;

/**
 * @brief 故障快照数据结构体
 *
 * 当系统进入故障状态时，捕获当前系统状态的所有关键参数，
 * 用于后续的故障分析和调试。快照通过二进制协议发送到上位机。
 */
typedef struct
{
    uint32_t time_ms;              /* 故障发生时间(毫秒) */
    uint32_t motor_cmd_age_ms;     /* 电机命令停滞时间(毫秒) */
    uint32_t line_age_ms;          /* 巡线传感器数据龄期(毫秒) */
    uint32_t gyro_age_ms;          /* 陀螺仪数据龄期(毫秒) */
    int32_t encoder_l;             /* 左编码器累计边沿数 */
    int32_t encoder_r;             /* 右编码器累计边沿数 */
    uint32_t encoder_qerr_l;       /* 左编码器无效跳变次数 */
    uint32_t encoder_qerr_r;       /* 右编码器无效跳变次数 */
    int16_t cmd_l;                 /* 最后发送的左电机PWM命令 */
    int16_t cmd_r;                 /* 最后发送的右电机PWM命令 */
    uint16_t battery_raw;          /* 电池ADC原始值 */
    uint16_t battery_mv;           /* 电池电压(毫伏) */
    int16_t line_pos;              /* 巡线位置偏差(-7~+7) */
    int16_t rpm_l;                 /* 左电机转速(RPM) */
    int16_t rpm_r;                 /* 右电机转速(RPM) */
    int32_t yaw_cdeg;              /* 偏航角(厘度) */
    int32_t wz_cdeg_s;             /* Z轴角速度(厘度/秒) */
    uint8_t line_bits;             /* T8传感器原始比特位 */
    uint8_t mode;                  /* 故障发生时的运行模式 */
    uint8_t mode_state;            /* 模式管理器状态 */
    uint8_t t8_failures;           /* T8传感器累计失败次数 */
    int8_t t8_status;              /* T8传感器最后状态 */
    uint8_t t8_i2c_status;         /* T8的I2C总线最后错误 */
    uint8_t gyro_status;           /* 航向传感器状态 */
    uint8_t gyro_i2c_status;       /* 航向传感器总线错误状态 */
    uint8_t battery_status;        /* 电池补偿状态 */
    bool motor_armed;              /* 电机是否有命令输出 */
    car_fault_reason_t reason;     /* 故障原因 */
} car_fault_snapshot_t;

/* ==================== 硬件驱动实例(全局) ==================== */
static dt_motor_config_t g_motor_l;           /* 左电机驱动配置 */
static dt_motor_config_t g_motor_r;           /* 右电机驱动配置 */
static dt_encoder_t g_enc_l;                  /* 左编码器 */
static dt_encoder_t g_enc_r;                  /* 右编码器 */
static dt_led_t g_leds[3];                    /* 板载LED(3个) */
static dt_heading_t g_heading;                /* 统一航向传感器 */
static dt_buzzer_config_t g_buzzer;            /* 蜂鸣器 */
static dt_oled_config_t g_oled;                /* OLED显示屏 */

/** 启动提示音序列：蜂鸣器响-停-响 */
static const dt_buzzer_step_t g_startup_tone[] = {
    {true, 80u},    /* 响80ms */
    {false, 80u},   /* 停80ms */
    {true, 80u}     /* 再响80ms */
};

/* ==================== I2C和传感器 ==================== */
static soft_iic_info_struct g_t8_iic;          /* T8灰度传感器的软件I2C */
static T8I2cDevice g_t8;                       /* T8灰度传感器设备 */

/* ==================== 模式管理和调参系统 ==================== */
static ec_mode_manager_t g_mode_manager;        /* 模式管理器(管理所有运行模式) */
static ec_menu_t g_menu;                        /* OLED菜单 */
static ec_parameter_menu_t g_parameter_menu;    /* 调参菜单 */
static ec_parameter_item_t g_parameter_items[18u + EC_ENABLE_VOFA];  /* 调参菜单项 */
static car_tuning_t g_tuning;                   /* 当前调优参数 */
static battery_compensation_t g_battery_compensation;  /* 电池补偿实例 */
static line_event_detector_t g_line_events;     /* 巡线事件检测器 */
#if EC_ENABLE_VOFA
static bool g_vofa_enabled = false;             /* VOFA上位机可视化使能 */
#endif

/* ==================== 巡线状态变量 ==================== */
static line_state_t g_line_state = LINE_STATE_TRACKING;  /* 当前巡线状态 */
static bool g_h2024_in_arc = false;               /* H2024任务是否处于弧线段 */
static float g_h2024_arc_entry_heading = 0.0f;    /* H2024进入弧线时刻的航向角 */
static uint8_t g_line_bits = 0xFFu;             /* T8传感器8位原始数据(0=黑线) */
static int16_t g_line_pos = 99;                 /* 巡线位置偏差值(-7~+7, 99=未检测到) */
static float g_line_pos_f = 99.0f;             /* 灰度连续巡线位置(浮点, -7~+7) */
static int16_t g_prev_pos = 0;                  /* 上一次有效的巡线位置 */
static uint8_t g_white_cnt = 0u;                /* 连续全白(丢线)计数 */
static uint8_t g_t8_failure_count = 0u;          /* T8传感器连续读取失败计数 */
static uint8_t g_t8_recovery_count = 0u;         /* T8传感器恢复计数 */
static T8Status g_t8_last_status = T8_ERROR;    /* T8传感器最后一次读取状态 */

/* ==================== 故障标志 ==================== */
static bool g_line_sensor_fault = false;        /* T8巡线传感器故障标志 */
static bool g_line_lost_fault = false;          /* 巡线丢失故障标志 */
static bool g_battery_fault = false;            /* 电池故障标志 */
static bool g_gyro_fault = false;               /* 陀螺仪故障标志 */
static bool g_encoder_feedback_ready = false;   /* 左右编码器均已成功初始化 */
static volatile bool g_motor_watchdog_fault = false;  /* 电机看门狗故障(ISR可写) */
static volatile bool g_emergency_latched = false;     /* 急停锁定标志(ISR可写) */
static volatile uint32_t g_right_enc_fault_count = 0u;/* 右编码器非法跳变检测计数 */

/* ==================== 控制变量 ==================== */
static float g_last_line_error = 0.0f;           /* 上一次巡线误差值(用于丢线时维持) */
static uint32_t g_find_start_ms = 0u;            /* 进入寻线模式的起始时间 */
static uint32_t g_heading_test_last_print_ms = 0u;

/* ==================== 航向控制变量 ==================== */
static float g_heading_target_deg = 0.0f;        /* 目标航向角(度) */
static float g_heading_error_deg = 0.0f;         /* 当前航向误差(度) */
static uint8_t g_heading_aligned_samples = 0u;   /* 航向对准确认样本计数 */

/* ==================== 路径点信号指示 ==================== */
static uint32_t g_point_led_deadline_ms = 0u;    /* 路径点LED点亮截止时间 */
static bool g_point_led_active = false;           /* 路径点LED激活标志 */

/* ==================== 故障快照 ==================== */
static car_fault_snapshot_t g_fault_snapshot;     /* 故障快照数据 */
static bool g_fault_snapshot_pending = false;      /* 有待发送的故障快照 */

/* ==================== 调试输出 ==================== */
static uint32_t g_last_diagnostic_log_ms = 0u;   /* 上次诊断日志发送时间 */
static uint8_t g_debug_tx_storage[CAR_TX_BUFFER_SIZE];  /* 串口发送缓冲区 */
static SerialTxBuffer g_debug_tx_buffer;           /* 串口发送缓冲区管理器 */

/**
 * @brief 调试串口发送服务(从环形缓冲区写入UART硬件)
 *
 * 将环形缓冲区中的数据逐个字节写入UART，直到缓冲区为空或UART忙。
 * 同时管理UART发送中断的使能状态：缓冲区非空时使能发送中断，
 * 缓冲区空时禁用以减少不必要的CPU开销。
 */
static void car_debug_tx_service(void)
{
    uint8_t byte;

    /* 循环取出缓冲区中的字节并尝试发送 */
    while (serial_tx_buffer_peek(&g_debug_tx_buffer, &byte))
    {
        if (uart_try_write_byte(DEBUG_UART_INDEX, byte) != ZF_TRUE)
        {
            break;  /* UART硬件忙，下次再发 */
        }
        (void)serial_tx_buffer_pop(&g_debug_tx_buffer);
    }

    /* 动态管理UART发送中断：缓冲区有数据时使能中断，空时禁用以减少CPU负载 */
    uart_set_interrupt_config(DEBUG_UART_INDEX,
        serial_tx_buffer_available(&g_debug_tx_buffer) == 0u ?
            UART_INTERRUPT_CONFIG_TX_DISABLE :
            UART_INTERRUPT_CONFIG_TX_ENABLE);
}

/**
 * @brief 调试UART中断回调函数
 *
 * 当UART发送完成或接收事件发生时由中断触发调用。
 * 目前只处理发送完成中断(TX)，用于驱动发送服务。
 *
 * @param state 中断状态标志位掩码
 * @param context 回调上下文(未使用)
 */
static void car_debug_uart_callback(uint32_t state, void *context)
{
    (void)context;
    if ((state & UART_INTERRUPT_STATE_TX) != 0u)
    {
        car_debug_tx_service();  /* 发送中断触发，继续发送缓冲区的数据 */
    }
}

/**
 * @brief 启动/触发调试串口发送(踢一脚)
 *
 * 在临界区内执行发送服务，防止中断竞争。
 * 此函数用于在写入数据后立即尝试发送，而不是等待下一个中断周期。
 */
static void car_debug_tx_kick(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();             /* 关中断，防止服务函数与中断竞争 */
    car_debug_tx_service();      /* 尝试直接发送 */
    if (primask == 0u)
    {
        __enable_irq();          /* 如果之前中断是使能的，恢复使能 */
    }
}

/**
 * @brief 向调试串口写入数据
 *
 * 将数据写入环形缓冲区，然后立即触发发送。
 * 如果缓冲区已满，返回false，数据丢弃。
 *
 * @param data 数据指针
 * @param length 数据长度
 * @return true 写入成功；false 缓冲区已满
 */
static bool car_debug_tx_write(const uint8_t *data, size_t length)
{
    if (!serial_tx_buffer_write(&g_debug_tx_buffer, data, length))
    {
        return false;
    }
    car_debug_tx_kick();  /* 写入后立即触发发送 */
    return true;
}

/**
 * @brief 通过调试串口发送调参协议数据帧
 *
 * 帧格式：0xAA 0x55 type len payload(最多120字节)
 * 帧头0xAA55用于上位机识别帧起始。
 *
 * @param type 帧类型(如调参响应、命令响应等)
 * @param pld 负载数据指针
 * @param len 负载数据长度(不能超过120字节)
 */
static void tune_send(uint8_t type, const uint8_t *pld, uint8_t len)
{
    uint8_t b[128];
    if (len > 120u) return;
    b[0]=0xAAu; b[1]=0x55u;       /* 帧头标记 */
    b[2]=type;                     /* 帧类型 */
    b[3]=len;                      /* 负载长度 */
    if(len>0u) memcpy(&b[4],pld,len);  /* 拷贝负载数据 */
    (void)car_debug_tx_write(b, (size_t)(4u + len));
}

/**
 * @brief 发送二进制格式的数据帧
 *
 * 如果二进制写入器(writer)的数据有效，则通过tune_send发送。
 * 此函数是调试数据发送的统一出口。
 *
 * @param type 帧类型
 * @param writer 二进制写入器(包含要发送的数据)
 */
static void car_binary_send(uint8_t type, const car_binary_writer_t *writer)
{
    if (writer->valid)
    {
        tune_send(type, writer->data, writer->length);
    }
}

/* ==================== 电池电压测量变量 ==================== */
static uint16_t g_bat_mv = 0u;    /* 电池电压(毫伏)，经过分压比换算 */
static uint16_t g_bat_raw = 0u;   /* 电池ADC原始采样值 */

/* ==================== PID控制器实例 ==================== */
static PidController g_spd_pid_l;    /* 左轮速度PID控制器 */
static PidController g_spd_pid_r;    /* 右轮速度PID控制器 */
static PidController g_line_pid;     /* 巡线位置PID控制器(PD控制，I=0) */

/* ==================== 电机控制变量 ==================== */
static float g_rpm_l = 0.0f;             /* 左轮计算转速(RPM，无符号) */
static float g_rpm_r = 0.0f;             /* 右轮计算转速(RPM，无符号) */
static float g_rpm_l_s = 0.0f;           /* 左轮带符号转速(正=前进) */
static float g_rpm_r_s = 0.0f;           /* 右轮带符号转速(正=前进) */
static float g_spd_out_l = 0.0f;         /* 左轮速度PID输出 */
static float g_spd_out_r = 0.0f;         /* 右轮速度PID输出 */
static float g_line_steer_pwm = 0.0f;    /* 经过斜率限制的位置PID修正量 */
static float g_steer_boost_pwm = 0.0f;   /* 防止内轮停转的渐进公共PWM补偿 */
static volatile int16_t g_cmd_l = 0;     /* 左电机最终PWM命令(ISR可读) */
static volatile int16_t g_cmd_r = 0;     /* 右电机最终PWM命令(ISR可读) */
static int8_t g_motor_dir = CAR_DEFAULT_MOTOR_DIRECTION;  /* 电机方向(+1或-1) */
static volatile uint32_t g_last_motor_cmd = 0u;  /* 最后电机命令时间戳(看门狗用) */
static volatile bool g_motor_cmd_armed = false;  /* 电机命令激活标志 */
static volatile bool g_car_running = false;      /* 车辆是否正在运行 */
static uint32_t g_line_startup_boost_deadline_ms = 0u; /* 巡线启动增强结束时间 */

/* ==================== 远程控制变量 ==================== */
static bool g_remote_cmd_active = false;         /* 远程控制是否激活 */
static int16_t g_remote_cmd_l = 0;               /* 远程控制左电机值 */
static int16_t g_remote_cmd_r = 0;               /* 远程控制右电机值 */
static uint32_t g_remote_cmd_last_rx_ms = 0u;    /* 最后远程命令接收时间 */

static motor_test_t g_motor_test;

/* ==================== 时序和采样统计 ==================== */
static uint32_t g_last_sensor = 0u;           /* 最后传感器采样时间 */
static uint32_t g_last_line_sample_ms = 0u;   /* 最后巡线采样时间 */
static uint32_t g_last_encoder_sample_ms = 0u;/* 最后编码器采样时间 */
static uint32_t g_last_control = 0u;           /* 最后控制计算时间 */
static uint32_t g_control_dt_ms = 0u;          /* 控制周期实际间隔(毫秒) */
static uint32_t g_control_dt_min_ms = UINT32_MAX;  /* 最小控制周期(毫秒) */
static uint32_t g_control_dt_max_ms = 0u;      /* 最大控制周期(毫秒) */

/**
 * @brief 巡线位置误差曲线映射
 *
 * 三段式映射：
 *   |pos| ≤ 2.0  → 线性 0.8× （中心区平缓，防抖）
 *   2.0 < |pos| ≤ 5.0  → 线性 1.0× （中间区正常响应）
 *   |pos| > 5.0  → 立方加速 0.02×pos³ （边缘区猛打角，防丢线）
 *
 * @param error 原始位置偏差（-7.0 ~ +7.0）
 * @return 映射后误差值，用于 PD 控制器
 */
static float car_shape_line_error(float error)
{
    float e = car_absf(error);
    float sign = (error >= 0.0f) ? 1.0f : -1.0f;

    if (e <= 2.0f)
    {
        return error * 1.0f;   /* 中心不衰减 */
    }
    else if (e <= 5.0f)
    {
        return error;
    }
    else
    {
        return sign * e * e * e * 0.04f;
    }
}

static void car_reset_line_controller(void)
{
    pid_reset(&g_line_pid);
    g_line_steer_pwm = 0.0f;
}

/**
 * @brief 巡线PD转向控制器
 *
 * 根据巡线位置偏差计算转向控制量(steer)，用于左右轮差速转向。
 * 使用PD控制(P比例+ D微分)，积分项I=0因为巡线不需要消除静差。
 *
 * dt_s异常(<=0或>0.1s)时重置PID并回退到默认dt，这可以防止
 * 长时间停滞后首次更新产生过大的微分项。
 *
 * @param error 巡线位置偏差
 * @param dt_s 控制周期(秒)
 * @return 转向控制量(正值=右转，用于修正line_kd的方向)
 */
static float car_line_steer(float error, float dt_s)
{
    float target_steer;
    float max_step;
    float step;

    /* dt_s异常检测：如果时间差无效或过大，重置积分和微分状态 */
    if (dt_s <= 0.0f || dt_s > 0.1f)
    {
        car_reset_line_controller();
        dt_s = 0.01f;  /* 默认10ms控制周期 */
    }
    /* 设置PD增益(巡线不使用积分项) */
    pid_set_gain(&g_line_pid, g_tuning.line_kp, 0.0f, g_tuning.line_kd);
    g_last_line_error = error;  /* 保存当前误差，用于丢线后维持 */
    target_steer = pid_update(&g_line_pid, car_shape_line_error(error), dt_s);

    /* 有符号修正量必须平滑经过零点，避免左右外轮补偿瞬间交换。 */
    max_step = CAR_LINE_STEER_SLEW_PWM_PER_S * dt_s;
    step = car_clampf(target_steer - g_line_steer_pwm, -max_step, max_step);
    g_line_steer_pwm += step;
    return g_line_steer_pwm;
}

/**
 * @brief 通过参数ID应用调优参数值(用于在线调参)
 *
 * 接收来自串口的调参命令，根据参数ID将请求值经过合法性检查后
 * 写入对应的调优结构体成员。
 *
 * @param id 参数ID(0~14，对应tuning结构体的各个成员)
 * @param requested 请求的目标值
 * @param applied 输出参数，返回实际应用的值(限幅后)
 * @return true 参数有效并应用成功；false 无效参数ID
 *
 * 参数ID映射：
 *   0: target_rpm     1: base_pwm       2: feedforward_gain
 *   3: speed_kp       4: speed_ki       5: speed_kd
 *   6: line_kp        7: line_kd        8: heading_kp
 *   9: heading_kd    10: heading_max_steer
 *  11: left_gain     12: right_gain    13: heading_steer_sign
 *  14: speed_loop_enabled
 *  15: vofa_enabled(仅EC_ENABLE_VOFA使能时)
 */
static bool car_apply_tuning_parameter(uint8_t id, float requested,
    float *applied)
{
    float value;

    if (requested != requested || applied == NULL) return false;  /* NaN检测 */
    value = requested;
    switch (id)
    {
        case 0:  /* 目标转速(RPM) */
            value = car_clampf(value, 0.0f, CAR_TUNE_TARGET_RPM_MAX);
            g_tuning.target_rpm = value;
            break;
        case 1:  /* 基础PWM值 */
            value = car_clampf(value, 0.0f, (float)MOTOR_PWM_DUTY_MAX);
            g_tuning.base_pwm = (int16_t)value;
            value = (float)g_tuning.base_pwm;
            break;
        case 2:  /* 前馈增益 */
            value = car_clampf(value, 0.0f, CAR_TUNE_FF_GAIN_MAX);
            g_tuning.feedforward_gain = value;
            break;
        case 3:  /* 速度环比例增益 */
            value = car_clampf(value, 0.0f, CAR_TUNE_SPEED_KP_MAX);
            g_tuning.speed_kp = value;
            break;
        case 4:  /* 速度环积分增益 */
            value = car_clampf(value, 0.0f, CAR_TUNE_SPEED_KI_MAX);
            g_tuning.speed_ki = value;
            break;
        case 5:  /* 速度环微分增益 */
            value = car_clampf(value, 0.0f, CAR_TUNE_SPEED_KD_MAX);
            g_tuning.speed_kd = value;
            break;
        case 6:  /* 巡线比例增益 */
            value = car_clampf(value, 0.0f, CAR_TUNE_LINE_KP_MAX);
            g_tuning.line_kp = value;
            break;
        case 7:  /* 巡线微分增益 */
            value = car_clampf(value, 0.0f, CAR_TUNE_LINE_KD_MAX);
            g_tuning.line_kd = value;
            break;
        case 8:  /* 航向比例增益 */
            value = car_clampf(value, 0.0f, CAR_TUNE_HEADING_KP_MAX);
            g_tuning.heading_kp = value;
            break;
        case 9:  /* 航向微分增益 */
            value = car_clampf(value, 0.0f, CAR_TUNE_HEADING_KD_MAX);
            g_tuning.heading_kd = value;
            break;
        case 10: /* 航向最大转向PWM */
            value = car_clampf(value, 0.0f, CAR_TUNE_HEADING_MAX_PWM);
            g_tuning.heading_max_steer = value;
            break;
        case 11: /* 左轮增益系数 */
            value = car_clampf(value, CAR_TUNE_WHEEL_GAIN_MIN,
                CAR_TUNE_WHEEL_GAIN_MAX);
            g_tuning.left_gain = value;
            break;
        case 12: /* 右轮增益系数 */
            value = car_clampf(value, CAR_TUNE_WHEEL_GAIN_MIN,
                CAR_TUNE_WHEEL_GAIN_MAX);
            g_tuning.right_gain = value;
            break;
        case 13: /* 航向转向符号(+1或-1) */
            g_tuning.heading_steer_sign = (value < 0.0f) ? -1 : 1;
            value = (float)g_tuning.heading_steer_sign;
            break;
        case 14: /* 速度闭环使能开关 */
            g_tuning.speed_loop_enabled = CAR_ENABLE_WHEEL_SPEED_PID != 0 &&
                value >= 0.5f &&
                g_encoder_feedback_ready;
            value = g_tuning.speed_loop_enabled ? 1.0f : 0.0f;
            break;
#if EC_ENABLE_VOFA
        case 15: /* VOFA上位机可视化使能 */
            g_vofa_enabled = value >= 0.5f;
            value = g_vofa_enabled ? 1.0f : 0.0f;
            break;
#endif
        default:
            return false;  /* 未知参数ID */
    }
    *applied = value;
    return true;
}

/**
 * @brief 获取当前航向角(经过方向修正)
 *
 * 根据heading_steer_sign修正陀螺仪的安装方向。
 * 如果sign为负，航向取反。
 * 注意：航向增加表示物理右转，与陀螺仪模块安装方向无关。
 */
static float car_route_heading(void)
{
    return car_wrap_heading(g_heading.yaw_deg *
        (float)((g_tuning.heading_steer_sign < 0) ? -1 : 1));
}

/**
 * @brief 获取当前Z轴角速度(经过方向修正)
 * @return 角速度(度/秒)
 */
static float car_route_wz(void)
{
    return g_heading.wz_dps *
        (float)((g_tuning.heading_steer_sign < 0) ? -1 : 1);
}

/**
 * @brief 检查陀螺仪数据是否新鲜(未超时)
 * @param now_ms 当前时间戳
 * @return true 陀螺仪数据有效且新鲜；false 数据陈旧
 */
static bool car_gyro_is_fresh(uint32_t now_ms)
{
    return dt_heading_is_fresh(&g_heading, now_ms,
        CAR_GYRO_STALE_TIMEOUT_MS);
}

/**
 * @brief 获取电池补偿状态名称(用于OLED显示)
 * @return 状态字符串
 */
static const char *battery_status_name(void)
{
    switch (g_battery_compensation.status)
    {
        case BATTERY_COMP_OK: return "OK";            /* 正常 */
        case BATTERY_COMP_INVALID: return "ADC";      /* ADC异常 */
        case BATTERY_COMP_UNDERVOLTAGE: return "LOW"; /* 欠压 */
        case BATTERY_COMP_STARTUP:
        default: return "WAIT";                        /* 等待初始化 */
    }
}

/**
 * @brief 确定当前故障原因(按优先级从高到低判断)
 * @return 故障原因枚举值
 */
static car_fault_reason_t car_fault_reason(void)
{
    if (g_emergency_latched) return CAR_FAULT_EMERGENCY;         /* 急停优先级最高 */
    if (g_motor_watchdog_fault) return CAR_FAULT_MOTOR_WATCHDOG; /* 电机看门狗 */
    if (g_battery_fault) return CAR_FAULT_BATTERY;               /* 电池故障 */
    if (g_gyro_fault) return CAR_FAULT_GYRO;                     /* 陀螺仪故障 */
    if (g_line_sensor_fault) return CAR_FAULT_LINE_SENSOR;       /* 巡线传感器故障 */
    if (g_line_lost_fault) return CAR_FAULT_LINE_LOST;           /* 巡线丢失 */
    return CAR_FAULT_UNKNOWN;                                     /* 未知原因 */
}

/**
 * @brief 捕获当前系统状态快照(故障分析用)
 *
 * 当系统进入故障状态时，此函数将当时的系统关键参数记录下来，
 * 包括电机命令、电池电压、巡线数据、编码器数据、陀螺仪数据等。
 * 快照通过二进制协议发送到上位机，供开发者分析故障原因。
 *
 * @param now_ms 故障发生时的系统时间
 */
static void car_capture_fault_snapshot(uint32_t now_ms)
{
    g_fault_snapshot.time_ms = now_ms;                           /* 故障时间 */
    g_fault_snapshot.reason = car_fault_reason();                /* 故障原因 */
    g_fault_snapshot.mode = g_mode_manager.active;               /* 当前运行模式 */
    g_fault_snapshot.mode_state = (uint8_t)g_mode_manager.state; /* 模式状态 */
    g_fault_snapshot.cmd_l = g_cmd_l;                            /* 左电机PWM命令 */
    g_fault_snapshot.cmd_r = g_cmd_r;                            /* 右电机PWM命令 */
    g_fault_snapshot.motor_armed = g_motor_cmd_armed;            /* 电机使能状态 */
    g_fault_snapshot.motor_cmd_age_ms = now_ms - g_last_motor_cmd; /* 电机命令龄期 */
    g_fault_snapshot.battery_raw = g_bat_raw;                    /* 电池ADC原始值 */
    g_fault_snapshot.battery_mv = g_bat_mv;                      /* 电池电压(毫伏) */
    g_fault_snapshot.battery_status = (uint8_t)g_battery_compensation.status; /* 电池状态 */
    g_fault_snapshot.line_bits = g_line_bits;                    /* T8原始数据 */
    g_fault_snapshot.line_pos = g_line_pos;                      /* 巡线位置 */
    g_fault_snapshot.line_age_ms = now_ms - g_last_line_sample_ms; /* 巡线数据龄期 */
    g_fault_snapshot.t8_failures = g_t8_failure_count;           /* T8失败次数 */
    g_fault_snapshot.t8_status = (int8_t)g_t8_last_status;       /* T8最后状态 */
    g_fault_snapshot.t8_i2c_status = (uint8_t)soft_iic_get_last_error(&g_t8_iic); /* T8的I2C错误 */
    g_fault_snapshot.rpm_l = (int16_t)g_rpm_l;                   /* 左轮RPM */
    g_fault_snapshot.rpm_r = (int16_t)g_rpm_r;                   /* 右轮RPM */
    g_fault_snapshot.encoder_l = dt_encoder_get_signed_edges(&g_enc_l); /* 左编码器 */
    g_fault_snapshot.encoder_r = dt_encoder_get_signed_edges(&g_enc_r); /* 右编码器 */
    g_fault_snapshot.encoder_qerr_l = dt_encoder_get_invalid_transitions(&g_enc_l); /* 左编码器无效跳变 */
    g_fault_snapshot.encoder_qerr_r = dt_encoder_get_invalid_transitions(&g_enc_r); /* 右编码器无效跳变 */
    g_fault_snapshot.gyro_status = (uint8_t)g_heading.status;
    g_fault_snapshot.gyro_i2c_status = g_heading.bus_status;
    g_fault_snapshot.gyro_age_ms = g_heading.ready ?
        now_ms - g_heading.last_update_ms : UINT32_MAX;
    g_fault_snapshot.yaw_cdeg = car_scale_float(
        g_heading.yaw_deg, 100.0f);
    g_fault_snapshot.wz_cdeg_s = car_scale_float(
        g_heading.wz_dps, 100.0f);
    g_fault_snapshot_pending = true;                              /* 标记有待发送的快照 */
}

/**
 * @brief 发送系统初始化完成事件帧
 *
 * 在系统初始化完成后发送，包含硬件配置信息，便于上位机了解车辆参数。
 * 内容：事件类型、时间戳、应用配置标识、编码器CPR、轮径、MPU6050状态等。
 *
 * @param now_ms 当前时间戳
 */
static void car_send_init_event(uint32_t now_ms)
{
    car_binary_writer_t writer;

    car_binary_init(&writer);
    car_binary_u8(&writer, CAR_EVENT_INIT);                    /* 事件类型：初始化 */
    car_binary_u32(&writer, now_ms);                           /* 时间戳 */
    car_binary_u8(&writer, EC_APP_PROFILE);                    /* 应用配置标识 */
    car_binary_u16(&writer, ENCODER_CPR);                      /* 编码器每转脉冲数 */
    car_binary_u16(&writer, (uint16_t)car_scale_float(WHEEL_DIAMETER_MM, 100.0f)); /* 轮径(百分之一毫米) */
    car_binary_u8(&writer, 1u);                                 /* 航向传感器启用 */
    car_binary_u8(&writer, (uint8_t)g_heading.source);          /* 航向源 */
    car_binary_u8(&writer, 1u);                                 /* 保留 */
    car_binary_u8(&writer, 2u);                                 /* 保留 */
    car_binary_send(CAR_FRAME_EVENT, &writer);
}

/**
 * @brief 发送按键事件帧
 * @param now_ms 当前时间戳
 * @param key 按键码
 */
static void car_send_key_event(uint32_t now_ms, uint8_t key)
{
    car_binary_writer_t writer;

    car_binary_init(&writer);
    car_binary_u8(&writer, CAR_EVENT_KEY);                     /* 事件类型：按键 */
    car_binary_u32(&writer, now_ms);
    car_binary_u8(&writer, key);                               /* 按键码 */
    car_binary_u8(&writer, (uint8_t)g_mode_manager.state);     /* 当前模式状态 */
    car_binary_u8(&writer, g_mode_manager.selected);            /* 当前选中模式 */
    car_binary_send(CAR_FRAME_EVENT, &writer);
}

/**
 * @brief 发送航向传感器初始化结果事件帧
 *
 * 包含校准结果、偏置值、方差等详细信息，用于调试陀螺仪初始化过程。
 *
 * @param now_ms 当前时间戳
 */
static void car_send_heading_init_event(uint32_t now_ms)
{
    car_binary_writer_t writer;

    car_binary_init(&writer);
    car_binary_u8(&writer, CAR_EVENT_HEADING_INIT);            /* 事件类型 */
    car_binary_u32(&writer, now_ms);
    car_binary_u8(&writer, (uint8_t)g_heading.status);
    car_binary_u8(&writer, g_heading.address);
    car_binary_u8(&writer, g_heading.device_id);
    car_binary_u8(&writer, g_heading.bus_status);
    car_binary_u32(&writer, g_heading.calibration_valid_samples);
    car_binary_i32(&writer, car_scale_float(g_heading.gyro_bias_z, 100.0f));
    car_binary_i32(&writer, car_scale_float(
        g_heading.calibration_variance_dps2, 1000.0f));
    car_binary_i32(&writer, car_scale_float(
        g_heading.calibration_min_gz_dps, 1000.0f));
    car_binary_i32(&writer, car_scale_float(
        g_heading.calibration_max_gz_dps, 1000.0f));
    car_binary_send(CAR_FRAME_EVENT, &writer);
}

/**
 * @brief 发送巡线状态切换事件帧
 *
 * 当巡线状态在TRACKING和FIND之间切换时发送。
 *
 * @param now_ms 当前时间戳
 */
static void car_send_line_state_event(uint32_t now_ms)
{
    car_binary_writer_t writer;

    car_binary_init(&writer);
    car_binary_u8(&writer, CAR_EVENT_LINE_STATE);              /* 事件类型：巡线状态 */
    car_binary_u32(&writer, now_ms);
    car_binary_u8(&writer, (uint8_t)g_line_state);             /* 当前巡线状态 */
    car_binary_u8(&writer, g_line_bits);                        /* T8传感器原始数据 */
    car_binary_i16(&writer, g_line_pos);                        /* 巡线位置 */
    car_binary_send(CAR_FRAME_EVENT, &writer);
}

/**
 * @brief 发送电机死区测试结果事件帧
 * @param now_ms 当前时间戳
 */
static void car_send_motor_deadzone_event(uint32_t now_ms)
{
    car_binary_writer_t writer;

    car_binary_init(&writer);
    car_binary_u8(&writer, CAR_EVENT_MOTOR_DEADZONE);          /* 事件类型：电机死区 */
    car_binary_u32(&writer, now_ms);
    car_binary_i16(&writer, g_motor_test.left_threshold);        /* 左轮死区阈值 */
    car_binary_i16(&writer, g_motor_test.right_threshold);       /* 右轮死区阈值 */
    car_binary_u32(&writer, g_motor_test.left_edge_count);       /* 左轮边沿数 */
    car_binary_u32(&writer, g_motor_test.right_edge_count);      /* 右轮边沿数 */
    car_binary_send(CAR_FRAME_EVENT, &writer);
}

/**
 * @brief 发送航向传感器数据帧
 *
 * MPU6050 提供完整六轴数据；M0 不支持的字段发送 0。
 *
 * @param now_ms 当前时间戳
 */
static void car_send_imu_frame(uint32_t now_ms)
{
    car_binary_writer_t writer;

    car_binary_init(&writer);
    car_binary_u32(&writer, now_ms);
    car_binary_u8(&writer, 1u);
    car_binary_u8(&writer, (uint8_t)g_heading.status);
    car_binary_u8(&writer, g_heading.device_id);
    car_binary_u8(&writer, g_heading.bus_status);
    /* 数据龄期(-1表示不可用) */
    car_binary_i32(&writer, g_heading.ready ?
        (int32_t)(now_ms - g_heading.last_update_ms) : -1);
    car_binary_u32(&writer, g_heading.sample_count);
    car_binary_u32(&writer, g_heading.read_error_count);
    /* 3轴加速度(毫g) */
    car_binary_i32(&writer, car_scale_float(g_heading.ax, 1000.0f));
    car_binary_i32(&writer, car_scale_float(g_heading.ay, 1000.0f));
    car_binary_i32(&writer, car_scale_float(g_heading.az, 1000.0f));
    /* 3轴角速度(百分之一度/秒) */
    car_binary_i32(&writer, car_scale_float(g_heading.gx, 100.0f));
    car_binary_i32(&writer, car_scale_float(g_heading.gy, 100.0f));
    car_binary_i32(&writer, car_scale_float(g_heading.gz, 100.0f));
    /* 温度(百分之一度) */
    car_binary_i32(&writer, car_scale_float(g_heading.temperature, 100.0f));
    /* 航向结果 */
    car_binary_i32(&writer, car_scale_float(g_heading.yaw_deg, 100.0f));
    car_binary_i32(&writer, car_scale_float(g_heading.wz_dps, 100.0f));
    car_binary_send(CAR_FRAME_IMU, &writer);
}

/**
 * @brief 发送故障快照帧
 *
 * 将之前捕获的故障快照通过二进制协议发送到上位机。
 * 包含电机状态、电池、巡线、编码器、陀螺仪等全面诊断信息。
 */
static void car_send_fault_frame(void)
{
    const car_fault_snapshot_t *fault = &g_fault_snapshot;
    car_binary_writer_t writer;

    car_binary_init(&writer);
    /* 基本信息 */
    car_binary_u32(&writer, fault->time_ms);                   /* 故障时间 */
    car_binary_u8(&writer, (uint8_t)fault->reason);            /* 故障原因 */
    car_binary_u8(&writer, fault->mode);                        /* 运行模式 */
    car_binary_u8(&writer, fault->mode_state);                  /* 模式状态 */
    /* 电机状态 */
    car_binary_u8(&writer, fault->motor_armed ? 1u : 0u);      /* 电机使能 */
    car_binary_u32(&writer, fault->motor_cmd_age_ms);           /* 命令龄期 */
    car_binary_i16(&writer, fault->cmd_l);                      /* 左电机PWM */
    car_binary_i16(&writer, fault->cmd_r);                      /* 右电机PWM */
    /* 电池 */
    car_binary_u16(&writer, fault->battery_raw);                /* ADC原始值 */
    car_binary_u16(&writer, fault->battery_mv);                 /* 毫伏值 */
    car_binary_u8(&writer, fault->battery_status);              /* 电池状态 */
    /* 巡线 */
    car_binary_u8(&writer, fault->line_bits);                    /* T8传感器位 */
    car_binary_i16(&writer, fault->line_pos);                    /* 位置偏差 */
    car_binary_u32(&writer, fault->line_age_ms);                 /* 数据龄期 */
    /* T8诊断 */
    car_binary_i8(&writer, fault->t8_status);                    /* 状态 */
    car_binary_u8(&writer, fault->t8_i2c_status);                /* I2C错误 */
    car_binary_u8(&writer, fault->t8_failures);                  /* 失败次数 */
    /* 编码器 */
    car_binary_i16(&writer, fault->rpm_l);                       /* 左轮RPM */
    car_binary_i16(&writer, fault->rpm_r);                       /* 右轮RPM */
    car_binary_i32(&writer, fault->encoder_l);                   /* 左编码器 */
    car_binary_i32(&writer, fault->encoder_r);                   /* 右编码器 */
    car_binary_u32(&writer, fault->encoder_qerr_l);              /* 左无效跳变 */
    car_binary_u32(&writer, fault->encoder_qerr_r);              /* 右无效跳变 */
    /* 陀螺仪 */
    car_binary_u8(&writer, fault->gyro_status);                  /* 状态 */
    car_binary_u8(&writer, fault->gyro_i2c_status);              /* I2C错误 */
    car_binary_u32(&writer, fault->gyro_age_ms);                 /* 数据龄期 */
    car_binary_i32(&writer, fault->yaw_cdeg);                    /* 偏航角 */
    car_binary_i32(&writer, fault->wz_cdeg_s);                   /* 角速度 */
    car_binary_send(CAR_FRAME_FAULT, &writer);
}

/**
 * @brief 发送任务调度统计帧
 *
 * 包含每个调度任务的运行次数和错过截止时间计数，
 * 用于监控系统实时性能。
 *
 * @param now_ms 当前时间戳
 * @param scheduler 调度器实例
 */
static void car_send_task_stats(uint32_t now_ms,
    const ec_scheduler_t *scheduler)
{
    car_binary_writer_t writer;
    uint8_t i;

    car_binary_init(&writer);
    car_binary_u32(&writer, now_ms);
    car_binary_u8(&writer, (uint8_t)g_mode_manager.state);
    car_binary_u8(&writer, scheduler->count);
    for (i = 0u; i < scheduler->count; i++)
    {
        car_binary_u32(&writer, scheduler->tasks[i].run_count);          /* 运行次数 */
        car_binary_u32(&writer, scheduler->tasks[i].missed_deadlines);   /* 错过截止时间次数 */
    }
    car_binary_send(CAR_FRAME_TASK_STATS, &writer);
}

/**
 * @brief 发送运行状态帧(运行时周期性发送)
 *
 * 包含车辆运行的核心数据，是上位机监控的主要数据源。
 * 数据量较大，包含几乎全部运行参数。
 *
 * @param now_ms 当前时间戳
 * @param total_skipped 总跳过调度次数
 * @param total_overruns 总超时运行次数
 * @param diagnostics ISR诊断数据
 */
static void car_send_run_frame(uint32_t now_ms, uint32_t total_skipped,
    uint32_t total_overruns, const isr_diagnostics_t *diagnostics)
{
    car_binary_writer_t writer;

    car_binary_init(&writer);
    car_binary_u32(&writer, now_ms);
    car_binary_u8(&writer, (uint8_t)g_mode_manager.state);       /* 模式状态 */
    car_binary_u8(&writer, g_mode_manager.active);               /* 当前运行模式 */
    car_binary_i16(&writer, (int16_t)g_rpm_l);                   /* 左轮RPM */
    car_binary_i16(&writer, (int16_t)g_rpm_r);                   /* 右轮RPM */
    car_binary_i16(&writer, g_cmd_l);                             /* 左电机PWM */
    car_binary_i16(&writer, g_cmd_r);                             /* 右电机PWM */
    car_binary_u8(&writer, g_line_bits);                          /* T8传感器数据 */
    car_binary_i16(&writer, g_line_pos);                          /* 巡线位置 */
    car_binary_u16(&writer, g_bat_mv);                            /* 电池电压 */
    car_binary_u32(&writer, g_control_dt_ms);                     /* 控制周期 */
    car_binary_u32(&writer, total_skipped);                       /* 调度跳过 */
    car_binary_u32(&writer, dt_encoder_get_invalid_transitions(&g_enc_l)); /* 左编码器错误 */
    car_binary_u32(&writer, dt_encoder_get_invalid_transitions(&g_enc_r)); /* 右编码器错误 */
    car_binary_u32(&writer, now_ms - g_last_line_sample_ms);      /* 巡线数据龄期 */
    car_binary_u32(&writer, now_ms - g_last_encoder_sample_ms);   /* 编码器数据龄期 */
    car_binary_u32(&writer, total_overruns);                      /* 超时运行次数 */
    car_binary_u32(&writer, diagnostics->gpio_drain_limit_hits);  /* GPIO排空限制命中 */
    car_binary_u32(&writer,
        diagnostics->uart_drain_limit_hits[DEBUG_UART_INDEX]);    /* UART排空限制命中 */
    car_binary_u16(&writer,
        (uint16_t)serial_tx_buffer_available(&g_debug_tx_buffer)); /* 缓冲区可用空间 */
    car_binary_u16(&writer,
        (uint16_t)serial_tx_buffer_high_watermark(&g_debug_tx_buffer)); /* 缓冲区最高水位 */
    car_binary_u32(&writer,
        (uint32_t)serial_tx_buffer_rejected_write_count(&g_debug_tx_buffer)); /* 拒绝写入计数 */
    car_binary_u32(&writer,
        (uint32_t)serial_tx_buffer_dropped_byte_count(&g_debug_tx_buffer)); /* 丢弃字节计数 */
    /* H2024任务状态 */
    car_binary_u8(&writer, (uint8_t)h2024_tasks_active_state(&g_mode_manager));
    /* 航向信息 */
    car_binary_i16(&writer, (int16_t)car_scale_float(car_route_heading(), 100.0f));     /* 当前航向 */
    car_binary_i16(&writer, (int16_t)car_scale_float(g_heading_target_deg, 100.0f));    /* 目标航向 */
    car_binary_i16(&writer, (int16_t)car_scale_float(g_heading_error_deg, 100.0f));     /* 航向误差 */
    /* 巡线事件统计 */
    car_binary_u32(&writer, g_line_events.enter_count);          /* 上线次数 */
    car_binary_u32(&writer, g_line_events.exit_count);           /* 离线次数 */
    car_binary_u8(&writer, g_line_events.stable_on_line ? 1u : 0u); /* 当前在线状态 */
    /* 巡线PD参数 */
    car_binary_i32(&writer, car_scale_float(g_tuning.line_kp, 100.0f));  /* 巡线KP(百分之一) */
    car_binary_i32(&writer, car_scale_float(g_tuning.line_kd, 100.0f));  /* 巡线KD(百分之一) */
    /* 编码器详细状态 */
    car_binary_u8(&writer, dt_encoder_get_ab_state(&g_enc_l));   /* 左编码器AB相 */
    car_binary_u8(&writer, dt_encoder_get_ab_state(&g_enc_r));   /* 右编码器AB相 */
    car_binary_u32(&writer, dt_encoder_get_edges(&g_enc_l));     /* 左编码器总边沿 */
    car_binary_u32(&writer, dt_encoder_get_edges(&g_enc_r));     /* 右编码器总边沿 */
    car_binary_u32(&writer, diagnostics->gpio_irq_count);        /* GPIO中断计数 */
    car_binary_u32(&writer, diagnostics->gpio_event_count);      /* GPIO事件计数 */
    car_binary_u8(&writer, 1u);                                   /* 保留 */
    car_binary_u32(&writer, dt_encoder_get_sampled_transitions(&g_enc_l)); /* 左编码器采样跳变 */
    car_binary_u32(&writer, dt_encoder_get_sampled_transitions(&g_enc_r)); /* 右编码器采样跳变 */
    car_binary_send(CAR_FRAME_RUN, &writer);
}

/**
 * @brief 发送传感器数据帧(非运行状态下周期性发送)
 *
 * 电池、T8巡线传感器、编码器、RPM和电机命令等传感器数据。
 *
 * @param now_ms 当前时间戳
 */
static void car_send_sensor_frame(uint32_t now_ms)
{
    car_binary_writer_t writer;

    car_binary_init(&writer);
    car_binary_u32(&writer, now_ms);
    car_binary_u8(&writer, (uint8_t)g_mode_manager.state);       /* 模式状态 */
    car_binary_u8(&writer, g_mode_manager.active);               /* 当前模式 */
    car_binary_u16(&writer, g_bat_raw);                           /* 电池ADC原始值 */
    car_binary_u16(&writer, g_bat_mv);                            /* 电池电压 */
    car_binary_u8(&writer, (uint8_t)g_battery_compensation.status); /* 电池状态 */
    car_binary_u8(&writer, g_line_bits);                          /* T8数据 */
    car_binary_i16(&writer, g_line_pos);                          /* 巡线位置 */
    car_binary_u32(&writer, now_ms - g_last_line_sample_ms);      /* 巡线龄期 */
    car_binary_i8(&writer, (int8_t)g_t8_last_status);             /* T8状态 */
    car_binary_u8(&writer, (uint8_t)soft_iic_get_last_error(&g_t8_iic)); /* T8的I2C错误 */
    car_binary_u8(&writer, g_t8_failure_count);                   /* T8失败次数 */
    car_binary_i16(&writer, (int16_t)g_rpm_l);                    /* 左轮RPM */
    car_binary_i16(&writer, (int16_t)g_rpm_r);                    /* 右轮RPM */
    car_binary_i32(&writer, dt_encoder_get_signed_edges(&g_enc_l)); /* 左编码器 */
    car_binary_i32(&writer, dt_encoder_get_signed_edges(&g_enc_r)); /* 右编码器 */
    car_binary_u32(&writer, dt_encoder_get_invalid_transitions(&g_enc_l)); /* 左编码器错误 */
    car_binary_u32(&writer, dt_encoder_get_invalid_transitions(&g_enc_r)); /* 右编码器错误 */
    car_binary_i16(&writer, g_cmd_l);                             /* 左电机PWM */
    car_binary_i16(&writer, g_cmd_r);                             /* 右电机PWM */
    car_binary_u8(&writer, dt_encoder_get_ab_state(&g_enc_l));    /* 编码器AB相 */
    car_binary_u8(&writer, dt_encoder_get_ab_state(&g_enc_r));
    car_binary_send(CAR_FRAME_SENSOR, &writer);
}

/**
 * @brief 发送时序统计帧(非运行状态下周期性发送)
 *
 * 包含控制周期统计、调度延迟、运行超时、ISR诊断数据等，
 * 用于分析系统实时性能和调度健康状态。
 *
 * @param now_ms 当前时间戳
 * @param total_skipped 总跳过调度次数
 * @param total_overruns 总超时运行次数
 * @param worst_lateness_index 最差延迟任务索引
 * @param worst_lateness_ms 最差启动延迟(毫秒)
 * @param worst_runtime_index 最差运行时间任务索引
 * @param worst_runtime_ms 最差运行时间(毫秒)
 * @param diagnostics ISR诊断数据
 */
static void car_send_timing_frame(uint32_t now_ms, uint32_t total_skipped,
    uint32_t total_overruns, uint8_t worst_lateness_index,
    uint32_t worst_lateness_ms, uint8_t worst_runtime_index,
    uint32_t worst_runtime_ms, const isr_diagnostics_t *diagnostics)
{
    car_binary_writer_t writer;

    car_binary_init(&writer);
    car_binary_u32(&writer, now_ms);
    /* 控制周期统计 */
    car_binary_u32(&writer, g_control_dt_ms);                    /* 当前控制周期 */
    car_binary_u32(&writer,
        g_control_dt_min_ms == UINT32_MAX ? 0u : g_control_dt_min_ms); /* 最小控制周期 */
    car_binary_u32(&writer, g_control_dt_max_ms);                 /* 最大控制周期 */
    car_binary_u32(&writer, now_ms - g_last_encoder_sample_ms);   /* 编码器数据龄期 */
    /* 调度统计 */
    car_binary_u32(&writer, total_skipped);                       /* 调度跳过次数 */
    car_binary_u32(&writer, total_overruns);                      /* 超时运行次数 */
    car_binary_u8(&writer, worst_lateness_index);                 /* 最差延迟任务索引 */
    car_binary_u32(&writer, worst_lateness_ms);                   /* 最差延迟 */
    car_binary_u8(&writer, worst_runtime_index);                  /* 最差运行时间任务索引 */
    car_binary_u32(&writer, worst_runtime_ms);                    /* 最差运行时间 */
    /* ISR诊断 */
    car_binary_u32(&writer, diagnostics->gpio_irq_count);        /* GPIO中断计数 */
    car_binary_u32(&writer, diagnostics->gpio_event_count);      /* GPIO事件计数 */
    car_binary_u32(&writer, diagnostics->gpio_drain_limit_hits); /* GPIO排空限制命中 */
    car_binary_u32(&writer, diagnostics->uart_irq_count[DEBUG_UART_INDEX]); /* UART中断计数 */
    car_binary_u32(&writer,
        diagnostics->uart_drain_limit_hits[DEBUG_UART_INDEX]);    /* UART排空限制 */
    /* 发送缓冲区状态 */
    car_binary_u16(&writer,
        (uint16_t)serial_tx_buffer_available(&g_debug_tx_buffer));     /* 缓冲区剩余 */
    car_binary_u16(&writer,
        (uint16_t)serial_tx_buffer_high_watermark(&g_debug_tx_buffer)); /* 最高占用 */
    car_binary_u32(&writer,
        (uint32_t)serial_tx_buffer_rejected_write_count(&g_debug_tx_buffer)); /* 拒绝写入 */
    car_binary_u32(&writer,
        (uint32_t)serial_tx_buffer_dropped_byte_count(&g_debug_tx_buffer)); /* 丢弃字节 */
    car_binary_send(CAR_FRAME_TIMING, &writer);
}

/**
 * @brief 限幅电机PWM命令值到合法范围
 *
 * 处理NaN、正负饱和等边界情况。NaN的检测利用IEEE754浮点特性
 * (value != value 当value为NaN时为true)。
 *
 * @param speed 期望的PWM速度值
 * @return 限幅后的int16_t类型PWM值，范围[-DT_MOTOR_DUTY_MAX, DT_MOTOR_DUTY_MAX]
 */
static int16_t car_clamp_motor_cmd(float speed)
{
    if (speed != speed)  /* NaN检测：NaN不等于自身 */
    {
        return 0;
    }
    if (speed > (float)DT_MOTOR_DUTY_MAX)
    {
        return (int16_t)DT_MOTOR_DUTY_MAX;
    }
    if (speed < (float)(-DT_MOTOR_DUTY_MAX))
    {
        return (int16_t)(-DT_MOTOR_DUTY_MAX);
    }
    return (int16_t)speed;
}

static int16_t car_drive_pwm_for_rpm(float target_rpm)
{
    if (CAR_ENABLE_SPEED_FEEDFORWARD != 0)
    {
        return car_tuning_feedforward_pwm_for_rpm(&g_tuning, target_rpm);
    }

    return car_clamp_motor_cmd((float)g_tuning.base_pwm);
}

/**
 * @brief 减速优先差速混控
 *
 * steer>0 表示右转，保持左外轮基准并降低右内轮；steer<0 时相反。
 * 该策略不会为纠偏主动提高任一车轮的 PWM。
 */
static void car_mix_deceleration_only(float base, float steer,
    float *left, float *right)
{
    *left = base;
    *right = base;
    if (steer > 0.0f)
    {
        *right = base - steer;
    }
    else if (steer < 0.0f)
    {
        *left = base + steer;
    }
}

static float car_signed_floor(float reference, float minimum)
{
    if (reference > 0.0f && reference < minimum)
    {
        return minimum;
    }
    if (reference < 0.0f && reference > -minimum)
    {
        return -minimum;
    }
    return reference;
}

static float car_update_steer_boost(float required_boost, float dt_s)
{
    float max_step;

    required_boost = car_clampf(required_boost, 0.0f,
        CAR_STEER_BOOST_MAX_PWM);
    if (dt_s <= 0.0f || dt_s > 0.1f)
    {
        dt_s = 0.01f;
    }
    max_step = CAR_STEER_BOOST_SLEW_PWM_PER_S * dt_s;
    if (g_steer_boost_pwm < required_boost)
    {
        g_steer_boost_pwm = car_clampf(g_steer_boost_pwm + max_step,
            0.0f, required_boost);
    }
    else
    {
        g_steer_boost_pwm = car_clampf(g_steer_boost_pwm - max_step,
            required_boost, CAR_STEER_BOOST_MAX_PWM);
    }
    return g_steer_boost_pwm;
}

static void car_prevent_steering_stall(float steer, float dt_s,
    float *left, float *right)
{
    float required_boost = 0.0f;
    float boost;

    /* 速度闭环模式下无需防停转保护 */
    if (CAR_ENABLE_WHEEL_SPEED_PID != 0 && g_tuning.speed_loop_enabled)
        return;

    if (steer > 0.0f)
    {
        required_boost = (float)MOTOR_MIN_RUN_PWM_R - *right;
    }
    else if (steer < 0.0f)
    {
        required_boost = (float)MOTOR_MIN_RUN_PWM_L - *left;
    }

    boost = car_update_steer_boost(required_boost, dt_s);
    *left += boost;
    *right += boost;
    *left = car_forward_floor(*left, (float)MOTOR_MIN_RUN_PWM_L);
    *right = car_forward_floor(*right, (float)MOTOR_MIN_RUN_PWM_R);
}

static float car_pwm_compensation_factor(void)
{
    return CAR_ENABLE_BATTERY_PWM_COMPENSATION != 0 ?
        battery_compensation_factor(&g_battery_compensation) : 1.0f;
}

static float car_apply_pwm_compensation(float reference_pwm)
{
    if (CAR_ENABLE_BATTERY_PWM_COMPENSATION != 0)
    {
        return battery_compensation_apply(&g_battery_compensation,
            reference_pwm, (float)DT_MOTOR_DUTY_MAX);
    }

    return battery_compensation_can_run(&g_battery_compensation) ?
        car_clampf(reference_pwm, (float)-DT_MOTOR_DUTY_MAX,
            (float)DT_MOTOR_DUTY_MAX) : 0.0f;
}

/**
 * @brief 应用原始PWM命令到电机硬件(最终输出函数)
 *
 * 这是电机PWM输出的最终关卡。所有控制计算、电池补偿和死区策略
 * 必须在调用此函数前完成。诊断系统直接使用此函数的输出作为可信的原始PWM值。
 *
 * 函数内部还会进行二次安全检查：如果存在任何故障标志，
 * 强制将命令设为0并调用电机紧急停止。
 *
 * 注意：此函数会在临界区内修改g_cmd_l/g_cmd_r，确保ISR读取的一致性。
 *
 * @param l_speed 左电机PWM命令
 * @param r_speed 右电机PWM命令
 * @param now_ms 当前时间戳(用于更新看门狗计时)
 */
static void apply_motor_raw_cmd(int16_t l_speed, int16_t r_speed, uint32_t now_ms)
{
    uint32_t primask;

    /* 限幅到合法PWM范围 */
    l_speed = car_clamp_motor_cmd((float)l_speed);
    r_speed = car_clamp_motor_cmd((float)r_speed);

    /* 临界区：更新共享变量 */
    primask = __get_PRIMASK();
    __disable_irq();

    /* 安全检查：任何故障标志存在时强制停止电机 */
    if (g_battery_fault || g_gyro_fault || g_motor_watchdog_fault ||
        g_emergency_latched)
    {
        l_speed = 0;
        r_speed = 0;
    }

    /* 更新电机命令和看门狗计时 */
    g_last_motor_cmd = now_ms;
    g_motor_cmd_armed = (l_speed != 0 || r_speed != 0);
    if (g_motor_cmd_armed)
    {
        g_car_running = true;
    }
    g_cmd_l = l_speed;
    g_cmd_r = r_speed;

    if (primask == 0u)
    {
        __enable_irq();
    }

    /* 最终输出到电机硬件(乘以输出方向符号) */
    dt_motor_set_speed(&g_motor_l,
        (int16_t)(l_speed * MOTOR_LEFT_OUTPUT_SIGN));
    dt_motor_set_speed(&g_motor_r,
        (int16_t)(r_speed * MOTOR_RIGHT_OUTPUT_SIGN));

    /* 二次安全检查：如果设置了故障，调用紧急停止 */
    if (g_battery_fault || g_gyro_fault || g_motor_watchdog_fault ||
        g_emergency_latched)
    {
        dt_motor_emergency_stop(&g_motor_l);
        dt_motor_emergency_stop(&g_motor_r);
        g_motor_cmd_armed = false;
    }
}

/**
 * @brief 停止所有电机并重置PID控制器状态
 *
 * 与紧急停止不同，此函数不设置故障标志，
 * 只是干净地停止电机并重置控制状态以便后续重新启动。
 */
static void car_stop_motors(void)
{
    g_car_running = false;             /* 清除运行标志 */
    g_spd_out_l = 0.0f;                /* 清除速度环输出 */
    g_spd_out_r = 0.0f;
    g_steer_boost_pwm = 0.0f;
    g_line_startup_boost_deadline_ms = 0u;
    car_reset_line_controller();       /* 重置所有PID控制器 */
    pid_reset(&g_spd_pid_l);
    pid_reset(&g_spd_pid_r);
    dt_motor_stop(&g_motor_l);          /* 立即停止左电机 */
    dt_motor_stop(&g_motor_r);          /* 立即停止右电机 */
    g_cmd_l = 0;                        /* 清除PWM命令 */
    g_cmd_r = 0;
    g_motor_cmd_armed = false;          /* 清除电机使能标志 */
}

/* ==================== 运行准备和故障处理 ==================== */

/**
 * @brief 准备进入运行状态
 *
 * 在进入任何需要电机转动的模式之前调用。
 * 重置所有PID控制器、清零速度输出、初始化控制周期计时器。
 *
 * @param now_ms 当前时间戳
 */
static void car_prepare_run(uint32_t now_ms)
{
    g_spd_out_l = 0.0f;              /* 清除速度PID输出 */
    g_spd_out_r = 0.0f;
    g_steer_boost_pwm = 0.0f;
    g_last_line_error = 0;           /* 清除上次巡线误差 */
    car_reset_line_controller();     /* 重置巡线PID */
    pid_reset(&g_spd_pid_l);         /* 重置左轮速度PID */
    pid_reset(&g_spd_pid_r);         /* 重置右轮速度PID */
    g_last_control = now_ms;         /* 初始化控制计时 */
    g_control_dt_ms = 0u;
    g_control_dt_min_ms = UINT32_MAX;
    g_control_dt_max_ms = 0u;
    g_last_motor_cmd = now_ms;       /* 更新电机命令时间(防止看门狗误触发) */
    g_motor_cmd_armed = false;       /* 电机尚未开始输出 */
    g_car_running = true;            /* 标记车辆正在运行 */
}

/**
 * @brief 进入故障状态
 *
 * 当系统检测到任何故障时调用。主要操作：
 * 1. 如果当前有运行中的模式，停止它
 * 2. 停止所有电机
 * 3. 如果是首次故障，捕获故障快照(包含故障发生时的电机命令)
 * 4. 将模式管理器状态设为FAULT
 * 5. 标记菜单需要刷新
 *
 * @param now_ms 故障发生时间
 */
static void car_enter_fault(uint32_t now_ms)
{
    bool first_fault = g_mode_manager.state != EC_MODE_FAULT;
    /* 在电机命令被清除前保存快照 */
    int16_t cmd_l = g_cmd_l;
    int16_t cmd_r = g_cmd_r;
    bool motor_armed = g_motor_cmd_armed;
    uint32_t motor_cmd_age_ms = now_ms - g_last_motor_cmd;

    /* 清除远程控制状态 */
    g_remote_cmd_active = false;
    g_remote_cmd_l = 0;
    g_remote_cmd_r = 0;

    /* 停止正在运行的模式或电机 */
    if (g_mode_manager.state == EC_MODE_RUNNING)
    {
        ec_mode_manager_stop(&g_mode_manager, now_ms);
    }
    else
    {
        car_stop_motors();
    }

    /* 首次故障时保存快照(记录完整的故障上下文) */
    if (first_fault)
    {
        car_capture_fault_snapshot(now_ms);
        /* 用前面保存的值覆盖(因为capture函数中命令已经被清除) */
        g_fault_snapshot.cmd_l = cmd_l;
        g_fault_snapshot.cmd_r = cmd_r;
        g_fault_snapshot.motor_armed = motor_armed;
        g_fault_snapshot.motor_cmd_age_ms = motor_cmd_age_ms;
    }

    g_mode_manager.state = EC_MODE_FAULT;  /* 切换为故障状态 */
    g_menu.dirty = true;                    /* 刷新OLED显示 */
}

/* ==================== 远程遥控命令处理 ==================== */

/**
 * @brief 处理远程控制命令
 *
 * 当远程控制激活时，此函数在控制任务中周期性调用。
 * 它会检查远程命令的超时和故障状态，然后应用电机命令。
 *
 * 远程控制只允许在STOPPED状态下使用(车辆静止)。
 * 如果检测到故障或超时，自动退出远程控制模式并停止。
 *
 * @param now_ms 当前时间戳
 * @return true 远程控制有效且已处理；false 未激活或已停止
 */
static bool car_service_remote_cmd(uint32_t now_ms)
{
    if (!g_remote_cmd_active)
    {
        return false;
    }
    /* 远程控制只允许在STOPPED状态 */
    if (g_mode_manager.state != EC_MODE_STOPPED)
    {
        g_remote_cmd_active = false;
        g_remote_cmd_l = 0;
        g_remote_cmd_r = 0;
        car_stop_motors();
        return false;
    }
    /* 检查故障状态 */
    if (g_battery_fault || g_motor_watchdog_fault || g_emergency_latched ||
        !battery_compensation_can_run(&g_battery_compensation))
    {
        car_enter_fault(now_ms);
        return false;
    }
    /* 远程命令超时检查(一段时间未收到新命令则停止) */
    if ((uint32_t)(now_ms - g_remote_cmd_last_rx_ms) >= MOTOR_REMOTE_TIMEOUT_MS)
    {
        g_remote_cmd_active = false;
        g_remote_cmd_l = 0;
        g_remote_cmd_r = 0;
        car_stop_motors();
        return false;
    }

    /* 应用远程命令到电机 */
    apply_motor_raw_cmd(g_remote_cmd_l, g_remote_cmd_r, now_ms);
    return true;
}

/* ==================== 电机看门狗 ==================== */

/**
 * @brief PIT定时器中断中调用的电机看门狗检查(ISR级别)
 *
 * 如果电机命令激活但超过MOTOR_COMMAND_TIMEOUT_MS未更新，
 * 在ISR中直接执行PWM紧急停止。
 * 这是安全关键的最后一道防线：即使主循环挂起，PWM也会被切断。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
static void car_motor_watchdog_tick(uint32_t now_ms, void *context)
{
    (void)context;
    if (g_motor_cmd_armed &&
        (uint32_t)(now_ms - g_last_motor_cmd) >= MOTOR_COMMAND_TIMEOUT_MS)
    {
        g_motor_cmd_armed = false;              /* 清除使能标志 */
        g_motor_watchdog_fault = true;          /* 设置看门狗故障 */
        dt_motor_emergency_stop(&g_motor_l);    /* 直接停止左电机 */
        dt_motor_emergency_stop(&g_motor_r);    /* 直接停止右电机 */
        g_cmd_l = 0;                             /* 清除PWM命令 */
        g_cmd_r = 0;
    }
}

/**
 * @brief 主循环层电机看门狗服务
 *
 * 在主循环中检查电机命令是否超时。如果超时，
 * 触发完整的故障处理流程(包括故障快照和模式切换)。
 * 实际的PWM停止已在ISR级别由car_motor_watchdog_tick完成。
 *
 * @param now_ms 当前时间戳
 */
static void car_service_motor_watchdog(uint32_t now_ms)
{
    if (g_motor_cmd_armed &&
        (uint32_t)(now_ms - g_last_motor_cmd) >= MOTOR_COMMAND_TIMEOUT_MS)
    {
        g_motor_watchdog_fault = true;
        car_enter_fault(now_ms);
    }
}

/**
 * @brief 紧急停止中断服务函数(按键触发)
 *
 * 按键中断触发时调用，立即停止电机并设置急停锁定。
 * 此函数在ISR上下文中执行，必须尽可能快。
 *
 * @param context 未使用
 */
static void car_emergency_isr_stop(void *context)
{
    (void)context;
    if (!g_car_running)
    {
        return;  /* 车未运行，无需处理 */
    }
    g_emergency_latched = true;                 /* 设置急停锁定 */
    g_motor_cmd_armed = false;                  /* 清除电机使能 */
    dt_motor_emergency_stop(&g_motor_l);        /* 直接停止左电机 */
    dt_motor_emergency_stop(&g_motor_r);        /* 直接停止右电机 */
}

/**
 * @brief 外部可调用的紧急停止函数(非ISR)
 *
 * 设置急停标志并通过car_enter_fault触发完整的故障处理流程。
 * 其他模块(如H2024任务)可以调用此函数来请求急停。
 */
void line_car_emergency_stop(void)
{
    g_emergency_latched = true;
    car_enter_fault(ec_time_ms());
}

/* ==================== 巡线传感器数据处理 ==================== */

/**
 * @brief 将T8灰度传感器的8位数据转换为位置值
 *
 * T8传感器输出为8位数据，每位对应一个传感器(低电平=黑线,高电平=白底)。
 * 此函数计算检测到黑线的传感器位置加权平均，得到线位置。
 *
 * 位置范围：-7(最左) ~ +7(最右)，99表示全白(未检测到黑线)。
 * 每个传感器的位置权重：i*2-7 (即 -7,-5,-3,-1,1,3,5,7)
 *
 * @param bits T8传感器8位数据(active-low)
 * @return 巡线位置值(-7~+7)或99(未检测到)
 */
static int16_t line_pos_from_bits(uint8_t bits)
{
    int16_t sum = 0;
    int16_t count = 0;
    int i;

    for (i = 0; i < 8; i++)
    {
        if (((bits >> i) & 1u) == 0u)
        {
            sum += (int16_t)(i * 2 - 7);
            count++;
        }
    }
    return (count > 0) ? (int16_t)(sum / count) : 99;
}

/**
 * @brief 更新巡线传感器数据
 *
 * T8灰度传感器通过I2C接口读取8个通道的数字信号。
 * 此函数处理：
 * 1. I2C通信故障检测和容错(连续失败超过阈值触发故障)
 * 2. 从原始传感器数据计算线位置
 * 3. 更新线事件检测器(用于路径点检测)
 * 4. 全黑(0x00)和全白(0xFF)的特殊处理
 * 5. 丢线时的位置保持(短时间使用上一次有效值)
 *
 * @param now_ms 当前时间戳
 */
/**
 * @brief 更新巡线传感器数据
 *
 * T8 八路二值读取，质心计算离散位置后经一阶 IIR 时域插值，
 * 在 7 个通道间隙各产生虚拟中间位置，输出连续浮点位置。
 *
 * @param now_ms 当前时间戳
 */
static void update_line_sensor(uint32_t now_ms)
{
    uint8_t line_bits;
    int16_t raw_pos;
    float raw_pos_f;

    /* 读 T8 硬件二值化位 */
    g_t8_last_status = t8_i2c_get_digital(&g_t8, &line_bits);

    if (g_t8_last_status != T8_OK)
    {
        g_t8_recovery_count = 0u;
        if (g_t8_failure_count < CAR_T8_FAILURE_LIMIT)
        {
            g_t8_failure_count++;
        }
        if (g_t8_failure_count >= CAR_T8_FAILURE_LIMIT && !g_line_sensor_fault)
        {
            g_line_sensor_fault = true;
            if (g_mode_manager.state == EC_MODE_RUNNING)
            {
                car_enter_fault(now_ms);
            }
        }
        return;
    }

    /* ---- 读取成功处理 ---- */
    g_t8_failure_count = 0u;
    g_last_line_sample_ms = now_ms;
    if (g_line_sensor_fault && g_t8_recovery_count < CAR_T8_RECOVERY_SAMPLES)
    {
        g_t8_recovery_count++;
    }

    g_line_bits = line_bits;
    line_event_detector_update(&g_line_events, line_bits != 0xFFu, now_ms);
    raw_pos = line_pos_from_bits(line_bits);

    /* 全黑 → 中心 */
    if (line_bits == 0x00u)
    {
        raw_pos = 0;
        g_white_cnt = 0u;
    }
    /* 全白 → 丢线保持 */
    else if (raw_pos == 99)
    {
        if (g_white_cnt < 0xFFu) g_white_cnt++;
        if (g_white_cnt <= 3u && g_prev_pos != 99)
        {
            raw_pos = g_prev_pos;
        }
    }
    else
    {
        g_white_cnt = 0u;
        if (raw_pos != 0) g_prev_pos = raw_pos;
    }

    /* 一阶 IIR 时域插值：7个通道间隙各产生虚拟中间位置 */
    raw_pos_f = (float)raw_pos;
    if (g_line_pos_f > 98.0f)  /* 首次或从丢线恢复，直接跳变 */
    {
        g_line_pos_f = raw_pos_f;
    }
    else
    {
        /* α=0.55: 新值占55%权重，2帧收敛到稳态 */
        g_line_pos_f = g_line_pos_f * 0.45f + raw_pos_f * 0.55f;
    }

    g_line_pos = (g_line_pos_f <= 98.0f) ? (int16_t)(g_line_pos_f + 0.5f) : raw_pos;
}

/**
 * @brief 陀螺仪更新任务
 *
 * 更新当前 CAR_GYRO_SOURCE 选择的航向传感器。
 * 如果读取成功，标记陀螺仪就绪并记录最新数据时间。
 * 此任务的调度频率决定了航向控制的更新率。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void line_car_gyro_task(uint32_t now_ms, void *context)
{
    (void)context;
    (void)dt_heading_update(&g_heading, now_ms);
    if (dt_gyro_z_data_received())
    {
        dt_led_toggle(&g_leds[2]);
    }
}

/**
 * @brief 传感器综合任务
 *
 * 在每个调度周期执行：
 * 1. 多次采样电池ADC并取平均(提高测量精度)
 * 2. 将ADC原始值转换为毫伏电压(考虑分压比)
 * 3. 更新电池补偿模块(滤波和状态机)
 * 4. 检查电池状态，故障时触发保护
 * 5. 计算左右电机的实时RPM
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void line_car_sensor_task(uint32_t now_ms, void *context)
{
    uint32_t dt_ms;
    uint32_t adc_sum = 0u;
    uint8_t sample_index;
    bool adc_valid = true;

    (void)context;

    dt_ms = now_ms - g_last_sensor;   /* 计算距上次采样的时间差 */
    g_last_sensor = now_ms;

    /* ---- 电池电压ADC采样(多次平均) ---- */
    for (sample_index = 0u; sample_index < BAT_ADC_SAMPLE_COUNT; sample_index++)
    {
        uint16_t sample;
        if (adc_convert_checked(BAT_ADC, &sample) != ZF_NO_ERROR)
        {
            adc_valid = false;
            break;
        }
        adc_sum += sample;
    }

    if (adc_valid)
    {
        /* 计算平均ADC值 */
        g_bat_raw = (uint16_t)(adc_sum / BAT_ADC_SAMPLE_COUNT);
        /* 转换为毫伏：ADC值 * 参考电压 * 分压比 / (满量程 * 分压系数) */
        g_bat_mv = (uint16_t)((uint64_t)g_bat_raw * BAT_ADC_REF_MV *
            BAT_DIVIDER / (BAT_ADC_FULL_SCALE * BAT_DIVIDER_SCALE));
    }

    /* 更新电池补偿模块(含低通滤波和状态机) */
    battery_compensation_update(&g_battery_compensation, g_bat_mv, adc_valid);

    /* 使用滤波后的电池电压覆盖原始测量值(更稳定) */
    if (g_battery_compensation.has_sample)
    {
        float filtered_mv = battery_compensation_voltage_mv(&g_battery_compensation);
        g_bat_mv = (filtered_mv >= 65535.0f) ? 65535u : (uint16_t)(filtered_mv + 0.5f);
    }

    /* 电池故障检测：INVALID或UNDERVOLTAGE时触发电池故障 */
    if ((g_battery_compensation.status == BATTERY_COMP_INVALID ||
         g_battery_compensation.status == BATTERY_COMP_UNDERVOLTAGE) &&
        !g_battery_fault)
    {
        g_battery_fault = true;
        if (g_mode_manager.state == EC_MODE_RUNNING)
        {
            car_enter_fault(now_ms);
        }
    }

    /* 计算电机RPM：根据编码器脉冲数和时间差计算转速 */
    g_rpm_l = dt_encoder_compute_rpm(&g_enc_l, dt_ms);
    g_rpm_r = dt_encoder_compute_rpm(&g_enc_r, dt_ms);
    g_rpm_l_s = dt_encoder_compute_signed_rpm(&g_enc_l, dt_ms);
    g_rpm_r_s = dt_encoder_compute_signed_rpm(&g_enc_r, dt_ms);

    /* 右编码器GPIO中断丢脉冲诊断 */
    if (!g_enc_r.hardware_quadrature)
    {
        uint32_t invalid = dt_encoder_get_invalid_transitions(&g_enc_r);
        /* 每累计100次非法跳变递增诊断计数 */
        if (invalid > 0u && (invalid % 100u) == 0u)
        {
            g_right_enc_fault_count++;
        }

        /* 对比 GPIO ISR 边沿计数与 5ms 轮询采样计数，检测 ISR 丢脉冲 */
        {
            static uint32_t s_last_isr = 0u;
            static uint32_t s_last_poll = 0u;
            uint32_t isr_now = dt_encoder_get_edges(&g_enc_r);
            uint32_t poll_now = dt_encoder_get_sampled_transitions(&g_enc_r);
            uint32_t isr_delta = isr_now - s_last_isr;
            uint32_t poll_delta = poll_now - s_last_poll;
            s_last_isr = isr_now;
            s_last_poll = poll_now;
            /* ISR 应捕获至少 poll 的 3 倍脉冲；落差过大说明 ISR 丢失 */
            if (isr_delta > 50u && poll_delta > 30u &&
                isr_delta * 3u < poll_delta * 5u)
            {
                g_right_enc_fault_count++;
            }
        }
    }

    g_last_encoder_sample_ms = now_ms;
}

/**
 * @brief 巡线传感器更新任务
 *
 * 读取T8灰度传感器并更新巡线位置和事件状态。
 * 独立于sensor_task，便于单独调整采样频率。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void line_car_line_sensor_task(uint32_t now_ms, void *context)
{
    (void)context;
    update_line_sensor(now_ms);
}

/**
 * @brief 速度控制主函数(巡线模式或纯速度模式)
 *
 * 这是车辆运动控制的核心函数，实现以下功能：
 *
 * 1. 计算控制周期dt并更新统计数据
 * 2. 前馈PWM计算(基于目标RPM)
 * 3. 巡线转向控制(PD控制器)
 * 4. 速度闭环控制(可选，由speed_loop_enabled控制)
 * 5. 寻线模式(FIND)的特殊处理
 * 6. 电池电压补偿
 * 7. 左右轮增益修正(补偿电机个体差异)
 * 8. 直行轮/外轮死区补偿(forward_floor函数)
 *
 * @param now_ms 当前时间戳
 * @param line_follow true=巡线模式(使用巡线转向)；false=纯速度模式(直线)
 */
static void run_speed_control(uint32_t now_ms, bool line_follow)
{
    uint32_t control_dt_ms;
    float control_dt_s;
    int16_t comp_speed;
    int16_t left_speed;
    int16_t right_speed;
    float left_reference;
    float right_reference;
    float left_base = 0.0f;
    float right_base = 0.0f;
    float steer = 0.0f;
    int8_t motor_dir;
    float compensation_factor;
    bool startup_boost_active = false;

    control_dt_ms = now_ms - g_last_control;    /* 计算控制周期 */
    g_last_control = now_ms;
    g_control_dt_ms = control_dt_ms;

    /* 更新控制周期统计数据 */
    if (control_dt_ms < g_control_dt_min_ms) g_control_dt_min_ms = control_dt_ms;
    if (control_dt_ms > g_control_dt_max_ms) g_control_dt_max_ms = control_dt_ms;
    control_dt_s = (float)control_dt_ms / 1000.0f;  /* 转换为秒 */

    /* 安全检查：电池状态不可运行则停止电机 */
    if (!battery_compensation_can_run(&g_battery_compensation))
    {
        car_stop_motors();
        return;
    }

    /* 计算当前驱动基准和启用后的 PWM 补偿因子。 */
    comp_speed = car_drive_pwm_for_rpm(g_tuning.target_rpm);
    compensation_factor = car_pwm_compensation_factor();
    if (line_follow && g_line_startup_boost_deadline_ms != 0u)
    {
        startup_boost_active =
            (int32_t)(now_ms - g_line_startup_boost_deadline_ms) < 0;
        if (!startup_boost_active)
        {
            g_line_startup_boost_deadline_ms = 0u;
        }
    }

    /* ========== 寻线模式(FIND)：特殊处理 ========== */
    if (line_follow && g_line_state == LINE_STATE_FIND)
    {
        float find_speed = car_clampf((float)comp_speed,
            CAR_LINE_FIND_PWM_MIN, CAR_LINE_FIND_PWM_MAX);

        /* 寻线时使用原地旋转搜索，不启用速度PID */
        g_spd_out_l = 0.0f;
        g_spd_out_r = 0.0f;
        left_base = 0.0f;
        right_base = 0.0f;

        /* 使用与正常巡线相同的误差方向，向最后一次见线的位置旋转。 */
        if (g_prev_pos > 0)
        {
            left_reference = find_speed;
            right_reference = -find_speed;
        }
        else
        {
            left_reference = -find_speed;
            right_reference = find_speed;
        }
    }
    else
    {
        /* ========== 正常循迹模式 ========== */

        /* 巡线模式且有有效位置时计算转向量 */
        if (!startup_boost_active && line_follow && g_line_pos != 99)
        {
            steer = (float)g_tuning.line_steer_sign *
                car_line_steer(g_line_pos_f, control_dt_s);
        }

        if (startup_boost_active)
        {
            left_base = (float)MOTOR_STARTUP_BOOST_PWM;
            right_base = (float)MOTOR_STARTUP_BOOST_PWM;
            g_spd_out_l = 0.0f;
            g_spd_out_r = 0.0f;
            pid_reset(&g_spd_pid_l);
            pid_reset(&g_spd_pid_r);
        }
        else if (CAR_ENABLE_WHEEL_SPEED_PID != 0 &&
            g_tuning.speed_loop_enabled && g_encoder_feedback_ready)
        {
            /* ===== 速度闭环控制(双闭环：位置PD外环 + 速度PI内环) ===== */
            float reference_limit = (float)DT_MOTOR_DUTY_MAX / compensation_factor;
            float rpm_per_pwm = car_absf(g_tuning.feedforward_gain);
            float steer_rpm;
            float left_target_rpm;
            float right_target_rpm;

            /* 防止除零：rpm_per_pwm至少为1 */
            if (rpm_per_pwm < 1.0f) rpm_per_pwm = 1.0f;

            /* 将转向控制量(steer)转换为左右轮的目标RPM差 */
            steer_rpm = steer / rpm_per_pwm;

            /* 减速优先：外轮保持基准PWM，只降内轮 */
            car_mix_deceleration_only(g_tuning.target_rpm, steer_rpm,
                &left_target_rpm, &right_target_rpm);
            left_target_rpm = car_clampf(left_target_rpm,
                g_tuning.target_rpm * CAR_LINE_MIN_TARGET_RPM_RATIO,
                CAR_TUNE_TARGET_RPM_MAX);
            right_target_rpm = car_clampf(right_target_rpm,
                g_tuning.target_rpm * CAR_LINE_MIN_TARGET_RPM_RATIO,
                CAR_TUNE_TARGET_RPM_MAX);

            /* 根据当前驱动策略计算左右轮基准 PWM。 */
            left_base = (float)car_drive_pwm_for_rpm(left_target_rpm);
            right_base = (float)car_drive_pwm_for_rpm(right_target_rpm);

            /* 仅在基值/gain变化时更新 PID 参数，避免每周期冗余重算 */
            {
                static float s_last_lb = -1.0f;
                static float s_last_rb = -1.0f;
                static float s_last_kp = -1.0f;
                static float s_last_ki = -1.0f;
                static float s_last_kd = -1.0f;
                if (left_base != s_last_lb || right_base != s_last_rb)
                {
                    pid_set_limits(&g_spd_pid_l,
                        -reference_limit - left_base,
                        reference_limit - left_base, -CAR_SPEED_PID_INTEGRAL_MAX,
                        CAR_SPEED_PID_INTEGRAL_MAX);
                    pid_set_limits(&g_spd_pid_r,
                        -reference_limit - right_base,
                        reference_limit - right_base, -CAR_SPEED_PID_INTEGRAL_MAX,
                        CAR_SPEED_PID_INTEGRAL_MAX);
                    s_last_lb = left_base;
                    s_last_rb = right_base;
                }
                if (g_tuning.speed_kp != s_last_kp ||
                    g_tuning.speed_ki != s_last_ki ||
                    g_tuning.speed_kd != s_last_kd)
                {
                    pid_set_gain(&g_spd_pid_l, g_tuning.speed_kp,
                        g_tuning.speed_ki, g_tuning.speed_kd);
                    pid_set_gain(&g_spd_pid_r, g_tuning.speed_kp,
                        g_tuning.speed_ki, g_tuning.speed_kd);
                    s_last_kp = g_tuning.speed_kp;
                    s_last_ki = g_tuning.speed_ki;
                    s_last_kd = g_tuning.speed_kd;
                }
            }

            /* 速度PID更新：误差 = 目标RPM - 实际RPM（带符号） */
            g_spd_out_l = pid_update(&g_spd_pid_l,
                left_target_rpm - g_rpm_l_s, control_dt_s);
            g_spd_out_r = pid_update(&g_spd_pid_r,
                right_target_rpm - g_rpm_r_s, control_dt_s);
        }
        else
        {
            /* ===== 开环控制(无速度反馈) ===== */
            left_base = (float)comp_speed;
            right_base = (float)comp_speed;
            if (line_follow)
            {
                /* 巡线时外轮保持基础 PWM，仅降低内轮。 */
                car_mix_deceleration_only((float)comp_speed, steer,
                    &left_base, &right_base);
            }
            /* 开环模式下速度PID输出为0 */
            g_spd_out_l = 0.0f;
            g_spd_out_r = 0.0f;
            pid_reset(&g_spd_pid_l);
            pid_reset(&g_spd_pid_r);
        }

        /* 最终参考值 = 驱动基准 + 可选速度 PID 输出。 */
        left_reference = left_base + g_spd_out_l;
        right_reference = right_base + g_spd_out_r;
    }

    /* ========== 最终输出处理 ========== */

    /* 电机方向 */
    motor_dir = (g_motor_dir <= 0) ? -1 : 1;
    g_motor_dir = motor_dir;

    /* 左右轮增益修正(补偿电机个体差异) */
    left_reference *= car_clampf(g_tuning.left_gain,
        CAR_TUNE_WHEEL_GAIN_MIN, CAR_TUNE_WHEEL_GAIN_MAX);
    right_reference *= car_clampf(g_tuning.right_gain,
        CAR_TUNE_WHEEL_GAIN_MIN, CAR_TUNE_WHEEL_GAIN_MAX);

    /* 启动阶段直接使用固定物理 PWM，不受轮增益和转向补偿影响。 */
    if (startup_boost_active && line_follow &&
        g_line_state == LINE_STATE_TRACKING)
    {
        left_reference = (float)MOTOR_STARTUP_BOOST_PWM;
        right_reference = (float)MOTOR_STARTUP_BOOST_PWM;
        g_steer_boost_pwm = 0.0f;
    }
    /* 低速阶段保持内轮持续运行，外轮通过渐进公共补偿建立差速。 */
    else if (line_follow && g_line_state == LINE_STATE_TRACKING)
    {
        car_prevent_steering_stall(steer, control_dt_s,
            &left_reference, &right_reference);
    }

    /* FIND 需要保留一侧反转命令；普通巡线仍禁止反向输出。 */
    if (line_follow && g_line_state == LINE_STATE_FIND)
    {
        float reference_limit = (float)DT_MOTOR_DUTY_MAX / compensation_factor;

        left_reference = car_signed_floor(left_reference,
            (float)MOTOR_MIN_RUN_PWM_L);
        right_reference = car_signed_floor(right_reference,
            (float)MOTOR_MIN_RUN_PWM_R);
        left_reference = car_clampf(left_reference, -reference_limit,
            reference_limit);
        right_reference = car_clampf(right_reference, -reference_limit,
            reference_limit);
    }
    else
    {
        left_reference = car_clampf(left_reference, 0.0f,
            (float)DT_MOTOR_DUTY_MAX / compensation_factor);
        right_reference = car_clampf(right_reference, 0.0f,
            (float)DT_MOTOR_DUTY_MAX / compensation_factor);
    }

    /* 应用电池补偿并转换方向，然后限幅到硬件PWM范围 */
    left_speed = car_clamp_motor_cmd(car_apply_pwm_compensation(
        left_reference * (float)motor_dir));
    right_speed = car_clamp_motor_cmd(car_apply_pwm_compensation(
        right_reference * (float)motor_dir));

    /* 最终输出到电机硬件 */
    apply_motor_raw_cmd(left_speed, right_speed, now_ms);
}

/**
 * @brief 统一巡线电机输出（标准 LINE FOLLOW 和 H2024 共用）
 *
 * 根据传入的转向量计算左右轮 PWM 并输出。控制逻辑与标准巡线
 * 的开环路径完全一致：基准 PWM 按 target_rpm 计算，差速转向，
 * 轮增益修正，防内轮停转，电池电压补偿。
 *
 * @param steer 转向控制量
 * @param now_ms 当前时间戳
 */
static void car_line_motor_output(float steer, uint32_t now_ms)
{
    float compensation_factor;
    float reference_limit;
    float left_reference;
    float right_reference;
    int16_t comp_speed;
    int16_t left_speed;
    int16_t right_speed;
    int8_t motor_dir;

    g_control_dt_ms = now_ms - g_last_control;
    g_last_control = now_ms;
    if (g_control_dt_ms < g_control_dt_min_ms)
        g_control_dt_min_ms = g_control_dt_ms;
    if (g_control_dt_ms > g_control_dt_max_ms)
        g_control_dt_max_ms = g_control_dt_ms;
    if (!battery_compensation_can_run(&g_battery_compensation))
    {
        car_stop_motors();
        return;
    }

    comp_speed = car_drive_pwm_for_rpm(g_tuning.target_rpm);
    compensation_factor = car_pwm_compensation_factor();
    reference_limit = (float)DT_MOTOR_DUTY_MAX / compensation_factor;
    car_mix_deceleration_only((float)comp_speed, steer,
        &left_reference, &right_reference);
    left_reference = car_clampf(left_reference, 0.0f, reference_limit);
    right_reference = car_clampf(right_reference, 0.0f, reference_limit);
    left_reference *= car_clampf(g_tuning.left_gain,
        CAR_TUNE_WHEEL_GAIN_MIN, CAR_TUNE_WHEEL_GAIN_MAX);
    right_reference *= car_clampf(g_tuning.right_gain,
        CAR_TUNE_WHEEL_GAIN_MIN, CAR_TUNE_WHEEL_GAIN_MAX);

    car_prevent_steering_stall(steer,
        (float)g_control_dt_ms / 1000.0f,
        &left_reference, &right_reference);

    left_reference = car_clampf(left_reference, 0.0f, reference_limit);
    right_reference = car_clampf(right_reference, 0.0f, reference_limit);
    motor_dir = (g_motor_dir <= 0) ? -1 : 1;
    left_speed = car_clamp_motor_cmd(car_apply_pwm_compensation(
        left_reference * (float)motor_dir));
    right_speed = car_clamp_motor_cmd(car_apply_pwm_compensation(
        right_reference * (float)motor_dir));
    apply_motor_raw_cmd(left_speed, right_speed, now_ms);
}

static void run_h2024_forward_control(uint32_t now_ms, float base_scale,
    float steer, bool prevent_wheel_stall)
{
    float compensation_factor;
    float reference_limit;
    float left_reference;
    float right_reference;
    int16_t comp_speed;
    int16_t left_speed;
    int16_t right_speed;
    int8_t motor_dir;

    g_control_dt_ms = now_ms - g_last_control;
    g_last_control = now_ms;
    if (g_control_dt_ms < g_control_dt_min_ms) g_control_dt_min_ms = g_control_dt_ms;
    if (g_control_dt_ms > g_control_dt_max_ms) g_control_dt_max_ms = g_control_dt_ms;
    if (!battery_compensation_can_run(&g_battery_compensation))
    {
        car_stop_motors();
        return;
    }

    comp_speed = car_clamp_motor_cmd((float)car_drive_pwm_for_rpm(g_tuning.target_rpm) *
        car_clampf(base_scale, 0.0f, 1.0f));
    compensation_factor = car_pwm_compensation_factor();
    reference_limit = (float)DT_MOTOR_DUTY_MAX / compensation_factor;
    car_mix_deceleration_only((float)comp_speed, steer,
        &left_reference, &right_reference);
    left_reference = car_clampf(left_reference, 0.0f, reference_limit);
    right_reference = car_clampf(right_reference, 0.0f, reference_limit);
    left_reference *= car_clampf(g_tuning.left_gain,
        CAR_TUNE_WHEEL_GAIN_MIN, CAR_TUNE_WHEEL_GAIN_MAX);
    right_reference *= car_clampf(g_tuning.right_gain,
        CAR_TUNE_WHEEL_GAIN_MIN, CAR_TUNE_WHEEL_GAIN_MAX);
    if (prevent_wheel_stall)
    {
        car_prevent_steering_stall(steer,
            (float)g_control_dt_ms / 1000.0f,
            &left_reference, &right_reference);
    }
    else
    {
        g_steer_boost_pwm = 0.0f;
    }
    left_reference = car_clampf(left_reference, 0.0f, reference_limit);
    right_reference = car_clampf(right_reference, 0.0f, reference_limit);
    motor_dir = (g_motor_dir <= 0) ? -1 : 1;
    left_speed = car_clamp_motor_cmd(car_apply_pwm_compensation(
        left_reference * (float)motor_dir));
    right_speed = car_clamp_motor_cmd(car_apply_pwm_compensation(
        right_reference * (float)motor_dir));
    apply_motor_raw_cmd(left_speed, right_speed, now_ms);
}

/* ==================== 通用车辆控制接口实现 ==================== */

/**
 * @brief 设置陀螺仪故障标志
 */
static void h2024_set_gyro_fault(void)
{
    g_gyro_fault = true;
    car_stop_motors();
    g_menu.dirty = true;
}

/**
 * @brief 读取车辆当前状态（航向、巡线事件、在线标志）
 * @param state 输出参数
 * @param now_ms 当前时间戳
 * @return true 成功；false 陀螺仪故障
 */
bool car_read_state(h2024_vehicle_state_t *state,
    uint32_t now_ms)
{
    if (state == NULL || !car_gyro_is_fresh(now_ms))
    {
        h2024_set_gyro_fault();
        return false;
    }
    state->heading_deg = car_route_heading();           /* 当前航向 */
    state->line_enter_count = g_line_events.enter_count; /* 上线次数 */
    state->line_exit_count = g_line_events.exit_count;   /* 离线次数 */
    state->on_line = g_line_events.stable_on_line;       /* 在线状态 */
    state->travel_mm = (dt_encoder_get_travel_mm(&g_enc_l) +
        dt_encoder_get_travel_mm(&g_enc_r)) * 0.5f;     /* 平均行驶距离 */
    return true;
}

/**
 * @brief 沿指定航向直线行驶（PD航向保持）
 *
 * 使用PD控制(比例+微分)保持车辆沿目标航向行驶。
 * 比例项对角度误差响应，微分项对角速度(角速率)提供阻尼，
 * 防止过度转向和振荡。
 *
 * @param heading_deg 目标航向角(度)
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void car_drive_heading(float heading_deg, uint32_t now_ms)
{
    float steer;

    g_h2024_in_arc = false;
    if (!car_gyro_is_fresh(now_ms))
    {
        h2024_set_gyro_fault();
        return;
    }
    /* 计算航向误差 */
    g_heading_target_deg = car_wrap_heading(heading_deg);
    g_heading_error_deg = car_wrap_heading(g_heading_target_deg -
        car_route_heading());

    /* PD控制：steer = Kp * angle_error - Kd * angular_velocity */
    steer = g_tuning.heading_kp * g_heading_error_deg -
        g_tuning.heading_kd * car_route_wz();
    steer = car_clampf(steer, -g_tuning.heading_max_steer,
        g_tuning.heading_max_steer);

    /* 航向控制期间不使用巡线PID */
    g_last_line_error = 0;
    car_reset_line_controller();
    g_heading_aligned_samples = 0u;

    run_h2024_forward_control(now_ms, 1.0f, steer, true);
}

/**
 * @brief 巡线行驶（位置PD + 陀螺仪辅助丢线恢复）
 *
 * 在黑线上循迹前进。使用巡线PD控制器计算转向量。
 * 转向方向通过LINE SIGN参数统一配置，配合HEADING SIGN保证航向
 * 和巡线的转向方向一致。
 *
 * 丢线时通过陀螺仪航向变化判断原因：
 * - 航向变化 >= 130°（接近弧线出口）：保持上次转向继续前向，快速到达路径点
 * - 航向变化 < 130°（弧中丢线）：原地 pivot 找线，与标准 LINE FOLLOW 一致
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void car_follow_line(uint32_t now_ms)
{
    float steer = 0.0f;
    float error;

    if (!g_h2024_in_arc)
    {
        g_h2024_arc_entry_heading = car_route_heading();
        g_h2024_in_arc = true;
    }

    if (g_line_pos != 99)
    {
        float dt_s = (float)(now_ms - g_last_control) / 1000.0f;
        error = g_line_pos_f;
        steer = (float)g_tuning.line_steer_sign *
            car_line_steer(error, dt_s);
    }
    else
    {
        float heading_change = car_absf(car_wrap_heading(
            car_route_heading() - g_h2024_arc_entry_heading));

        if (heading_change < H2024_ARC_MIN_TURN_DEG)
        {
            float find_speed;
            float compensation_factor;
            float ref_limit;
            float left_ref;
            float right_ref;
            int8_t motor_dir;
            int16_t left_pwm;
            int16_t right_pwm;

            find_speed = car_clampf(
                (float)car_drive_pwm_for_rpm(g_tuning.target_rpm),
                CAR_LINE_FIND_PWM_MIN, CAR_LINE_FIND_PWM_MAX);
            compensation_factor = car_pwm_compensation_factor();
            ref_limit = (float)DT_MOTOR_DUTY_MAX / compensation_factor;
            motor_dir = (g_motor_dir <= 0) ? -1 : 1;

            if (g_prev_pos > 0)
            {
                left_ref = find_speed;
                right_ref = -find_speed;
            }
            else
            {
                left_ref = -find_speed;
                right_ref = find_speed;
            }
            left_ref = car_signed_floor(left_ref,
                (float)MOTOR_MIN_RUN_PWM_L);
            right_ref = car_signed_floor(right_ref,
                (float)MOTOR_MIN_RUN_PWM_R);
            left_ref = car_clampf(left_ref, -ref_limit, ref_limit);
            right_ref = car_clampf(right_ref, -ref_limit, ref_limit);
            left_ref *= car_clampf(g_tuning.left_gain,
                CAR_TUNE_WHEEL_GAIN_MIN, CAR_TUNE_WHEEL_GAIN_MAX);
            right_ref *= car_clampf(g_tuning.right_gain,
                CAR_TUNE_WHEEL_GAIN_MIN, CAR_TUNE_WHEEL_GAIN_MAX);
            left_pwm = car_clamp_motor_cmd(
                car_apply_pwm_compensation(left_ref * (float)motor_dir));
            right_pwm = car_clamp_motor_cmd(
                car_apply_pwm_compensation(right_ref * (float)motor_dir));

            g_last_control = now_ms;
            g_spd_out_l = 0.0f;
            g_spd_out_r = 0.0f;
            pid_reset(&g_spd_pid_l);
            pid_reset(&g_spd_pid_r);
            car_reset_line_controller();
            g_heading_aligned_samples = 0u;
            apply_motor_raw_cmd(left_pwm, right_pwm, now_ms);
            return;
        }

        float dt_s = (float)(now_ms - g_last_control) / 1000.0f;
        error = (g_last_line_error != 0) ? g_last_line_error : g_prev_pos;
        steer = (float)g_tuning.line_steer_sign *
            car_line_steer(error, dt_s);
    }
    g_heading_aligned_samples = 0u;
    car_line_motor_output(steer, now_ms);
}

/**
 * @brief 原地差速转向对准目标航向
 *
 * 使用差速转向原地调整车头方向，直到对准目标航向。
 * 对准判定使用迟滞比较(连续H2024_ALIGN_CONFIRM_SAMPLES次在容差内)。
 * 转向时使用较低的速度(H2024_ALIGN_BASE_SCALE)以获得更高的精度。
 *
 * @param heading_deg 目标航向角(度)
 * @param now_ms 当前时间戳
 * @param context 未使用
 * @return true 对准完成；false 还在调整中
 */
bool car_align_heading(float heading_deg,
    uint32_t now_ms)
{
    float pivot_pwm;

    if (!car_gyro_is_fresh(now_ms))
    {
        h2024_set_gyro_fault();
        return false;
    }
    g_heading_target_deg = car_wrap_heading(heading_deg);
    g_heading_error_deg = car_wrap_heading(g_heading_target_deg -
        car_route_heading());

    /* 如果角度误差在容差范围内，认为对准完成 */
    if (car_absf(g_heading_error_deg) <= H2024_ALIGN_TOLERANCE_DEG)
    {
        car_stop_motors();
        /* 需要连续多次确认，防止噪声导致误判 */
        if (g_heading_aligned_samples < H2024_ALIGN_CONFIRM_SAMPLES)
        {
            g_heading_aligned_samples++;
        }
        return g_heading_aligned_samples >= H2024_ALIGN_CONFIRM_SAMPLES;
    }

    /* 误差超出容差，继续调整 */
    g_heading_aligned_samples = 0u;
    pivot_pwm = (float)car_drive_pwm_for_rpm(g_tuning.target_rpm) *
        H2024_ALIGN_BASE_SCALE;  /* 使用降低的速度提高对准精度 */
    run_h2024_forward_control(now_ms, H2024_ALIGN_BASE_SCALE,
        (g_heading_error_deg > 0.0f) ? pivot_pwm : -pivot_pwm, false);
    return false;
}

/**
 * @brief 路径点到达信号（LED + 蜂鸣器）
 *
 * 点亮LED(第1个)并驱动蜂鸣器发出持续H2024_POINT_SIGNAL_MS的声音，
 * 指示车辆到达了关键路径点(如B点、C点等)。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void car_signal_point(uint32_t now_ms)
{
    dt_led_on(&g_leds[0]);                               /* 点亮路径点LED */
    g_point_led_active = true;
    g_point_led_deadline_ms = now_ms + H2024_POINT_SIGNAL_MS;  /* 设置熄灭时间 */
    dt_buzzer_beep_async(&g_buzzer, H2024_POINT_SIGNAL_MS, now_ms);  /* 蜂鸣器响 */
}

/* ==================== 电机死区测试模式 ==================== */

/**
 * @brief 巡线模式启动回调
 *
 * 检查电池状态，就绪后初始化巡线运行参数。
 * 将初始状态设为TRACKING，等待传感器数据。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
static void line_mode_start(uint32_t now_ms, void *context)
{
    (void)context;
    /* 安全检查：电池必须可运行 */
    if (!battery_compensation_can_run(&g_battery_compensation))
    {
        g_battery_fault = true;
        car_enter_fault(now_ms);
        return;
    }
    car_prepare_run(now_ms);                     /* 重置控制状态 */
    g_line_state = LINE_STATE_TRACKING;          /* 初始为循迹状态 */
    g_line_pos = 99;                              /* 初始化位置为无效值 */
    g_line_pos_f = 99.0f;                         /* 浮点位置同步清零 */
    g_prev_pos = 0;                               /* 上次有效位置清0 */
    g_white_cnt = 0u;                             /* 全白计数清零 */
    g_find_start_ms = 0u;                         /* 寻线计时清零 */
    g_line_startup_boost_deadline_ms = now_ms +
        MOTOR_STARTUP_BOOST_DURATION_MS;
}

/**
 * @brief 巡线模式运行回调(周期性执行)
 *
 * 核心巡线逻辑：
 * 1. 检测巡线状态切换(TRACKING <-> FIND)
 * 2. 状态切换时重置PID并发送事件
 * 3. FIND模式持续原地旋转，直到中央传感器重新检测到黑线
 * 4. 执行速度控制(包含循迹转向)
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
static void line_mode_run(uint32_t now_ms, void *context)
{
    line_state_t next_state = g_line_state;

    (void)context;

    /* FIND 期间必须将线转到中央区域，不能在边缘刚见线时立即前进。 */
    if (g_line_state == LINE_STATE_FIND)
    {
        if (g_line_pos != 99 &&
            car_absf(g_line_pos_f) <= (float)CAR_LINE_FIND_CENTER_MAX_ERROR)
        {
            next_state = LINE_STATE_TRACKING;
        }
    }
    /* 连续全白超过3次且无有效位置 -> 进入FIND模式。 */
    else if (g_line_pos == 99 && g_white_cnt > 3u)
    {
        next_state = LINE_STATE_FIND;
    }

    /* 状态切换时执行初始化 */
    if (next_state != g_line_state)
    {
        g_line_state = next_state;
        car_send_line_state_event(now_ms);       /* 发送状态切换事件 */
        g_find_start_ms = (next_state == LINE_STATE_FIND) ? now_ms : 0u;  /* 记录寻线开始时间 */
        /* 重置所有PID控制器 */
        g_spd_out_l = 0.0f;
        g_spd_out_r = 0.0f;
        g_steer_boost_pwm = 0.0f;
        if (next_state == LINE_STATE_FIND)
        {
            g_line_startup_boost_deadline_ms = 0u;
        }
        g_last_line_error = 0;
        car_reset_line_controller();
        pid_reset(&g_spd_pid_l);
        pid_reset(&g_spd_pid_r);
    }

    /* 非零配置可恢复超时保护；当前为0，持续寻找直到线进入中央。 */
    if (CAR_LINE_FIND_TIMEOUT_MS != 0u && g_line_state == LINE_STATE_FIND &&
        (uint32_t)(now_ms - g_find_start_ms) >= CAR_LINE_FIND_TIMEOUT_MS)
    {
        g_line_lost_fault = true;                 /* 触发丢线故障 */
        car_enter_fault(now_ms);
        return;
    }

    run_speed_control(now_ms, true);              /* 执行速度控制(含循迹) */
}

/* ==================== 速度测试模式 ==================== */

/**
 * @brief 速度测试模式启动回调
 *
 * 检查电池状态后准备运行参数，与巡线模式类似但不设置巡线状态。
 * 此模式用于测试车辆在直线上的速度控制性能。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
static void speed_mode_start(uint32_t now_ms, void *context)
{
    (void)context;
    if (!battery_compensation_can_run(&g_battery_compensation))
    {
        g_battery_fault = true;
        car_enter_fault(now_ms);
        return;
    }
    car_prepare_run(now_ms);
}

/**
 * @brief 速度测试模式运行回调
 *
 * 执行速度控制(无巡线转向，直线行驶)。
 * 用于测试和调优速度PID参数。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
static void speed_mode_run(uint32_t now_ms, void *context)
{
    (void)context;
    run_speed_control(now_ms, false);  /* line_follow=false，直线行驶 */
}

static void car_motor_test_apply(int16_t left, int16_t right,
    uint32_t now_ms, void *context)
{
    (void)context;
    apply_motor_raw_cmd(left, right, now_ms);
}

static uint32_t car_motor_test_left_edges(void *context)
{
    (void)context;
    return dt_encoder_get_edges(&g_enc_l);
}

static uint32_t car_motor_test_right_edges(void *context)
{
    (void)context;
    return dt_encoder_get_edges(&g_enc_r);
}

static void car_motor_test_reset_left(void *context)
{
    (void)context;
    dt_encoder_reset_odometry(&g_enc_l);
}

static void car_motor_test_reset_right(void *context)
{
    (void)context;
    dt_encoder_reset_odometry(&g_enc_r);
}

static bool car_motor_test_prepare(uint32_t now_ms)
{
    if (!battery_compensation_can_run(&g_battery_compensation))
    {
        g_battery_fault = true;
        car_enter_fault(now_ms);
        return false;
    }

    car_stop_motors();
    car_prepare_run(now_ms);
    return true;
}

static void motor_deadzone_mode_start(uint32_t now_ms, void *context)
{
    (void)context;
    if (g_encoder_feedback_ready && car_motor_test_prepare(now_ms))
    {
        (void)motor_test_start_deadzone(&g_motor_test, now_ms);
    }
}

static void motor_deadzone_mode_run(uint32_t now_ms, void *context)
{
    bool was_done = motor_test_is_done(&g_motor_test);

    (void)context;
    motor_test_update(&g_motor_test, now_ms);
    if (!was_done && motor_test_is_done(&g_motor_test))
    {
        car_send_motor_deadzone_event(now_ms);
    }
}

static void raw_motor_left_start(uint32_t now_ms, void *context)
{
    (void)context;
    if (car_motor_test_prepare(now_ms))
    {
        (void)motor_test_start_raw_left(&g_motor_test, now_ms);
    }
}

static void raw_motor_right_start(uint32_t now_ms, void *context)
{
    (void)context;
    if (car_motor_test_prepare(now_ms))
    {
        (void)motor_test_start_raw_right(&g_motor_test, now_ms);
    }
}

static void boost_motor_left_start(uint32_t now_ms, void *context)
{
    (void)context;
    if (car_motor_test_prepare(now_ms))
    {
        (void)motor_test_start_boost_left(&g_motor_test, now_ms);
    }
}

static void boost_motor_right_start(uint32_t now_ms, void *context)
{
    (void)context;
    if (car_motor_test_prepare(now_ms))
    {
        (void)motor_test_start_boost_right(&g_motor_test, now_ms);
    }
}

static void raw_motor_run(uint32_t now_ms, void *context)
{
    (void)context;
    motor_test_update(&g_motor_test, now_ms);
    if (motor_test_is_done(&g_motor_test))
    {
        ec_mode_manager_stop(&g_mode_manager, now_ms);
    }
}

static void motor_test_mode_stop(uint32_t now_ms, void *context)
{
    (void)context;
    motor_test_stop(&g_motor_test, now_ms);
    car_stop_motors();
}

static void mode_stop(uint32_t now_ms, void *context)
{
    (void)now_ms;
    (void)context;
    car_stop_motors();
}

static void heading_test_mode_start(uint32_t now_ms, void *context)
{
    (void)context;
    car_stop_motors();
    g_heading_test_last_print_ms = now_ms - 500u;
}

static void tuning_mode_start(uint32_t now_ms, void *context)
{
    (void)now_ms;
    (void)context;
    car_stop_motors();
    g_parameter_menu.editing = false;
    g_parameter_menu.dirty = true;
}

static void tuning_mode_run(uint32_t now_ms, void *context)
{
    (void)now_ms;
    (void)context;
    car_stop_motors();
}

static void tuning_exit(void *context)
{
    ec_mode_manager_t *manager = (ec_mode_manager_t *)context;
    ec_mode_manager_stop(manager, ec_time_ms());
}

static void heading_test_mode_run(uint32_t now_ms, void *context)
{
    (void)context;
    car_stop_motors();
    if ((uint32_t)(now_ms - g_heading_test_last_print_ms) >= 500u)
    {
        g_heading_test_last_print_ms = now_ms;
        car_send_imu_frame(now_ms);
    }
}

/**
 * @brief 准备运行：检查电池/陀螺仪/巡线传感器，初始化控制状态
 *
 * 检查电池、陀螺仪、巡线传感器的状态，全部就绪后初始化运行参数。
 * 这是H2024任务启动时的安全检查门禁。
 *
 * @param task 任务ID(未使用)
 * @param now_ms 当前时间戳
 * @param context 未使用
 * @return true 准备就绪；false 准备失败(已触发故障)
 */
bool car_prepare(uint32_t now_ms)
{
    /* 安全检查：电池、陀螺仪、巡线传感器都必须正常 */
    if (!battery_compensation_can_run(&g_battery_compensation) ||
        !car_gyro_is_fresh(now_ms) || g_line_sensor_fault)
    {
        if (!battery_compensation_can_run(&g_battery_compensation))
            g_battery_fault = true;
        if (!car_gyro_is_fresh(now_ms))
            g_gyro_fault = true;
        car_enter_fault(now_ms);
        return false;
    }
    car_prepare_run(now_ms);
    /* 记录当前航向作为初始航向参考 */
    g_heading_target_deg = car_route_heading();
    g_heading_error_deg = 0.0f;
    g_heading_aligned_samples = 0u;
    return true;
}

/**
 * @brief 重置编码器里程计（临界区内清零）
 *
 * 在任务开始时重置编码器位置，确保距离测量从零开始。
 * 操作在临界区内执行，防止中断干扰。
 *
 * @param context 未使用
 */
void car_reset_odometry(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();                               /* 关中断保护 */
    dt_encoder_reset_odometry(&g_enc_l);           /* 左编码器清零 */
    dt_encoder_reset_odometry(&g_enc_r);           /* 右编码器清零 */
    if (primask == 0u)
    {
        __enable_irq();                            /* 恢复中断 */
    }
}

/**
 * @brief 停止车辆运动
 * @param now_ms 当前时间戳(未使用)
 * @param context 未使用
 */
void car_stop(uint32_t now_ms)
{
    (void)now_ms;
    car_stop_motors();
}

uint8_t car_get_line_bits(void)
{
    return g_line_bits;
}

void car_get_imu_accel(float *ax, float *ay, float *az)
{
    if (ax != NULL)
    {
        *ax = g_heading.ax;
    }
    if (ay != NULL)
    {
        *ay = g_heading.ay;
    }
    if (az != NULL)
    {
        *az = g_heading.az;
    }
}

float car_get_heading_deg(void)
{
    return g_heading.yaw_deg;
}

float car_get_wz_dps(void)
{
    return g_heading.wz_dps;
}

/**
 * @brief 获取车辆运行状态描述字符串
 *
 * 按优先级从高到低返回当前车辆的状态描述：
 * 故障 > 巡线状态(FIND) > H2024任务状态 > 正常运行模式
 * 用于OLED主菜单显示。
 *
     * 故障优先级：E-STOP > BAT > GYRO > LINE LOST > T8 > CMD
 *
 * @param manager 模式管理器实例
 * @return 状态描述字符串(静态内存，无需释放)
 *
 * @note 以下 extern 声明由 ec_app.c 链接，避免 line_car.c 直接 include 上层任务模块
 */
extern bool e2025_tasks_is_active(const ec_mode_manager_t *manager);
extern const char *e2025_tasks_active_status(const ec_mode_manager_t *manager);

static const char *car_status_name(const ec_mode_manager_t *manager)
{
    /* ---- 故障状态(优先级从高到低) ---- */
    if (g_emergency_latched)
    {
        return "E-STOP";          /* 急停锁定 */
    }
    if (g_battery_fault)
    {
        return "BAT FAULT";       /* 电池故障 */
    }
    if (g_gyro_fault)
    {
        return "GYRO FAULT";      /* 陀螺仪故障 */
    }
    if (g_line_lost_fault)
    {
        return "LINE LOST";       /* 巡线丢失 */
    }
    if (g_line_sensor_fault)
    {
        return "T8 FAULT";        /* T8传感器故障 */
    }
    if (g_motor_watchdog_fault)
    {
        return "CMD FAULT";       /* 电机命令看门狗故障 */
    }
    if (!g_encoder_feedback_ready)
    {
        return "ENC CONFIG";      /* 编码器配置无效；速度闭环已锁定 */
    }

    /* ---- 正常运行状态 ---- */
    if (manager != NULL && manager->state == EC_MODE_RUNNING &&
        manager->active == CAR_MODE_LINE_FOLLOW && g_line_state == LINE_STATE_FIND)
    {
        return "FIND";            /* 寻线模式 */
    }
    if (manager != NULL && e2025_tasks_is_active(manager))
    {
        return e2025_tasks_active_status(manager);
    }
    if (manager != NULL && h2024_tasks_is_active(manager))
    {
        return h2024_tasks_active_status(manager);
    }
    if (manager != NULL && manager->state == EC_MODE_RUNNING)
    {
        if (manager->active == CAR_MODE_LINE_FOLLOW)
            return g_tuning.speed_loop_enabled ? "FOLLOW CL" : "FOLLOW OL";  /* 巡线(闭环/开环) */
        if (manager->active == CAR_MODE_SPEED_TEST)
            return g_tuning.speed_loop_enabled ? "SPEED CL" : "SPEED OL";    /* 速度测试(闭环/开环) */
        return ec_mode_manager_active_name(manager);  /* 其他模式名称 */
    }
    return "STOP";                /* 停止状态 */
}

/**
 * @brief 在OLED指定行显示字符串(自动填充至21字符宽度)
 * @param oled OLED配置实例
 * @param row 行号(0~7)
 * @param text 要显示的字符串
 */
static void oled_show_line(dt_oled_config_t *oled, uint8_t row, const char *text)
{
    char line[22];

    snprintf(line, sizeof(line), "%-21s", text);
    dt_oled_show_string(oled, 0u, row, line);
}

/**
 * @brief 电机死区测试OLED显示函数
 *
 * 实时显示死区测试的状态：当前阶段、PWM值、编码器边沿数、
 * 已检测到的死区阈值和编码器AB相状态。
 */
static void motor_deadzone_render(void)
{
    const char *phase = "IDLE";
    uint32_t left_edges = dt_encoder_get_edges(&g_enc_l);
    uint32_t right_edges = dt_encoder_get_edges(&g_enc_r);
    char text[22];

    switch (g_motor_test.state)
    {
        case MOTOR_TEST_STATE_DEADZONE_LEFT: phase = "LEFT RAMP"; break;
        case MOTOR_TEST_STATE_DEADZONE_PAUSE: phase = "PAUSE"; break;
        case MOTOR_TEST_STATE_DEADZONE_RIGHT: phase = "RIGHT RAMP"; break;
        case MOTOR_TEST_STATE_DONE: phase = "DONE"; break;
        default: break;
    }

    oled_show_line(&g_oled, 0u, "   MOTOR DEADZONE");
    snprintf(text, sizeof(text), "PHASE:%s", phase);
    oled_show_line(&g_oled, 1u, text);
    snprintf(text, sizeof(text), "PWM:%4d STEP:%3d", (int)g_motor_test.current_pwm,
        MOTOR_DEADZONE_TEST_STEP_PWM);
    oled_show_line(&g_oled, 2u, text);
    snprintf(text, sizeof(text), "EDGE L:%4u R:%4u", (unsigned)left_edges,
        (unsigned)right_edges);
    oled_show_line(&g_oled, 3u, text);
    if (g_motor_test.left_threshold < 0)
        snprintf(text, sizeof(text), "LEFT : NO EDGE");
    else if (g_motor_test.left_threshold == 0)
        snprintf(text, sizeof(text), "LEFT : ----");
    else
        snprintf(text, sizeof(text), "LEFT : %4d", (int)g_motor_test.left_threshold);
    oled_show_line(&g_oled, 4u, text);
    if (g_motor_test.right_threshold < 0)
        snprintf(text, sizeof(text), "RIGHT: NO EDGE");
    else if (g_motor_test.right_threshold == 0)
        snprintf(text, sizeof(text), "RIGHT: ----");
    else
        snprintf(text, sizeof(text), "RIGHT: %4d", (int)g_motor_test.right_threshold);
    oled_show_line(&g_oled, 5u, text);
    {
        uint8_t left_ab = dt_encoder_get_ab_state(&g_enc_l);
        uint8_t right_ab = dt_encoder_get_ab_state(&g_enc_r);
        snprintf(text, sizeof(text), "AB L:%u%u R:%u%u",
            (unsigned)((left_ab >> 1u) & 1u), (unsigned)(left_ab & 1u),
            (unsigned)((right_ab >> 1u) & 1u), (unsigned)(right_ab & 1u));
    }
    oled_show_line(&g_oled, 6u, text);
    oled_show_line(&g_oled, 7u, "K1 E-STOP K3 EXIT");
}

static void heading_test_render(void)
{
    char text[22];

    oled_show_line(&g_oled, 0u, "HEADING SAFE TEST");
    snprintf(text, sizeof(text), "SRC:%s ST:%s",
        dt_heading_source_name(g_heading.source),
        dt_heading_status_name(g_heading.status));
    oled_show_line(&g_oled, 1u, text);
    snprintf(text, sizeof(text), "Y:%4d WZ:%4d",
        (int)g_heading.yaw_deg,
        (int)g_heading.wz_dps);
    oled_show_line(&g_oled, 2u, text);
    snprintf(text, sizeof(text), "N:%5u E:%5u",
        (unsigned)g_heading.sample_count,
        (unsigned)g_heading.read_error_count);
    oled_show_line(&g_oled, 3u, text);
    snprintf(text, sizeof(text), "BUS:%u CHK:%u",
        (unsigned)g_heading.bus_status,
        (unsigned)g_heading.checksum_error_count);
    oled_show_line(&g_oled, 4u, text);
    if (g_heading.source == DT_HEADING_SOURCE_M0)
    {
        snprintf(text, sizeof(text), "RAW:%6d %6d",
            (int)g_heading.yaw_raw, (int)g_heading.wz_raw);
        oled_show_line(&g_oled, 5u, text);
        snprintf(text, sizeof(text), "OVF:%u AGE:%u",
            (unsigned)g_heading.rx_overflow,
            (unsigned)(g_heading.ready ?
                ec_time_ms() - g_heading.last_update_ms : UINT32_MAX));
    }
    else
    {
        snprintf(text, sizeof(text), "A:%5d %5d %5d",
            (int)car_scale_float(g_heading.ax, 1000.0f),
            (int)car_scale_float(g_heading.ay, 1000.0f),
            (int)car_scale_float(g_heading.az, 1000.0f));
        oled_show_line(&g_oled, 5u, text);
        snprintf(text, sizeof(text), "ID:%02X GZ:%5d",
            (unsigned)g_heading.device_id,
            (int)car_scale_float(g_heading.gz, 100.0f));
    }
    oled_show_line(&g_oled, 6u, text);
    oled_show_line(&g_oled, 7u, "K3 EXIT MOTORS OFF");
}

static void line_car_menu_render(const ec_mode_manager_t *manager,
    uint32_t now_ms, void *context)
{
    dt_oled_config_t *oled = (dt_oled_config_t *)context;
    const char *run_state;
    char text[22];

    (void)now_ms;
    if (oled == NULL || manager == NULL)
    {
        return;
    }

    if (manager->state == EC_MODE_RUNNING) run_state = "RUN";
    else if (manager->state == EC_MODE_FAULT) run_state = "FAULT";
    else run_state = "STOP";

    snprintf(text, sizeof(text), "TASK:%s", ec_mode_manager_selected_name(manager));
    oled_show_line(oled, 0u, text);
    snprintf(text, sizeof(text), "%s CAR:%s", run_state, car_status_name(manager));
    oled_show_line(oled, 1u, text);
    if (h2024_tasks_is_active(manager))
        snprintf(text, sizeof(text), "Y:%4d T:%4d E:%4d",
            (int)car_route_heading(), (int)g_heading_target_deg,
            (int)g_heading_error_deg);
    else if (manager->state == EC_MODE_RUNNING &&
        (manager->active == CAR_MODE_MOTOR_LEFT_RAW ||
         manager->active == CAR_MODE_MOTOR_RIGHT_RAW ||
         manager->active == CAR_MODE_MOTOR_LEFT_BOOST ||
         manager->active == CAR_MODE_MOTOR_RIGHT_BOOST))
        snprintf(text, sizeof(text), "PWM L:%4d R:%4d", (int)g_cmd_l, (int)g_cmd_r);
    else
        snprintf(text, sizeof(text), "RPM L:%4d R:%4d", (int)g_rpm_l_s, (int)g_rpm_r_s);
    oled_show_line(oled, 2u, text);
    if (g_battery_fault)
        snprintf(text, sizeof(text), "BAT %s %5umV", battery_status_name(),
            (unsigned)g_bat_mv);
    else if (g_line_sensor_fault)
        snprintf(text, sizeof(text), "T8 I2C FAULT %u/%u",
            (unsigned)g_t8_recovery_count,
            (unsigned)CAR_T8_RECOVERY_SAMPLES);
    else if (g_line_lost_fault)
        snprintf(text, sizeof(text), "LINE LOST - REPLACE");
    else if (g_gyro_fault)
        snprintf(text, sizeof(text), "GYRO AGE:%5ums",
            (unsigned)(now_ms - g_heading.last_update_ms));
    else if (h2024_tasks_is_active(manager))
        snprintf(text, sizeof(text), "LINE:%u IN:%3u OUT:%3u",
            g_line_events.stable_on_line ? 1u : 0u,
            (unsigned)g_line_events.enter_count,
            (unsigned)g_line_events.exit_count);
    else if (manager->state == EC_MODE_RUNNING &&
        (manager->active == CAR_MODE_MOTOR_LEFT_BOOST ||
         manager->active == CAR_MODE_MOTOR_RIGHT_BOOST))
        snprintf(text, sizeof(text), "RPM L:%4d R:%4d",
            (int)g_rpm_l_s, (int)g_rpm_r_s);
    else
        snprintf(text, sizeof(text), "LINE:%02X POS:%3d", g_line_bits, g_line_pos);
    oled_show_line(oled, 3u, text);
    snprintf(text, sizeof(text), "BAT:%5u %s x%u.%02u",
        (unsigned)g_bat_mv,
        battery_status_name(),
        (unsigned)car_pwm_compensation_factor(),
        (unsigned)(car_pwm_compensation_factor() * 100.0f) % 100u);
    oled_show_line(oled, 4u, text);
    oled_show_line(oled, 5u,
        manager->state == EC_MODE_RUNNING ? "K1 EMERGENCY STOP" : "K1 PREV   K2 NEXT");
    oled_show_line(oled, 6u,
        manager->state == EC_MODE_FAULT ? "K3 ACK FAULT" : "K3 START / STOP");
    snprintf(text, sizeof(text), "RAW K:%u%u%u",
        gpio_get_level(KEY1_PIN) == GPIO_LOW ? 0u : 1u,
        gpio_get_level(KEY2_PIN) == GPIO_LOW ? 0u : 1u,
        gpio_get_level(KEY3_PIN) == GPIO_LOW ? 0u : 1u);
    oled_show_line(oled, 7u, text);
}

static void line_car_parameter_render(void)
{
    const ec_parameter_item_t *item = ec_parameter_menu_current(&g_parameter_menu);
    char text[22];
    char value[16];

    ec_parameter_menu_format_value(item, value, sizeof(value));
    oled_show_line(&g_oled, 0u, "       TUNING");
    snprintf(text, sizeof(text), "ITEM %u/%u",
        (unsigned)(g_parameter_menu.selected + 1u),
        (unsigned)g_parameter_menu.count);
    oled_show_line(&g_oled, 1u, text);
    oled_show_line(&g_oled, 2u, item != NULL ? item->name : "-");
    snprintf(text, sizeof(text), "VALUE: %s", value);
    oled_show_line(&g_oled, 3u, text);
    snprintf(text, sizeof(text), "Y:%4d W:%4d %s",
        (int)car_route_heading(), (int)car_route_wz(),
        g_parameter_menu.editing ? "EDIT" : "SELECT");
    oled_show_line(&g_oled, 4u, text);
    oled_show_line(&g_oled, 5u, "K1 - / PREV");
    oled_show_line(&g_oled, 6u, "K2 + / NEXT");
    oled_show_line(&g_oled, 7u, "K3 EDIT / ENTER");
    g_parameter_menu.dirty = false;
}

static void register_parameters(void)
{
    g_parameter_items[0] = (ec_parameter_item_t){"TARGET RPM", EC_PARAM_FLOAT,
        &g_tuning.target_rpm, 0.0f, CAR_TUNE_TARGET_RPM_MAX,
        CAR_TUNE_TARGET_RPM_STEP, NULL, NULL};
    g_parameter_items[1] = (ec_parameter_item_t){"BASE PWM", EC_PARAM_INT16,
        &g_tuning.base_pwm, 0.0f, (float)MOTOR_PWM_DUTY_MAX,
        CAR_TUNE_BASE_PWM_STEP, NULL, NULL};
    g_parameter_items[2] = (ec_parameter_item_t){"FF GAIN", EC_PARAM_FLOAT,
        &g_tuning.feedforward_gain, 0.0f, CAR_TUNE_FF_GAIN_MAX,
        CAR_TUNE_FF_GAIN_STEP, NULL, NULL};
    g_parameter_items[3] = (ec_parameter_item_t){"SPEED KP", EC_PARAM_FLOAT,
        &g_tuning.speed_kp, 0.0f, CAR_TUNE_SPEED_KP_MAX,
        CAR_TUNE_SPEED_KP_STEP, NULL, NULL};
    g_parameter_items[4] = (ec_parameter_item_t){"SPEED KI", EC_PARAM_FLOAT,
        &g_tuning.speed_ki, 0.0f, CAR_TUNE_SPEED_KI_MAX,
        CAR_TUNE_SPEED_KI_STEP, NULL, NULL};
    g_parameter_items[5] = (ec_parameter_item_t){"SPEED KD", EC_PARAM_FLOAT,
        &g_tuning.speed_kd, 0.0f, CAR_TUNE_SPEED_KD_MAX,
        CAR_TUNE_SPEED_KD_STEP, NULL, NULL};
    g_parameter_items[6] = (ec_parameter_item_t){"LINE KP", EC_PARAM_FLOAT,
        &g_tuning.line_kp, 0.0f, CAR_TUNE_LINE_KP_MAX,
        CAR_TUNE_LINE_KP_STEP, NULL, NULL};
    g_parameter_items[7] = (ec_parameter_item_t){"LINE KD", EC_PARAM_FLOAT,
        &g_tuning.line_kd, 0.0f, CAR_TUNE_LINE_KD_MAX,
        CAR_TUNE_LINE_KD_STEP, NULL, NULL};
    g_parameter_items[8] = (ec_parameter_item_t){"LINE SIGN", EC_PARAM_INT8,
        &g_tuning.line_steer_sign, -1.0f, 1.0f, 2.0f, NULL, NULL};
    g_parameter_items[9] = (ec_parameter_item_t){"LEFT GAIN", EC_PARAM_FLOAT,
        &g_tuning.left_gain, CAR_TUNE_WHEEL_GAIN_MIN,
        CAR_TUNE_WHEEL_GAIN_MAX, CAR_TUNE_WHEEL_GAIN_STEP, NULL, NULL};
    g_parameter_items[10] = (ec_parameter_item_t){"RIGHT GAIN", EC_PARAM_FLOAT,
        &g_tuning.right_gain, CAR_TUNE_WHEEL_GAIN_MIN,
        CAR_TUNE_WHEEL_GAIN_MAX, CAR_TUNE_WHEEL_GAIN_STEP, NULL, NULL};
    g_parameter_items[11] = (ec_parameter_item_t){"MOTOR DIR", EC_PARAM_INT8,
        &g_motor_dir, -1.0f, 1.0f, 2.0f, NULL, NULL};
    g_parameter_items[12] = (ec_parameter_item_t){"SPEED LOOP", EC_PARAM_BOOL,
        &g_tuning.speed_loop_enabled, 0.0f,
        (CAR_ENABLE_WHEEL_SPEED_PID != 0 && g_encoder_feedback_ready) ?
            1.0f : 0.0f, 1.0f, NULL, NULL};
    g_parameter_items[13] = (ec_parameter_item_t){"HEADING KP", EC_PARAM_FLOAT,
        &g_tuning.heading_kp, 0.0f, CAR_TUNE_HEADING_KP_MAX,
        CAR_TUNE_HEADING_KP_STEP, NULL, NULL};
    g_parameter_items[14] = (ec_parameter_item_t){"HEADING KD", EC_PARAM_FLOAT,
        &g_tuning.heading_kd, 0.0f, CAR_TUNE_HEADING_KD_MAX,
        CAR_TUNE_HEADING_KD_STEP, NULL, NULL};
    g_parameter_items[15] = (ec_parameter_item_t){"HEADING MAX", EC_PARAM_FLOAT,
        &g_tuning.heading_max_steer, 0.0f, CAR_TUNE_HEADING_MAX_PWM,
        CAR_TUNE_HEADING_MAX_STEP, NULL, NULL};
    g_parameter_items[16] = (ec_parameter_item_t){"HEADING SIGN", EC_PARAM_INT8,
        &g_tuning.heading_steer_sign, -1.0f, 1.0f, 2.0f, NULL, NULL};
#if EC_ENABLE_VOFA
    g_parameter_items[17] = (ec_parameter_item_t){"VOFA", EC_PARAM_BOOL,
        &g_vofa_enabled, 0.0f, 1.0f, 1.0f, NULL, NULL};
    g_parameter_items[18] = (ec_parameter_item_t){"EXIT", EC_PARAM_ACTION,
        NULL, 0.0f, 0.0f, 0.0f, tuning_exit, &g_mode_manager};
#else
    g_parameter_items[17] = (ec_parameter_item_t){"EXIT", EC_PARAM_ACTION,
        NULL, 0.0f, 0.0f, 0.0f, tuning_exit, &g_mode_manager};
#endif
    ec_parameter_menu_init(&g_parameter_menu, g_parameter_items,
        (uint8_t)(sizeof(g_parameter_items) / sizeof(g_parameter_items[0])));
}

static void register_car_modes(void)
{
    static const ec_mode_t modes[] = {
        {"LINE FOLLOW", NULL, line_mode_start, line_mode_run, mode_stop, NULL},
        {"SPEED TEST", NULL, speed_mode_start, speed_mode_run, mode_stop, NULL},
        {"MOTOR L RAW", NULL, raw_motor_left_start,
            raw_motor_run, motor_test_mode_stop, NULL},
        {"MOTOR R RAW", NULL, raw_motor_right_start,
            raw_motor_run, motor_test_mode_stop, NULL},
        {"MOTOR L START", NULL, boost_motor_left_start,
            raw_motor_run, motor_test_mode_stop, NULL},
        {"MOTOR R START", NULL, boost_motor_right_start,
            raw_motor_run, motor_test_mode_stop, NULL},
        {"MOTOR DEADZONE", NULL, motor_deadzone_mode_start,
            motor_deadzone_mode_run, motor_test_mode_stop, NULL},
        {"TUNING", NULL, tuning_mode_start, tuning_mode_run, mode_stop, NULL},
        {"GYRO TEST", NULL, heading_test_mode_start,
            heading_test_mode_run, mode_stop, NULL}
    };
    uint32_t i;

    ec_mode_manager_init(&g_mode_manager);
    for (i = 0u; i < (uint32_t)(sizeof(modes) / sizeof(modes[0])); i++)
    {
        if (!ec_mode_manager_add(&g_mode_manager, &modes[i]))
        {
            g_mode_manager.state = EC_MODE_FAULT;
            car_stop_motors();
            break;
        }
    }
}

ec_mode_manager_t *line_car_get_mode_manager(void)
{
    return &g_mode_manager;
}

static bool t8_write(uint8_t addr, const uint8_t *data, size_t len, void *context)
{
    soft_iic_info_struct *iic = (soft_iic_info_struct *)context;
    iic->addr = addr;
    soft_iic_write_8bit_array(iic, data, len);
    return soft_iic_get_last_error(iic) == SOFT_IIC_STATUS_OK;
}

static bool t8_read(uint8_t addr, uint8_t *data, size_t len,
    uint32_t timeout_ms, void *context)
{
    soft_iic_info_struct *iic = (soft_iic_info_struct *)context;
    (void)timeout_ms;
    iic->addr = addr;
    soft_iic_read_8bit_array(iic, data, len);
    return soft_iic_get_last_error(iic) == SOFT_IIC_STATUS_OK;
}

/**
 * @brief 智能车系统初始化
 *
 * 初始化所有硬件和软件模块，包括：
 *
 * 硬件层：
 * - 调试串口(UART)及发送缓冲区
 * - 左右电机(PWM驱动)
 * - 左右编码器(AB相正交解码)
 * - 板载LED(3个)
 * - 蜂鸣器
 * - M0或MPU6050航向传感器
 * - T8灰度传感器(I2C)
 * - OLED显示屏(I2C)
 * - 电池ADC
 * - 按键(带紧急停止ISR)
 *
 * 软件层：
 * - 调优参数加载默认值
 * - 电池补偿模块初始化
 * - 所有PID控制器初始化及参数配置
 * - 巡线事件检测器初始化
 * - 中断优先级配置
 * - 模式管理器和菜单系统注册
 * - 调参菜单项注册
 * - PIT定时器看门狗钩子注册
 *
 * 在系统启动时(caller的main函数)只调用一次。
 */
void line_car_init(void)
{
    static const battery_compensation_config_t battery_config = {
        BAT_REFERENCE_MV,
        BAT_VALID_MIN_MV,
        BAT_VALID_MAX_MV,
        BAT_UNDERVOLTAGE_MV,
        BAT_RECOVERY_MV,
        BAT_COMP_MIN_FACTOR,
        BAT_COMP_MAX_FACTOR,
        BAT_FILTER_ALPHA,
        BAT_FAULT_SAMPLES,
        BAT_RECOVERY_SAMPLES
    };
    static const motor_test_config_t motor_test_config = {
        .apply = car_motor_test_apply,
        .left_edges = car_motor_test_left_edges,
        .right_edges = car_motor_test_right_edges,
        .reset_left = car_motor_test_reset_left,
        .reset_right = car_motor_test_reset_right,
        .context = NULL,
        .raw_left_pwm = MOTOR_STARTUP_BOOST_PWM,
        .raw_right_pwm = MOTOR_STARTUP_BOOST_PWM,
        .raw_duration_ms = MOTOR_RAW_TEST_DURATION_MS,
        .startup_boost_pwm = MOTOR_STARTUP_BOOST_PWM,
        .startup_boost_duration_ms = MOTOR_STARTUP_BOOST_DURATION_MS,
        .startup_low_pwm = MOTOR_STARTUP_LOW_PWM,
        .startup_low_duration_ms = MOTOR_STARTUP_LOW_DURATION_MS,
        .deadzone_start_pwm = MOTOR_DEADZONE_TEST_START_PWM,
        .deadzone_step_pwm = MOTOR_DEADZONE_TEST_STEP_PWM,
        .deadzone_max_pwm = MOTOR_PWM_DUTY_MAX,
        .deadzone_step_ms = MOTOR_DEADZONE_TEST_STEP_MS,
        .deadzone_pause_ms = MOTOR_DEADZONE_TEST_PAUSE_MS,
        .deadzone_edge_count = MOTOR_DEADZONE_TEST_EDGE_COUNT
    };
    ec_keys_config_t keys = {
        KEY1_PIN,
        KEY2_PIN,
        KEY3_PIN,
        50u,
        CAR_KEY_STARTUP_LOCK_MS,
        car_emergency_isr_stop,
        NULL
    };
    T8I2cTransport t8_transport = {t8_write, t8_read, &g_t8_iic};

    serial_tx_buffer_init(&g_debug_tx_buffer, g_debug_tx_storage,
        sizeof(g_debug_tx_storage));
    uart_set_callback(DEBUG_UART_INDEX, car_debug_uart_callback, NULL);
    uart_set_interrupt_config(DEBUG_UART_INDEX,
        UART_INTERRUPT_CONFIG_TX_DISABLE);

    car_tuning_defaults(&g_tuning);
    battery_compensation_init(&g_battery_compensation, &battery_config);

    g_motor_l.in1_pin = MOTOR_L_IN1;
    g_motor_l.in2_pin = MOTOR_L_IN2;
    g_motor_l.pwm_freq = MOTOR_PWM_FREQ;
    dt_motor_init(&g_motor_l);

    g_motor_r.in1_pin = MOTOR_R_IN1;
    g_motor_r.in2_pin = MOTOR_R_IN2;
    g_motor_r.pwm_freq = MOTOR_PWM_FREQ;
    dt_motor_init(&g_motor_r);
    ec_time_set_tick_hook(car_motor_watchdog_tick, NULL);

    pid_init(&g_spd_pid_l);
    pid_init(&g_spd_pid_r);
    pid_init(&g_line_pid);
    pid_set_gain(&g_spd_pid_l, g_tuning.speed_kp, g_tuning.speed_ki, g_tuning.speed_kd);
    pid_set_gain(&g_spd_pid_r, g_tuning.speed_kp, g_tuning.speed_ki, g_tuning.speed_kd);
    pid_set_limits(&g_spd_pid_l, -CAR_SPEED_PID_INITIAL_OUTPUT_MAX,
        CAR_SPEED_PID_INITIAL_OUTPUT_MAX, -CAR_SPEED_PID_INTEGRAL_MAX,
        CAR_SPEED_PID_INTEGRAL_MAX);
    pid_set_limits(&g_spd_pid_r, -CAR_SPEED_PID_INITIAL_OUTPUT_MAX,
        CAR_SPEED_PID_INITIAL_OUTPUT_MAX, -CAR_SPEED_PID_INTEGRAL_MAX,
        CAR_SPEED_PID_INTEGRAL_MAX);
    pid_set_limits(&g_line_pid, -CAR_LINE_STEER_MAX_PWM,
        CAR_LINE_STEER_MAX_PWM, 0.0f, 0.0f);
    pid_set_derivative_lpf(&g_line_pid, CAR_LINE_DERIVATIVE_LPF);
    pid_set_derivative_lpf(&g_spd_pid_l, CAR_SPEED_DERIVATIVE_LPF);
    pid_set_derivative_lpf(&g_spd_pid_r, CAR_SPEED_DERIVATIVE_LPF);

    g_enc_l.a_pin = ENCODER1_A_PIN;
    g_enc_l.b_pin = ENCODER1_B_PIN;
    g_enc_l.counts_per_rev = ENCODER_CPR;
    g_enc_l.wheel_circumference_mm = WHEEL_DIAMETER_MM * 3.1415926f;
    g_enc_l.direction_sign = ENCODER1_DIRECTION_SIGN;
    g_enc_l.rpm_lpf_alpha = ENCODER_RPM_LPF_ALPHA;
    g_enc_l.quadrature_enabled = ENCODER_USE_QUADRATURE != 0;
    g_enc_l.hardware_quadrature = ENCODER1_USE_HARDWARE_QUADRATURE != 0;
    g_enc_l.timer_index = ENCODER1_TIMER;
    g_enc_l.channel_a = ENCODER1_CHANNEL_A;
    g_enc_l.channel_b = ENCODER1_CHANNEL_B;
    g_encoder_feedback_ready = dt_encoder_init(&g_enc_l);

    g_enc_r.a_pin = ENCODER2_A_PIN;
    g_enc_r.b_pin = ENCODER2_B_PIN;
    g_enc_r.counts_per_rev = ENCODER_CPR;
    g_enc_r.wheel_circumference_mm = WHEEL_DIAMETER_MM * 3.1415926f;
    g_enc_r.direction_sign = ENCODER2_DIRECTION_SIGN;
    g_enc_r.rpm_lpf_alpha = ENCODER_RPM_LPF_ALPHA;
    g_enc_r.quadrature_enabled = ENCODER_USE_QUADRATURE != 0;
    g_enc_r.hardware_quadrature = ENCODER2_USE_HARDWARE_QUADRATURE != 0;
    g_enc_r.timer_index = ENCODER2_TIMER;
    g_enc_r.channel_a = ENCODER2_CHANNEL_A;
    g_enc_r.channel_b = ENCODER2_CHANNEL_B;
    if (!dt_encoder_init(&g_enc_r))
    {
        g_encoder_feedback_ready = false;
    }
    if (CAR_ENABLE_WHEEL_SPEED_PID == 0 || !g_encoder_feedback_ready)
    {
        g_tuning.speed_loop_enabled = false;
    }
    if (!motor_test_init(&g_motor_test, &motor_test_config))
    {
        g_motor_watchdog_fault = true;
    }

    g_buzzer.pin = BUZZER_PIN;
    dt_buzzer_init(&g_buzzer);
    g_leds[0].pin = PIN_LED1;
    g_leds[1].pin = PIN_LED2;
    g_leds[2].pin = PIN_LED3;
    for (uint8_t i = 0u; i < 3u; i++)
    {
        g_leds[i].on_level = GPIO_HIGH;
        g_leds[i].off_level = GPIO_LOW;
        dt_led_init(&g_leds[i]);
    }
    gpio_init(RELAY_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    (void)dt_heading_init(&g_heading);

    soft_iic_init(&g_t8_iic, T8_DEFAULT_I2C_ADDRESS, 100, TRACE_SCL, TRACE_SDA);
    t8_i2c_init(&g_t8, &t8_transport, T8_DEFAULT_I2C_ADDRESS);

    soft_iic_init(&g_oled.iic, DT_OLED_DEFAULT_ADDR,
        CAR_OLED_SOFT_I2C_DELAY,
        OLED_SCL, OLED_SDA);
    dt_oled_init(&g_oled);
    dt_oled_clear(&g_oled);

    adc_init(BAT_ADC, BAT_ADC_RESOLUTION);
    line_event_detector_init(&g_line_events,
        CAR_LINE_EVENT_DEBOUNCE_SAMPLES);
    ec_keys_init(&keys);
    interrupt_set_priority(TIMG0_INT_IRQn, 0u);
    interrupt_set_priority(GPIOA_INT_IRQn, 1u);
    interrupt_set_priority(DEBUG_UART_PRIORITY, 2u);
    register_car_modes();
    register_parameters();
    ec_menu_init(&g_menu, &g_mode_manager, line_car_menu_render,
        &g_oled, CAR_MENU_PERIOD_MS);
    g_last_sensor = ec_time_ms();
    g_last_line_sample_ms = g_last_sensor;
    g_last_encoder_sample_ms = g_last_sensor;
    g_last_diagnostic_log_ms = g_last_sensor;
    car_stop_motors();
    car_send_init_event(g_last_sensor);
    car_send_heading_init_event(g_last_sensor);
    dt_buzzer_play_sequence(&g_buzzer, g_startup_tone,
        (uint8_t)(sizeof(g_startup_tone) / sizeof(g_startup_tone[0])),
        g_last_sensor);
}

/**
 * @brief 输入采集任务(由调度器周期性调用)
 *
 * 功能：
 * 1. 采集编码器AB相输入信号
 * 2. 检测编码器活动时闪烁LED(调试辅助)
 * 3. 执行电机看门狗检查
 * 4. 处理紧急停止请求(按键中断待处理)
 * 5. 运行状态下自动检测故障并进入故障模式
 * 6. 弹出按键事件，分发到各模式处理器
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void line_car_input_task(uint32_t now_ms, void *context)
{
    uint8_t key;

    (void)context;
    dt_encoder_sample_inputs(&g_enc_l);
    dt_encoder_sample_inputs(&g_enc_r);
    {
        static uint32_t last_raw_edges;
        uint32_t now_edges = dt_encoder_get_edges(&g_enc_l)
                           + dt_encoder_get_edges(&g_enc_r);
        if (now_edges != last_raw_edges)
        {
            last_raw_edges = now_edges;
            dt_led_toggle(&g_leds[1]);
        }
    }
    car_service_motor_watchdog(now_ms);
    if (ec_keys_emergency_pending() && g_mode_manager.state == EC_MODE_RUNNING)
    {
        line_car_emergency_stop();
    }
    if ((g_line_sensor_fault || g_line_lost_fault || g_battery_fault || g_gyro_fault ||
         g_motor_watchdog_fault || g_emergency_latched) &&
        g_mode_manager.state == EC_MODE_RUNNING)
    {
        car_enter_fault(now_ms);
    }

    while (ec_keys_pop(&key))
    {
        car_send_key_event(now_ms, key);
        if (key >= (uint8_t)EC_MENU_KEY_PREVIOUS
            && key <= (uint8_t)EC_MENU_KEY_CONFIRM)
        {
            if (g_mode_manager.state == EC_MODE_RUNNING &&
                g_mode_manager.active == CAR_MODE_TUNING)
            {
                ec_parameter_menu_handle_key(&g_parameter_menu, (ec_menu_key_t)key);
            }
            else if (g_mode_manager.state == EC_MODE_RUNNING &&
                key == (uint8_t)EC_MENU_KEY_PREVIOUS)
            {
                line_car_emergency_stop();
            }
            else if (g_mode_manager.state == EC_MODE_FAULT)
            {
                bool battery_ready = !g_battery_fault;

                if (key == (uint8_t)EC_MENU_KEY_CONFIRM && g_battery_fault)
                {
                    battery_ready = battery_compensation_can_run(&g_battery_compensation) ||
                        battery_compensation_acknowledge(&g_battery_compensation);
                }
                if (key == (uint8_t)EC_MENU_KEY_CONFIRM && battery_ready &&
                    gpio_get_level(KEY1_PIN) != GPIO_LOW &&
                    (!g_line_sensor_fault ||
                     g_t8_recovery_count >= CAR_T8_RECOVERY_SAMPLES) &&
                    (!g_line_lost_fault || g_line_pos != 99) &&
                    (!g_gyro_fault || car_gyro_is_fresh(now_ms)))
                {
                    g_line_sensor_fault = false;
                    g_line_lost_fault = false;
                    g_battery_fault = false;
                    g_gyro_fault = false;
                    g_motor_watchdog_fault = false;
                    g_emergency_latched = false;
                    g_t8_failure_count = 0u;
                    g_t8_recovery_count = 0u;
                    car_stop_motors();
                    g_mode_manager.state = EC_MODE_STOPPED;
                    g_menu.dirty = true;
                }
            }
            else
                ec_menu_handle_key(&g_menu, (ec_menu_key_t)key, now_ms);
        }
    }
}

/**
 * @brief 控制任务(主控制环路)
 *
 * 在每个调度周期执行：
 * 1. 检查电机看门狗(主循环层)
 * 2. 如果有远程控制命令，执行它
 * 3. 否则运行模式管理器(执行当前模式的run回调)
 *
 * 这是车辆运动控制的执行驱动，所有模式的具体控制逻辑
 * 都在模式管理器的run回调中实现。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void line_car_control_task(uint32_t now_ms, void *context)
{
    (void)context;
    car_service_motor_watchdog(now_ms);             /* 检查电机看门狗 */
    if (car_service_remote_cmd(now_ms))              /* 有远程命令先执行 */
    {
        return;
    }
    ec_mode_manager_run(&g_mode_manager, now_ms);   /* 运行当前模式 */
}

/**
 * @brief 菜单/显示更新任务
 *
 * 根据当前运行模式选择对应的OLED渲染函数。
 * 特殊模式(LED测试、调参、死区测试、MPU测试)有自己的全屏渲染，
 * 其他模式使用ec_menu的标准菜单渲染。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void line_car_menu_task(uint32_t now_ms, void *context)
{
    (void)context;
    if (g_mode_manager.state == EC_MODE_RUNNING &&
        g_mode_manager.active == CAR_MODE_TUNING)
        line_car_parameter_render();
    else if (g_mode_manager.state == EC_MODE_RUNNING &&
        g_mode_manager.active == CAR_MODE_MOTOR_DEADZONE)
        motor_deadzone_render();
    else if (g_mode_manager.state == EC_MODE_RUNNING &&
        g_mode_manager.active == CAR_MODE_HEADING_TEST)
        heading_test_render();
    else
        ec_menu_update(&g_menu, now_ms);
}

/**
 * @brief 蜂鸣器服务任务
 *
 * 更新蜂鸣器异步鸣叫序列(非阻塞播放)。
 * 同时管理路径点指示LED的定时熄灭：
 * 当到达点亮截止时间时自动关闭LED。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void line_car_buzzer_task(uint32_t now_ms, void *context)
{
    (void)context;
    dt_buzzer_service(&g_buzzer, now_ms);
    if (g_point_led_active &&
        (int32_t)(now_ms - g_point_led_deadline_ms) >= 0)
    {
        dt_led_off(&g_leds[0]);
        g_point_led_active = false;
    }
}

/**
 * @brief OLED刷新任务
 *
 * 增量刷新OLED显示：只刷新标记为"脏"(dirty)的区域。
 * 这种设计减少了I2C通信量，提高整体系统性能，
 * 因为OLED通过I2C接口通信，频繁刷新会占用总线带宽。
 *
 * @param now_ms 当前时间戳(未使用)
 * @param context 未使用
 */
void line_car_oled_task(uint32_t now_ms, void *context)
{
    (void)now_ms;
    (void)context;
    dt_oled_refresh_one_dirty(&g_oled);  /* 增量刷新一个脏区域 */
}

#if EC_ENABLE_VOFA
static bool line_car_vofa_write(const uint8_t *data, size_t length, void *context)
{
    (void)context;
    return car_debug_tx_write(data, length);
}
#endif

/**
 * @brief 遥测数据发送任务
 *
 * 通过VOFA协议(一种浮点数据流协议)向上位机发送实时数据。
 * VOFA支持在PC端进行可视化监控和数据记录。
 *
 * 发送的数据包含：目标RPM、实际RPM、PWM命令、电池电压、
 * 巡线位置、补偿因子、航向信息、编码器数据等32个通道。
 *
 * 只发送非航向测试模式下的数据，避免与诊断帧冲突。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void line_car_telemetry_task(uint32_t now_ms, void *context)
{
    (void)context;
#if EC_ENABLE_VOFA
    {
        static const VofaTransport vofa_transport = {line_car_vofa_write, NULL};
        static uint32_t last_vofa = 0u;
        if (g_vofa_enabled &&
            !(g_mode_manager.state == EC_MODE_RUNNING &&
              g_mode_manager.active == CAR_MODE_HEADING_TEST) &&
            (uint32_t)(now_ms - last_vofa) >= 50u)
        {
            float vofa_data[32];
            last_vofa = now_ms;
            vofa_data[0] = g_tuning.target_rpm;
            vofa_data[1] = (car_absf(g_rpm_l) + car_absf(g_rpm_r)) * 0.5f;
            vofa_data[2] = car_absf(g_rpm_l);
            vofa_data[3] = car_absf(g_rpm_r);
            vofa_data[4] = (float)g_cmd_l;
            vofa_data[5] = (float)g_cmd_r;
            vofa_data[6] = g_spd_out_l;
            vofa_data[7] = g_spd_out_r;
            vofa_data[8] = (float)car_drive_pwm_for_rpm(g_tuning.target_rpm);
            vofa_data[9] = (float)g_mode_manager.active;
            vofa_data[10] = (float)g_bat_mv;
            vofa_data[11] = g_line_pos_f;
            vofa_data[12] = car_pwm_compensation_factor();
            vofa_data[13] = g_tuning.speed_loop_enabled ? 1.0f : 0.0f;
            vofa_data[14] = (float)g_battery_compensation.status;
            vofa_data[15] = dt_encoder_get_distance_mm(&g_enc_l);
            vofa_data[16] = dt_encoder_get_distance_mm(&g_enc_r);
            vofa_data[17] = (float)dt_encoder_get_invalid_transitions(&g_enc_l);
            vofa_data[18] = (float)dt_encoder_get_invalid_transitions(&g_enc_r);
            vofa_data[19] = car_route_heading();
            vofa_data[20] = car_route_wz();
            vofa_data[21] = g_heading_target_deg;
            vofa_data[22] = g_heading_error_deg;
            vofa_data[23] = g_heading.ready ?
                (float)(uint32_t)(now_ms - g_heading.last_update_ms) : -1.0f;
            vofa_data[24] = g_line_events.stable_on_line ? 1.0f : 0.0f;
            vofa_data[25] = (float)g_line_events.enter_count;
            vofa_data[26] = (float)g_line_events.exit_count;
            vofa_data[27] = (g_mode_manager.state == EC_MODE_FAULT) ?
                (float)g_fault_snapshot.reason : 0.0f;
            vofa_data[28] = (float)g_t8_last_status;
            vofa_data[29] = (float)soft_iic_get_last_error(&g_t8_iic);
            vofa_data[30] = (float)(uint32_t)(now_ms - g_last_line_sample_ms);
            vofa_data[31] = (float)g_control_dt_ms;
            (void)vofa_send(&vofa_transport, vofa_data, 32u);
        }
    }
#else
    (void)now_ms;
#endif
}

/**
 * @brief 调试数据发送任务
 *
 * 当VOFA未启用时，通过二进制协议向上位机发送各种调试数据帧。
 * 发送策略：
 * - 运行中(RUNNING)：发送run_frame(核心运行数据)
 * - 非运行状态：发送task_stats + sensor_frame + imu_frame + timing_frame
 *
 * 还负责：
 * - 发送挂起的故障快照帧(首次触发)
 * - 统计最差调度延迟和最差运行时间的任务
 * - 记录ISR诊断数据
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void line_car_debug_task(uint32_t now_ms, void *context)
{
    const ec_scheduler_t *scheduler;
    const ec_task_t *worst_lateness = NULL;
    const ec_task_t *worst_runtime = NULL;
    isr_diagnostics_t isr_diagnostics;
    uint32_t total_missed = 0u;
    uint32_t total_overruns = 0u;
    uint8_t worst_lateness_index = 0xFFu;
    uint8_t worst_runtime_index = 0xFFu;
    uint8_t i;

    (void)context;
#if EC_ENABLE_VOFA
    if (g_vofa_enabled)
    {
        return;
    }
#endif
    scheduler = ec_app_get_scheduler();
    if (scheduler == NULL) return;

    if (g_fault_snapshot_pending)
    {
        car_send_fault_frame();
        g_fault_snapshot_pending = false;
    }

    if ((uint32_t)(now_ms - g_last_diagnostic_log_ms) <
        ((g_mode_manager.state == EC_MODE_RUNNING) ? CAR_RUN_LOG_PERIOD_MS :
         CAR_DIAGNOSTIC_LOG_PERIOD_MS))
    {
        return;
    }
    g_last_diagnostic_log_ms = now_ms;

    for (i = 0u; i < scheduler->count; i++)
    {
        const ec_task_t *task = &scheduler->tasks[i];
        total_missed += task->missed_deadlines;
        total_overruns += task->overrun_count;
        if (worst_lateness == NULL || task->max_start_lateness_ms >
            worst_lateness->max_start_lateness_ms)
        {
            worst_lateness = task;
            worst_lateness_index = i;
        }
        if (worst_runtime == NULL || task->max_runtime_ms >
            worst_runtime->max_runtime_ms)
        {
            worst_runtime = task;
            worst_runtime_index = i;
        }
    }
    isr_get_diagnostics(&isr_diagnostics);

    if (g_mode_manager.state == EC_MODE_RUNNING)
    {
        car_send_run_frame(now_ms, total_missed, total_overruns,
            &isr_diagnostics);
        return;
    }

    car_send_task_stats(now_ms, scheduler);
    car_send_sensor_frame(now_ms);
    car_send_imu_frame(now_ms);
    car_send_timing_frame(now_ms, total_missed, total_overruns,
        worst_lateness_index, worst_lateness != NULL ?
            worst_lateness->max_start_lateness_ms : 0u,
        worst_runtime_index, worst_runtime != NULL ?
            worst_runtime->max_runtime_ms : 0u, &isr_diagnostics);
}

/**
 * @brief 调参/遥控接收任务
 *
 * 从调试串口读取数据并解析两种协议：
 *
 * 1. 远程遥控命令(REMOTE_CMD)：
 *    帧头0xCD + 魔法字0x4D + 左轮PWM(2字节) + 右轮PWM(2字节) + 校验和
 *    用于上位机直接控制车辆运动。
 *
 * 2. 调参命令(CC协议)：
 *    帧头0xCC + 参数ID(1字节) + 参数值(4字节float)
 *    用于在线调整PID参数和调优变量。
 *
 * 两种协议通过帧头区分(0xCC vs 0xCD)。
 *
 * @param now_ms 当前时间戳
 * @param context 未使用
 */
void line_car_tune_task(uint32_t now_ms, void *context)
{
    static uint8_t rx[7];
    static uint8_t rx_pos;
    static uint8_t rx_len;
    uint8_t byte;

    (void)context;

    while (debug_read_ring_buffer(&byte, 1u) == 1u)
    {
        if (rx_pos == 0u)
        {
            if (byte == 0xCCu)
            {
                rx_len = 6u;
            }
            else if (byte == REMOTE_CMD_MARKER)
            {
                rx_len = 7u;
            }
            else
            {
                continue;
            }
        }
        if (rx_pos >= sizeof(rx))
        {
            rx_pos = 0u;
            rx_len = 0u;
            continue;
        }
        rx[rx_pos++] = byte;
        if (rx_pos >= rx_len)
        {
            uint8_t marker = rx[0];

            rx_pos = 0u;
            rx_len = 0u;

            if (marker == REMOTE_CMD_MARKER)
            {
                uint8_t response[5];
                uint8_t checksum = 0u;
                uint8_t status = 0u;
                int16_t left = 0;
                int16_t right = 0;
                uint8_t i;

                for (i = 0u; i < 6u; i++)
                {
                    checksum = (uint8_t)(checksum + rx[i]);
                }
                if (rx[1] != REMOTE_CMD_MAGIC || checksum != rx[6])
                {
                    status = 3u;
                }
                else
                {
                    left = (int16_t)((uint16_t)rx[2] |
                        ((uint16_t)rx[3] << 8u));
                    right = (int16_t)((uint16_t)rx[4] |
                        ((uint16_t)rx[5] << 8u));
                    left = car_clamp_motor_cmd((float)left);
                    right = car_clamp_motor_cmd((float)right);

                    if (left == 0 && right == 0)
                    {
                        g_remote_cmd_active = false;
                        g_remote_cmd_l = 0;
                        g_remote_cmd_r = 0;
                        car_stop_motors();
                    }
                    else if (g_mode_manager.state != EC_MODE_STOPPED)
                    {
                        status = 1u;
                    }
                    else if (g_battery_fault || g_motor_watchdog_fault ||
                        g_emergency_latched ||
                        !battery_compensation_can_run(&g_battery_compensation))
                    {
                        status = 2u;
                    }
                    else
                    {
                        bool changed = !g_remote_cmd_active ||
                            left != g_remote_cmd_l || right != g_remote_cmd_r;

                        g_remote_cmd_l = left;
                        g_remote_cmd_r = right;
                        g_remote_cmd_last_rx_ms = now_ms;
                        g_remote_cmd_active = true;
                        if (!changed)
                        {
                            continue;
                        }
                    }
                }

                response[0] = status;
                response[1] = (uint8_t)((uint16_t)left & 0xFFu);
                response[2] = (uint8_t)((uint16_t)left >> 8u);
                response[3] = (uint8_t)((uint16_t)right & 0xFFu);
                response[4] = (uint8_t)((uint16_t)right >> 8u);
                tune_send(CAR_FRAME_CMD_RESPONSE, response, sizeof(response));
                continue;
            }

            uint8_t  pid = rx[1];
            uint32_t raw = (uint32_t)rx[2] | ((uint32_t)rx[3] << 8u)
                         | ((uint32_t)rx[4] << 16u) | ((uint32_t)rx[5] << 24u);
            float val;
            memcpy(&val, &raw, sizeof(val));

            if (!car_apply_tuning_parameter(pid, val, &val)) continue;

            uint8_t resp[6];
            resp[0] = 0xCCu;
            resp[1] = pid;
            memcpy(&resp[2], &val, sizeof(val));
            tune_send(CAR_FRAME_TUNE_RESPONSE, resp, 6u);
        }
    }
}
