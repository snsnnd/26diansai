/*********************************************************************************************************************
 * config.h — 智能车系统配置定义
 *
 * 本文件定义了智能车硬件配置常数和控制参数，包含：
 *   - 应用配置文件选择（硬件测试 / 循迹小车 / 空工程）
 *   - 外设引脚复用配置（PWM / UART / I2C / ADC）
 *   - 电机控制参数（PWM 频率、死区补偿、PID 预设值）
 *   - 编码器参数（分辨率、方向、车轮直径）
 *   - 传感器配置（T8 灰度、MPU6050 陀螺仪、OLED）
 *   - 电池检测参数
 *   - 调试与通信配置（VOFA、蓝牙、串口缓冲区）
 ********************************************************************************************************************/

#ifndef _CONFIG_H_
#define _CONFIG_H_

/*
 * 应用配置文件选择。
 * 每次构建只选择一个 profile，它们故意保持冲突的 UART/引脚分配，
 * 防止同时编译到同一个固件映像中。
 *
 * EC_APP_PROFILE_HARDWARE_TEST = 1: 硬件测试模式（测试各外设功能）
 * EC_APP_PROFILE_LINE_CAR      = 2: 循迹小车模式（主控模式）
 * EC_APP_PROFILE_EMPTY         = 3: 空工程模式（最小模板）
 */
/* Build exactly one hardware profile. Profiles intentionally keep conflicting
 * UART/pin assignments out of the same firmware image. */
#define EC_APP_PROFILE_HARDWARE_TEST  1
#define EC_APP_PROFILE_LINE_CAR       2
#define EC_APP_PROFILE_EMPTY          3

/* 默认配置值：这些将被实际构建配置覆盖 */
#ifndef EC_APP_PROFILE
#error "EC_APP_PROFILE must be defined by the build configuration"
#endif

/* 云台默认控制模式：位置模式（vs 速度模式） */
#ifndef GIMBAL_DEFAULT_CONTROL_MODE
#define GIMBAL_DEFAULT_CONTROL_MODE GIMBAL_CONTROL_POSITION
#endif

/* VOFA+ 上位机数据可视化：默认开启（1=开，0=关） */
#ifndef EC_ENABLE_VOFA
#define EC_ENABLE_VOFA 1
#endif

/* HC05 蓝牙模块：默认关闭（需额外硬件） */
#ifndef EC_ENABLE_HC05
#define EC_ENABLE_HC05 0
#endif

/* 云台使能校准：上电时自动校准陀螺仪零点 */
#ifndef GIMBAL_ENABLE_CALIBRATION
#define GIMBAL_ENABLE_CALIBRATION 1
#endif

/* 编译时检查：确保开关类宏的值在 0/1 范围内 */
#if (EC_ENABLE_VOFA != 0) && (EC_ENABLE_VOFA != 1)
#error "EC_ENABLE_VOFA must be 0 or 1"
#endif

#if (EC_ENABLE_HC05 != 0) && (EC_ENABLE_HC05 != 1)
#error "EC_ENABLE_HC05 must be 0 or 1"
#endif

#if (GIMBAL_ENABLE_CALIBRATION != 0) && (GIMBAL_ENABLE_CALIBRATION != 1)
#error "GIMBAL_ENABLE_CALIBRATION must be 0 or 1"
#endif

/* 检查应用配置文件值是否有效 */
#if (EC_APP_PROFILE < EC_APP_PROFILE_HARDWARE_TEST) || (EC_APP_PROFILE > EC_APP_PROFILE_EMPTY)
#error "Invalid EC_APP_PROFILE"
#endif

/*
 * ============================================================
 *  UART 外设驱动规范 — 本工程所有串口设备均遵循此方案
 * ============================================================
 *
 * 方案:  UART RX 中断 + SerialRxBuffer 环形缓冲区
 * 原因:  轮询 uart_query_byte() 只读 4 字节 FIFO，高速率持续
 *        数据会导致 FIFO 溢出丢字节。改用中断自动收数进环
 *        缓冲区，主循环安全取数。
 *
 * 模板 (新增 UART 设备时照此写):
 *
 *   1) 头文件引用:
 *      #include "lib/serial_rx_buffer.h"
 *
 *   2) 定义静态缓冲区:
 *      static uint8_t rx_storage[512];
 *      static SerialRxBuffer rx_ring;
 *
 *   3) 中断回调 (在 ISR 中由逐飞库自动调用):
 *      static void my_uart_isr(uint32_t state, void *ptr)
 *      {
 *          uint8_t data;
 *          (void)ptr;
 *          if ((state & UART_INTERRUPT_STATE_RX) == 0) return;
 *          while (uart_query_byte(MY_UART, &data) == ZF_TRUE)
 *              serial_rx_buffer_push(&rx_ring, data);
 *      }
 *
 *   4) 初始化:
 *      serial_rx_buffer_init(&rx_ring, rx_storage, sizeof(rx_storage));
 *      uart_init(MY_UART, 115200, MY_TX_PIN, MY_RX_PIN);
 *      uart_set_callback(MY_UART, my_uart_isr, NULL);
 *      uart_set_interrupt_config(MY_UART, UART_INTERRUPT_CONFIG_RX_ENABLE);
 *
 *   5) 主循环取数:
 *      uint8_t byte;
 *      while (serial_rx_buffer_pop(&rx_ring, &byte))
 *      {
 *          // 按设备协议解析 byte
 *      }
 *
 * 参考实现:
 *   gimbal/gimbal.c       — UART2 EMM 半双工
 *   gimbal/maixcam2_protocol.c — UART3 MaixCam2 视觉
 *   driver/dt_gyro_z.c    — UART1 陀螺仪 (本驱动)
 *
 * 注意:
 *   - 云台 EMM 是半双工 (TX float)，其他设备是全双工
 *   - 环形缓冲区必须 ≥ 256 字节，高速设备建议 512+
 *   - 不要在 ISR 里做耗时操作 (只 push 不进解析)
 *   - 串口分配见 pin_mapping.h
 *
 * ============================================================
 */

/*==================== 电机 PWM 配置 ====================*/

/*
 * 电机驱动：使用 TIMA0 定时器的 4 个 CCP 通道输出 PWM。
 * 两个直流有刷电机各需 2 路 PWM 信号（IN1/IN2）通过 H 桥驱动。
 *
 * 注意：MSPM0G3507 的 TIMA0 支持互补 PWM 输出和刹车功能，
 * 但这里使用独立的两路 PWM 来实现双电机控制。
 *
 * 左轮 (电机1): TIMA0 CCP0/CCP1.
 * 使用逐飞库的 PWM 宏定义，自动配置定时器通道和 GPIO 复用功能。
 */
#define MOTOR_L_IN1       PWM_TIM_A0_CH0_A8
#define MOTOR_L_IN2       PWM_TIM_A0_CH1_B20

/* 右轮 (电机2): TIMA0 CCP2/CCP3. */
#define MOTOR_R_IN1       PWM_TIM_A0_CH3_A17
#define MOTOR_R_IN2       PWM_TIM_A0_CH2_A15

/*
 * 电机 PWM 载波频率 20kHz，超出人耳听觉范围，
 * 避免可听见的电机啸叫声。
 */
#define MOTOR_PWM_FREQ       20000u

/* PWM 最大占空比值（10000 = 100%）。逐飞库的 PWM 函数通常使用 0~10000 的范围。 */
#define MOTOR_PWM_DUTY_MAX   8000

/* 电机输出方向校正：如果电机接反，调整此符号而非重焊线。 */
#define MOTOR_LEFT_OUTPUT_SIGN      1   /* 左轮正方向 */
#define MOTOR_RIGHT_OUTPUT_SIGN     1   /* 右轮正方向（负号表示与左轮物理方向相反） */

/* 默认电机方向 */
#define CAR_DEFAULT_MOTOR_DIRECTION -1

/* 电机指令超时：超过此时间未收到新指令，电机自动停止（安全保护） */
#define MOTOR_COMMAND_TIMEOUT_MS    200u
/* 遥控器信号超时：超过此时间未收到遥控指令，视为遥控断开 */
#define MOTOR_REMOTE_TIMEOUT_MS     300u

/* 电机最小运行 PWM 值。
 * 由于电机存在启动死区（PWM 较小时电机不转），此值定义了
 * 使电机持续旋转的最小 PWM。需根据实际电机和负载情况校准。 */
/* Loaded continuous-running thresholds. These are physical PWM commands,
 * not offsets added to another command. Recalibrate with MOTOR DEADZONE. */
#define MOTOR_MIN_RUN_PWM_L  1000  /* 闭环下由速度PID防停转 */
#define MOTOR_MIN_RUN_PWM_R  1000

/* 电机死区测试参数（用于自动校准死区补偿值）。
 * 测试过程：从 START_PWM 开始逐渐增加 PWM，直到电机开始旋转，
 * 记录每个"开始旋转->停止->反向旋转"的边界。 */
/* Raw-PWM dead-zone test. The test deliberately bypasses all compensation. */
#define MOTOR_DEADZONE_TEST_START_PWM  4000   /* 测试起始 PWM 值 */
#define MOTOR_DEADZONE_TEST_STEP_PWM    250   /* 每步增加的 PWM 量 */
#define MOTOR_DEADZONE_TEST_STEP_MS     400u  /* 每步的持续时长（ms） */
#define MOTOR_DEADZONE_TEST_PAUSE_MS    500u  /* 测试间的暂停（ms） */
#define MOTOR_DEADZONE_TEST_EDGE_COUNT    4u  /* 检测的边沿次数 */
#define MOTOR_RAW_TEST_DURATION_MS      5000u

/* 低速启动增强测试：高 PWM 启动后切换到较低的持续 PWM。 */
#define MOTOR_STARTUP_BOOST_PWM          5500
#define MOTOR_STARTUP_BOOST_DURATION_MS   20u
#define MOTOR_STARTUP_LOW_PWM            4500
#define MOTOR_STARTUP_LOW_DURATION_MS       0u  /* 0=持续运行，直到手动停止 */

/*==================== 控制算法默认参数 ====================*/

/*
 * 这些参数是小车出厂默认值，在运行时可通过蓝牙/VOFA 在线调节。
 * PID 参数经过初步调试，但最终需根据实际机械结构微调。
 *
 * base_pwm = feedforward base + feedforward_gain * target_rpm
 * 这是前馈控制的开环 PWM 预估值。
 */
/* Runtime tuning defaults. base PWM is the physical feedforward intercept. */
#define CAR_DEFAULT_TARGET_RPM          40.0f    /* 默认目标转速（RPM） */
#define CAR_DEFAULT_BASE_PWM            4500     /* 须高于 MOTOR_MIN_RUN，提供转向减速余量 */
#define CAR_DEFAULT_FEEDFORWARD_GAIN    13.25f   /* 前馈增益（PWM/RPM） */

/* 速度闭环 PID 默认值 */
#define CAR_DEFAULT_SPEED_KP            35.0f    /* 比例增益 */
#define CAR_DEFAULT_SPEED_KI            15.0f    /* 积分增益 */
#define CAR_DEFAULT_SPEED_KD             0.0f    /* 微分增益（速度环不需要） */
#define CAR_SPEED_DERIVATIVE_LPF        0.35f    /* 速度微分低通滤波系数 */

/* 循线位置 PID 默认值（控制车身与循迹线的偏移） */
#define CAR_DEFAULT_LINE_KP            600.0f   /* 比例增益：速度PID分支下需更高 */
#define CAR_DEFAULT_LINE_KD            200.0f   /* 微分增益：抑制过冲穿线 */
#define CAR_DEFAULT_LINE_SIGN              1

/* 航向角 PID 默认值（MPU6050 陀螺仪航向保持） */
#define CAR_DEFAULT_HEADING_KP          90.0f
#define CAR_DEFAULT_HEADING_KD           3.0f
#define CAR_DEFAULT_HEADING_MAX_PWM   2500.0f   /* 航向修正最大 PWM */

/* 左右轮独立增益校正（补偿左右机械差异，如轮径差异、摩擦差异） */
#define CAR_DEFAULT_LEFT_GAIN            1.0f
#define CAR_DEFAULT_RIGHT_GAIN           1.0f

/* 航向方向符号（约定正航向对应左转还是右转） */
#define CAR_DEFAULT_HEADING_SIGN            1

/* 速度闭环默认禁用（仅使用前馈开环控制） */
#define CAR_DEFAULT_SPEED_LOOP_ENABLED       1

/* 底盘控制策略：前馈 + 电池补偿，编码器速度闭环可选。 */
#define CAR_ENABLE_SPEED_FEEDFORWARD          1
#define CAR_ENABLE_BATTERY_PWM_COMPENSATION   1
#define CAR_ENABLE_WHEEL_SPEED_PID            1

#if (CAR_ENABLE_SPEED_FEEDFORWARD != 0) && (CAR_ENABLE_SPEED_FEEDFORWARD != 1)
#error "CAR_ENABLE_SPEED_FEEDFORWARD must be 0 or 1"
#endif
#if (CAR_ENABLE_BATTERY_PWM_COMPENSATION != 0) && \
    (CAR_ENABLE_BATTERY_PWM_COMPENSATION != 1)
#error "CAR_ENABLE_BATTERY_PWM_COMPENSATION must be 0 or 1"
#endif
#if (CAR_ENABLE_WHEEL_SPEED_PID != 0) && (CAR_ENABLE_WHEEL_SPEED_PID != 1)
#error "CAR_ENABLE_WHEEL_SPEED_PID must be 0 or 1"
#endif

/*==================== T8 灰度传感器循线控制参数 ====================*/

/*
 * T8 8 路灰度传感器用于检测地面循迹线位置。
 * 位置值 = 归一化的线位置（-7.0 ~ +7.0），
 * 0 表示线在正中央，正负表示偏左/偏右。
 * 差速转向混合器根据位置值计算左右轮 PWM 差值。
 */
/* 整个循迹线运动范围：8 个传感器在 105mm 轮距上约 ±7.0 归一化单位 */
#define CAR_LINE_POSITION_MAX_ABS        7.0f

/* 位置 PID 输出限幅：最大转向修正 PWM */
#define CAR_LINE_STEER_MAX_PWM        4000.0f

/* 位置 PID 修正量变化斜率，防止误差换向时左右轮补偿瞬间交换。 */
#define CAR_LINE_STEER_SLEW_PWM_PER_S  999999.0f  /* 斜率限制：关闭 */

/* 位置误差微分的低通滤波系数（0.20=较强滤波，抑制传感器噪声引起的抖动） */
#define CAR_LINE_DERIVATIVE_LPF           0.20f

/* 最小目标转速比率：当线位置偏离中心时，最低允许速度为目标值的 25% */
#define CAR_LINE_MIN_TARGET_RPM_RATIO     0.25f

/* 巡线模式下寻找循迹线时固定使用低速原地旋转。 */
#define CAR_LINE_FIND_PWM_MIN           4500.0f
#define CAR_LINE_FIND_PWM_MAX           4500.0f

/* 找到线后必须进入中央两路范围，才从原地找线恢复正常前进。 */
#define CAR_LINE_FIND_CENTER_MAX_ERROR     1

/* 防停转动态补偿：外轮增速量的上限及变化斜率。 */
#define CAR_STEER_BOOST_MAX_PWM           800.0f
#define CAR_STEER_BOOST_SLEW_PWM_PER_S   6000.0f

/* 巡线寻找超时；0 表示持续旋转，直到中央传感器重新检测到黑线。 */
#define CAR_LINE_FIND_TIMEOUT_MS             0u

/* 巡线事件消抖样本数：连续 N 次检测到相同状态才确认为事件 */
#define CAR_LINE_EVENT_DEBOUNCE_SAMPLES      3u

/*==================== 速度 PID 限幅 ====================*/

/* 速度 PID 初始输出限幅：限制闭环调节的最大附加 PWM */
/* Wheel-speed PID correction limits in physical PWM units. */
#define CAR_SPEED_PID_INITIAL_OUTPUT_MAX  2000.0f

/* 速度 PID 积分项上限：防止积分饱和 */
#define CAR_SPEED_PID_INTEGRAL_MAX        3000.0f

/*==================== 调度与故障诊断参数 ====================*/

/* Line-car scheduling, fault qualification and diagnostics. */
#define CAR_KEY_STARTUP_LOCK_MS          1000u   /* 按键自锁时间：上电后 1 秒内按键无效 */
#define CAR_MENU_PERIOD_MS                200u   /* OLED 菜单刷新周期（ms） */
#define CAR_T8_FAILURE_LIMIT               10u   /* T8 传感器连续通信失败阈值 */
#define CAR_T8_RECOVERY_SAMPLES             3u   /* T8 传感器恢复所需的连续成功通信次数 */
#define CAR_GYRO_STALE_TIMEOUT_MS          250u  /* 陀螺仪数据陈旧超时（超过此时间无新数据视为故障） */
#define CAR_DIAGNOSTIC_LOG_PERIOD_MS        2000u /* 诊断日志输出周期（ms） */
#define CAR_RUN_LOG_PERIOD_MS                200u /* 运行日志输出周期（ms） */
#define CAR_OLED_SOFT_I2C_DELAY                10u /* 软件模拟 I2C 的时钟延迟（单位任意，影响 OLED 通信速度） */

/*==================== 在线调参范围与步长 ====================*/

/*
 * 以下参数定义了通过蓝牙/OLED 在线调节各控制参数时的范围限值和步进值。
 * 这些限制防止误操作将参数设置到极端值导致小车失控。
 */
#define CAR_TUNE_TARGET_RPM_MAX          500.0f
#define CAR_TUNE_TARGET_RPM_STEP          10.0f
#define CAR_TUNE_BASE_PWM_STEP           100.0f
#define CAR_TUNE_FF_GAIN_MAX             100.0f
#define CAR_TUNE_FF_GAIN_STEP              0.5f
#define CAR_TUNE_SPEED_KP_MAX            100.0f
#define CAR_TUNE_SPEED_KP_STEP             0.5f
#define CAR_TUNE_SPEED_KI_MAX            100.0f
#define CAR_TUNE_SPEED_KI_STEP             0.1f
#define CAR_TUNE_SPEED_KD_MAX             20.0f
#define CAR_TUNE_SPEED_KD_STEP             0.01f
#define CAR_TUNE_LINE_KP_MAX             1000.0f
#define CAR_TUNE_LINE_KP_STEP              10.0f
#define CAR_TUNE_LINE_KD_MAX               50.0f
#define CAR_TUNE_LINE_KD_STEP               0.1f
#define CAR_TUNE_WHEEL_GAIN_MIN              0.5f
#define CAR_TUNE_WHEEL_GAIN_MAX              1.5f
#define CAR_TUNE_WHEEL_GAIN_STEP             0.01f
#define CAR_TUNE_HEADING_KP_MAX             500.0f
#define CAR_TUNE_HEADING_KP_STEP              5.0f
#define CAR_TUNE_HEADING_KD_MAX              50.0f
#define CAR_TUNE_HEADING_KD_STEP              0.5f
#define CAR_TUNE_HEADING_MAX_PWM          5000.0f
#define CAR_TUNE_HEADING_MAX_STEP          100.0f

/*==================== 编码器配置 ====================*/

/*
 * 编码器用于测量车轮转速，实现速度闭环控制。
 * 使用电机自带的霍尔编码器（13 PPR，1:28 减速比）。
 *
 * 编码方式：AB 正交解码（Quadrature Decoder）
 *   - 左轮 TIMG8 使用硬件 QEI
 *   - 右轮现有 PA14/PB24 属于 TIMG12；该定时器不支持 QEI，因此使用 GPIO 双边沿解码
 *   - 两种方式均按四倍频计数：13 PPR × 28 减速比 × 4 = 1456 CPR
 */
#define ENCODER_USE_QUADRATURE          1
#define ENCODER1_USE_HARDWARE_QUADRATURE 1
#define ENCODER2_USE_HARDWARE_QUADRATURE 0
#define ENCODER1_TIMER       TIM_G8
#define ENCODER1_CHANNEL_A   TIMG8_ENCODER1_CH1_A26
#define ENCODER1_CHANNEL_B   TIMG8_ENCODER1_CH2_A27
#define ENCODER1_A_PIN       A26
#define ENCODER1_B_PIN       A27

#define ENCODER1_DIRECTION_SIGN  1

/* 编码器2 (右轮): PA14/PB24 作为 GPIO 双边沿输入；TIMG12 仅保留引脚来源信息。 */
#define ENCODER2_TIMER       TIM_G12
#define ENCODER2_CHANNEL_A   TIMG12_ENCODER1_CH1_A14
#define ENCODER2_CHANNEL_B   TIMG12_ENCODER1_CH2_B24
#define ENCODER2_A_PIN       A14
#define ENCODER2_B_PIN       B24
#define ENCODER2_DIRECTION_SIGN  1

/*
 * 编码器每转脉冲数（Counts Per Revolution）。
 * MG513 电机标称值：13 PPR（脉冲/转），1:30 减速比，AB 四倍频。
 * 计算：13 × 30 × 4 = 1560
 * 即电机输出轴每转一圈，编码器产生 1560 个计数。
 */
/* 13PPR, 1:28, AB x4 = 1456。 */
#define ENCODER_CPR      1456

/*
 * 车轮直径（毫米）。
 * 65mm 是标称值，实际有效直径受胎压、负载、磨损影响。
 * 建议：在实际路面上测量 10 圈行驶距离，反推出有效直径。
 * 此值用于计算行驶速度和里程。
 */
/* 仅为初始值：65 mm 标称轮径；应以负载下实测有效周长修正里程。 */
#define WHEEL_DIAMETER_MM 65.0f

/*
 * 编码器 RPM 低通滤波系数。
 * sensor 任务已从 50ms 提高到 10ms，alpha 从 0.35 提高到 0.55，
 * 减少滤波延迟（时间常数 ~18ms），适配 100Hz 速度闭环。
 */
#define ENCODER_RPM_LPF_ALPHA 0.55f

/*==================== GPIO 引脚定义 ====================*/

/*
 * 板载按键（默认高电平，按下拉低）。
 * KEY1/KEY2 用户自定义功能（如菜单切换、参数调整）。
 * KEY3 可配置为运行/停止切换。
 */
/* 按键 */
#define KEY1_PIN         B6
#define KEY2_PIN         B7
#define KEY3_PIN         B23

/* 蜂鸣器 (有源蜂鸣器，高电平驱动) */
#define BUZZER_PIN       A31
/* 继电器控制引脚（用于控制大电流负载或电源管理） */
#define RELAY_PIN        A28

/*==================== 传感器与外部模块引脚 ====================*/

/* 航向传感器后端。只修改 CAR_GYRO_SOURCE 即可切换。 */
#define CAR_GYRO_SOURCE_M0       1
#define CAR_GYRO_SOURCE_MPU6050  2

#ifndef CAR_GYRO_SOURCE
#define CAR_GYRO_SOURCE CAR_GYRO_SOURCE_M0 // 使用M0
#endif

#if (CAR_GYRO_SOURCE != CAR_GYRO_SOURCE_M0) && \
    (CAR_GYRO_SOURCE != CAR_GYRO_SOURCE_MPU6050)
#error "CAR_GYRO_SOURCE must be M0 or MPU6050"
#endif

/*
 * MPU6050 六轴陀螺仪/加速度计（软件模拟 I2C 接口）。
 * 在 line-car 模式中用作航向（偏航角）测量源。
 * 使用软件 I2C（GPIO 位操作）而非硬件 I2C，以节省外设资源。
 * 软件 I2C 的时序由 CAR_OLED_SOFT_I2C_DELAY 控制。
 *
 * 当 CAR_GYRO_SOURCE 为 CAR_GYRO_SOURCE_MPU6050 时启用。
 */
/* MPU6050 (软件 I2C，line-car 航向源) */
#define MPU6050_SCL      A1
#define MPU6050_SDA      A0

/*
 * OLED 显示屏 (SSD1306, 128x64 像素, I2C 接口)。
 * 使用软件模拟 I2C（与 MPU6050 共用时序控制）。
 * 用于显示运行时参数（速度、PID 值、电池电压等）和调试信息。
 * SCL/SDA 连接 B9/B8（非硬件 I2C 外设引脚）。
 */
/* OLED (I2C SSD1306 128x64) */
#define OLED_SCL         B9
#define OLED_SDA         B8

/*==================== 电池电压监测 ====================*/

/*
 * 电池电压通过分压电阻连接至 ADC 引脚。
 * 使用 MSPM0G3507 内置 ADC（12 位分辨率）采样。
 *
 * 电压计算：
 *   实际电压(mV) = ADC值 × ADC参考电压(mV) / ADC满量程 × 分压比
 *
 * 故障处理：包含欠压保护（UNDERVOLTAGE）、恢复迟滞（RECOVERY）、
 * 滤波（FILTER_ALPHA）和消抖（FAULT_SAMPLES/RECOVERY_SAMPLES）。
 */
#define BAT_ADC          ADC0_CH2_A25          /* 电池电压 ADC 通道：ADC0 通道 2，引脚 A25 */
#define BAT_REFERENCE_MV       14800u           /* 电池参考电压：14.8V（典型 4S 锂电池满电电压） */
#define BAT_DIVIDER              447u           /* 分压比 ×100（实际 4.47:1） */
#define BAT_ADC_REF_MV          3300u           /* ADC 参考电压：3.3V（MSPM0G3507 的 VDDA） */
#define BAT_VALID_MIN_MV        6000u           /* 最低有效电压 6.0V（低于此视为电池未接） */
#define BAT_VALID_MAX_MV       18000u           /* 最高有效电压 18.0V（高于此视为测量错误） */
#define BAT_UNDERVOLTAGE_MV    12000u           /* 欠压阈值 12.0V */
#define BAT_RECOVERY_MV        13000u           /* 欠压恢复阈值 13.0V（带迟滞） */
#define BAT_COMP_MIN_FACTOR      0.85f           /* 电压补偿最小系数 */
#define BAT_COMP_MAX_FACTOR      1.30f           /* 电压补偿最大系数 */
#define BAT_FILTER_ALPHA          0.10f           /* 一阶低通滤波系数（越小越平滑） */
#define BAT_FAULT_SAMPLES            10u         /* 欠压判定所需连续采样数 */
#define BAT_RECOVERY_SAMPLES         15u         /* 恢复判定所需连续采样数 */
#define BAT_ADC_SAMPLE_COUNT           8u         /* 每次 ADC 采样的过采样数（取均值） */
#define BAT_ADC_FULL_SCALE          4095u         /* 12 位 ADC 满量程值 */
#define BAT_DIVIDER_SCALE            100u         /* 分压比放大倍数（整数运算用） */
#define BAT_ADC_RESOLUTION       ADC_12BIT        /* ADC 分辨率设置：12 位 */

/*
 * T8 灰度循迹模块（I2C 模式接口）。
 * 注意：T8 模块支持 UART 和 I2C 双模式。
 * 在 config.h 中此处仅定义 I2C 引脚（B2/B3），
 * 但具体使用哪种模式由上层代码框架选择。
 * 实际项目中 T8 可能通过 UART1（A8/A9）连接。
 */
/* 循迹模块 (I2C) */
#define TRACE_SCL        B2
#define TRACE_SDA        B3

/*
 * 蓝牙透明串口（HC-05/HC-06 或类似模块）。
 * 在 line-car 模式中，此串口同时承担三重角色：
 *   1. 调试串口（debug_printf 输出）
 *   2. VOFA+ 上位机数据可视化通道
 *   3. 在线参数调节命令通道
 *
 * 使用 UART2，波特率 115200bps。
 */
/* 蓝牙透明串口，同时作为 line-car debug/VOFA/在线调参端口。 */
#define BLUETOOTH_UART       UART_2              /* 蓝牙串口号 */
#define BLUETOOTH_TX_PIN     UART2_TX_B15        /* TX 引脚：PB15 */
#define BLUETOOTH_RX_PIN     UART2_RX_B16        /* RX 引脚：PB16 */
#define BLUETOOTH_BAUDRATE   115200u             /* 波特率 115200 */

/*
 * M0 单轴 Z 轴陀螺仪 — 支持 UART 和 I2C 两种传输方式。
 * 通过 GYRO_Z_TRANSPORT 切换：1=UART (PA10/PA11)，2=I2C (PA0/PA1，addr 0x48)。
 * hardware-test 固定使用该模块；line-car 由 CAR_GYRO_SOURCE 选择。
 */
/* M0 单轴 Z 陀螺仪传输方式：1=UART, 2=I2C */
#ifndef GYRO_Z_TRANSPORT
#define GYRO_Z_TRANSPORT                2
#endif

/* M0 单轴 Z 陀螺仪 UART */
#define GYRO_Z_UART      UART_0                  /* 陀螺仪串口号 */
#define GYRO_Z_TX_PIN    UART0_TX_A10            /* TX 引脚：PA10 */
#define GYRO_Z_RX_PIN    UART0_RX_A11            /* RX 引脚：PA11 */
#define GYRO_Z_BAUD      115200                   /* 手册默认寄存器值 0x0002 对应 9600 */

/* M0 单轴 Z 陀螺仪 I2C */
#define GYRO_Z_IIC_SCL    A1                     /* SCL：PA1 */
#define GYRO_Z_IIC_SDA    A0                     /* SDA：PA0 */
#define GYRO_Z_IIC_ADDR   0x48u                  /* 器件地址 7 位 */
#define GYRO_Z_IIC_DELAY  100u                    /* 软件 I2C 延时，与 MPU6050 一致 */

/*
 * 在线调参说明：
 * 调参命令从蓝牙串口（UART2）的中断接收环形缓冲区读取，
 * 而非从调试串口读取。这是为了在调试输出和调参输入复用同一条串口时
 * 能够正确区分数据和命令。
 */

#endif /* _CONFIG_H_ */
