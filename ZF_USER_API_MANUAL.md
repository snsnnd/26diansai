# MSPM0G3507 用户层 API 参考手册

> **层归属**: 用户应用层 (user/inc/)  
> **依赖**: 本层构建在 [逐飞 SeekFree 底层库](ZF_SEEKFREE_API_MANUAL.md) 之上  
> **平台**: MSPM0G3507 + 天猛星开发板 | **生成日期**: 2026-07-20  
> **工程模式**: hardware-test / line-car / empty  

---

## 目录

1. [架构定位](#1-架构定位)
2. [系统配置 — config.h / pin_mapping.h](#2-系统配置--configh--pin_mappingh)
3. [框架层 — framework](#3-框架层--framework)
   - [3.1 协作式调度器 — ec_scheduler](#31-协作式调度器--ec_scheduler)
   - [3.2 时间管理 — ec_time](#32-时间管理--ec_time)
   - [3.3 按键框架 — ec_keys](#33-按键框架--ec_keys)
   - [3.4 模式管理 — ec_mode_manager](#34-模式管理--ec_mode_manager)
   - [3.5 菜单 — ec_menu](#35-菜单--ec_menu)
   - [3.6 参数菜单 — ec_parameter_menu](#36-参数菜单--ec_parameter_menu)
   - [3.7 应用入口 — ec_app](#37-应用入口--ec_app)
4. [驱动层 — driver](#4-驱动层--driver)
   - [4.1 直流电机 — dt_motor](#41-直流电机--dt_motor)
   - [4.2 编码器 — dt_encoder](#42-编码器--dt_encoder)
   - [4.3 MPU6050 — dt_mpu6050](#43-mpu6050--dt_mpu6050)
   - [4.4 航向传感器 — dt_heading / dt_mpu6050_heading](#44-航向传感器--dt_heading--dt_mpu6050_heading)
   - [4.5 单轴陀螺仪 — dt_gyro_z](#45-单轴陀螺仪--dt_gyro_z)
   - [4.6 Mahony 姿态 — dt_imu_mahony](#46-mahony-姿态--dt_imu_mahony)
   - [4.7 OLED (SSD1306 I2C) — dt_oled](#47-oled-ssd1306-i2c--dt_oled)
   - [4.8 LED — dt_led](#48-led--dt_led)
   - [4.9 RGB 灯 — dt_rgb](#49-rgb-灯--dt_rgb)
   - [4.10 蜂鸣器 — dt_buzzer](#410-蜂鸣器--dt_buzzer)
   - [4.11 蓝牙 HC05 — dt_hc05](#411-蓝牙-hc05--dt_hc05)
5. [工具库 — lib](#5-工具库--lib)
   - [5.1 PID 控制器 — pid_controller](#51-pid-控制器--pid_controller)
   - [5.2 串口接收环形缓冲区 — serial_rx_buffer](#52-串口接收环形缓冲区--serial_rx_buffer)
   - [5.3 串口发送环形缓冲区 — serial_tx_buffer](#53-串口发送环形缓冲区--serial_tx_buffer)
   - [5.4 通用工具 — car_utils](#54-通用工具--car_utils)
6. [设备驱动 — device](#6-设备驱动--device)
   - [6.1 T8 灰度传感器 — t8_gray_sensor](#61-t8-灰度传感器--t8_gray_sensor)
7. [通信协议 — protocol](#7-通信协议--protocol)
   - [7.1 VOFA+ 数据可视化 — vofa](#71-vofa-数据可视化--vofa)
8. [应用层 — app](#8-应用层--app)
   - [8.1 循迹小车 — line_car](#81-循迹小车--line_car)
   - [8.2 竞赛任务 — h2024_tasks](#82-竞赛任务--h2024_tasks)
   - [8.3 硬件测试 — hardware_test](#83-硬件测试--hardware_test)
   - [8.4 电机测试 — motor_test](#84-电机测试--motor_test)
   - [8.5 电池补偿 — battery_compensation](#85-电池补偿--battery_compensation)
   - [8.6 参数调校 — car_tuning](#86-参数调校--car_tuning)
   - [8.7 巡线事件检测 — line_event_detector](#87-巡线事件检测--line_event_detector)
9. [云台控制 — gimbal](#9-云台控制--gimbal)
   - [9.1 EMM 步进电机 — emm_stepper](#91-emm-步进电机--emm_stepper)
   - [9.2 云台主控 — gimbal](#92-云台主控--gimbal)
   - [9.3 MaixCam2 视觉 — maixcam2_protocol](#93-maixcam2-视觉--maixcam2_protocol)
   - [9.4 位置控制 — position_control](#94-位置控制--position_control)
   - [9.5 速度控制 — speed_control](#95-速度控制--speed_control)
10. [其他 — isr.h / zf_ccs_compat.h](#10-其他--isrh--zf_ccs_compath)
11. [从底层到用户层的典型数据流](#11-从底层到用户层的典型数据流)

---

## 1. 架构定位

```
┌──────────────────────────────────────────────────────────┐
│  app/  应用层                                            │
│  (line_car, h2024_tasks, hardware_test, gimbal)          │
│  ┌────────────────────────────────────────────────────┐  │
│  │  framework/  框架层                                │  │
│  │  (scheduler, mode_manager, menu, keys, time)       │  │
│  │  ┌──────────────────────────────────────────────┐  │  │
│  │  │  driver/  用户驱动层                         │  │  │
│  │  │  (motor, encoder, mpu6050, heading, oled...) │  │  │
│  │  │  ┌────────────────────────────────────────┐  │  │  │
│  │  │  │  lib/  工具库 (pid, ringbuf, utils)     │  │  │  │
│  │  │  │  ┌──────────────────────────────────┐  │  │  │  │
│  │  │  │  │  SeekFree 底层库                 │  │  │  │  │
│  │  │  │  │  (见 ZF_SEEKFREE_API_MANUAL.md)  │  │  │  │  │
│  │  │  │  └──────────────────────────────────┘  │  │  │  │
│  │  │  └────────────────────────────────────────┘  │  │  │
│  │  └──────────────────────────────────────────────┘  │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
```

**本手册所有内容** 均在 `user/inc/` 和 `user/src/` 中实现，不修改逐飞底层库。用户层通过 `#include "zf_common_headfile.h"` 获得底层能力，然后在此基础上构建应用。

---

## 2. 系统配置 — config.h / pin_mapping.h

### 2.1 config.h — 核心配置

**文件**: [user/inc/config.h](user/inc/config.h)

这是整个工程的 **唯一配置入口**。所有引脚、PID参数、控制阈值均在此定义。

#### 应用配置文件

```c
#define EC_APP_PROFILE_HARDWARE_TEST  1    // 硬件测试模式
#define EC_APP_PROFILE_LINE_CAR       2    // 循迹小车模式（默认）
#define EC_APP_PROFILE_EMPTY          3    // 空工程模板
```

构建时通过 `-DEC_APP_PROFILE=2` 选择。

#### 电机 PWM 配置

```c
#define MOTOR_L_IN1       PWM_TIM_A0_CH0_A8     // 左轮 IN1 (TIMA0 CH0, PA8)
#define MOTOR_L_IN2       PWM_TIM_A0_CH1_B20    // 左轮 IN2 (TIMA0 CH1, PB20)
#define MOTOR_R_IN1       PWM_TIM_A0_CH3_A17    // 右轮 IN1 (TIMA0 CH3, PA17)
#define MOTOR_R_IN2       PWM_TIM_A0_CH2_A15    // 右轮 IN2 (TIMA0 CH2, PA15)
#define MOTOR_PWM_FREQ       20000u             // 载波 20kHz（超听觉范围）
#define MOTOR_PWM_DUTY_MAX   8000               // 占空比 0~8000
```

#### PID 默认参数

```c
#define CAR_DEFAULT_SPEED_KP     35.0f     // 速度环 P
#define CAR_DEFAULT_SPEED_KI     15.0f     // 速度环 I
#define CAR_DEFAULT_LINE_KP     600.0f     // 循线位置 P
#define CAR_DEFAULT_LINE_KD     200.0f     // 循线位置 D
#define CAR_DEFAULT_HEADING_KP   90.0f     // 航向环 P
#define CAR_DEFAULT_HEADING_KD    3.0f     // 航向环 D
#define CAR_DEFAULT_BASE_PWM     4500      // 前馈基础 PWM
#define CAR_DEFAULT_FEEDFORWARD_GAIN 13.25f // 前馈增益 (PWM/RPM)
#define CAR_DEFAULT_TARGET_RPM    40.0f    // 默认目标转速
```

#### 编码器配置

```c
#define ENCODER1_TIMER       TIM_G8                  // 左轮: 硬件 QEI (TIMG8)
#define ENCODER1_CHANNEL_A   TIMG8_ENCODER1_CH1_A26  // A相: PA26
#define ENCODER1_CHANNEL_B   TIMG8_ENCODER1_CH2_A27  // B相: PA27
#define ENCODER2_TIMER       TIM_G12                 // 右轮: GPIO 双边沿 (TIMG12无QEI)
#define ENCODER2_CHANNEL_A   TIMG12_ENCODER1_CH1_A14 // A相: PA14
#define ENCODER2_CHANNEL_B   TIMG12_ENCODER1_CH2_B24 // B相: PB24
#define ENCODER_CPR          1456                    // 13PPR×28减速比×4倍频
#define WHEEL_DIAMETER_MM    65.0f                   // 轮径 (mm)
```

#### 外设引脚

```c
// 按键/蜂鸣器/继电器
#define KEY1_PIN    B6 | KEY2_PIN B7 | KEY3_PIN B23
#define BUZZER_PIN  A31 | RELAY_PIN A28

// I2C 设备
#define OLED_SCL    B9  | OLED_SDA    B8       // OLED (软件 I2C, addr 0x3C)
#define MPU6050_SCL A1  | MPU6050_SDA A0       // MPU6050 (软件 I2C)
#define GYRO_Z_IIC_ADDR  0x48u                 // M0 单轴陀螺仪

// 电池 ADC
#define BAT_ADC          ADC0_CH2_A25
#define BAT_DIVIDER      447u                  // 分压比 ×100 (4.47:1)
#define BAT_UNDERVOLTAGE_MV  12000u            // 12V 欠压阈值

// 蓝牙/UART
#define BLUETOOTH_UART      UART_2
#define BLUETOOTH_TX_PIN    UART2_TX_B15
#define BLUETOOTH_RX_PIN    UART2_RX_B16
```

#### 控制策略开关

```c
#define CAR_ENABLE_SPEED_FEEDFORWARD       1    // 前馈控制
#define CAR_ENABLE_BATTERY_PWM_COMPENSATION 1   // 电池电压补偿
#define CAR_ENABLE_WHEEL_SPEED_PID         1    // 速度闭环
```

### 2.2 pin_mapping.h — 引脚映射

**文件**: [user/inc/pin_mapping.h](user/inc/pin_mapping.h)

天猛星开发板的标准化引脚别名：

```c
// LED
#define PIN_LED1  A30  | PIN_LED2  A7  | PIN_LED3  B27

// RGB (共阳/共阴可配置)
#define PIN_RGB_R B0   | PIN_RGB_G B1  | PIN_RGB_B A29
#define PIN_RGB_ON_LEVEL   GPIO_LOW
#define PIN_RGB_OFF_LEVEL  GPIO_HIGH

// 调试串口 — Line-car 用蓝牙 (UART2), 其他用 UART1
#if EC_APP_PROFILE == EC_APP_PROFILE_LINE_CAR
  #define BOARD_DEBUG_UART     BLUETOOTH_UART      // UART2
#else
  #define BOARD_DEBUG_UART     UART_1              // UART1
#endif

// EMM 云台 UART:  UART2 (PB15/PB16)
// T8 传感器 UART: UART1 (PA8/PA9)
// MaixCam2 UART:   UART3 (PB2/PB3)
```

---

## 3. 框架层 — framework

### 3.1 协作式调度器 — ec_scheduler

**文件**: [user/inc/framework/ec_scheduler.h](user/inc/framework/ec_scheduler.h)

> 非抢占式周期任务调度器。主循环中无竞态条件，每个任务自行返回。

```c
#define EC_SCHEDULER_MAX_TASKS  12u

typedef void (*ec_task_fn)(uint32_t now_ms, void *context);

typedef struct {
    const char *name;                    // 任务名
    ec_task_fn  run;                     // 执行函数
    void       *context;                 // 用户上下文
    uint32_t    period_ms;              // 执行周期
    uint32_t    next_run_ms;            // 下次执行时刻
    // 运行时统计
    uint32_t    run_count;              // 累计执行次数
    uint32_t    missed_deadlines;       // 错过截止期次数
    uint32_t    max_runtime_ms;         // 最大执行时间
    uint32_t    overrun_count;          // 超时次数
    bool        enabled;
} ec_task_t;

typedef struct {
    ec_task_t tasks[EC_SCHEDULER_MAX_TASKS];
    uint8_t count;
} ec_scheduler_t;

void ec_scheduler_init(ec_scheduler_t *scheduler);
bool ec_scheduler_add(ec_scheduler_t *scheduler, const char *name,
    ec_task_fn run, void *context, uint32_t period_ms, uint32_t start_ms);
void ec_scheduler_run(ec_scheduler_t *scheduler, uint32_t now_ms);
```

**使用**:
```c
ec_scheduler_t sched;
ec_scheduler_init(&sched);

void sensor_10ms(uint32_t now, void *ctx) { /* 读传感器 */ }
void control_10ms(uint32_t now, void *ctx) { /* PID控制 */ }

ec_scheduler_add(&sched, "sensor",  sensor_10ms,  NULL, 10, 0);
ec_scheduler_add(&sched, "control", control_10ms, NULL, 10, 5);  // 错开5ms

while (1) {
    ec_scheduler_run(&sched, ec_time_ms());
}
```

---

### 3.2 时间管理 — ec_time

**文件**: [user/inc/framework/ec_time.h](user/inc/framework/ec_time.h)

```c
typedef void (*ec_time_tick_hook_t)(uint32_t now_ms, void *context);

void     ec_time_init(void);
uint32_t ec_time_ms(void);                                               // 系统 ms 时间戳
bool     ec_time_elapsed(uint32_t now_ms, uint32_t since_ms, uint32_t period_ms);
void     ec_time_set_tick_hook(ec_time_tick_hook_t hook, void *context); // 注册 tick 钩子
```

---

### 3.3 按键框架 — ec_keys

**文件**: [user/inc/framework/ec_keys.h](user/inc/framework/ec_keys.h)

> 对 zf_device_key 的封装，提供事件队列和紧急停止钩子。

```c
typedef void (*ec_keys_emergency_hook_t)(void *context);

typedef struct {
    gpio_pin_enum key1_pin, key2_pin, key3_pin;
    uint32_t debounce_ms;               // 消抖时间
    uint32_t startup_lock_ms;           // 上电锁定时间
    ec_keys_emergency_hook_t emergency_hook;
    void *emergency_context;
} ec_keys_config_t;

void ec_keys_init(const ec_keys_config_t *config);
bool ec_keys_pop(uint8_t *key);         // 弹出一个按键事件（非阻塞）
bool ec_keys_emergency_pending(void);   // 紧急停止挂起标志
```

---

### 3.4 模式管理 — ec_mode_manager

**文件**: [user/inc/framework/ec_mode_manager.h](user/inc/framework/ec_mode_manager.h)

> 管理运行/停止/故障等多种模式的切换。最多 16 种模式。

```c
typedef bool (*ec_mode_init_fn)(void *context);    // 返回 false=初始化失败
typedef void (*ec_mode_fn)(uint32_t now_ms, void *context);

typedef enum { EC_MODE_STOPPED = 0, EC_MODE_RUNNING, EC_MODE_FAULT } ec_mode_state_t;

typedef struct {
    const char *name;
    ec_mode_init_fn init;
    ec_mode_fn start, run, stop;         // 生命周期回调
    void *context;
} ec_mode_t;

typedef struct {
    ec_mode_t modes[16];
    uint8_t count, selected, active;
    ec_mode_state_t state;
} ec_mode_manager_t;

// API
void ec_mode_manager_init(ec_mode_manager_t *mgr);
bool ec_mode_manager_add(ec_mode_manager_t *mgr, const ec_mode_t *mode);
void ec_mode_manager_select_next/previous(ec_mode_manager_t *mgr);
bool ec_mode_manager_start(ec_mode_manager_t *mgr, uint32_t now_ms);
void ec_mode_manager_stop(ec_mode_manager_t *mgr, uint32_t now_ms);
void ec_mode_manager_run(ec_mode_manager_t *mgr, uint32_t now_ms);
const char *ec_mode_manager_selected/active_name(const ec_mode_manager_t *mgr);
```

---

### 3.5 菜单 — ec_menu

**文件**: [user/inc/framework/ec_menu.h](user/inc/framework/ec_menu.h)

> OLED 菜单导航，与 ec_mode_manager 联动。

```c
typedef void (*ec_menu_render_fn)(const ec_mode_manager_t *mgr, uint32_t now_ms, void *ctx);

typedef struct {
    ec_mode_manager_t *manager;
    ec_menu_render_fn  render;
    void              *render_context;
    uint32_t           render_period_ms;
    bool               dirty;
} ec_menu_t;

void ec_menu_init(ec_menu_t *menu, ec_mode_manager_t *mgr,
    ec_menu_render_fn render, void *ctx, uint32_t render_period_ms);
void ec_menu_handle_key(ec_menu_t *menu, ec_menu_key_t key, uint32_t now_ms);
void ec_menu_update(ec_menu_t *menu, uint32_t now_ms);
```

---

### 3.6 参数菜单 — ec_parameter_menu

**文件**: [user/inc/framework/ec_parameter_menu.h](user/inc/framework/ec_parameter_menu.h)

> OLED 在线调参：选择参数 → 修改值 → 实时生效。

```c
typedef enum {
    EC_PARAM_INT8, EC_PARAM_INT16, EC_PARAM_UINT16,
    EC_PARAM_FLOAT, EC_PARAM_BOOL, EC_PARAM_ACTION
} ec_parameter_type_t;

typedef void (*ec_parameter_action_fn)(void *context);

typedef struct {
    const char *name;
    ec_parameter_type_t type;
    void *value;                          // 指向实际变量的指针
    float min_value, max_value, step;
    ec_parameter_action_fn action;        // ACTION 类型的回调
    void *context;
} ec_parameter_item_t;

typedef struct {
    ec_parameter_item_t *items;
    uint8_t count, selected;
    bool editing, dirty;
} ec_parameter_menu_t;

void ec_parameter_menu_init(ec_parameter_menu_t *menu, ec_parameter_item_t *items, uint8_t count);
void ec_parameter_menu_handle_key(ec_parameter_menu_t *menu, ec_menu_key_t key);
const ec_parameter_item_t *ec_parameter_menu_current(const ec_parameter_menu_t *menu);
void ec_parameter_menu_format_value(const ec_parameter_item_t *item, char *buf, size_t cap);
```

---

### 3.7 应用入口 — ec_app

**文件**: [user/inc/framework/ec_app.h](user/inc/framework/ec_app.h)

```c
void ec_app_init(void);                   // 应用初始化（注册所有模式）
void ec_app_run(void);                    // 应用主循环
int  ec_app_emergency_stop(void);         // 紧急停止所有电机
const ec_scheduler_t *ec_app_get_scheduler(void);
```

---

## 4. 驱动层 — driver

### 4.1 直流电机 — dt_motor

**文件**: [user/inc/driver/dt_motor.h](user/inc/driver/dt_motor.h)

> H桥直流电机，双路PWM，支持正反转/停止/急停/刹车。

```c
#define DT_MOTOR_DUTY_MAX  MOTOR_PWM_DUTY_MAX    // = 8000

typedef struct {
    pwm_channel_enum in1_pin;     // H桥 IN1
    pwm_channel_enum in2_pin;     // H桥 IN2
    uint32_t         pwm_freq;    // 载波频率 (Hz)
} dt_motor_config_t;

void dt_motor_init(dt_motor_config_t *cfg);
void dt_motor_set_speed(dt_motor_config_t *cfg, int16_t speed);  // [-8000, +8000]
void dt_motor_stop(dt_motor_config_t *cfg);                       // 停止 (两路=0)
void dt_motor_emergency_stop(dt_motor_config_t *cfg);             // 强制拉低 (pwm_force_low)
void dt_motor_brake(dt_motor_config_t *cfg);                      // 电磁制动 (同侧导通)
```

**使用**:
```c
dt_motor_config_t left  = { .in1_pin=MOTOR_L_IN1, .in2_pin=MOTOR_L_IN2, .pwm_freq=20000 };
dt_motor_init(&left);
dt_motor_set_speed(&left,  5000);    // 正转 62.5%
dt_motor_set_speed(&left, -3000);    // 反转 37.5%
```

---

### 4.2 编码器 — dt_encoder

**文件**: [user/inc/driver/dt_encoder.h](user/inc/driver/dt_encoder.h)

> 支持 GPIO 双边沿解码和硬件 QEI 正交解码。自动计算 RPM 和行驶距离。

```c
typedef struct {
    gpio_pin_enum    a_pin, b_pin;        // A/B 相引脚
    uint16_t         counts_per_rev;      // 每转脉冲 (CPR)
    float            wheel_circumference_mm; // 车轮周长
    volatile uint32_t edge_count;         // 有效脉冲总数
    volatile int32_t  signed_edge_count;  // 带符号脉冲总数
    volatile uint32_t invalid_transition_count; // 非法跳变 (诊断)
    bool             quadrature_enabled;  // 正交解码使能
    bool             hardware_quadrature; // 硬件 QEI 方式
    timer_index_enum timer_index;         // 硬件定时器
    encoder_channel1_enum channel_a;
    encoder_channel2_enum channel_b;
    int8_t           direction_sign;      // 方向符号 (1或-1)
    float            rpm_lpf_alpha;       // RPM 低通滤波系数
    float            rpm;                 // 滤波后转速 (RPM)
    float            rpm_signed;          // 带符号转速
} dt_encoder_t;

// API
bool     dt_encoder_init(dt_encoder_t *enc);
bool     dt_encoder_is_ready(const dt_encoder_t *enc);
void     dt_encoder_sample_inputs(dt_encoder_t *enc);     // 轮询采样 (诊断)
void     dt_encoder_reset_odometry(dt_encoder_t *enc);    // 清零里程 (仅停车时)
uint32_t dt_encoder_get_edges(dt_encoder_t *enc);
int32_t  dt_encoder_get_signed_edges(dt_encoder_t *enc);
uint32_t dt_encoder_get_delta(dt_encoder_t *enc);         // 增量脉冲
int32_t  dt_encoder_get_signed_delta(dt_encoder_t *enc);
float    dt_encoder_compute_rpm(dt_encoder_t *enc, uint32_t dt_ms);
float    dt_encoder_compute_signed_rpm(dt_encoder_t *enc, uint32_t dt_ms);
float    dt_encoder_get_travel_mm(dt_encoder_t *enc);
float    dt_encoder_get_distance_mm(dt_encoder_t *enc);   // 有符号位移
```

**使用**:
```c
dt_encoder_t enc1 = {
    .a_pin = A26, .b_pin = A27,
    .counts_per_rev = 1456,
    .wheel_circumference_mm = 3.14159f * 65.0f,
    .quadrature_enabled = true, .hardware_quadrature = true,
    .timer_index = TIM_G8,
    .channel_a = TIMG8_ENCODER1_CH1_A26,
    .channel_b = TIMG8_ENCODER1_CH2_A27,
    .direction_sign = 1, .rpm_lpf_alpha = 0.55f,
};
dt_encoder_init(&enc1);
float rpm = dt_encoder_compute_signed_rpm(&enc1, 10);     // 每10ms
float dist_mm = dt_encoder_get_distance_mm(&enc1);
```

---

### 4.3 MPU6050 — dt_mpu6050

**文件**: [user/inc/driver/dt_mpu6050.h](user/inc/driver/dt_mpu6050.h)

> 六轴 IMU，软件 I2C。提供 HAL（单寄存器读写）和高级接口。

```c
#define DT_MPU6050_DEFAULT_ADDR  0x68

// 量程宏
#define DT_MPU6050_ACCEL_FS_2G   0    // ±2g  (16384 LSB/g)
#define DT_MPU6050_ACCEL_FS_4G   1    // ±4g  (8192 LSB/g)
#define DT_MPU6050_ACCEL_FS_8G   2    // ±8g  (4096 LSB/g)
#define DT_MPU6050_ACCEL_FS_16G  3    // ±16g (2048 LSB/g)
#define DT_MPU6050_GYRO_FS_250   0    // ±250°/s  (131 LSB/(°/s))
#define DT_MPU6050_GYRO_FS_500   1    // ±500°/s  (65.5)
#define DT_MPU6050_GYRO_FS_1000  2    // ±1000°/s (32.8)
#define DT_MPU6050_GYRO_FS_2000  3    // ±2000°/s (16.4)

typedef struct {
    soft_iic_info_struct iic;
    uint8_t accel_fs, gyro_fs;
} dt_mpu6050_config_t;

typedef struct {
    float ax, ay, az;     // 加速度 (g)
    float gx, gy, gz;     // 角速度 (°/s)
    float temp;           // 温度 (°C)
} dt_mpu6050_data_t;

// HAL 接口 — 单寄存器读写（可被其他传感器复用）
uint8_t dt_mpu6050_hal_init(soft_iic_info_struct *iic, uint8_t sample_rate_div,
                            uint8_t dlpf_cfg, uint8_t gyro_fs, uint8_t accel_fs);
uint8_t dt_mpu6050_hal_write_reg(soft_iic_info_struct *iic, uint8_t reg, uint8_t value);
uint8_t dt_mpu6050_hal_write_regs(soft_iic_info_struct *iic, uint8_t reg, const uint8_t *data, uint16_t len);
uint8_t dt_mpu6050_hal_read_reg(soft_iic_info_struct *iic, uint8_t reg, uint8_t *value);
uint8_t dt_mpu6050_hal_read_regs(soft_iic_info_struct *iic, uint8_t reg, uint8_t *data, uint16_t len);

// 高级接口
uint8_t dt_mpu6050_init(dt_mpu6050_config_t *cfg);
uint8_t dt_mpu6050_read_all(dt_mpu6050_config_t *cfg, dt_mpu6050_data_t *data);
```

**使用**:
```c
dt_mpu6050_config_t mpu = { .accel_fs = DT_MPU6050_ACCEL_FS_8G, .gyro_fs = DT_MPU6050_GYRO_FS_2000 };
soft_iic_init(&mpu.iic, 0x68, 10, MPU6050_SCL, MPU6050_SDA);
dt_mpu6050_init(&mpu);

dt_mpu6050_data_t data;
dt_mpu6050_read_all(&mpu, &data);
// data.gz = Z轴角速度 (°/s), data.ay = Y轴加速度 (g)
```

---

### 4.4 航向传感器 — dt_heading / dt_mpu6050_heading

**文件**: [user/inc/driver/dt_heading.h](user/inc/driver/dt_heading.h)

> 统一的航向传感器抽象，自动按 CAR_GYRO_SOURCE 选择 M0 单轴或 MPU6050。

```c
typedef enum {
    DT_HEADING_SOURCE_M0 = 1, DT_HEADING_SOURCE_MPU6050 = 2
} dt_heading_source_t;

typedef enum {
    DT_HEADING_STATUS_UNINITIALIZED, DT_HEADING_STATUS_WAITING_DATA,
    DT_HEADING_STATUS_READY, DT_HEADING_STATUS_BUS_ERROR,
    DT_HEADING_STATUS_ID_ERROR, DT_HEADING_STATUS_CONFIG_ERROR,
    DT_HEADING_STATUS_CALIBRATION_ERROR,
} dt_heading_status_t;

typedef struct {
    dt_heading_source_t source;
    dt_heading_status_t status;
    float yaw_deg, wz_dps;                  // 航向角/角速度
    uint32_t last_update_ms;
    uint32_t read_error_count, checksum_error_count, rx_overflow;
    dt_mpu6050_heading_t mpu6050;           // MPU6050 子结构 (source=MPU6050时)
    // ... 更多诊断字段
} dt_heading_t;

bool dt_heading_init(dt_heading_t *heading);
bool dt_heading_update(dt_heading_t *heading, uint32_t now_ms);
void dt_heading_zero(dt_heading_t *heading);          // 当前航向归零
bool dt_heading_is_fresh(const dt_heading_t *heading, uint32_t now_ms, uint32_t timeout_ms);
```

---

### 4.5 单轴陀螺仪 — dt_gyro_z

**文件**: [user/inc/driver/dt_gyro_z.h](user/inc/driver/dt_gyro_z.h)

> M0 单轴 Z 陀螺仪，支持 UART 和 I2C。

```c
typedef struct {
    uart_index_enum uart; uint32_t baud;
    uart_tx_pin_enum tx_pin; uart_rx_pin_enum rx_pin;
} dt_gyro_z_config_t;

typedef struct {
    float yaw_deg, wz_dps;
    int16_t yaw_raw, wz_raw;
    uint32_t frame_count, checksum_error_count, rx_overflow;
} dt_gyro_z_data_t;

void dt_gyro_z_init(const dt_gyro_z_config_t *config);
uint8_t dt_gyro_z_update(uint32_t now_ms);
const dt_gyro_z_data_t *dt_gyro_z_get_data(void);
float dt_gyro_z_get_yaw(void);
float dt_gyro_z_get_wz(void);
void dt_gyro_z_zero_yaw(void);
void dt_gyro_z_start_bias_cal(void);
```

---

### 4.6 Mahony 姿态 — dt_imu_mahony

**文件**: [user/inc/driver/dt_imu_mahony.h](user/inc/driver/dt_imu_mahony.h)

> Mahony 互补滤波器，从 MPU6050 九轴数据解算姿态欧拉角。

```c
typedef struct {
    dt_mpu6050_config_t mpu;
    uint8_t ready;
    float q0, q1, q2, q3;               // 四元数
    float gbias_x, gbias_y, gbias_z;     // 陀螺仪零偏
    float roll, pitch, yaw;             // 欧拉角 (°)
} dt_imu_mahony_t;

uint8_t dt_imu_mahony_init(dt_imu_mahony_t *imu, gpio_pin_enum scl, gpio_pin_enum sda);
void    dt_imu_mahony_update(dt_imu_mahony_t *imu, float dt_s);
void    dt_imu_mahony_zero_yaw(dt_imu_mahony_t *imu);
```

---

### 4.7 OLED (SSD1306 I2C) — dt_oled

**文件**: [user/inc/driver/dt_oled.h](user/inc/driver/dt_oled.h)

> 软件 I2C 驱动的 OLED，带 framebuffer 和脏页标记，支持增量刷新。

```c
#define DT_OLED_DEFAULT_ADDR  0x3C
#define DT_OLED_WIDTH         128
#define DT_OLED_HEIGHT        64

typedef struct {
    soft_iic_info_struct iic;
    uint8_t framebuffer[8][128];         // 8页 × 128列
    uint8_t dirty_pages;                 // 脏页位掩码
} dt_oled_config_t;

// API
void dt_oled_init(dt_oled_config_t *cfg);
void dt_oled_clear(dt_oled_config_t *cfg);
void dt_oled_fill(dt_oled_config_t *cfg, uint8_t data);
void dt_oled_show_char(dt_oled_config_t *cfg, uint8_t x, uint8_t y, char ch);
void dt_oled_show_string(dt_oled_config_t *cfg, uint8_t x, uint8_t y, const char *str);
void dt_oled_show_num(dt_oled_config_t *cfg, uint8_t x, uint8_t y, int32_t num, uint8_t len);
void dt_oled_show_hex(dt_oled_config_t *cfg, uint8_t x, uint8_t y, uint32_t num, uint8_t len);
void dt_oled_show_float(dt_oled_config_t *cfg, uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len);
void dt_oled_mark_page_dirty(dt_oled_config_t *cfg, uint8_t page);
void dt_oled_refresh_page(dt_oled_config_t *cfg, uint8_t page);
void dt_oled_refresh_one_dirty(dt_oled_config_t *cfg);   // 只刷新一个脏页
void dt_oled_refresh_dirty(dt_oled_config_t *cfg);       // 刷新全部脏页
void dt_oled_refresh_task(uint32_t now_ms, void *context); // 调度器任务包装
```

**使用**:
```c
dt_oled_config_t oled;
soft_iic_init(&oled.iic, 0x3C, 10, OLED_SCL, OLED_SDA);
dt_oled_init(&oled);
dt_oled_show_string(&oled, 0, 0, "Hello");
dt_oled_refresh_dirty(&oled);     // 只写脏页到OLED，支持增量刷新
```

---

### 4.8 LED — dt_led

**文件**: [user/inc/driver/dt_led.h](user/inc/driver/dt_led.h)

```c
typedef struct {
    gpio_pin_enum pin;
    gpio_level_enum on_level, off_level;
    bool is_on;
} dt_led_t;

void dt_led_init(dt_led_t *led);
void dt_led_set(dt_led_t *led, bool on);
void dt_led_on/dt_led_off/dt_led_toggle(dt_led_t *led);
bool dt_led_is_on(const dt_led_t *led);
```

---

### 4.9 RGB 灯 — dt_rgb

**文件**: [user/inc/driver/dt_rgb.h](user/inc/driver/dt_rgb.h)

```c
typedef enum {
    DT_RGB_COLOR_OFF     = 0x00,
    DT_RGB_COLOR_RED     = 0x01,
    DT_RGB_COLOR_GREEN   = 0x02,
    DT_RGB_COLOR_BLUE    = 0x04,
    DT_RGB_COLOR_YELLOW  = RED|GREEN,
    DT_RGB_COLOR_MAGENTA = RED|BLUE,
    DT_RGB_COLOR_CYAN    = GREEN|BLUE,
    DT_RGB_COLOR_WHITE   = RED|GREEN|BLUE,
} dt_rgb_color_t;

typedef struct {
    gpio_pin_enum red_pin, green_pin, blue_pin;
    gpio_level_enum on_level, off_level;
    dt_rgb_color_t color;
} dt_rgb_t;

void dt_rgb_init(dt_rgb_t *rgb);
void dt_rgb_set_color(dt_rgb_t *rgb, dt_rgb_color_t color);
void dt_rgb_off(dt_rgb_t *rgb);
```

---

### 4.10 蜂鸣器 — dt_buzzer

**文件**: [user/inc/driver/dt_buzzer.h](user/inc/driver/dt_buzzer.h)

> 支持单次鸣叫、异步定时鸣叫和音序播放。

```c
typedef struct {
    bool on; uint32_t duration_ms;
} dt_buzzer_step_t;

typedef struct {
    gpio_pin_enum pin;
    const dt_buzzer_step_t *sequence;    // 当前音序
    uint32_t deadline_ms;                // 序列截止时间
    uint8_t sequence_length, sequence_index;
    bool service_active;
} dt_buzzer_config_t;

void dt_buzzer_init(dt_buzzer_config_t *cfg);
void dt_buzzer_on/dt_buzzer_off(dt_buzzer_config_t *cfg);
void dt_buzzer_beep(dt_buzzer_config_t *cfg, uint32_t duration_ms);
void dt_buzzer_beep_async(dt_buzzer_config_t *cfg, uint32_t duration_ms, uint32_t now_ms);
void dt_buzzer_play_sequence(dt_buzzer_config_t *cfg, const dt_buzzer_step_t *seq, uint8_t len, uint32_t now_ms);
void dt_buzzer_service(dt_buzzer_config_t *cfg, uint32_t now_ms);
void dt_buzzer_service_task(uint32_t now_ms, void *context);  // 调度器任务包装
```

---

### 4.11 蓝牙 HC05 — dt_hc05

**文件**: [user/inc/driver/dt_hc05.h](user/inc/driver/dt_hc05.h)

```c
typedef enum {
    DT_HC05_STATUS_DISABLED, DT_HC05_STATUS_IDLE, DT_HC05_STATUS_BUSY,
    DT_HC05_STATUS_READY, DT_HC05_STATUS_ERROR_RESPONSE, DT_HC05_STATUS_ERROR_TIMEOUT,
} dt_hc05_status_t;

void dt_hc05_init(gpio_pin_enum en_pin);
bool dt_hc05_begin(gpio_pin_enum en_pin, uint32_t now_ms);   // 进入 AT 模式
void dt_hc05_update(uint32_t now_ms);                          // 状态机推进
dt_hc05_status_t dt_hc05_get_status(void);
```

---

## 5. 工具库 — lib

### 5.1 PID 控制器 — pid_controller

**文件**: [user/inc/lib/pid_controller.h](user/inc/lib/pid_controller.h)

> 位置式 PID。支持积分限幅、输出限幅、死区、微分低通滤波、积分分离。

```c
typedef struct {
    float kp, ki, kd;               // 增益
    float integral;                 // 积分累加
    float prev_error;               // 上次误差
    float prev_derivative;          // 上次微分 (滤波)
    float output_min, output_max;   // 输出限幅
    float integral_min, integral_max; // 积分限幅
    float deadband;                 // 死区 (|error| ≤ deadband → 输出0)
    float derivative_lpf;           // 微分低通 α (0~1, 1=无滤波)
    bool  has_prev_error;
} PidController;

void  pid_init(PidController *pid);
void  pid_reset(PidController *pid);
void  pid_set_gain(PidController *pid, float kp, float ki, float kd);
void  pid_set_limits(PidController *pid, float out_min, float out_max,
                     float int_min, float int_max);
void  pid_set_deadband(PidController *pid, float deadband);
void  pid_set_derivative_lpf(PidController *pid, float alpha);
float pid_update(PidController *pid, float error, float dt_s);
```

**使用**:
```c
PidController spd;
pid_init(&spd);
pid_set_gain(&spd, 35.0f, 15.0f, 0.0f);
pid_set_limits(&spd, -2000, 2000, -3000, 3000);
pid_set_derivative_lpf(&spd, 0.35f);

float pwm_correction = pid_update(&spd, target_rpm - actual_rpm, 0.01f);
```

---

### 5.2 串口接收环形缓冲区 — serial_rx_buffer

**文件**: [user/inc/lib/serial_rx_buffer.h](user/inc/lib/serial_rx_buffer.h)

> 经典生产者-消费者环形缓冲区。ISR push → 主循环 pop。

```c
typedef struct {
    uint8_t *data;
    size_t capacity;
    volatile size_t head;              // ISR 写位置
    volatile size_t tail;              // 主循环读位置
    volatile size_t overflow_count;    // 溢出计数
    uint32_t *timestamps_ms;           // 可选时间戳数组
} SerialRxBuffer;

void   serial_rx_buffer_init(SerialRxBuffer *buf, uint8_t *storage, size_t capacity);
void   serial_rx_buffer_init_timed(SerialRxBuffer *buf, uint8_t *storage,
                                   uint32_t *ts_storage, size_t capacity);
void   serial_rx_buffer_clear(SerialRxBuffer *buf);
bool   serial_rx_buffer_push(SerialRxBuffer *buf, uint8_t byte);          // ISR 调用
bool   serial_rx_buffer_push_timed(SerialRxBuffer *buf, uint8_t byte, uint32_t rx_time_ms);
bool   serial_rx_buffer_pop(SerialRxBuffer *buf, uint8_t *byte);          // 主循环调用
bool   serial_rx_buffer_pop_timed(SerialRxBuffer *buf, uint8_t *byte, uint32_t *rx_time_ms);
bool   serial_rx_buffer_peek(const SerialRxBuffer *buf, size_t offset, uint8_t *byte);
size_t serial_rx_buffer_available(const SerialRxBuffer *buf);
size_t serial_rx_buffer_overflow_count(const SerialRxBuffer *buf);
void   serial_rx_buffer_drop(SerialRxBuffer *buf, size_t length);
```

---

### 5.3 串口发送环形缓冲区 — serial_tx_buffer

**文件**: [user/inc/lib/serial_tx_buffer.h](user/inc/lib/serial_tx_buffer.h)

> 多生产者单消费者发送缓冲区，ISR 从其中取数发送。

```c
typedef struct {
    uint8_t *data; size_t capacity;
    volatile size_t head, tail;
    volatile size_t rejected_write_count;
    volatile size_t dropped_byte_count;
    volatile size_t high_watermark;
} SerialTxBuffer;

void   serial_tx_buffer_init(SerialTxBuffer *buf, uint8_t *storage, size_t capacity);
bool   serial_tx_buffer_write(SerialTxBuffer *buf, const uint8_t *data, size_t length);
bool   serial_tx_buffer_peek(const SerialTxBuffer *buf, uint8_t *byte);
bool   serial_tx_buffer_pop(SerialTxBuffer *buf);
size_t serial_tx_buffer_available(const SerialTxBuffer *buf);
size_t serial_tx_buffer_free(const SerialTxBuffer *buf);
```

---

### 5.4 通用工具 — car_utils

**文件**: [user/inc/lib/car_utils.h](user/inc/lib/car_utils.h)

```c
// 二进制打包器 (VOFA 等协议)
typedef struct { uint8_t data[120]; uint8_t length; bool valid; } car_binary_writer_t;
void car_binary_init(car_binary_writer_t *w);
void car_binary_u8/w/u16/i8/i16/i32/w/i32(car_binary_writer_t *w, ...);

// 数学工具
float   car_absf(float value);
float   car_forward_floor(float reference, float minimum);  // 前向最小值
float   car_clampf(float value, float minimum, float maximum);
int32_t car_scale_float(float value, float scale);          // value * scale → int32
float   car_wrap_heading(float heading_deg);                // 航向角 [-180, +180) 归一化
```

---

## 6. 设备驱动 — device

### 6.1 T8 灰度传感器 — t8_gray_sensor

**文件**: [user/inc/device/t8_gray_sensor.h](user/inc/device/t8_gray_sensor.h)

> 8路灰度循迹传感器，支持 UART 和 I2C 双模式。协议完整，含校验和超时重试。

```c
#define T8_SENSOR_COUNT  8u

// 状态码
typedef enum {
    T8_OK, T8_ERROR, T8_ERROR_INVALID_ARG, T8_ERROR_IO,
    T8_ERROR_TIMEOUT, T8_ERROR_BAD_FRAME, T8_ERROR_CHECKSUM,
} T8Status;

// 协议命令
typedef enum {
    T8_CMD_GRAY8_ALL = 0x09, T8_CMD_ADC16_ALL = 0x19,
    T8_CMD_DIGITAL = 0x0C, T8_CMD_VERSION = 0xAE, ...
} T8Command;

// 传输层抽象 (回调注入)
typedef size_t (*T8UartWriteFn)(const uint8_t *data, size_t length, void *user_data);
typedef size_t (*T8UartReadFn)(uint8_t *data, size_t length, uint32_t timeout_ms, void *user_data);
typedef void   (*T8UartFlushFn)(void *user_data);

// UART 模式 API
T8Status t8_uart_get_gray8_all(T8UartDevice *dev, uint8_t values[8]);
T8Status t8_uart_get_adc16_all(T8UartDevice *dev, uint16_t values[8]);
T8Status t8_uart_get_digital(T8UartDevice *dev, uint8_t *bits);
T8Status t8_uart_start_continuous(T8UartDevice *dev, uint8_t cmd, uint8_t period); // 连续采集
T8Status t8_uart_stop_continuous(T8UartDevice *dev);

// I2C 模式 API (相同功能，接口不同)
T8Status t8_i2c_get_gray8_all(T8I2cDevice *dev, uint8_t values[8]);

// 位置计算
void t8_compute_position(const uint8_t gray[8], uint8_t *bits_out, float *pos_out);
// pos_out: -7.0 ~ +7.0 (归一化线位置，0=正中)
```

---

## 7. 通信协议 — protocol

### 7.1 VOFA+ 数据可视化 — vofa

**文件**: [user/inc/protocol/vofa.h](user/inc/protocol/vofa.h)

> VOFA+ JustFloat 协议，发送 float 数组到上位机绘图。

```c
#define VOFA_JUSTFLOAT_MAX_CHANNELS  32u

typedef bool (*VofaWriteCallback)(const uint8_t *data, size_t length, void *context);

typedef struct {
    VofaWriteCallback write;
    void *context;
} VofaTransport;

bool vofa_send(const VofaTransport *transport, const float *data, uint8_t count);
```

**使用**:
```c
float vofa_buf[4] = {rpm_L, rpm_R, yaw_deg, battery_v};
vofa_send(&vofa_transport, vofa_buf, 4);
```

---

## 8. 应用层 — app

### 8.1 循迹小车 — line_car

**文件**: [user/inc/app/line_car.h](user/inc/app/line_car.h)

> 循迹小车的主控制器。注册到调度器的任务族。

```c
// 调度器任务 (由 ec_scheduler 管理)
void line_car_input_task(uint32_t now_ms, void *context);      // 按键处理
void line_car_gyro_task(uint32_t now_ms, void *context);       // 航向更新
void line_car_line_sensor_task(uint32_t now_ms, void *context); // T8 传感器
void line_car_sensor_task(uint32_t now_ms, void *context);     // 综合传感器
void line_car_control_task(uint32_t now_ms, void *context);    // PID 控制
void line_car_menu_task(uint32_t now_ms, void *context);       // OLED 菜单
void line_car_buzzer_task(uint32_t now_ms, void *context);     // 蜂鸣器
void line_car_oled_task(uint32_t now_ms, void *context);       // OLED 刷新
void line_car_telemetry_task(uint32_t now_ms, void *context);  // VOFA 遥测
void line_car_debug_task(uint32_t now_ms, void *context);      // 诊断日志
void line_car_tune_task(uint32_t now_ms, void *context);       // 在线调参

// 控制接口
void line_car_init(void);
bool car_prepare(uint32_t now_ms);
void car_reset_odometry(void);
bool car_read_state(h2024_vehicle_state_t *state, uint32_t now_ms);
void car_drive_heading(float heading_deg, uint32_t now_ms);     // 定航向行驶
void car_follow_line(uint32_t now_ms);                          // 循线行驶
bool car_align_heading(float heading_deg, uint32_t now_ms);     // 航向对齐
void car_signal_point(uint32_t now_ms);                         // 途经点信号
void car_stop(uint32_t now_ms);
void line_car_emergency_stop(void);                             // 紧急停止
```

---

### 8.2 竞赛任务 — h2024_tasks

**文件**: [user/inc/app/h2024_tasks.h](user/inc/app/h2024_tasks.h)

> 2024 竞速组赛道任务状态机（入环岛/出环岛/十字路口）。

```c
typedef enum {
    H2024_TASK_1, H2024_TASK_2, H2024_TASK_3, H2024_TASK_4, H2024_TASK_COUNT
} h2024_task_id_t;

typedef enum {
    H2024_STATE_STOPPED, H2024_STATE_INIT_TURN, H2024_STATE_LEAVE_A,
    H2024_STATE_FIRST_STRAIGHT, H2024_STATE_PRE_FIRST_ARC,
    H2024_STATE_FIRST_ARC, ..., H2024_STATE_DONE, H2024_STATE_TIMEOUT, H2024_STATE_FAULT
} h2024_task_state_t;

typedef struct {
    float heading_deg;
    uint32_t line_enter_count, line_exit_count;
    bool on_line;
    float travel_mm;
} h2024_vehicle_state_t;

bool h2024_tasks_register(ec_mode_manager_t *manager);  // 注册为调度模式
bool h2024_tasks_is_active(const ec_mode_manager_t *manager);
h2024_task_state_t h2024_tasks_active_state(const ec_mode_manager_t *manager);
const char *h2024_tasks_active_status(const ec_mode_manager_t *manager);
```

**默认参数**:
```c
#define H2024_DIAGONAL_TURN_DEG   38.0f
#define H2024_ARC_MIN_DURATION_MS 800u
#define H2024_ALIGN_TOLERANCE_DEG  3.0f
#define H2024_POINT_SIGNAL_MS     250u
```

---

### 8.3 硬件测试 — hardware_test

**文件**: [user/inc/app/hardware_test.h](user/inc/app/hardware_test.h)

> 硬件测试模式，支持逐个外设测试和云台调试。

```c
void hardware_test_init(void);
void hardware_test_run(void);
void hardware_test_set_gimbal_control_mode(GimbalControlMode mode);
GimbalControlMode hardware_test_get_gimbal_control_mode(void);
GimbalStatus hardware_test_emergency_stop(void);
GimbalStatus hardware_test_rearm(void);
bool hardware_test_is_gimbal_armed(void);
bool hardware_test_is_emergency_latched(void);
```

---

### 8.4 电机测试 — motor_test

**文件**: [user/inc/app/motor_test.h](user/inc/app/motor_test.h)

> 电机死区自动测试：逐步增加 PWM 直到电机转动，记录死区值。

```c
typedef void (*motor_test_apply_fn)(int16_t left, int16_t right, uint32_t now_ms, void *ctx);
typedef uint32_t (*motor_test_edges_fn)(void *ctx);

typedef struct {
    motor_test_apply_fn apply;           // 电机驱动回调
    motor_test_edges_fn left_edges;      // 左编码器读边沿
    motor_test_edges_fn right_edges;
    int16_t deadzone_start_pwm, deadzone_step_pwm;
    uint32_t deadzone_step_ms, deadzone_pause_ms;
    uint8_t  deadzone_edge_count;
    // ...
} motor_test_config_t;

typedef struct {
    motor_test_state_t state;           // IDLE → RAW → BOOST → DEADZONE → DONE
    int16_t left_threshold, right_threshold; // ← 测试结果
} motor_test_t;

bool motor_test_init(motor_test_t *test, const motor_test_config_t *config);
bool motor_test_start_raw_left/right(motor_test_t *test, uint32_t now_ms);
bool motor_test_start_boost_left/right(motor_test_t *test, uint32_t now_ms);
bool motor_test_start_deadzone(motor_test_t *test, uint32_t now_ms);
void motor_test_update(motor_test_t *test, uint32_t now_ms);
void motor_test_stop(motor_test_t *test, uint32_t now_ms);
```

---

### 8.5 电池补偿 — battery_compensation

**文件**: [user/inc/app/battery_compensation.h](user/inc/app/battery_compensation.h)

> 电池电压一阶滤波 + 欠压保护（带迟滞）+ PWM 补偿。

```c
typedef enum {
    BATTERY_COMP_STARTUP, BATTERY_COMP_OK, BATTERY_COMP_INVALID, BATTERY_COMP_UNDERVOLTAGE,
} battery_compensation_status_t;

typedef struct {
    uint16_t reference_mv;              // 参考电压 (14.8V)
    uint16_t cutoff_mv;                 // 欠压阈值 (12V)
    uint16_t recovery_mv;               // 恢复阈值 (13V)
    float minimum_factor, maximum_factor; // 补偿系数范围 [0.85, 1.30]
    float filter_alpha;                 // 滤波系数
    uint8_t fault_samples, recovery_samples; // 消抖样本数
} battery_compensation_config_t;

typedef struct {
    battery_compensation_config_t config;
    float filtered_mv, factor;
    battery_compensation_status_t status;
} battery_compensation_t;

void  battery_compensation_init(battery_compensation_t *comp, const battery_compensation_config_t *cfg);
void  battery_compensation_update(battery_compensation_t *comp, uint16_t sample_mv, bool valid);
float battery_compensation_factor(const battery_compensation_t *comp);
float battery_compensation_voltage_mv(const battery_compensation_t *comp);
float battery_compensation_apply(const battery_compensation_t *comp, float ref_pwm, float max_pwm);
```

**公式**: 补偿后 PWM = 参考PWM × (参考电压 / 当前电压) × 补偿系数。

---

### 8.6 参数调校 — car_tuning

**文件**: [user/inc/app/car_tuning.h](user/inc/app/car_tuning.h)

> 运行时在线调参的数据容器，所有 PID/前馈参数均可通过菜单修改。

```c
typedef struct {
    float target_rpm, feedforward_gain;
    int16_t base_pwm;
    float speed_kp, speed_ki, speed_kd;
    float line_kp, line_kd;
    int8_t  line_steer_sign, heading_steer_sign;
    float heading_kp, heading_kd, heading_max_steer;
    float left_gain, right_gain;
    bool speed_loop_enabled;
} car_tuning_t;

void car_tuning_defaults(car_tuning_t *tuning);
int16_t car_tuning_feedforward_pwm(const car_tuning_t *tuning);
int16_t car_tuning_feedforward_pwm_for_rpm(const car_tuning_t *tuning, float target_rpm);
```

---

### 8.7 巡线事件检测 — line_event_detector

**文件**: [user/inc/app/line_event_detector.h](user/inc/app/line_event_detector.h)

> 循迹线进入/离开事件的消抖检测器。

```c
typedef struct {
    bool initialized, stable_on_line;
    uint8_t debounce_samples, candidate_samples;
    uint32_t enter_count, exit_count;
    uint32_t last_transition_ms;
} line_event_detector_t;

void line_event_detector_init(line_event_detector_t *det, uint8_t debounce_samples);
void line_event_detector_update(line_event_detector_t *det, bool raw_on_line, uint32_t now_ms);
```

---

## 9. 云台控制 — gimbal

### 9.1 EMM 步进电机 — emm_stepper

**文件**: [user/inc/gimbal/emm_stepper.h](user/inc/gimbal/emm_stepper.h)

> EMM 步进电机伺服驱动协议库。完整的全双工半双工协议栈，~67 个 API。

```c
// 传输层注入 (依赖反转)
typedef size_t (*EmmWriteFn)(const uint8_t *data, size_t length, void *user_data);
typedef size_t (*EmmReadFn)(uint8_t *data, size_t length, uint32_t timeout_ms, void *user_data);
typedef void   (*EmmFlushFn)(void *user_data);
typedef void   (*EmmDelayFn)(uint32_t delay_ms, void *user_data);

typedef struct {
    EmmWriteFn write; EmmReadFn read;
    EmmFlushFn flush_input, flush_output;
    EmmDelayFn delay_ms;
    void *user_data;
} EmmTransport;

typedef struct {
    uint8_t address; EmmChecksumMode checksum_mode;
    uint32_t timeout_ms; uint8_t max_retries;
    EmmTransport transport;
    uint8_t rx_buffer[256];
    // ...
} EmmDevice;

// 常用 API
void emm_init(EmmDevice *dev, const EmmTransport *transport, uint8_t address);
EmmStatus emm_enable(EmmDevice *dev, bool enable, EmmSyncFlag sync);
EmmStatus emm_jog(EmmDevice *dev, const EmmJogParams *params);          // JOG 速度模式
EmmStatus emm_move_pulses(EmmDevice *dev, const EmmPositionParams *params);
EmmStatus emm_move_degrees(EmmDevice *dev, float deg, uint16_t rpm, uint8_t acc, ...);
EmmStatus emm_stop(EmmDevice *dev, EmmSyncFlag sync);
EmmStatus emm_home(EmmDevice *dev, EmmHomingMode mode, EmmSyncFlag sync);
EmmStatus emm_zero_position(EmmDevice *dev);                            // 当前位置设为零
EmmStatus emm_get_encoder(EmmDevice *dev, uint16_t *encoder);
EmmStatus emm_get_encoder_degrees(EmmDevice *dev, float *degrees);
EmmStatus emm_get_realtime_position(EmmDevice *dev, float *degrees);
EmmStatus emm_get_realtime_speed(EmmDevice *dev, int16_t *speed_rpm);
EmmStatus emm_get_system_status(EmmDevice *dev, EmmSystemStatusParams *params);
EmmStatus emm_calibrate_encoder(EmmDevice *dev);
EmmStatus emm_clear_stall_and_recover(EmmDevice *dev);                  // 清除堵转并恢复
// ... 总共约67个协议操作
```

---

### 9.2 云台主控 — gimbal

**文件**: [user/inc/gimbal/gimbal.h](user/inc/gimbal/gimbal.h)

> 双轴云台控制器：管理 pitch/yaw 两个 EMM 电机。

```c
typedef enum {
    GIMBAL_OK, GIMBAL_ERROR, GIMBAL_ERROR_MOTOR,
    GIMBAL_ERROR_SENSOR, GIMBAL_ERROR_CALIB,
    GIMBAL_ERROR_NOT_HOMED, GIMBAL_ERROR_SAFETY_LATCHED,
} GimbalStatus;

typedef struct {
    EmmDevice yaw, pitch;              // 两个 EMM 电机
    float yaw_angle_deg, pitch_angle_deg;
    float yaw_commanded_deg, pitch_commanded_deg;
    bool homed, position_valid, safety_fault_latched;
    // ... 限位/校准数据
} Gimbal;

extern Gimbal g_gimbal;    // 全局单例

GimbalStatus gimbal_init(Gimbal *gimbal);
GimbalStatus gimbal_enable(Gimbal *gimbal, bool enable);
GimbalStatus gimbal_move_to(Gimbal *gimbal, float yaw_deg, float pitch_deg);    // 绝对角度
GimbalStatus gimbal_move_relative(Gimbal *gimbal, float yaw_delta, float pitch_delta);
GimbalStatus gimbal_auto_calibrate(Gimbal *gimbal);
GimbalStatus gimbal_read_actual_position(Gimbal *gimbal, float *yaw, float *pitch);
GimbalStatus gimbal_clear_safety_fault(Gimbal *gimbal);
```

---

### 9.3 MaixCam2 视觉 — maixcam2_protocol

**文件**: [user/inc/gimbal/maixcam2_protocol.h](user/inc/gimbal/maixcam2_protocol.h)

> K210/MaixCam2 视觉模块通信协议。接收目标坐标。

```c
typedef enum {
    VISION_STATE_IDLE, VISION_STATE_SEARCHING, VISION_STATE_CANDIDATE,
    VISION_STATE_LOCKED, VISION_STATE_TRACKING, VISION_STATE_LOST,
} VisionState;

typedef struct {
    uint8_t  target_valid;
    int16_t  error_x, error_y;             // 目标相对画面中心的像素误差
    uint8_t  vision_state;                 // 视觉跟踪状态
} MaixVisionTarget;

void maixcam2_init(void);
void maixcam2_update(uint32_t now_ms);
bool maixcam2_get_latest_target(MaixVisionTarget *target, uint32_t *rx_time_ms);
bool maixcam2_target_semantically_valid(const MaixVisionTarget *target);
const MaixProtocolStats *maixcam2_get_stats(void);   // 帧/CRC/溢出统计
```

---

### 9.4 位置控制 — position_control

**文件**: [user/inc/gimbal/position_control.h](user/inc/gimbal/position_control.h)

> 基于视觉误差的位置闭环云台控制器。

```c
typedef struct {
    float kp, ki, kd;
    float max_delta_deg;                 // 单次最大步进
    float max_output_dps;                // 输出角速度上限
    float deadband_px;                   // 像素死区
    int16_t large_error_px;              // 大误差阈值
    uint32_t vision_timeout_ms, lost_hold_ms;
} PositionControlConfig;

typedef struct {
    VisionControlState state;
    PidController yaw_pid, pitch_pid;
    uint32_t last_update_ms;
    bool failsafe_latched;
} PositionController;

void position_control_init(PositionController *ctrl);
void position_control_update(PositionController *ctrl, Gimbal *gimbal, uint32_t now_ms);
VisionControlState position_control_get_state(const PositionController *ctrl);
```

---

### 9.5 速度控制 — speed_control

**文件**: [user/inc/gimbal/speed_control.h](user/inc/gimbal/speed_control.h)

> 基于视觉误差的速度闭环云台控制器（更平滑的跟踪）。

```c
typedef struct {
    float kp, ki, kd;
    float stiction_comp_rpm;             // 静摩擦补偿
    uint16_t max_motor_rpm;
    uint32_t vision_timeout_ms, feedback_period_ms;
    // ...
} SpeedControlConfig;

typedef struct {
    VisionControlState state;
    PidController yaw_pid, pitch_pid;
    uint16_t last_yaw_motor_rpm, last_pitch_motor_rpm;
    bool failsafe_latched;
} SpeedController;

void speed_control_init(SpeedController *ctrl);
void speed_control_update(SpeedController *ctrl, Gimbal *gimbal, uint32_t now_ms);
void speed_control_stop(SpeedController *ctrl, Gimbal *gimbal);
```

---

## 10. 其他 — isr.h / zf_ccs_compat.h

### isr.h

```c
typedef struct {
    uint32_t uart_irq_count[4];          // 各 UART 中断次数
    uint32_t uart_drain_limit_hits[4];   // UART FIFO 溢流次数
    uint32_t gpio_irq_count;             // GPIO 中断总次数
    uint32_t gpio_event_count;           // GPIO 事件次数
} isr_diagnostics_t;

void isr_get_diagnostics(isr_diagnostics_t *diag);
```

### zf_ccs_compat.h

CCS IDE 兼容层 — 声明逐飞库在 TI SDK 中未暴露的外设函数：
```c
extern void DL_Timer_Count_CCP(GPTIMER_Regs *gptimer);  // 自定义 CCP 计数模式
```

---

## 11. 从底层到用户层的典型数据流

以**速度闭环控制**为例，展示逐飞底层库 → 用户层的完整数据流：

```
┌─ 硬件层 ─────────────────────────────────────────────────────────┐
│  TIMG8 QEI 编码器计数 → CPU GPIO 中断                            │
│  TIMA0 PWM → H桥 → 直流电机转动                                  │
│  SysTick → 1ms 时基                                              │
└──────────────────────────────────────────────────────────────────┘
         ↓
┌─ 逐飞底层 (SeekFree/) ──────────────────────────────────────────┐
│  encoder_quad_init(TIM_G8, ...)      ← 配置硬件 QEI              │
│  pwm_init(PWM_TIM_A0_CH0_A8, ...)    ← 配置 PWM                  │
│  pit_ms_init(PIT_TIM_G6, 10, isr)    ← 10ms 中断                 │
│  encoder_get_delta(TIM_G8)           ← 读脉冲增量 (16bit)        │
│  pwm_set_duty(...)                   ← 写占空比                  │
└──────────────────────────────────────────────────────────────────┘
         ↓
┌─ 用户驱动层 (user/driver/) ─────────────────────────────────────┐
│  dt_encoder_init(&enc1)              ← 封装 QEI 和 GPIO 两种方式 │
│  dt_encoder_compute_signed_rpm(&enc, 10) ← 脉冲→RPM 转换+低通    │
│  dt_motor_init(&left_motor)          ← 双路 PWM 封装             │
│  dt_motor_set_speed(&left, speed)    ← 带符号→IN1/IN2 占空比     │
└──────────────────────────────────────────────────────────────────┘
         ↓
┌─ 用户框架/工具层 ───────────────────────────────────────────────┐
│  pid_set_gain(&speed_pid, 35, 15, 0) ← 配置速度环 PID           │
│  pid_update(&speed_pid, err, 0.01)   ← 计算 PWM 修正量          │
│  ec_scheduler_add("control", cb, 10) ← 注册 10ms 控制任务       │
└──────────────────────────────────────────────────────────────────┘
         ↓
┌─ 应用层 (user/app/) ────────────────────────────────────────────┐
│  line_car_control_task():                                        │
│    float rpm = dt_encoder_compute_signed_rpm(&enc, 10);          │
│    float correction = pid_update(&pid, target - rpm, 0.01);      │
│    float pwm = feedforward + correction * battery_factor;        │
│    dt_motor_set_speed(&motor, pwm);                              │
└──────────────────────────────────────────────────────────────────┘
```

**分层原则**: 每一层只依赖下层，不"越级调用"。用户层代码不应直接操作 TI 寄存器。

---

> **版本**: v2.0 (解耦版) | **生成日期**: 2026-07-20  
> **依赖文档**: [ZF_SEEKFREE_API_MANUAL.md](ZF_SEEKFREE_API_MANUAL.md) — 逐飞 SeekFree 底层库 API 手册
