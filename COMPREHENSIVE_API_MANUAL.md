# MSPM0G3507 ZF 开源库 — 完整 API 参考手册 (已拆分)

> ⚠️ **本文件已拆分为两个独立手册，请使用新文档：**

| 手册 | 文件 | 覆盖范围 |
|------|------|---------|
| **逐飞 SeekFree 底层库 API** | [ZF_SEEKFREE_API_MANUAL.md](ZF_SEEKFREE_API_MANUAL.md) | `SeekFree/zf_common/` `SeekFree/zf_driver/` `SeekFree/zf_device/` `SeekFree/zf_components/` |
| **用户应用层 API** | [ZF_USER_API_MANUAL.md](ZF_USER_API_MANUAL.md) | `user/inc/` `user/src/` — framework, driver, lib, app, gimbal, protocol, device |

> 拆分原因：底层库（逐飞实现）与上层应用（用户代码）的职责和受众不同，解耦后便于独立查阅和维护。

---

<details>
<summary>📜 原始内容（已过期，保留供参考）</summary>

> **平台**: MSPM0G3507 (TI ARM Cortex-M0+)  
> **开源库**: SeekFree (逐飞科技) GPL3.0  
> **生成日期**: 2026-07-20  
> **适用范围**: 天猛星 MSPM0G3507 开发板、智能车竞赛  
> **状态**: ⚠️ 已拆分，请使用上方链接的新文档  

---

## 目录

1. [项目架构总览](#1-项目架构总览)
2. [类型定义层 — zf_common_typedef](#2-类型定义层--zf_common_typedef)
3. [公共工具层 — zf_common](#3-公共工具层--zf_common)
   - [3.1 系统时钟 — zf_common_clock](#31-系统时钟--zf_common_clock)
   - [3.2 调试输出 — zf_common_debug](#32-调试输出--zf_common_debug)
   - [3.3 FIFO 队列 — zf_common_fifo](#33-fifo-队列--zf_common_fifo)
   - [3.4 字体颜色 — zf_common_font](#34-字体颜色--zf_common_font)
   - [3.5 实用函数 — zf_common_function](#35-实用函数--zf_common_function)
   - [3.6 中断控制 — zf_common_interrupt](#36-中断控制--zf_common_interrupt)
   - [3.7 总头文件 — zf_common_headfile](#37-总头文件--zf_common_headfile)
4. [驱动层 — zf_driver](#4-驱动层--zf_driver)
   - [4.1 GPIO — zf_driver_gpio](#41-gpio--zf_driver_gpio)
   - [4.2 PWM — zf_driver_pwm](#42-pwm--zf_driver_pwm)
   - [4.3 UART — zf_driver_uart](#43-uart--zf_driver_uart)
   - [4.4 ADC — zf_driver_adc](#44-adc--zf_driver_adc)
   - [4.5 延时 — zf_driver_delay](#45-延时--zf_driver_delay)
   - [4.6 定时器 — zf_driver_timer](#46-定时器--zf_driver_timer)
   - [4.7 周期中断 PIT — zf_driver_pit](#47-周期中断-pit--zf_driver_pit)
   - [4.8 SPI — zf_driver_spi](#48-spi--zf_driver_spi)
   - [4.9 软件 I2C — zf_driver_soft_iic](#49-软件-i2c--zf_driver_soft_iic)
   - [4.10 外部中断 EXTI — zf_driver_exti](#410-外部中断-exti--zf_driver_exti)
   - [4.11 Flash — zf_driver_flash](#411-flash--zf_driver_flash)
   - [4.12 编码器 — zf_driver_encoder](#412-编码器--zf_driver_encoder)
5. [设备层 — zf_device](#5-设备层--zf_device)
   - [5.1 IMU660RA 六轴传感器](#51-imu660ra-六轴传感器)
   - [5.2 IMU963RA 九轴传感器](#52-imu963ra-九轴传感器)
   - [5.3 OLED 显示屏](#53-oled-显示屏)
   - [5.4 TFT180 彩色屏](#54-tft180-彩色屏)
   - [5.5 IPS200 彩色屏](#55-ips200-彩色屏)
   - [5.6 TSL1401 线阵 CCD](#56-tsl1401-线阵-ccd)
   - [5.7 DL1B 激光测距](#57-dl1b-激光测距)
   - [5.8 GS08RA 灰度传感器](#58-gs08ra-灰度传感器)
   - [5.9 按键管理](#59-按键管理)
   - [5.10 无线串口模块](#510-无线串口模块)
   - [5.11 设备类型管理](#511-设备类型管理)
   - [5.12 其他设备](#512-其他设备)
6. [组件层 — zf_components](#6-组件层--zf_components)
   - [6.1 逐飞助手上位机](#61-逐飞助手上位机)
7. [用户应用层 — user](#7-用户应用层--user)
   - [7.1 系统配置 — config.h](#71-系统配置--configh)
   - [7.2 引脚映射 — pin_mapping.h](#72-引脚映射--pin_mappingh)
   - [7.3 电机驱动 — dt_motor](#73-电机驱动--dt_motor)
   - [7.4 编码器驱动 — dt_encoder](#74-编码器驱动--dt_encoder)
   - [7.5 MPU6050 驱动 — dt_mpu6050](#75-mpu6050-驱动--dt_mpu6050)
   - [7.6 PID 控制器 — pid_controller](#76-pid-控制器--pid_controller)
   - [7.7 协作式调度器 — ec_scheduler](#77-协作式调度器--ec_scheduler)
   - [7.8 串口接收环形缓冲区 — serial_rx_buffer](#78-串口接收环形缓冲区--serial_rx_buffer)
   - [7.9 时间管理 — ec_time](#79-时间管理--ec_time)
   - [7.10 菜单框架 — ec_menu / ec_parameter_menu](#710-菜单框架--ec_menu--ec_parameter_menu)
   - [7.11 模式管理 — ec_mode_manager / ec_app](#711-模式管理--ec_mode_manager--ec_app)
   - [7.12 按键框架 — ec_keys](#712-按键框架--ec_keys)
8. [快速参考速查表](#8-快速参考速查表)
   - [8.1 常用 GPIO 引脚映射](#81-常用-gpio-引脚映射)
   - [8.2 PWM 通道速查](#82-pwm-通道速查)
   - [8.3 UART 引脚速查](#83-uart-引脚速查)
   - [8.4 定时器资源分配](#84-定时器资源分配)
9. [典型应用模板](#9-典型应用模板)

---

## 1. 项目架构总览

```
MSPM0G3507_ZF/
├── SeekFree/                        # 逐飞科技官方开源库
│   ├── zf_common/                   # 公共工具层
│   │   ├── zf_common_typedef.h      # 基础类型定义
│   │   ├── zf_common_clock.h        # 时钟初始化
│   │   ├── zf_common_debug.h        # 调试/断言
│   │   ├── zf_common_fifo.h         # FIFO 队列
│   │   ├── zf_common_font.h         # 字体/颜色定义
│   │   ├── zf_common_function.h     # 通用函数（限幅、字符串转换等）
│   │   ├── zf_common_interrupt.h    # 中断管理
│   │   └── zf_common_headfile.h     # 总头文件（一键引入）
│   ├── zf_driver/                   # 外设驱动层
│   │   ├── zf_driver_gpio.h         # GPIO 控制
│   │   ├── zf_driver_pwm.h          # PWM 输出
│   │   ├── zf_driver_uart.h         # UART 串口
│   │   ├── zf_driver_adc.h          # ADC 模数转换
│   │   ├── zf_driver_delay.h        # 延时函数
│   │   ├── zf_driver_timer.h        # 定时器
│   │   ├── zf_driver_pit.h          # 周期中断定时器
│   │   ├── zf_driver_spi.h          # SPI 总线
│   │   ├── zf_driver_soft_iic.h     # 软件 I2C
│   │   ├── zf_driver_exti.h         # 外部中断
│   │   ├── zf_driver_flash.h        # 内部 Flash
│   │   └── zf_driver_encoder.h      # 编码器
│   ├── zf_device/                   # 外部设备驱动层
│   │   ├── zf_device_imu660ra.h     # 6轴 IMU (SPI/I2C)
│   │   ├── zf_device_imu963ra.h     # 9轴 IMU (SPI/I2C)
│   │   ├── zf_device_oled.h         # OLED 128x64
│   │   ├── zf_device_tft180.h       # TFT 1.8寸彩屏
│   │   ├── zf_device_ips200.h       # IPS 2.0寸彩屏
│   │   ├── zf_device_ips114.h       # IPS 1.14寸彩屏
│   │   ├── zf_device_tsl1401.h      # 线阵 CCD
│   │   ├── zf_device_dl1b.h         # ToF 激光测距
│   │   ├── zf_device_gs08ra.h       # 8路灰度传感器
│   │   ├── zf_device_key.h          # 按键管理
│   │   ├── zf_device_type.h         # 设备类型枚举
│   │   ├── zf_device_wireless_uart.h# 无线串口
│   │   └── ...
│   ├── zf_components/               # 应用组件层
│   │   └── seekfree_assistant.h     # 逐飞助手上位机协议
│   ├── ti_config/                   # TI SDK 配置
│   └── dl_timer.c                   # 死区时间控制
├── user/                            # 用户应用层
│   ├── inc/
│   │   ├── config.h                 # 系统配置
│   │   ├── pin_mapping.h            # 引脚映射
│   │   ├── driver/                  # 用户驱动
│   │   ├── framework/               # 框架层
│   │   ├── lib/                     # 工具库
│   │   ├── app/                     # 应用任务
│   │   └── gimbal/                  # 云台控制
│   └── src/                         # 实现文件
└── Debug/                           # 调试输出
```

**API 调用层次关系**:
```
App Layer (user/app/*.c)
  └─> Framework Layer (user/framework/ec_*.c, user/lib/pid_controller.c)
        └─> User Driver Layer (user/driver/dt_*.c)
              └─> Device Layer (SeekFree/zf_device/zf_device_*.h)
                    └─> Driver Layer (SeekFree/zf_driver/zf_driver_*.h)
                          └─> TI SDK (ti_msp_dl_config.h)
```

---

## 2. 类型定义层 — zf_common_typedef

**文件**: [zf_common_typedef.h](SeekFree/zf_common/zf_common_typedef.h)

### 2.1 基础类型别名

```c
// 无符号类型
typedef unsigned char       uint8;       // 无符号 8 bits
typedef unsigned short int  uint16;      // 无符号 16 bits
typedef unsigned int        uint32;      // 无符号 32 bits
typedef unsigned long long  uint64;      // 无符号 64 bits

// 有符号类型
typedef signed char         int8;        // 有符号 8 bits
typedef signed short int    int16;       // 有符号 16 bits
typedef signed int          int32;       // 有符号 32 bits
typedef signed long long    int64;       // 有符号 64 bits

// volatile 类型
typedef volatile uint8      vuint8;
typedef volatile uint16     vuint16;
typedef volatile uint32     vuint32;
typedef volatile uint64     vuint64;
typedef volatile int8       vint8;
typedef volatile int16      vint16;
typedef volatile int32      vint32;
typedef volatile int64      vint64;
```

可通过 `#define USE_ZF_TYPEDEF 0` 禁用这些类型别名。

### 2.2 通用枚举与宏

```c
// 数据位宽
typedef enum {
    COMMON_DATA_SIZE_8BIT  = 1,
    COMMON_DATA_SIZE_16BIT = 2,
    COMMON_DATA_SIZE_32BIT = 4,
} common_data_size_enum;

// 通用返回值
#define ZF_NO_ERROR     ( 0 )    // 无异常
#define ZF_ERROR        ( 1 )    // 异常码

// 使能/禁止
#define ZF_ENABLE       ( 1 )
#define ZF_DISABLE      ( 0 )

// 布尔值
#define ZF_TRUE         ( 1 )
#define ZF_FALSE        ( 0 )
```

### 2.3 函数指针类型

```c
typedef void   (*void_function_void)(void);
typedef void   (*void_function_uint32)(uint32 parameter);
typedef void   (*void_function_ptr)(void *ptr);
typedef uint32 (*uint32_function_void)(void);
typedef uint32 (*uint32_function_uint32)(uint32 parameter);
typedef uint32 (*uint32_function_ptr)(void *ptr);
typedef void*  (*ptr_function_void)(void);
typedef void*  (*ptr_function_uint32)(uint32 parameter);
typedef void*  (*ptr_function_ptr)(void *ptr);
typedef void   (*void_callback_uint32_ptr)(uint32 state, void *ptr);  // 常用回调类型
```

### 2.4 IDE 适配宏

```c
// IDE 类型检测
#define IDE_MDK  (0x01)
#define IDE_IAR  (0x02)
#define IDE_ADS  (0x04)
#define IDE_MRS  (0x08)

// 内联函数
#define ZF_INLINE   static inline
#define ZF_WEAK     __attribute__((weak))

// 紧凑结构体
#define ZF_PACKED   __attribute__((packed))

// 内存屏障
#define ZF_DSB()    __DSB()
#define ZF_ISB()    __ISB()
#define ZF_DMB()    __DMB()

// 调试定位
#define ZF_FILE_MESSAGE  ( __FILE__ )
#define ZF_LINE_MESSAGE  ( __LINE__ )
```

---

## 3. 公共工具层 — zf_common

### 3.1 系统时钟 — zf_common_clock

**文件**: [zf_common_clock.h](SeekFree/zf_common/zf_common_clock.h)

```c
// 宏定义
#define BOARD_XTAL_FREQ      ( 40000000 )    // 晶振频率 40MHz
#define XTAL_STARTUP_TIMEOUT ( 0x0F00 )      // 晶振就绪超时

// 系统时钟频率枚举
typedef enum {
    SYSTEM_CLOCK_80M = 80000000,             // 80MHz
} system_clock_enum;

// 全局变量 — 系统/总线时钟频率
extern uint32 system_clock;                  // AHB 系统时钟
extern uint32 bus_clock;                     // APB 总线时钟

// 函数
void clock_init(uint32 clock);               // 初始化系统时钟（参数用 SYSTEM_CLOCK_80M）
```

**使用示例**:
```c
clock_init(SYSTEM_CLOCK_80M);  // 系统启动时调用，设置主频 80MHz
```

---

### 3.2 调试输出 — zf_common_debug

**文件**: [zf_common_debug.h](SeekFree/zf_common/zf_common_debug.h)

```c
// 调试串口配置（根据 EC_APP_PROFILE 自动选择）
// Line-car 模式：使用 BLUETOOTH_UART (UART2)
// 其他模式：使用 UART_1
#define DEBUG_UART_BAUDRATE  ( 115200 )

// 断言宏 — 条件为假时输出错误信息和文件行号
#define zf_assert(x)  ( debug_assert_handler((x), __FILE__, __LINE__) )

// 日志输出宏 — 条件为假时输出日志
#define zf_log(x, str)  ( debug_log_handler((x), (str), __FILE__, __LINE__) )

// 调试输出配置结构体
typedef struct {
    uint16 type_index;
    uint16 display_x_max;
    uint16 display_y_max;
    uint8  font_x_size;
    uint8  font_y_size;
    void (*output_uart)(const char *str);
    void (*output_screen)(uint16 x, uint16 y, const char *str);
    void (*output_screen_clear)(void);
} debug_output_struct;

// 环形缓冲区长度
#define DEBUG_RING_BUFFER_LEN  ( 64 )

// 函数
void     debug_init(void);                                  // 初始化调试串口
void     debug_assert_enable(void);                         // 启用断言
void     debug_assert_disable(void);                        // 禁用断言
void     debug_assert_handler(uint8 pass, char *file, int line);  // 断言处理
void     debug_log_handler(uint8 pass, char *str, char *file, int line);
void     debug_output_struct_init(debug_output_struct *info);
void     debug_output_init(debug_output_struct *info);
void     debug_interrupr_handler(void);                     // 中断处理
uint32   debug_send_buffer(const uint8 *buff, uint32 len);  // 发送数据
uint32   debug_read_ring_buffer(uint8 *buff, uint32 len);   // 读取环形缓冲区
```

**使用示例**:
```c
debug_init();                                       // 初始化调试输出
zf_assert(ptr != NULL);                             // 断言检查
zf_log(ret != 0, "UART init failed");               // 日志输出
debug_printf("Value = %d\r\n", value);              // 格式化输出
```

---

### 3.3 FIFO 队列 — zf_common_fifo

**文件**: [zf_common_fifo.h](SeekFree/zf_common/zf_common_fifo.h)

```c
// FIFO 操作状态
typedef enum {
    FIFO_SUCCESS,              // 操作成功
    FIFO_RESET_UNDO,           // 重置未执行
    FIFO_CLEAR_UNDO,           // 清空未执行
    FIFO_BUFFER_NULL,          // 缓冲区异常
    FIFO_WRITE_UNDO,           // 写入未执行
    FIFO_SPACE_NO_ENOUGH,      // 缓冲区空间不足
    FIFO_READ_UNDO,            // 读取未执行
    FIFO_DATA_NO_ENOUGH,       // 数据长度不足
} fifo_state_enum;

// FIFO 执行状态（用于防止中断嵌套冲突）
typedef enum {
    FIFO_IDLE  = 0x00,
    FIFO_RESET = 0x01,         // 正在重置
    FIFO_CLEAR = 0x02,         // 正在清空
    FIFO_WRITE = 0x04,         // 正在写入
    FIFO_READ  = 0x08,         // 正在读取
} fifo_execution_enum;

// 读取模式
typedef enum {
    FIFO_READ_AND_CLEAN,       // 读后清空（释放空间）
    FIFO_READ_ONLY,            // 仅读取（不释放）
} fifo_operation_enum;

// 数据位宽
typedef enum {
    FIFO_DATA_8BIT,
    FIFO_DATA_16BIT,
    FIFO_DATA_32BIT,
} fifo_data_type_enum;

// FIFO 结构体
typedef struct __attribute__((packed)) {
    volatile uint8      execution;    // 执行步骤
    fifo_data_type_enum type;         // 数据类型
    void                *buffer;      // 缓存指针
    volatile uint32     head;         // 头指针（写位置）
    volatile uint32     end;          // 尾指针（读位置）
    volatile uint32     size;         // 剩余大小
    uint32              max;          // 总大小
} fifo_struct;

// 函数
fifo_state_enum fifo_init(fifo_struct *fifo, fifo_data_type_enum type,
                          void *buffer_addr, uint32 size);
fifo_state_enum fifo_clear(fifo_struct *fifo);
uint32          fifo_used(fifo_struct *fifo);               // 查询已用空间
fifo_state_enum fifo_write_element(fifo_struct *fifo, uint32 dat);
fifo_state_enum fifo_write_buffer(fifo_struct *fifo, void *dat, uint32 length);
fifo_state_enum fifo_read_element(fifo_struct *fifo, void *dat, fifo_operation_enum flag);
fifo_state_enum fifo_read_buffer(fifo_struct *fifo, void *dat, uint32 *length, fifo_operation_enum flag);
fifo_state_enum fifo_read_tail_buffer(fifo_struct *fifo, void *dat, uint32 *length, fifo_operation_enum flag);
```

**使用示例**:
```c
static uint8  my_buffer[256];
static fifo_struct my_fifo;

fifo_init(&my_fifo, FIFO_DATA_8BIT, my_buffer, sizeof(my_buffer));
fifo_write_element(&my_fifo, 0x55);                    // 写入单个元素
uint8 data;
uint32 len = 1;
fifo_read_element(&my_fifo, &data, FIFO_READ_AND_CLEAN); // 读取并释放
```

---

### 3.4 字体颜色 — zf_common_font

**文件**: [zf_common_font.h](SeekFree/zf_common/zf_common_font.h)

```c
// RGB565 常用颜色
typedef enum {
    RGB565_WHITE   = 0xFFFF,
    RGB565_BLACK   = 0x0000,
    RGB565_BLUE    = 0x001F,
    RGB565_PURPLE  = 0xF81F,
    RGB565_PINK    = 0xFE19,
    RGB565_RED     = 0xF800,
    RGB565_MAGENTA = 0xF81F,
    RGB565_GREEN   = 0x07E0,
    RGB565_CYAN    = 0x07FF,
    RGB565_YELLOW  = 0xFFE0,
    RGB565_BROWN   = 0xBC40,
    RGB565_GRAY    = 0x8430,
} rgb565_color_enum;

// 外部字体数据
extern const uint8 ascii_font_8x16[][16];       // 8x16 ASCII 字库
extern const uint8 ascii_font_6x8[][6];          // 6x8 ASCII 字库
extern const uint8 oled_16x16_chinese[][16];     // 16x16 中文字库
extern const uint8 gImage_seekfree_logo[38400];  // 逐飞 LOGO (160×160)
```

---

### 3.5 实用函数 — zf_common_function

**文件**: [zf_common_function.h](SeekFree/zf_common/zf_common_function.h)

```c
// 宏定义函数
#define func_abs(x)            ((x) >= 0 ? (x) : -(x))         // 绝对值 [-32767, 32767]
#define func_limit(x, y)       ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))  // 双边限幅
#define func_limit_ab(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))     // 区间限幅

// 常规函数
void     func_get_sin_amplitude_table(uint32 *data_buffer, uint32 sample_max,
                                      uint32 amplitude_max, uint32 offset_degree);
uint32   func_get_greatest_common_divisor(uint32 num1, uint32 num2);
void     func_soft_delay(volatile long t);                    // 软件延时（不精确）

// 字符串转换
int32    func_str_to_int(char *str);
void     func_int_to_str(char *str, int32 number);
uint32   func_str_to_uint(char *str);
void     func_uint_to_str(char *str, uint32 number);
float    func_str_to_float(char *str);
void     func_float_to_str(char *str, float number, uint8 point_bit);
double   func_str_to_double(char *str);
void     func_double_to_str(char *str, double number, uint8 point_bit);
uint32   func_str_to_hex(char *str);
void     func_hex_to_str(char *str, uint32 number);

// 简易 printf
uint32   zf_sprintf(int8 *buff, const int8 *format, ...);
```

---

### 3.6 中断控制 — zf_common_interrupt

**文件**: [zf_common_interrupt.h](SeekFree/zf_common/zf_common_interrupt.h)

```c
void     interrupt_global_enable(uint32 primask);     // 恢复全局中断（传入 interrupt_global_disable 的返回值）
uint32   interrupt_global_disable(void);               // 关闭全局中断并返回之前的状态
void     interrupt_enable(IRQn_Type irqn);             // 使能指定中断
void     interrupt_disable(IRQn_Type irqn);            // 禁止指定中断
void     interrupt_set_priority(IRQn_Type irqn, uint8 priority); // 设置中断优先级
void     interrupt_init(void);                         // 中断初始化
```

**使用示例**:
```c
uint32 primask = interrupt_global_disable();  // 临界区起始
// ... 临界区代码 ...
interrupt_global_enable(primask);             // 临界区结束

interrupt_set_priority(UART2_INT_IRQn, 1);   // 设置 UART2 中断优先级
interrupt_enable(UART2_INT_IRQn);             // 使能 UART2 中断
```

---

### 3.7 总头文件 — zf_common_headfile

**文件**: [zf_common_headfile.h](SeekFree/zf_common/zf_common_headfile.h)

这是一个总入口头文件，`#include "zf_common_headfile.h"` 即可引入整个逐飞库的绝大部分头文件：

```c
#include "zf_ccs_compat.h"      // CCS IDE 兼容层
#include "ti_msp_dl_config.h"   // TI SDK 底层配置
// 公共层
#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_common_font.h"
#include "zf_common_function.h"
#include "zf_common_interrupt.h"
// 驱动层 (全部)
// 设备层 (全部)
// 组件层 (全部)
```

---

## 4. 驱动层 — zf_driver

### 4.1 GPIO — zf_driver_gpio

**文件**: [zf_driver_gpio.h](SeekFree/zf_driver/zf_driver_gpio.h)

#### 引脚枚举

```c
typedef enum {
    A0, A1, A2, A3, A4, A5, A6, A7,
    A8, A9, A10, A11, A12, A13, A14, A15,
    A16, A17, A18, A19, A20, A21, A22, A23,
    A24, A25, A26, A27, A28, A29, A30, A31,

    B0, B1, B2, B3, B4, B5, B6, B7,
    B8, B9, B10, B11, B12, B13, B14, B15,
    B16, B17, B18, B19, B20, B21, B22, B23,
    B24, B25, B26, B27,
    GPIO_MAX,
} gpio_pin_enum;
```

#### 方向与模式枚举

```c
// 输入/输出方向
typedef enum {
    GPI = 0x00,   // 输入
    GPO = 0x10,   // 输出
} gpio_dir_enum;

// 引脚模式
typedef enum {
    GPI_ANAOG_IN     = 0x01,  // 模拟输入
    GPI_FLOATING_IN  = 0x02,  // 浮空输入
    GPI_PULL_DOWN    = 0x03,  // 下拉输入
    GPI_PULL_UP      = 0x04,  // 上拉输入
    GPO_PUSH_PULL    = 0x11,  // 推挽输出
    GPO_OPEN_DTAIN   = 0x12,  // 开漏输出
    GPO_AF_PUSH_PULL = 0x13,  // 复用推挽输出
    GPO_AF_OPEN_DTAIN= 0x14,  // 复用开漏输出
} gpio_mode_enum;

// 复用功能
typedef enum {
    GPIO_AF0 ... GPIO_AF15
} gpio_af_enum;

// 电平
typedef enum {
    GPIO_LOW  = 0x00,
    GPIO_HIGH = 0x01,
} gpio_level_enum;
```

#### GPIO API 函数

```c
// 宏函数 — 快速设置引脚电平（最常用）
#define gpio_high(pin)  (gpio_group[...]->DOUTSET31_0 |= (1 << (pin & 0x1F)))
#define gpio_low(pin)   (gpio_group[...]->DOUTCLR31_0 |= (1 << (pin & 0x1F)))

// 函数
void   gpio_set_level(gpio_pin_enum pin, const uint8 dat);                   // dat=GPIO_HIGH/GPIO_LOW
uint8  gpio_get_level(gpio_pin_enum pin);                                    // 读取引脚电平
void   gpio_toggle_level(gpio_pin_enum pin);                                 // 翻转电平
void   gpio_set_dir(gpio_pin_enum pin, gpio_dir_enum dir, gpio_mode_enum mode); // 设置方向

// 初始化
void   gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, const uint8 dat, gpio_mode_enum mode);
void   afio_init(gpio_pin_enum pin, gpio_dir_enum dir, gpio_af_enum af, gpio_mode_enum mode);
```

**使用示例**:
```c
// 输出模式
gpio_init(A31, GPO, GPIO_HIGH, GPO_PUSH_PULL);    // PA31 推挽输出，初始高电平
gpio_high(A31);                                    // 输出高电平
gpio_low(A31);                                     // 输出低电平
gpio_toggle_level(A31);                            // 翻转

// 输入模式
gpio_init(B6, GPI, 0, GPI_PULL_UP);               // PB6 上拉输入（按键）
uint8 level = gpio_get_level(B6);                  // 读取按键状态
```

**内部实现细节**:  
- 每个 GPIO 引脚编码的低 5 bit 存储引脚号（0-31），第 5 bit 以后存储组号（0=A, 1=B）
- `gpio_high/gpio_low` 直接操作 DOUTSET/DOUTCLR 寄存器，实现原子写
- `gpio_group[]` 数组映射 GPIOA/GPIOB 寄存器基址

---

### 4.2 PWM — zf_driver_pwm

**文件**: [zf_driver_pwm.h](SeekFree/zf_driver/zf_driver_pwm.h)

```c
#define PWM_DUTY_MAX  ( 8000 )    // 最大占空比 (0 ~ 8000)

// PWM 定时器索引
typedef enum {
    PWM_TIM_A0,       // 高级定时器 A0 (4通道)
    PWM_TIM_A1,       // 高级定时器 A1 (2通道)
    PWM_TIM_G0,       // 通用定时器 G0 (2通道)
    PWM_TIM_G6,       // 通用定时器 G6 (2通道)
    PWM_TIM_G7,       // 通用定时器 G7 (2通道)
    PWM_TIM_G8,       // 通用定时器 G8 (2通道)
    PWM_TIM_G12,      // 通用定时器 G12 (2通道)
} pwm_index_enum;

// PWM 通道 (CH0/CH1/CH2/CH3)
typedef enum { PWM_CH0, PWM_CH1, PWM_CH2, PWM_CH3 } pwm_channel_index_enum;

// PWM 引脚枚举 — 编码了 (定时器索引, 通道, 复用功能, GPIO引脚)
// 例如: PWM_TIM_A0_CH0_A8 = TIMA0通道0，引脚PA8，AF5
typedef enum {
    PWM_TIM_A0_CH0_A8,  PWM_TIM_A0_CH0_A21, ...
    PWM_TIM_A0_CH1_A9,  PWM_TIM_A0_CH1_B20, ...
    PWM_TIM_A0_CH2_A15, ...
    PWM_TIM_A0_CH3_A17, ...
    // ... 其余定时器通道类似
} pwm_channel_enum;
```

**PWM API**:
```c
void pwm_set_duty(pwm_channel_enum pin, const uint32 duty);   // 设置占空比 (0~8000)
void pwm_force_low(pwm_channel_enum pin);                      // 强制输出低电平
void pwm_init(pwm_channel_enum pin, const uint32 freq, const uint32 duty); // 初始化
```

**使用示例**:
```c
// 电机PWM初始化
pwm_init(MOTOR_L_IN1, MOTOR_PWM_FREQ, 0);  // PWM_TIM_A0_CH0_A8, 20kHz, 占空比0
pwm_init(MOTOR_L_IN2, MOTOR_PWM_FREQ, 0);  // PWM_TIM_A0_CH1_B20, 20kHz
pwm_init(MOTOR_R_IN1, MOTOR_PWM_FREQ, 0);  // PWM_TIM_A0_CH3_A17, 20kHz
pwm_init(MOTOR_R_IN2, MOTOR_PWM_FREQ, 0);  // PWM_TIM_A0_CH2_A15, 20kHz

// 设置占空比
pwm_set_duty(MOTOR_L_IN1, 5000);  // 50% 占空比
pwm_set_duty(MOTOR_L_IN2, 0);
```

---

### 4.3 UART — zf_driver_uart

**文件**: [zf_driver_uart.h](SeekFree/zf_driver/zf_driver_uart.h)

```c
#define UART_NUM  ( 4 )     // MSPM0G3507 有 4 个 UART

// UART 编号
typedef enum { UART_0, UART_1, UART_2, UART_3 } uart_index_enum;

// TX 引脚
typedef enum {
    UART0_TX_A0,  UART0_TX_A10, UART0_TX_A28, UART0_TX_B0,
    UART1_TX_A8,  UART1_TX_A17, UART1_TX_B4,  UART1_TX_B6,
    UART2_TX_A21, UART2_TX_A23, UART2_TX_B15, UART2_TX_B17,
    UART3_TX_A14, UART3_TX_A26, UART3_TX_B2,  UART3_TX_B12,
} uart_tx_pin_enum;

// RX 引脚
typedef enum {
    UART0_RX_A1,  UART0_RX_A11, UART0_RX_A31, UART0_RX_B1,
    UART1_RX_A9,  UART1_RX_A18, UART1_RX_B5,  UART1_RX_B7,
    UART2_RX_A22, UART2_RX_A24, UART2_RX_B16, UART2_RX_B18,
    UART3_RX_A13, UART3_RX_A25, UART3_RX_B3,  UART3_RX_B13,
} uart_rx_pin_enum;

// 中断配置
typedef enum {
    UART_INTERRUPT_CONFIG_TX_DISABLE,
    UART_INTERRUPT_CONFIG_RX_DISABLE,
    UART_INTERRUPT_CONFIG_ALL_DISABLE,
    UART_INTERRUPT_CONFIG_TX_ENABLE,
    UART_INTERRUPT_CONFIG_RX_ENABLE,
    UART_INTERRUPT_CONFIG_ALL_ENABLE,
} uart_interrupt_config_enum;

// 中断状态
typedef enum {
    UART_INTERRUPT_STATE_NONE = 0x00,
    UART_INTERRUPT_STATE_RX   = 0x01,
    UART_INTERRUPT_STATE_TX   = 0x02,
    UART_INTERRUPT_STATE_ALL  = 0x03,
} uart_interrupt_state_enum;
```

**UART API**:
```c
// 发送
void   uart_write_byte(uart_index_enum uart_index, const uint8 dat);
void   uart_write_buffer(uart_index_enum uart_index, const uint8 *buff, uint32 len);
void   uart_write_string(uart_index_enum uart_index, const char *str);
uint8  uart_try_write_byte(uart_index_enum uart_index, const uint8 data);  // 非阻塞发送

// 接收
uint8  uart_read_byte(uart_index_enum uart_index, uint8 *data);    // 阻塞读取（有超时）
uint8  uart_query_byte(uart_index_enum uart_index, uint8 *data);   // 非阻塞查询

// 中断
void   uart_set_callback(uart_index_enum uart_index, void_callback_uint32_ptr callback, void *ptr);
void   uart_set_interrupt_config(uart_index_enum uart_index, uart_interrupt_config_enum config);

// 初始化
void   uart_init(uart_index_enum uart_index, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);
```

**模板 — UART RX 中断 + 环形缓冲区** (推荐模式):
```c
#include "lib/serial_rx_buffer.h"

static uint8_t rx_storage[512];
static SerialRxBuffer rx_ring;

// 中断回调（在 ISR 中调用）
static void my_uart_isr(uint32_t state, void *ptr) {
    uint8_t data;
    (void)ptr;
    if ((state & UART_INTERRUPT_STATE_RX) == 0) return;
    while (uart_query_byte(MY_UART, &data) == ZF_TRUE)
        serial_rx_buffer_push(&rx_ring, data);
}

// 初始化
serial_rx_buffer_init(&rx_ring, rx_storage, sizeof(rx_storage));
uart_init(MY_UART, 115200, MY_TX_PIN, MY_RX_PIN);
uart_set_callback(MY_UART, my_uart_isr, NULL);
uart_set_interrupt_config(MY_UART, UART_INTERRUPT_CONFIG_RX_ENABLE);

// 主循环取数
uint8_t byte;
while (serial_rx_buffer_pop(&rx_ring, &byte)) {
    // 按协议解析 byte
}
```

---

### 4.4 ADC — zf_driver_adc

**文件**: [zf_driver_adc.h](SeekFree/zf_driver/zf_driver_adc.h)

```c
#define ADC_NUM               ( 2 )            // ADC0 和 ADC1
#define ADC_CONVERSION_TIMEOUT ( 0x001FFFFF )

typedef enum { ADC_0, ADC_1 } adc_index_enum;
typedef enum {
    ADC_CHANNEL_0 ... ADC_CHANNEL_7
} adc_channel_enum;

// ADC 引脚 — 编码了 (ADC索引, 通道, GPIO引脚)
typedef enum {
    ADC0_CH0_A27, ADC0_CH1_A26, ADC0_CH2_A25, ADC0_CH3_A24,
    ADC0_CH4_B25, ADC0_CH5_B24, ADC0_CH6_B20, ADC0_CH7_A22,
    ADC1_CH0_A15, ADC1_CH1_A16, ADC1_CH2_A17, ADC1_CH3_A18,
    ADC1_CH4_B17, ADC1_CH5_B18, ADC1_CH6_B19, ADC1_CH7_A21,
} adc_pin_enum;

typedef enum { ADC_12BIT, ADC_10BIT, ADC_8BIT } adc_resolution_enum;
```

**ADC API**:
```c
uint16 adc_convert(adc_pin_enum adc_pin);                              // 单次转换（阻塞）
uint8  adc_convert_checked(adc_pin_enum adc_pin, uint16 *result);     // 带超时保护的转换
uint16 adc_mean_filter_convert(adc_pin_enum adc_pin, const uint8 count); // 多次采样取均值
void   adc_init(adc_pin_enum adc_pin, adc_resolution_enum resolution); // 初始化
```

**使用示例**:
```c
adc_init(BAT_ADC, ADC_12BIT);                    // 电池ADC初始化，12位精度
uint16 raw = adc_convert(BAT_ADC);               // 单次采样 (0~4095)
uint16 filtered = adc_mean_filter_convert(BAT_ADC, 8); // 8次平均滤波

// 电池电压计算
uint32 bat_mv = (uint32)filtered * 3300 / 4095 * 447 / 100;
```

---

### 4.5 延时 — zf_driver_delay

**文件**: [zf_driver_delay.h](SeekFree/zf_driver/zf_driver_delay.h)

```c
void system_delay_ms(uint32 time);   // 毫秒延时（阻塞）
void system_delay_us(uint32 time);   // 微秒延时（阻塞）
```

---

### 4.6 定时器 — zf_driver_timer

**文件**: [zf_driver_timer.h](SeekFree/zf_driver/zf_driver_timer.h)

```c
typedef enum {
    TIM_A0, TIM_A1, TIM_G0, TIM_G6, TIM_G7, TIM_G8, TIM_G12, TIM_MAX
} timer_index_enum;

// 计时模式
typedef enum {
    TIMER_SYSTEM_CLOCK,    // 系统频率计时（最大 0xFFFF）
    TIMER_US,              // 微秒计时（最大 0xFFFF）
    TIMER_MS,              // 毫秒计时（最大 0xFFFF/2）
} timer_mode_enum;

// 功能状态（内部使用）
typedef enum {
    TIMER_FUNCTION_INIT    = 0,
    TIMER_FUNCTION_TIMER,  // 计时器
    TIMER_FUNCTION_PIT,    // 周期中断
    TIMER_FUNCTION_PWM,    // PWM
    TIMER_FUNCTION_ENCODER,// 编码器
} timer_function_enum;

#define TIM_NUM ( 7 )

// API
uint8  timer_funciton_check(timer_index_enum index, timer_function_enum mode);
void   timer_clock_enable(timer_index_enum index);
void   timer_start(timer_index_enum index);
void   timer_stop(timer_index_enum index);
uint16 timer_get(timer_index_enum index);    // 获取计数值
void   timer_clear(timer_index_enum index);  // 清零计数
void   timer_init(timer_index_enum index, timer_mode_enum mode);
```

---

### 4.7 周期中断 PIT — zf_driver_pit

**文件**: [zf_driver_pit.h](SeekFree/zf_driver/zf_driver_pit.h)

```c
typedef enum {
    PIT_TIM_A0, PIT_TIM_A1, PIT_TIM_G0, PIT_TIM_G6,
    PIT_TIM_G7, PIT_TIM_G8, PIT_TIM_G12,
} pit_index_enum;

#define PIT_NUM  ( 7 )

// 回调函数列表（外部声明）
extern void_callback_uint32_ptr pit_callback_list[PIT_NUM];
extern void *pit_callback_ptr_list[PIT_NUM];

// API
void pit_enable(pit_index_enum pit_n);
void pit_disable(pit_index_enum pit_n);
void pit_init(pit_index_enum pit_n, uint32 period, void_callback_uint32_ptr callback, void *ptr);
void pit_us_init(pit_index_enum pit_n, uint32 period, void_callback_uint32_ptr callback, void *ptr);
void pit_ms_init(pit_index_enum pit_n, uint32 period, void_callback_uint32_ptr callback, void *ptr);
```

**使用示例**:
```c
// 1ms 周期中断
void my_isr(uint32 event, void *ptr) {
    // 每1ms执行一次
}
pit_ms_init(PIT_TIM_G0, 1, my_isr, NULL);
pit_enable(PIT_TIM_G0);

// 100us 周期中断
pit_us_init(PIT_TIM_G6, 100, another_isr, NULL);
```

---

### 4.8 SPI — zf_driver_spi

**文件**: [zf_driver_spi.h](SeekFree/zf_driver/zf_driver_spi.h)

```c
#define SPI_NUM  ( 2 )     // SPI0 和 SPI1

typedef enum { SPI_0, SPI_1 } spi_index_enum;
typedef enum { SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3 } spi_mode_enum;

// SPI 引脚枚举
// SCK:  SPI0_SCK_A6, SPI0_SCK_A11, SPI0_SCK_A12, SPI0_SCK_B18
//       SPI1_SCK_A17, SPI1_SCK_B9,  SPI1_SCK_B16, SPI1_SCK_B23
// MOSI: SPI0_MOSI_A5, SPI0_MOSI_A9, SPI0_MOSI_A14, SPI0_MOSI_B17
//       SPI1_MOSI_A18, SPI1_MOSI_B8, SPI1_MOSI_B15, SPI1_MOSI_B22
// MISO: SPI0_MISO_A4, SPI0_MISO_A10, SPI0_MISO_A13, SPI0_MISO_B19
//       SPI1_MISO_A16, SPI1_MISO_B7, SPI1_MISO_B14, SPI1_MISO_B21
// CS:   SPI0_CS_A2, SPI0_CS_A8, SPI0_CS_B25
//       SPI1_CS_A2, SPI1_CS_A26, SPI1_CS_B6, SPI1_CS_B20
// 特殊值: SPI_MISO_NULL = 0xFFFFF, SPI_CS_NULL = 0xFFFFF (不用硬件MISO/CS)
```

**SPI API**:
```c
// 写操作
void spi_write_8bit(spi_index_enum spi_index, const uint8 data);
void spi_write_8bit_array(spi_index_enum spi_index, const uint8 *data, uint32 len);
void spi_write_16bit(spi_index_enum spi_index, const uint16 data);
void spi_write_16bit_array(spi_index_enum spi_index, const uint16 *data, uint32 len);

// 寄存器写
void spi_write_8bit_register(spi_index_enum spi_index, const uint8 reg, const uint8 data);
void spi_write_8bit_registers(spi_index_enum spi_index, const uint8 reg, const uint8 *data, uint32 len);
void spi_write_16bit_register(spi_index_enum spi_index, const uint16 reg, const uint16 data);
void spi_write_16bit_registers(spi_index_enum spi_index, const uint16 reg, const uint16 *data, uint32 len);

// 读操作
uint8  spi_read_8bit(spi_index_enum spi_index);
void   spi_read_8bit_array(spi_index_enum spi_index, uint8 *data, uint32 len);
uint16 spi_read_16bit(spi_index_enum spi_index);
void   spi_read_16bit_array(spi_index_enum spi_index, uint16 *data, uint32 len);
uint8  spi_read_8bit_register(spi_index_enum spi_index, const uint8 reg);
void   spi_read_8bit_registers(spi_index_enum spi_index, const uint8 reg, uint8 *data, uint32 len);
uint16 spi_read_16bit_register(spi_index_enum spi_index, const uint16 reg);
void   spi_read_16bit_registers(spi_index_enum spi_index, const uint16 reg, uint16 *data, uint32 len);

// 全双工传输
void spi_transfer_8bit(spi_index_enum spi_index, const uint8 *wbuf, uint8 *rbuf, uint32 len);
void spi_transfer_16bit(spi_index_enum spi_index, const uint16 *wbuf, uint16 *rbuf, uint32 len);

// 初始化
void spi_init(spi_index_enum spi_index, spi_mode_enum mode, uint32 baud,
              spi_sck_pin_enum sck, spi_mosi_pin_enum mosi,
              spi_miso_pin_enum miso, spi_cs_pin_enum cs);
```

---

### 4.9 软件 I2C — zf_driver_soft_iic

**文件**: [zf_driver_soft_iic.h](SeekFree/zf_driver/zf_driver_soft_iic.h)

```c
#define SOFT_IIC_CLOCK_STRETCH_TIMEOUT  ( 10000u )

typedef enum {
    SOFT_IIC_STATUS_OK = 0,
    SOFT_IIC_STATUS_NACK,
    SOFT_IIC_STATUS_INVALID_PARAM,
    SOFT_IIC_STATUS_CLOCK_STRETCH_TIMEOUT,
    SOFT_IIC_STATUS_BUS_STUCK,
} soft_iic_status_enum;

// 软件IIC对象结构体
typedef struct {
    uint32              scl_pin;        // SCL 引脚
    uint32              sda_pin;        // SDA 引脚
    uint8               addr;           // 7位器件地址
    uint32              delay;          // 时钟延时参数（越小越快）
    soft_iic_status_enum last_error;     // 最后一次错误状态
    uint8               transaction_active;
} soft_iic_info_struct;
```

**软件 I2C API**:
```c
// 基础操作
void   soft_iic_start(soft_iic_info_struct *obj);
void   soft_iic_stop(soft_iic_info_struct *obj);
uint8  soft_iic_send_data(soft_iic_info_struct *obj, const uint8 data);  // 返回 ACK 状态
uint8  soft_iic_read_data(soft_iic_info_struct *obj, uint8 ack);

// 写操作
void   soft_iic_write_8bit(soft_iic_info_struct *obj, const uint8 data);
void   soft_iic_write_8bit_array(soft_iic_info_struct *obj, const uint8 *data, uint32 len);
void   soft_iic_write_16bit(soft_iic_info_struct *obj, const uint16 data);
void   soft_iic_write_16bit_array(soft_iic_info_struct *obj, const uint16 *data, uint32 len);
void   soft_iic_write_8bit_register(soft_iic_info_struct *obj, const uint8 reg, const uint8 data);
void   soft_iic_write_8bit_registers(soft_iic_info_struct *obj, const uint8 reg, const uint8 *data, uint32 len);
soft_iic_status_enum soft_iic_write_8bit_register_checked(soft_iic_info_struct *obj, const uint8 reg, const uint8 data);   // 带状态返回
soft_iic_status_enum soft_iic_write_8bit_registers_checked(soft_iic_info_struct *obj, const uint8 reg, const uint8 *data, uint32 len);
void   soft_iic_write_16bit_register(soft_iic_info_struct *obj, const uint16 reg, const uint16 data);
void   soft_iic_write_16bit_registers(soft_iic_info_struct *obj, const uint16 reg, const uint16 *data, uint32 len);

// 读操作
uint8  soft_iic_read_8bit(soft_iic_info_struct *obj);
void   soft_iic_read_8bit_array(soft_iic_info_struct *obj, uint8 *data, uint32 len);
uint16 soft_iic_read_16bit(soft_iic_info_struct *obj);
void   soft_iic_read_16bit_array(soft_iic_info_struct *obj, uint16 *data, uint32 len);
uint8  soft_iic_read_8bit_register(soft_iic_info_struct *obj, const uint8 reg);
void   soft_iic_read_8bit_registers(soft_iic_info_struct *obj, const uint8 reg, uint8 *data, uint32 len);
soft_iic_status_enum soft_iic_read_8bit_register_checked(soft_iic_info_struct *obj, const uint8 reg, uint8 *data);         // 带状态返回
soft_iic_status_enum soft_iic_read_8bit_registers_checked(soft_iic_info_struct *obj, const uint8 reg, uint8 *data, uint32 len);
uint16 soft_iic_read_16bit_register(soft_iic_info_struct *obj, const uint16 reg);
void   soft_iic_read_16bit_registers(soft_iic_info_struct *obj, const uint16 reg, uint16 *data, uint32 len);

// 高级操作
void   soft_iic_transfer_8bit_array(soft_iic_info_struct *obj, const uint8 *wdata, uint32 wlen, uint8 *rdata, uint32 rlen);
void   soft_iic_transfer_16bit_array(soft_iic_info_struct *obj, const uint16 *wdata, uint32 wlen, uint16 *rdata, uint32 rlen);
void   soft_iic_sccb_write_register(soft_iic_info_struct *obj, const uint8 reg, uint8 data);    // SCCB 协议（类似 I2C）
uint8  soft_iic_sccb_read_register(soft_iic_info_struct *obj, const uint8 reg);
void   soft_iic_write_splicing_array(soft_iic_info_struct *obj, const uint8 *p1, uint32 l1, const uint8 *p2, uint32 l2); // 拼接写
soft_iic_status_enum soft_iic_write_splicing_array_checked(soft_iic_info_struct *obj, const uint8 *p1, uint32 l1, const uint8 *p2, uint32 l2);
soft_iic_status_enum soft_iic_get_last_error(const soft_iic_info_struct *obj);

// 初始化
void   soft_iic_init(soft_iic_info_struct *obj, uint8 addr, uint32 delay,
                     gpio_pin_enum scl_pin, gpio_pin_enum sda_pin);
```

**使用示例**:
```c
static soft_iic_info_struct my_i2c;
soft_iic_init(&my_i2c, 0x68, 10, B9, B8);   // 地址 0x68, SCL=B9, SDA=B8

soft_iic_write_8bit_register(&my_i2c, 0x6B, 0x80);   // 写寄存器
uint8 val = soft_iic_read_8bit_register(&my_i2c, 0x75); // 读寄存器
```

---

### 4.10 外部中断 EXTI — zf_driver_exti

**文件**: [zf_driver_exti.h](SeekFree/zf_driver/zf_driver_exti.h)

```c
typedef enum {
    EXTI_TRIGGER_RISING  = 1,   // 上升沿触发
    EXTI_TRIGGER_FALLING,       // 下降沿触发
    EXTI_TRIGGER_BOTH,          // 双边沿触发
} exti_trigger_enum;

// 回调列表
extern void_callback_uint32_ptr exti_callback_list[GPIO_GROUP_MAX * GPIO_GROUP_PIN_NUMBER_MAX];
extern void *exti_callback_ptr_list[GPIO_GROUP_MAX * GPIO_GROUP_PIN_NUMBER_MAX];

// API
void exti_enable(gpio_pin_enum pin);
void exti_disable(gpio_pin_enum pin);
void exti_init(gpio_pin_enum pin, exti_trigger_enum trigger,
               void_callback_uint32_ptr callback, void *ptr);
```

**使用示例**:
```c
void encoder_isr(uint32 state, void *ptr) {
    dt_encoder_t *enc = (dt_encoder_t *)ptr;
    // 处理编码器脉冲
}
exti_init(A26, EXTI_TRIGGER_BOTH, encoder_isr, &my_encoder);
```

---

### 4.11 Flash — zf_driver_flash

**文件**: [zf_driver_flash.h](SeekFree/zf_driver/zf_driver_flash.h)

```c
#define FLASH_BASE_ADDR        ( 0x00016000 )     // Flash 存储起始地址
#define FLASH_PAGE_SIZE        ( 0x00000400 )     // 1KB 每页
#define FLASH_SECTION_SIZE     ( FLASH_PAGE_SIZE * 2 )  // 2KB 每扇区
#define FLASH_MAX_PAGE_INDEX   ( 2 )
#define FLASH_DATA_BUFFER_SIZE ( FLASH_PAGE_SIZE / 4 )  // 256 个 32bit 元素

// 数据联合体
typedef union {
    float   float_type;
    uint32  uint32_type;
    int32   int32_type;
    uint16  uint16_type;
    int16   int16_type;
    uint8   uint8_type;
    int8    int8_type;
} flash_data_union;

extern flash_data_union flash_union_buffer[FLASH_DATA_BUFFER_SIZE];

// API
uint8 flash_check(uint32 sector_num, uint32 page_num);       // 检查页是否为空
uint8 flash_erase_page(uint32 sector_num, uint32 page_num);  // 擦除页
void  flash_read_page(uint32 sector_num, uint32 page_num, uint32 *buf, uint16 len);
uint8 flash_write_page(uint32 sector_num, uint32 page_num, const uint32 *buf, uint16 len);

// 缓冲区模式
void  flash_read_page_to_buffer(uint32 sector_num, uint32 page_num);
uint8 flash_write_page_from_buffer(uint32 sector_num, uint32 page_num);
void  flash_buffer_clear(void);
```

**使用示例**:
```c
// 直接写入
uint32 data[4] = {0x12345678, 0x9ABCDEF0, 0xAABBCCDD, 0x11223344};
flash_erase_page(0, 0);
flash_write_page(0, 0, data, 4);

// 缓冲区模式
flash_buffer_clear();
flash_union_buffer[0].float_type = 3.14f;
flash_union_buffer[1].uint32_type = 42;
flash_write_page_from_buffer(0, 1);
```

---

### 4.12 编码器 — zf_driver_encoder

**文件**: [zf_driver_encoder.h](SeekFree/zf_driver/zf_driver_encoder.h)

```c
// 编码器通道引脚 — 编码 (定时器索引, 复用功能, GPIO)
// CH1: TIMA0_ENCODER1_CH1_A0, TIMA0_ENCODER1_CH1_A8, ...
//      TIMG8_ENCODER1_CH1_A26, TIMG8_ENCODER1_CH1_B10, ...
// CH2: TIMA0_ENCODER1_CH2_A1, TIMA0_ENCODER1_CH2_A9, ...
//      TIMG8_ENCODER1_CH2_A27, TIMG8_ENCODER1_CH2_B11, ...

// API
int16 encoder_get_count(timer_index_enum index);   // 获取当前计数值
int16 encoder_get_delta(timer_index_enum index);   // 获取增量（自上次调用后）
void  encoder_clear_count(timer_index_enum index); // 清零

// 初始化
void  encoder_quad_init(timer_index_enum index,
                        encoder_channel1_enum ch1_pin,
                        encoder_channel2_enum ch2_pin);         // 正交解码模式
void  encoder_dir_init(timer_index_enum index,
                       encoder_channel1_enum lsb_pin,
                       gpio_pin_enum dir_pin);                  // 方向+脉冲模式
```

**使用示例**:
```c
// 硬件正交解码
encoder_quad_init(TIM_G8, TIMG8_ENCODER1_CH1_A26, TIMG8_ENCODER1_CH2_A27);
int16 count = encoder_get_count(TIM_G8);
int16 delta = encoder_get_delta(TIM_G8);
```

---

## 5. 设备层 — zf_device

### 5.1 IMU660RA 六轴传感器

**文件**: [zf_device_imu660ra.h](SeekFree/zf_device/zf_device_imu660ra.h)

> 6轴 IMU：3轴加速度计 + 3轴陀螺仪，支持 SPI/I2C 双模式。默认使用硬件 SPI。

```c
// 通信模式配置
#define IMU660RA_USE_IIC       ( 0 )     // 0=SPI, 1=I2C
#define IMU660RA_USE_SOFT_IIC  ( 1 )     // 在I2C模式下: 0=硬件I2C, 1=软件I2C

// 引脚定义
#define IMU660RA_SPI         ( SPI_1 )
#define IMU660RA_SPC_PIN     ( SPI1_SCK_B23  )    // SPI SCK = PB23
#define IMU660RA_SDI_PIN     ( SPI1_MOSI_B22 )    // SPI MOSI = PB22
#define IMU660RA_SDO_PIN     ( SPI1_MISO_B21 )    // SPI MISO = PB21
#define IMU660RA_CS_PIN      ( B19 )              // 片选 = PB19
#define IMU660RA_CS(x)       ((x) ? gpio_high(IMU660RA_CS_PIN) : gpio_low(IMU660RA_CS_PIN))

// 量程配置
typedef enum {
    IMU660RA_ACC_SAMPLE_SGN_2G,     // ±2g
    IMU660RA_ACC_SAMPLE_SGN_4G,     // ±4g
    IMU660RA_ACC_SAMPLE_SGN_8G,     // ±8g  (默认)
    IMU660RA_ACC_SAMPLE_SGN_16G,    // ±16g
} imu660ra_acc_sample_config;

typedef enum {
    IMU660RA_GYRO_SAMPLE_SGN_125DPS,
    IMU660RA_GYRO_SAMPLE_SGN_250DPS,
    IMU660RA_GYRO_SAMPLE_SGN_500DPS,
    IMU660RA_GYRO_SAMPLE_SGN_1000DPS,
    IMU660RA_GYRO_SAMPLE_SGN_2000DPS,   // 默认
} imu660ra_gyro_sample_config;

#define IMU660RA_ACC_SAMPLE_DEFAULT   ( IMU660RA_ACC_SAMPLE_SGN_8G )
#define IMU660RA_GYRO_SAMPLE_DEFAULT  ( IMU660RA_GYRO_SAMPLE_SGN_2000DPS )
#define IMU660RA_DEV_ADDR             ( 0x69 )              // 7位I2C地址

// 全局变量 — 原始数据
extern int16 imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z;
extern int16 imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z;
extern float imu660ra_transition_factor[2];     // [0]=加速度转换因子, [1]=陀螺仪转换因子

// API
uint8 imu660ra_init(void);                              // 返回 0=失败, 1=成功
void  imu660ra_get_acc(void);                           // 读取加速度计数据
void  imu660ra_get_gyro(void);                          // 读取陀螺仪数据

// 转换宏 — 将原始值转换为物理量
#define imu660ra_acc_transition(acc)    ((float)(acc) / imu660ra_transition_factor[0])    // 单位: g
#define imu660ra_gyro_transition(gyro)  ((float)(gyro) / imu660ra_transition_factor[1])   // 单位: °/s
```

**使用示例**:
```c
imu660ra_init();
imu660ra_get_gyro();                    // 更新陀螺仪数据
imu660ra_get_acc();                     // 更新加速度计数据
float gz = imu660ra_gyro_transition(imu660ra_gyro_z);  // Z轴角速度 (°/s)
float ay = imu660ra_acc_transition(imu660ra_acc_y);    // Y轴加速度 (g)
```

---

### 5.2 IMU963RA 九轴传感器

**文件**: [zf_device_imu963ra.h](SeekFree/zf_device/zf_device_imu963ra.h)

> 9轴 IMU：3轴加速度计 + 3轴陀螺仪 + 3轴磁力计。接口与 IMU660RA 类似，额外支持磁力计。

```c
#define IMU963RA_DEV_ADDR  ( 0x6B )

// 额外量程（相比660RA增加了 ±4000dps 陀螺仪量程和磁力计量程）
typedef enum {
    IMU963RA_MAG_SAMPLE_2G,   // 磁力计 ±2 Gs
    IMU963RA_MAG_SAMPLE_8G,   // 磁力计 ±8 Gs (默认)
} imu963ra_mag_sample_config;

// 全局变量
extern int16 imu963ra_acc_x,  imu963ra_acc_y,  imu963ra_acc_z;
extern int16 imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z;
extern int16 imu963ra_mag_x,  imu963ra_mag_y,  imu963ra_mag_z;
extern float imu963ra_transition_factor[3];  // [0]=acc, [1]=gyro, [2]=mag

// API
uint8 imu963ra_init(void);
void  imu963ra_get_acc(void);
void  imu963ra_get_gyro(void);
void  imu963ra_get_mag(void);                              // 读取磁力计数据

#define imu963ra_acc_transition(acc)    ((float)acc / imu963ra_transition_factor[0])
#define imu963ra_gyro_transition(gyro)  ((float)gyro / imu963ra_transition_factor[1])
#define imu963ra_mag_transition(mag)    ((float)mag / imu963ra_transition_factor[2])
```

---

### 5.3 OLED 显示屏

**文件**: [zf_device_oled.h](SeekFree/zf_device/zf_device_oled.h)

> SSD1306 驱动，128×64 像素，SPI 接口

```c
#define OLED_X_MAX  ( 128 )
#define OLED_Y_MAX  ( 64 )

typedef enum { OLED_PORTAIT = 0, OLED_PORTAIT_180 = 1 } oled_dir_enum;
typedef enum {
    OLED_6X8_FONT  = 0,
    OLED_8X16_FONT = 1,
    OLED_16X16_FONT = 2,   // 目前不支持
} oled_font_size_enum;

// 硬件 SPI 配置（默认）
#define OLED_SPI       ( SPI_0 )
#define OLED_D0_PIN    ( SPI0_SCK_A12  )     // SCK = PA12
#define OLED_D1_PIN    ( SPI0_MOSI_A9  )     // MOSI = PA9
#define OLED_RES_PIN   ( A7 )                // RES = PA7
#define OLED_DC_PIN    ( A15 )               // DC = PA15
#define OLED_CS_PIN    ( A8 )                // CS = PA8

// CS/DC/RES 控制宏
#define OLED_CS(x)   ((x) ? gpio_high(OLED_CS_PIN)  : gpio_low(OLED_CS_PIN))
#define OLED_DC(x)   ((x) ? gpio_high(OLED_DC_PIN)  : gpio_low(OLED_DC_PIN))
#define OLED_RES(x)  ((x) ? gpio_high(OLED_RES_PIN) : gpio_low(OLED_RES_PIN))

// API
void oled_init(void);
void oled_clear(void);
void oled_full(const uint8 color);
void oled_set_dir(oled_dir_enum dir);
void oled_set_font(oled_font_size_enum font);
void oled_draw_point(uint16 x, uint16 y, const uint8 color);
void oled_show_string(uint16 x, uint16 y, const char ch[]);
void oled_show_int(uint16 x, uint16 y, const int32 dat, uint8 num);
void oled_show_uint(uint16 x, uint16 y, const uint32 dat, uint8 num);
void oled_show_float(uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum);
void oled_show_binary_image(uint16 x, uint16 y, const uint8 *image,
                            uint16 width, uint16 height, uint16 dis_w, uint16 dis_h);
void oled_show_gray_image(uint16 x, uint16 y, const uint8 *image,
                          uint16 width, uint16 height, uint16 dis_w, uint16 dis_h, uint8 threshold);
void oled_show_wave(uint16 x, uint16 y, const uint16 *wave,
                    uint16 width, uint16 value_max, uint16 dis_w, uint16 dis_vmax);
void oled_show_chinese(uint16 x, uint16 y, uint8 size, const uint8 *chinese_buffer, uint8 number);
```

**使用示例**:
```c
oled_init();
oled_set_font(OLED_8X16_FONT);
oled_show_string(0, 0, "Hello SeekFree");
oled_show_float(0, 2, 3.14, 3, 2);    // 显示 "3.14"
oled_show_int(0, 4, -42, 3);           // 显示 "-42"
```

---

### 5.4 TFT180 彩色屏

**文件**: [zf_device_tft180.h](SeekFree/zf_device/zf_device_tft180.h)

> 1.8寸 TFT 彩屏，160×128，SPI 接口，支持 RGB565 颜色。

```c
typedef enum {
    TFT180_PORTAIT = 0, TFT180_PORTAIT_180, TFT180_CROSSWISE, TFT180_CROSSWISE_180
} tft180_dir_enum;

// API — 与 OLED 类似，但增加了颜色设置和 RGB565 图像显示
void tft180_init(void);
void tft180_clear(void);
void tft180_full(const uint16 color);
void tft180_set_dir(tft180_dir_enum dir);
void tft180_set_font(tft180_font_size_enum font);
void tft180_set_color(const uint16 pen, const uint16 bgcolor);
void tft180_draw_point(uint16 x, uint16 y, const uint16 color);
void tft180_draw_line(uint16 xs, uint16 ys, uint16 xe, uint16 ye, const uint16 color);
void tft180_show_char(uint16 x, uint16 y, const char dat);
void tft180_show_string(uint16 x, uint16 y, const char dat[]);
void tft180_show_int(uint16 x, uint16 y, const int32 dat, uint8 num);
void tft180_show_uint(uint16 x, uint16 y, const uint32 dat, uint8 num);
void tft180_show_float(uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum);
void tft180_show_binary_image(...);
void tft180_show_gray_image(...);
void tft180_show_rgb565_image(uint16 x, uint16 y, const uint16 *image,
                              uint16 w, uint16 h, uint16 dw, uint16 dh, uint8 color_mode);
void tft180_show_wave(...);
void tft180_show_chinese(...);

// 快捷宏
#define tft180_displayimage7725(p, w, h)   // 显示小钻风图像
#define tft180_displayimage03x(p, w, h)    // 显示总钻风图像
#define tft180_displayimage8660(p, w, h)   // 显示凌瞳图像
```

---

### 5.5 IPS200 彩色屏

**文件**: [zf_device_ips200.h](SeekFree/zf_device/zf_device_ips200.h)

> 2.0寸 IPS 彩屏，320×240，SPI 接口。API 与 TFT180 基本一致。

```c
typedef enum { IPS200_TYPE_SPI, IPS200_TYPE_PARALLEL8 } ips200_type_enum;
void ips200_init(ips200_type_enum type_select);  // 初始化时需要选择接口类型

// 其余 API: ips200_clear, ips200_full, ips200_show_string, ips200_show_float 等
// 命名规则与 TFT180 一致，将 tft180_ 替换为 ips200_
```

---

### 5.6 TSL1401 线阵 CCD

**文件**: [zf_device_tsl1401.h](SeekFree/zf_device/zf_device_tsl1401.h)

> 128 像素线阵 CCD，ADC 采集。用于循迹线检测等。

```c
#define TSL1401_DATA_LEN     ( 128 )          // CCD 数据长度 128 像素
#define TSL1401_EXPOSURE_TIME ( 10 )          // 曝光时间 10ms
#define TSL1401_PIT_INDEX    ( PIT_TIM_G8 )   // 使用 PIT 定时中断采集
#define TSL1401_AD_RESOLUTION ( ADC_12BIT )   // ADC 12 位精度
#define TSL1401_AO_PIN_MAX   ( 2 )
#define TSL1401_AO_PIN_LIST  { ADC0_CH2_A25, ADC0_CH3_A24 }

// 全局变量
extern uint16 tsl1401_data[TSL1401_AO_PIN_MAX][TSL1401_DATA_LEN];
extern vuint8  tsl1401_finish_flag;    // 采集完成标志

// API
void tsl1401_collect_pit_handler(uint32 event, void *ptr);  // PIT ISR 中进行采集
void tsl1401_send_data(uart_index_enum uart_n, uint8 index);  // 发送至上位机
void tsl1401_init(uint8 index);                              // 初始化（指定 ADC 索引）
```

**使用示例**:
```c
tsl1401_init(0);   // 初始化第0路AO
// PIT 中断每隔 EXPOSURE_TIME ms 自动调用 tsl1401_collect_pit_handler
if (tsl1401_finish_flag) {
    tsl1401_finish_flag = 0;
    // tsl1401_data[0][0..127] 包含最新曝光数据
}
```

---

### 5.7 DL1B 激光测距

**文件**: [zf_device_dl1b.h](SeekFree/zf_device/zf_device_dl1b.h)

> VL53L1X ToF 激光测距模块，I2C 接口，最大测距 4m。

```c
#define DL1B_DEV_ADDR  ( 0x52 >> 1 )    // 0x29 (7位)

// 软件I2C引脚配置
#define DL1B_SOFT_IIC_DELAY  ( 10 )
#define DL1B_SCL_PIN         ( B8 )
#define DL1B_SDA_PIN         ( B26 )
#define DL1B_XS_PIN          ( B14 )    // XSHUT 引脚

// 全局变量
extern uint8  dl1b_finsh_flag;           // 测量完成标志
extern uint16 dl1b_distance_mm;          // 距离 (mm)

// API
uint8 dl1b_init(void);                   // 返回 0=失败, 1=成功
void  dl1b_get_distance(void);           // 触发一次测距
void  dl1b_int_handler(void);            // INT 引脚中断处理
```

**使用示例**:
```c
dl1b_init();
dl1b_get_distance();
if (dl1b_finsh_flag) {
    dl1b_finsh_flag = 0;
    // dl1b_distance_mm 是最新距离
}
```

---

### 5.8 GS08RA 灰度传感器

**文件**: [zf_device_gs08ra.h](SeekFree/zf_device/zf_device_gs08ra.h)

> 8 路模拟灰度传感器，通过模拟开关 + ADC 采集。用于巡线检测。

```c
#define GS08A_CHANNEL_NUM  ( 8 )         // 8路通道
#define GS08RA_S0_PIN      ( A16 )       // 通道选择 A
#define GS08RA_S1_PIN      ( A17 )       // 通道选择 B
#define GS08RA_S2_PIN      ( B17 )       // 通道选择 C
#define GS08RA_OUT_PIN     ( ADC0_CH4_B25 )  // ADC 采集引脚

// 全局变量
extern uint8 gs08ra_threshold;           // 二值化阈值
extern uint8 gs08ra_max_val[8];          // 8路最大值
extern uint8 gs08ra_min_val[8];          // 8路最小值
extern uint8 gs08ra_raw_val[8];          // 8路原始灰度
extern uint8 gs08ra_deal_val[8];         // 归一化后的值
extern uint8 gs08ra_bin_val[8];          // 二值化结果

// API
void gs08ra_set_max(void);               // 记录各通道当前值作为最大参考
void gs08ra_set_min(void);               // 记录各通道当前值作为最小参考
void gs08ra_set_threshold(uint8 threshold);
void gs08ra_scan_read(void);             // 扫描读取所有8路数据
void gs08ra_init(void);
```

**使用示例**:
```c
gs08ra_init();
// 初始化后放置传感器在黑色/白色上分别调用 set_min/set_max
gs08ra_scan_read();   // 扫描一次
// gs08ra_bin_val[0..7] 包含二值化结果
```

---

### 5.9 按键管理

**文件**: [zf_device_key.h](SeekFree/zf_device/zf_device_key.h)

> 支持最多 4 路按键，自动消抖和长按检测。

```c
#define KEY_LIST              { A30, A31, B0, B1 }   // 4个按键引脚
#define KEY_RELEASE_LEVEL     ( GPIO_HIGH )           // 释放状态电平（上拉）
#define KEY_MAX_SHOCK_PERIOD  ( 10 )                  // 消抖时间 10ms
#define KEY_LONG_PRESS_PERIOD ( 1000 )                // 长按阈值 1000ms

typedef enum { KEY_1, KEY_2, KEY_3, KEY_4, KEY_NUMBER } key_index_enum;
typedef enum {
    KEY_RELEASE,         // 释放
    KEY_SHORT_PRESS,     // 短按
    KEY_LONG_PRESS,      // 长按
} key_state_enum;

// API
void           key_scanner(void);                      // 扫描按键（通过PIT周期调用）
key_state_enum key_get_state(key_index_enum key_n);    // 获取按键状态
void           key_clear_state(key_index_enum key_n);  // 清除指定按键状态
void           key_clear_all_state(void);              // 清除所有按键状态
void           key_init(uint32 period);                // 初始化（period=扫描间隔ms）
```

**使用示例**:
```c
key_init(10);   // 每 10ms 扫描一次
// 通过 PIT 中断调用 key_scanner():
// pit_ms_init(PIT_TIM_G6, 10, key_scanner_isr, NULL);

key_state_enum state = key_get_state(KEY_1);
if (state == KEY_SHORT_PRESS) {
    key_clear_state(KEY_1);
    // 处理短按
}
```

---

### 5.10 无线串口模块

**文件**: [zf_device_wireless_uart.h](SeekFree/zf_device/zf_device_wireless_uart.h)

> 无线转串口模块（逐飞配套），支持自动波特率。

```c
#define WIRELESS_UART_INDEX     ( UART_1 )
#define WIRELESS_UART_BUAD_RATE ( 115200 )
#define WIRELESS_UART_TX_PIN    ( UART1_RX_B5 )
#define WIRELESS_UART_RX_PIN    ( UART1_TX_B6 )
#define WIRELESS_UART_RTS_PIN   ( B2 )

// API
uint32 wireless_uart_send_byte(const uint8 data);
uint32 wireless_uart_send_buffer(const uint8 *buff, uint32 len);
uint32 wireless_uart_send_string(const char *str);
uint32 wireless_uart_read_buffer(uint8 *buff, uint32 len);
void   wireless_uart_callback(void);
uint8  wireless_uart_init(void);
```

---

### 5.11 设备类型管理

**文件**: [zf_device_type.h](SeekFree/zf_device/zf_device_type.h)

> 统一管理摄像头、无线模块、ToF 等外设类型的注册和中断回调分发。

```c
typedef enum {
    NO_CAMERE = 0,
    CAMERA_BIN_IIC,      // 小钻风 I2C 版本
    CAMERA_BIN_UART,     // 小钻风 UART 版本
    CAMERA_GRAYSCALE,    // 总钻风
    CAMERA_COLOR,        // 凌瞳
} camera_type_enum;

typedef enum {
    NO_WIRELESS = 0,
    WIRELESS_UART,
    BLUETOOTH_CH9141,
    WIFI_UART,
    WIFI_SPI,
} wireless_type_enum;

typedef enum {
    NO_TOF = 0, TOF_DL1A, TOF_DL1B,
} tof_type_enum;

typedef void (*callback_function)(void);

extern wireless_type_enum wireless_type;
extern callback_function wireless_module_uart_handler;
extern callback_function wireless_module_spi_handler;
extern camera_type_enum camera_type;
extern callback_function camera_dma_handler;
extern callback_function camera_vsync_handler;
extern callback_function camera_uart_handler;
extern tof_type_enum tof_type;
extern callback_function tof_module_exti_handler;

// API
void set_camera_type(camera_type_enum type, callback_function vsync,
                     callback_function dma, callback_function uart);
void set_wireless_type(wireless_type_enum type, callback_function cb);
void set_tof_type(tof_type_enum type, callback_function exti_cb);
```

---

### 5.12 其他设备

简要接口列表：

| 设备 | 文件 | 关键 API |
|------|------|----------|
| **IMU660RB** | zf_device_imu660rb.h | 同 IMU660RA，量程配置略有差异 |
| **IMU660RC** | zf_device_imu660rc.h | 同 IMU660RA, 超时配置不同 |
| **IPS114** | zf_device_ips114.h | 1.14寸 IPS，API 同 IPS200 |
| **IPS200pro** | zf_device_ips200pro.h | IPS200 增强版，API 同 IPS200 |
| **DL1A** | zf_device_dl1a.h | 同 DL1B，软件 I2C |
| **绝对式编码器** | zf_device_absolute_encoder.h | 编码器设备配置（初始化文件） |
| **WiFi SPI** | zf_device_wifi_spi.h | SPI WiFi 模块 |
| **WiFi UART** | zf_device_wifi_uart.h | UART WiFi 模块 |

---

## 6. 组件层 — zf_components

### 6.1 逐飞助手上位机

**文件**: [seekfree_assistant.h](SeekFree/zf_components/seekfree_assistant.h)

> 与逐飞官方上位机的通信协议库。支持发送 CCD 图像、摄像头图像、虚拟示波器数据、调参数据。

```c
#define SEEKFREE_ASSISTANT_OSCILLOSCOPE_MAX  ( 16 )    // 示波器最大通道数
#define SEEKFREE_ASSISTANT_DEBUG_PARAM_ENABLE ( 1 )     // 启用在线调参
#define SEEKFREE_ASSISTANT_DEBUG_PARAM_MAX   ( 8 )      // 调参通道数

// 数据传输回调
typedef uint32 (*seekfree_assistant_transfer_callback_function)(const uint8 *buff, uint32 length);
typedef uint32 (*seekfree_assistant_receive_callback_function)(uint8 *buff, uint32 length);

// CCD 图像配置和发送
void seekfree_assistant_ccd_config(seekfree_assistant_ccd_struct *ccd_obj,
    seekfree_assistant_data_type_enum data_type, uint8 channel_index,
    uint16 channel_color, uint16 data_length, void *data_buffer);
void seekfree_assistant_ccd_send(seekfree_assistant_ccd_struct *ccd_obj);

// 摄像头图像配置和发送
void seekfree_assistant_camera_config(seekfree_assistant_camera_struct *camera_obj,
    seekfree_assistant_camera_type_enum camera_type, uint16 w, uint16 h, void *buf);
void seekfree_assistant_camera_send(seekfree_assistant_camera_struct *camera_obj);

// 边界框发送
void seekfree_assistant_camera_boundary_config(...);
void seekfree_assistant_camera_boundary_send(...);
void seekfree_assistant_camera_rectangular_send(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color);

// 虚拟示波器（最多16通道）
void seekfree_assistant_oscilloscope_config(seekfree_assistant_oscilloscope_struct *obj,
    uint8 channel_max, void *data_buffer);
void seekfree_assistant_oscilloscope_send(seekfree_assistant_oscilloscope_struct *obj);

// 在线调参
void seekfree_assistant_debug_param_analysis(seekfree_assistant_debug_param_struct *obj);
```

---

## 7. 用户应用层 — user

### 7.1 系统配置 — config.h

**文件**: [config.h](user/inc/config.h)

这是整个工程的核心配置文件，定义了所有控制参数、引脚定义和算法参数。

#### 应用配置文件选择

```c
#define EC_APP_PROFILE_HARDWARE_TEST  1    // 硬件测试模式
#define EC_APP_PROFILE_LINE_CAR       2    // 循迹小车模式
#define EC_APP_PROFILE_EMPTY          3    // 空工程模式
```

#### 电机配置

```c
#define MOTOR_L_IN1       PWM_TIM_A0_CH0_A8    // 左轮 IN1
#define MOTOR_L_IN2       PWM_TIM_A0_CH1_B20   // 左轮 IN2
#define MOTOR_R_IN1       PWM_TIM_A0_CH3_A17   // 右轮 IN1
#define MOTOR_R_IN2       PWM_TIM_A0_CH2_A15   // 右轮 IN2
#define MOTOR_PWM_FREQ    20000u               // PWM 20kHz
#define MOTOR_PWM_DUTY_MAX  8000               // 最大占空比
```

#### PID 参数默认值

```c
#define CAR_DEFAULT_SPEED_KP     35.0f
#define CAR_DEFAULT_SPEED_KI     15.0f
#define CAR_DEFAULT_SPEED_KD      0.0f
#define CAR_DEFAULT_LINE_KP     600.0f
#define CAR_DEFAULT_LINE_KD     200.0f
#define CAR_DEFAULT_HEADING_KP   90.0f
#define CAR_DEFAULT_HEADING_KD    3.0f
```

#### 编码器配置

```c
#define ENCODER1_TIMER       TIM_G8
#define ENCODER1_CHANNEL_A   TIMG8_ENCODER1_CH1_A26
#define ENCODER1_CHANNEL_B   TIMG8_ENCODER1_CH2_A27
#define ENCODER_CPR          1456          // 13PPR × 1:28 × AB×4
#define WHEEL_DIAMETER_MM    65.0f
```

#### 外设引脚

```c
#define KEY1_PIN          B6
#define KEY2_PIN          B7
#define KEY3_PIN          B23
#define BUZZER_PIN        A31
#define OLED_SCL          B9
#define OLED_SDA          B8
#define MPU6050_SCL       A1
#define MPU6050_SDA       A0
#define BAT_ADC           ADC0_CH2_A25
#define BLUETOOTH_UART    UART_2
#define BLUETOOTH_TX_PIN  UART2_TX_B15
#define BLUETOOTH_RX_PIN  UART2_RX_B16
```

---

### 7.2 引脚映射 — pin_mapping.h

**文件**: [pin_mapping.h](user/inc/pin_mapping.h)

天猛星开发板的引脚分配表：

```c
// LED
#define PIN_LED1  A30
#define PIN_LED2  A7
#define PIN_LED3  B27

// RGB
#define PIN_RGB_G  B1
#define PIN_RGB_R  B0
#define PIN_RGB_B  A29
#define PIN_RGB_ON_LEVEL   GPIO_LOW
#define PIN_RGB_OFF_LEVEL  GPIO_HIGH

// 调试串口（按 EC_APP_PROFILE 自动选择）
// Line-car: UART2 (PB15/PB16)
// 其他: UART1 (PA8/PA9)

// 云台 EMM 电机通信: UART2 (PB15/PB16)
// T8 灰度传感器: UART1 (PA8/PA9)
// MaixCam2 视觉模块: UART3 (PB2/PB3)
```

---

### 7.3 电机驱动 — dt_motor

**文件**: [dt_motor.h](user/inc/driver/dt_motor.h)

> H 桥直流电机驱动，双路 PWM 控制，支持正反转和刹车。

```c
#define DT_MOTOR_DUTY_MAX  MOTOR_PWM_DUTY_MAX   // = 8000

typedef struct {
    pwm_channel_enum in1_pin;
    pwm_channel_enum in2_pin;
    uint32_t         pwm_freq;
} dt_motor_config_t;

void dt_motor_init(dt_motor_config_t *cfg);
void dt_motor_set_speed(dt_motor_config_t *cfg, int16_t speed);  // speed: [-8000, +8000]
void dt_motor_stop(dt_motor_config_t *cfg);                       // 两路PWM=0
void dt_motor_emergency_stop(dt_motor_config_t *cfg);             // 强制拉低
void dt_motor_brake(dt_motor_config_t *cfg);                      // H桥同侧导通刹车
```

**使用示例**:
```c
dt_motor_config_t left_motor = {
    .in1_pin = MOTOR_L_IN1,
    .in2_pin = MOTOR_L_IN2,
    .pwm_freq = MOTOR_PWM_FREQ,
};
dt_motor_init(&left_motor);
dt_motor_set_speed(&left_motor, 5000);   // 正转 62.5%
dt_motor_set_speed(&left_motor, -3000);  // 反转 37.5%
dt_motor_stop(&left_motor);             // 停止
```

---

### 7.4 编码器驱动 — dt_encoder

**文件**: [dt_encoder.h](user/inc/driver/dt_encoder.h)

> 支持 GPIO 中断和硬件正交解码。自动计算转速和行驶距离。

```c
typedef struct {
    gpio_pin_enum    a_pin;                   // A相引脚
    gpio_pin_enum    b_pin;                   // B相引脚
    uint16_t         counts_per_rev;          // 每转脉冲数 (CPR)
    float            wheel_circumference_mm;  // 车轮周长 (mm)
    volatile uint32_t edge_count;             // 有效脉冲总数
    volatile int32_t  signed_edge_count;      // 带符号脉冲总数
    volatile uint32_t invalid_transition_count; // 非法跳变计数(诊断)
    bool             quadrature_enabled;      // 正交解码使能
    bool             hardware_quadrature;     // 硬件正交解码
    timer_index_enum timer_index;             // 硬件定时器索引
    encoder_channel1_enum channel_a;
    encoder_channel2_enum channel_b;
    int8_t           direction_sign;          // 方向符号
    float            rpm_lpf_alpha;           // RPM 低通滤波系数
    float            rpm;                     // 滤波后转速 (RPM)
    float            rpm_signed;              // 带符号转速
} dt_encoder_t;

// API
bool     dt_encoder_init(dt_encoder_t *enc);
bool     dt_encoder_is_ready(const dt_encoder_t *enc);
uint8_t  dt_encoder_get_ab_state(const dt_encoder_t *enc);
void     dt_encoder_sample_inputs(dt_encoder_t *enc);
void     dt_encoder_reset_odometry(dt_encoder_t *enc);
uint32_t dt_encoder_get_edges(dt_encoder_t *enc);
int32_t  dt_encoder_get_signed_edges(dt_encoder_t *enc);
uint32_t dt_encoder_get_invalid_transitions(dt_encoder_t *enc);
uint32_t dt_encoder_get_delta(dt_encoder_t *enc);
int32_t  dt_encoder_get_signed_delta(dt_encoder_t *enc);
float    dt_encoder_compute_signed_rpm(dt_encoder_t *enc, uint32_t dt_ms);
float    dt_encoder_compute_rpm(dt_encoder_t *enc, uint32_t dt_ms);
float    dt_encoder_get_travel_mm(dt_encoder_t *enc);
float    dt_encoder_get_distance_mm(dt_encoder_t *enc);
```

**使用示例**:
```c
dt_encoder_t enc1 = {
    .a_pin = A26, .b_pin = A27,
    .counts_per_rev = ENCODER_CPR,
    .wheel_circumference_mm = 3.14159f * WHEEL_DIAMETER_MM,
    .quadrature_enabled = true,
    .hardware_quadrature = true,
    .timer_index = TIM_G8,
    .channel_a = TIMG8_ENCODER1_CH1_A26,
    .channel_b = TIMG8_ENCODER1_CH2_A27,
    .direction_sign = 1,
    .rpm_lpf_alpha = 0.55f,
};
dt_encoder_init(&enc1);
float rpm = dt_encoder_compute_signed_rpm(&enc1, 10);  // 每10ms计算一次
float distance = dt_encoder_get_distance_mm(&enc1);
```

---

### 7.5 MPU6050 驱动 — dt_mpu6050

**文件**: [dt_mpu6050.h](user/inc/driver/dt_mpu6050.h)

> MPU6050 六轴传感器驱动，基于软件 I2C。支持硬件抽象层（HAL）和高级接口。

```c
#define DT_MPU6050_DEFAULT_ADDR     0x68

// 加速度计量程
#define DT_MPU6050_ACCEL_FS_2G      0     // ±2g,  灵敏度 16384 LSB/g
#define DT_MPU6050_ACCEL_FS_4G      1     // ±4g,  灵敏度  8192 LSB/g
#define DT_MPU6050_ACCEL_FS_8G      2     // ±8g,  灵敏度  4096 LSB/g
#define DT_MPU6050_ACCEL_FS_16G     3     // ±16g, 灵敏度  2048 LSB/g

// 陀螺仪量程
#define DT_MPU6050_GYRO_FS_250      0     // ±250°/s,  灵敏度 131 LSB/(°/s)
#define DT_MPU6050_GYRO_FS_500      1     // ±500°/s,  灵敏度 65.5 LSB/(°/s)
#define DT_MPU6050_GYRO_FS_1000     2     // ±1000°/s, 灵敏度 32.8 LSB/(°/s)
#define DT_MPU6050_GYRO_FS_2000     3     // ±2000°/s, 灵敏度 16.4 LSB/(°/s)

// 配置和数据结构体
typedef struct {
    soft_iic_info_struct  iic;
    uint8_t               accel_fs;
    uint8_t               gyro_fs;
} dt_mpu6050_config_t;

typedef struct {
    float ax, ay, az;     // 加速度 (g)
    float gx, gy, gz;     // 角速度 (°/s)
    float temp;           // 温度 (°C)
} dt_mpu6050_data_t;

// HAL 接口 — 单寄存器读写
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

**使用示例**:
```c
dt_mpu6050_config_t mpu_cfg = {0};
soft_iic_init(&mpu_cfg.iic, DT_MPU6050_DEFAULT_ADDR, 10, MPU6050_SCL, MPU6050_SDA);
mpu_cfg.accel_fs = DT_MPU6050_ACCEL_FS_8G;
mpu_cfg.gyro_fs = DT_MPU6050_GYRO_FS_2000;
dt_mpu6050_init(&mpu_cfg);

dt_mpu6050_data_t mpu_data;
dt_mpu6050_read_all(&mpu_cfg, &mpu_data);
// mpu_data.gz — Z轴角速度 (°/s)
// mpu_data.ay — Y轴加速度 (g)
```

---

### 7.6 PID 控制器 — pid_controller

**文件**: [pid_controller.h](user/inc/lib/pid_controller.h)

> 位置式 PID 控制器，支持积分限幅、输出限幅、死区、微分低通滤波、积分分离。

```c
typedef struct {
    float kp;                // 比例增益
    float ki;                // 积分增益
    float kd;                // 微分增益
    float integral;          // 积分累计值
    float prev_error;        // 上次误差
    float prev_derivative;   // 上次微分（滤波用）
    float output_min;        // 输出下限
    float output_max;        // 输出上限
    float integral_min;      // 积分下限
    float integral_max;      // 积分上限
    float deadband;          // 死区
    float derivative_lpf;    // 微分低通滤波系数 α (0~1)
    bool  has_prev_error;    // 是否有历史误差
} PidController;

// API
void  pid_init(PidController *pid);
void  pid_reset(PidController *pid);       // 保留增益和限幅，清空内部状态
void  pid_set_gain(PidController *pid, float kp, float ki, float kd);
void  pid_set_limits(PidController *pid, float out_min, float out_max,
                     float int_min, float int_max);
void  pid_set_deadband(PidController *pid, float deadband);
void  pid_set_derivative_lpf(PidController *pid, float alpha);
float pid_update(PidController *pid, float error, float dt_s);
```

**使用示例**:
```c
PidController speed_pid;
pid_init(&speed_pid);
pid_set_gain(&speed_pid, 35.0f, 15.0f, 0.0f);
pid_set_limits(&speed_pid, -2000.0f, 2000.0f, -3000.0f, 3000.0f);
pid_set_deadband(&speed_pid, 1.0f);
pid_set_derivative_lpf(&speed_pid, 0.35f);

float correction = pid_update(&speed_pid, target_rpm - actual_rpm, 0.01f);
```

---

### 7.7 协作式调度器 — ec_scheduler

**文件**: [ec_scheduler.h](user/inc/framework/ec_scheduler.h)

> 基于优先级的协作式任务调度器，支持周期任务执行和运行时统计。

```c
#define EC_SCHEDULER_MAX_TASKS  12u

typedef void (*ec_task_fn)(uint32_t now_ms, void *context);

typedef struct {
    const char *name;
    ec_task_fn run;
    void *context;
    uint32_t period_ms;
    uint32_t next_run_ms;
    // 运行时统计
    uint32_t run_count;
    uint32_t missed_deadlines;
    uint32_t last_start_lateness_ms;
    uint32_t max_start_lateness_ms;
    uint32_t last_runtime_ms;
    uint32_t max_runtime_ms;
    uint32_t overrun_count;
    bool enabled;
} ec_task_t;

typedef struct {
    ec_task_t tasks[EC_SCHEDULER_MAX_TASKS];
    uint8_t count;
} ec_scheduler_t;

// API
void ec_scheduler_init(ec_scheduler_t *scheduler);
bool ec_scheduler_add(ec_scheduler_t *scheduler, const char *name, ec_task_fn run,
                      void *context, uint32_t period_ms, uint32_t start_ms);
void ec_scheduler_run(ec_scheduler_t *scheduler, uint32_t now_ms);
```

**使用示例**:
```c
ec_scheduler_t scheduler;
ec_scheduler_init(&scheduler);

void sensor_task(uint32_t now, void *ctx) { /* 传感器读取 */ }
void control_task(uint32_t now, void *ctx) { /* PID控制 */ }

ec_scheduler_add(&scheduler, "sensor",  sensor_task,  NULL, 10, 0);  // 每10ms
ec_scheduler_add(&scheduler, "control", control_task, NULL, 10, 5);  // 每10ms, 偏移5ms

// 主循环
while (1) {
    ec_scheduler_run(&scheduler, system_time_ms());
}
```

---

### 7.8 串口接收环形缓冲区 — serial_rx_buffer

**文件**: [serial_rx_buffer.h](user/inc/lib/serial_rx_buffer.h)

> 线程安全的环形缓冲区，专为 UART ISR → 主循环 数据传递设计。

```c
typedef struct {
    uint8_t *data;
    size_t capacity;
    volatile size_t head;
    volatile size_t tail;
    volatile size_t overflow_count;
    uint32_t *timestamps_ms;      // 可选时间戳数组
} SerialRxBuffer;

// API
void   serial_rx_buffer_init(SerialRxBuffer *buf, uint8_t *storage, size_t capacity);
void   serial_rx_buffer_init_timed(SerialRxBuffer *buf, uint8_t *storage,
                                   uint32_t *ts_storage, size_t capacity);
void   serial_rx_buffer_clear(SerialRxBuffer *buf);
bool   serial_rx_buffer_push(SerialRxBuffer *buf, uint8_t byte);
bool   serial_rx_buffer_push_timed(SerialRxBuffer *buf, uint8_t byte, uint32_t rx_time_ms);
bool   serial_rx_buffer_pop(SerialRxBuffer *buf, uint8_t *byte);
bool   serial_rx_buffer_pop_timed(SerialRxBuffer *buf, uint8_t *byte, uint32_t *rx_time_ms);
bool   serial_rx_buffer_peek(const SerialRxBuffer *buf, size_t offset, uint8_t *byte);
size_t serial_rx_buffer_available(const SerialRxBuffer *buf);
size_t serial_rx_buffer_overflow_count(const SerialRxBuffer *buf);
void   serial_rx_buffer_drop(SerialRxBuffer *buf, size_t length);
```

---

### 7.9 时间管理 — ec_time

**文件**: [ec_time.h](user/inc/framework/ec_time.h)

> 系统毫秒/微秒时间管理。

```c
uint32_t ec_time_ms(void);   // 获取系统毫秒时间戳
uint32_t ec_time_us(void);   // 获取系统微秒时间戳
void     ec_time_init(void); // 初始化
```

---

### 7.10 菜单框架 — ec_menu / ec_parameter_menu

**文件**: 
- [ec_menu.h](user/inc/framework/ec_menu.h) — 导航菜单系统
- [ec_parameter_menu.h](user/inc/framework/ec_parameter_menu.h) — 参数调节菜单

关键接口：
```c
// ec_menu — 多级菜单导航
void ec_menu_init(void);
void ec_menu_handle_key(key_index_enum key);
void ec_menu_display(void);

// ec_parameter_menu — 参数在线调节
void ec_parameter_menu_init(void);
void ec_parameter_menu_register(const char *name, float *value, float min, float max, float step);
void ec_parameter_menu_handle_input(char cmd);
```

---

### 7.11 模式管理 — ec_mode_manager / ec_app

**文件**:
- [ec_mode_manager.h](user/inc/framework/ec_mode_manager.h) — 运行模式切换
- [ec_app.h](user/inc/framework/ec_app.h) — 应用注册和生命周期

关键接口：
```c
// ec_app — 应用模板
void ec_app_init(void);
void ec_app_run(void);

// ec_mode_manager — 模式切换
typedef enum { MODE_STOP, MODE_RUN, MODE_TUNE, MODE_TEST } ec_mode_t;
void ec_mode_manager_init(void);
void ec_mode_manager_set(ec_mode_t mode);
ec_mode_t ec_mode_manager_get(void);
```

---

### 7.12 按键框架 — ec_keys

**文件**: [ec_keys.h](user/inc/framework/ec_keys.h)

> 对 zf_device_key 的二次封装，提供按键事件回调注册。

```c
void ec_keys_init(void);
void ec_keys_register_callback(key_index_enum key, key_state_enum state, void (*cb)(void));
void ec_keys_process(void);   // 主循环中调用
```

---

## 8. 快速参考速查表

### 8.1 常用 GPIO 引脚映射

| 功能 | 引脚 | 说明 |
|------|------|------|
| LED1 | PA30 | 板载红色 |
| LED2 | PA7 | 板载绿色 |
| LED3 | PB27 | 板载蓝色 |
| RGB_R | PB0 | RGB 红灯 |
| RGB_G | PB1 | RGB 绿灯 |
| RGB_B | PA29 | 蓝灯（非标准RGB蓝） |
| KEY1 | PB6 | 按键1 |
| KEY2 | PB7 | 按键2 |
| KEY3 | PB23 | 按键3 |
| BUZZER | PA31 | 蜂鸣器 |
| RELAY | PA28 | 继电器 |
| OLED_SCL | PB9 | OLED I2C |
| OLED_SDA | PB8 | OLED I2C |
| MPU6050_SCL | PA1 | MPU6050 I2C |
| MPU6050_SDA | PA0 | MPU6050 I2C |

### 8.2 PWM 通道速查

| 定时器 | CH0 | CH1 | CH2 | CH3 |
|--------|-----|-----|-----|-----|
| TIMA0 | PA0,PA8,PA21,PB8,PB14 | PA1,PA3,PA7,PA9,PA22,PB9,PB12,PB20 | PA3,PA7,PA10,PA15,PB0,PB4,PB12,PB17,PB20 | PA4,PA12,PA17,PA23,PA25,PA28,PB2,PB13,PB24,PB26 |
| TIMA1 | PA10,PA15,PA17,PA28,PB0,PB2,PB4,PB17,PB26 | PA11,PA16,PA18,PA24,PA31,PB1,PB3,PB5,PB18,PB27 | — | — |
| TIMG0 | PA5,PA12,PA23,PB10 | PA6,PA13,PA24,PB11 | — | — |
| TIMG6 | PA5,PA21,PA29,PB2,PB6,PB10,PB26 | PA6,PA22,PA30,PB3,PB7,PB11,PB27 | — | — |
| TIMG7 | PA3,PA17,PA23,PA26,PA28,PB15 | PA2,PA4,PA7,PA18,PA24,PA27,PA31,PB16,PB19 | — | — |
| TIMG8 | PA1,PA3,PA5,PA7,PA21,PA23,PA26,PA29,PB6,PB10,PB15,PB21 | PA0,PA2,PA4,PA6,PA22,PA27,PA30,PB7,PB11,PB16,PB19,PB22 | — | — |
| TIMG12 | PA10,PA14,PB13,PB20 | PA25,PA31,PB14,PB24 | — | — |

### 8.3 UART 引脚速查

| UART | TX 引脚 | RX 引脚 |
|------|---------|---------|
| UART0 | PA0, PA10, PA28, PB0 | PA1, PA11, PA31, PB1 |
| UART1 | PA8, PA17, PB4, PB6 | PA9, PA18, PB5, PB7 |
| UART2 | PA21, PA23, PB15, PB17 | PA22, PA24, PB16, PB18 |
| UART3 | PA14, PA26, PB2, PB12 | PA13, PA25, PB3, PB13 |

### 8.4 定时器资源分配

| 定时器 | 本工程用途 |
|--------|-----------|
| TIMA0 | 电机 PWM (CH0/CH1/CH2/CH3) |
| TIMA1 | 备用 |
| TIMG0 | 备用 |
| TIMG6 | PIT (10ms 传感器扫描) |
| TIMG7 | 备用 |
| TIMG8 | 编码器1 (硬件正交解码) / TSL1401 PIT |
| TIMG12 | 编码器2 (GPIO 中断解码) |

---

## 9. 典型应用模板

### 9.1 最小工程模板

```c
#include "zf_common_headfile.h"
#include "config.h"
#include "pin_mapping.h"

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);     // 1. 时钟 80MHz
    interrupt_init();                  // 2. 中断初始化
    debug_init();                      // 3. 调试串口
    system_delay_ms(100);             // 4. 等待外设稳定

    gpio_init(PIN_LED, GPO, GPIO_HIGH, GPO_PUSH_PULL);  // 5. 初始化外设

    while (1) {
        gpio_toggle_level(PIN_LED);
        system_delay_ms(500);
    }
}
```

### 9.2 UART 中断接收模板

```c
static uint8_t uart_rx_buf[256];
static SerialRxBuffer uart_ring;

void uart_rx_isr(uint32_t state, void *ptr) {
    uint8_t byte;
    (void)ptr;
    if ((state & UART_INTERRUPT_STATE_RX) == 0) return;
    while (uart_query_byte(UART_2, &byte) == ZF_TRUE)
        serial_rx_buffer_push(&uart_ring, byte);
}

int main(void) {
    // ... 基础初始化 ...
    serial_rx_buffer_init(&uart_ring, uart_rx_buf, sizeof(uart_rx_buf));
    uart_init(UART_2, 115200, UART2_TX_B15, UART2_RX_B16);
    uart_set_callback(UART_2, uart_rx_isr, NULL);
    uart_set_interrupt_config(UART_2, UART_INTERRUPT_CONFIG_RX_ENABLE);

    while (1) {
        uint8_t byte;
        while (serial_rx_buffer_pop(&uart_ring, &byte)) {
            // 处理接收数据
        }
    }
}
```

### 9.3 PIT + 调度器模板

```c
ec_scheduler_t sched;

void sensor_task(uint32_t now, void *ctx) {
    static uint32_t last = 0;
    if (now - last >= 10) {
        last = now;
        // 10ms 传感器采集
    }
}

void control_task(uint32_t now, void *ctx) {
    // PID 控制
}

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    interrupt_init();
    debug_init();
    ec_scheduler_init(&sched);
    ec_scheduler_add(&sched, "sensor",  sensor_task,  NULL, 10, 0);
    ec_scheduler_add(&sched, "control", control_task, NULL, 10, 5);

    while (1) {
        ec_scheduler_run(&sched, ec_time_ms());
    }
}
```

### 9.4 电机 + 编码器 + PID 完整控制模板

```c
dt_motor_config_t left_motor  = { .in1_pin=MOTOR_L_IN1, .in2_pin=MOTOR_L_IN2, .pwm_freq=20000 };
dt_motor_config_t right_motor = { .in1_pin=MOTOR_R_IN1, .in2_pin=MOTOR_R_IN2, .pwm_freq=20000 };

dt_encoder_t left_enc = {
    .a_pin=A26, .b_pin=A27, .counts_per_rev=ENCODER_CPR,
    .wheel_circumference_mm=204.2f, .quadrature_enabled=true,
    .hardware_quadrature=true, .timer_index=TIM_G8,
    .channel_a=TIMG8_ENCODER1_CH1_A26,
    .channel_b=TIMG8_ENCODER1_CH2_A27,
    .direction_sign=1, .rpm_lpf_alpha=0.55f,
};

PidController speed_pid;

void control_loop(uint32_t now, void *ctx) {
    float dt = 0.01f;  // 10ms
    float actual_rpm = dt_encoder_compute_signed_rpm(&left_enc, 10);
    float correction = pid_update(&speed_pid, 40.0f - actual_rpm, dt);
    dt_motor_set_speed(&left_motor, (int16_t)(4500 + correction));
}

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    interrupt_init();
    debug_init();

    dt_motor_init(&left_motor);
    dt_motor_init(&right_motor);
    dt_encoder_init(&left_enc);

    pid_init(&speed_pid);
    pid_set_gain(&speed_pid, 35.0f, 15.0f, 0.0f);
    pid_set_limits(&speed_pid, -2000, 2000, -3000, 3000);

    // 注册 10ms PIT 中断触发 control_loop
    // ...

    while (1) { /* 主循环 */ }
}
```

---

> **版本**: v1.0 | **生成日期**: 2026-07-20  
> **基于**: MSPM0G3507 ZF 逐飞开源库 (2025-06) | **适用**: 天猛星 MSPM0G3507 开发板  
> **维护**: 本文档基于实际代码分析生成，所有 API 签名均来自对应头文件
> **状态**: ⚠️ 已拆分，请使用 [ZF_SEEKFREE_API_MANUAL.md](ZF_SEEKFREE_API_MANUAL.md) 和 [ZF_USER_API_MANUAL.md](ZF_USER_API_MANUAL.md)

</details>
