# MSPM0G3507 逐飞 SeekFree 底层库 API 参考手册

> **层归属**: 逐飞科技官方库 (SeekFree/)  
> **平台**: MSPM0G3507 (TI ARM Cortex-M0+)  
> **许可证**: GPL3.0  
> **版本**: 2025-06-1 | **生成日期**: 2026-07-20  
> **被依赖**: 本层直接被 [用户层 API 手册](ZF_USER_API_MANUAL.md) 调用  

---

## 目录

1. [架构定位](#1-架构定位)
2. [类型定义层 — zf_common_typedef](#2-类型定义层--zf_common_typedef)
3. [公共工具层 — zf_common](#3-公共工具层--zf_common)
   - [3.1 系统时钟 — zf_common_clock](#31-系统时钟--zf_common_clock)
   - [3.2 调试输出 — zf_common_debug](#32-调试输出--zf_common_debug)
   - [3.3 FIFO 队列 — zf_common_fifo](#33-fifo-队列--zf_common_fifo)
   - [3.4 字体颜色 — zf_common_font](#34-字体颜色--zf_common_font)
   - [3.5 实用函数 — zf_common_function](#35-实用函数--zf_common_function)
   - [3.6 中断控制 — zf_common_interrupt](#36-中断控制--zf_common_interrupt)
4. [外设驱动层 — zf_driver](#4-外设驱动层--zf_driver)
   - [4.01 GPIO — zf_driver_gpio](#401-gpio--zf_driver_gpio)
   - [4.02 PWM — zf_driver_pwm](#402-pwm--zf_driver_pwm)
   - [4.03 UART — zf_driver_uart](#403-uart--zf_driver_uart)
   - [4.04 ADC — zf_driver_adc](#404-adc--zf_driver_adc)
   - [4.05 延时 — zf_driver_delay](#405-延时--zf_driver_delay)
   - [4.06 定时器 — zf_driver_timer](#406-定时器--zf_driver_timer)
   - [4.07 PIT 周期中断 — zf_driver_pit](#407-pit-周期中断--zf_driver_pit)
   - [4.08 SPI — zf_driver_spi](#408-spi--zf_driver_spi)
   - [4.09 软件 I2C — zf_driver_soft_iic](#409-软件-i2c--zf_driver_soft_iic)
   - [4.10 外部中断 EXTI — zf_driver_exti](#410-外部中断-exti--zf_driver_exti)
   - [4.11 Flash — zf_driver_flash](#411-flash--zf_driver_flash)
   - [4.12 编码器 — zf_driver_encoder](#412-编码器--zf_driver_encoder)
5. [设备层 — zf_device](#5-设备层--zf_device)
   - [5.1 IMU660RA 六轴传感器](#51-imu660ra-六轴传感器)
   - [5.2 IMU660RB 六轴传感器](#52-imu660rb-六轴传感器)
   - [5.3 IMU660RC 六轴传感器（含四元数）](#53-imu660rc-六轴传感器含四元数)
   - [5.4 IMU963RA 九轴传感器](#54-imu963ra-九轴传感器)
   - [5.5 OLED SPI 屏](#55-oled-spi-屏)
   - [5.6 TFT180 彩屏](#56-tft180-彩屏)
   - [5.7 IPS114 彩屏](#57-ips114-彩屏)
   - [5.8 IPS200 彩屏](#58-ips200-彩屏)
   - [5.9 IPS200PRO 智能屏](#59-ips200pro-智能屏)
   - [5.10 TSL1401 线阵 CCD](#510-tsl1401-线阵-ccd)
   - [5.11 DL1A / DL1B 激光测距](#511-dl1a--dl1b-激光测距)
   - [5.12 GS08RA 灰度传感器](#512-gs08ra-灰度传感器)
   - [5.13 按键管理](#513-按键管理)
   - [5.14 绝对式编码器](#514-绝对式编码器)
   - [5.15 无线串口模块](#515-无线串口模块)
   - [5.16 WiFi SPI / UART 模块](#516-wifi-spi--uart-模块)
   - [5.17 设备类型管理](#517-设备类型管理)
6. [组件层 — zf_components](#6-组件层--zf_components)
   - [6.1 逐飞助手上位机协议](#61-逐飞助手上位机协议)
   - [6.2 上位机接口适配](#62-上位机接口适配)
7. [资源速查表](#7-资源速查表)
   - [7.1 GPIO 引脚索引](#71-gpio-引脚索引)
   - [7.2 PWM 通道索引](#72-pwm-通道索引)
   - [7.3 UART 引脚索引](#73-uart-引脚索引)
   - [7.4 定时器资源索引](#74-定时器资源索引)
8. [与用户层的边界约定](#8-与用户层的边界约定)

---

## 1. 架构定位

```
┌──────────────────────────────────────────────────┐
│  user/inc/  用户层 API（见 ZF_USER_API_MANUAL）  │
│  ┌──────────────────────────────────────────────┐│
│  │        zf_components  组件层                 ││
│  │  ┌────────────────────────────────────────┐  ││
│  │  │       zf_device  外设设备层            │  ││
│  │  │  ┌──────────────────────────────────┐  │  ││
│  │  │  │     zf_driver  外设驱动层        │  │  ││
│  │  │  │  ┌────────────────────────────┐  │  │  ││
│  │  │  │  │  zf_common  公共工具层     │  │  │  ││
│  │  │  │  │  ┌──────────────────────┐  │  │  │  ││
│  │  │  │  │  │ ti_msp_dl_config.h   │  │  │  │  ││
│  │  │  │  │  │ (TI SDK 寄存器层)    │  │  │  │  ││
│  │  │  │  │  └──────────────────────┘  │  │  │  ││
│  │  │  │  └────────────────────────────┘  │  │  ││
│  │  │  └──────────────────────────────────┘  │  ││
│  │  └────────────────────────────────────────┘  ││
│  └──────────────────────────────────────────────┘│
└──────────────────────────────────────────────────┘
```

**本手册覆盖范围**: `SeekFree/zf_common/` + `SeekFree/zf_driver/` + `SeekFree/zf_device/` + `SeekFree/zf_components/` + `SeekFree/dl_timer.c`

**不覆盖**: `user/inc/`, `user/src/` — 这些属于 [用户层 API 手册](ZF_USER_API_MANUAL.md)

### 各子层职责

| 子层 | 目录 | 职责 |
|------|------|------|
| **zf_common** | `SeekFree/zf_common/` | 类型定义、系统时钟、调试、FIFO、字符串转换、中断NVIC |
| **zf_driver** | `SeekFree/zf_driver/` | 直接封装 TI SDK 寄存器，提供 GPIO/PWM/UART/ADC/SPI/I2C 等外设 API |
| **zf_device** | `SeekFree/zf_device/` | 外部传感器/模块驱动（IMU、OLED、ToF、CCD 等），依赖 zf_driver |
| **zf_components** | `SeekFree/zf_components/` | 上位机通信协议、数据可视化 |

---

## 2. 类型定义层 — zf_common_typedef

**文件**: [SeekFree/zf_common/zf_common_typedef.h](SeekFree/zf_common/zf_common_typedef.h)

### 2.1 基础类型

```c
typedef unsigned char       uint8;       // 无符号  8 bits
typedef unsigned short int  uint16;      // 无符号 16 bits
typedef unsigned int        uint32;      // 无符号 32 bits
typedef unsigned long long  uint64;      // 无符号 64 bits

typedef signed char         int8;        // 有符号  8 bits
typedef signed short int    int16;       // 有符号 16 bits
typedef signed int          int32;       // 有符号 32 bits
typedef signed long long    int64;       // 有符号 64 bits

typedef volatile uint8      vuint8;      // volatile 无符号 8
typedef volatile uint16     vuint16;     // volatile 无符号 16
typedef volatile uint32     vuint32;     // volatile 无符号 32
typedef volatile uint64     vuint64;     // volatile 无符号 64
typedef volatile int8       vint8;       // volatile 有符号 8
typedef volatile int16      vint16;      // volatile 有符号 16
typedef volatile int32      vint32;      // volatile 有符号 32
typedef volatile int64      vint64;      // volatile 有符号 64
```

可通过 `#define USE_ZF_TYPEDEF 0` 禁用别名。

### 2.2 通用状态宏

```c
#define ZF_NO_ERROR     ( 0 )     // 无异常（逐飞库所有函数的标准成功返回值）
#define ZF_ERROR        ( 1 )     // 异常码
#define ZF_ENABLE       ( 1 )
#define ZF_DISABLE      ( 0 )
#define ZF_TRUE         ( 1 )
#define ZF_FALSE        ( 0 )
```

### 2.3 数据位宽枚举

```c
typedef enum {
    COMMON_DATA_SIZE_8BIT  = 1,
    COMMON_DATA_SIZE_16BIT = 2,
    COMMON_DATA_SIZE_32BIT = 4,
} common_data_size_enum;
```

### 2.4 函数指针类型

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
typedef void   (*void_callback_uint32_ptr)(uint32 state, void *ptr);  // 回调签名（中断/事件）
```

### 2.5 IDE 多平台适配

```c
#define IDE_MDK  (0x01)    // Keil MDK (ARMCC/GNUC)
#define IDE_IAR  (0x02)    // IAR (ICCARM)
#define IDE_ADS  (0x04)    // ADS
#define IDE_MRS  (0x08)    // MRS

// 自动检测
#if defined(__ICCARM__)
#define IDE_TYPE ( IDE_IAR )
#else
#define IDE_TYPE ( IDE_MDK )
#endif

// 通用修饰
#define ZF_INLINE   static inline
#define ZF_WEAK     __attribute__((weak))
#define ZF_PACKED   __attribute__((packed))
#define ZF_DSB()    __DSB()
#define ZF_ISB()    __ISB()
#define ZF_DMB()    __DMB()
#define ZF_FILE_MESSAGE  ( __FILE__ )
#define ZF_LINE_MESSAGE  ( __LINE__ )
```

---

## 3. 公共工具层 — zf_common

### 3.1 系统时钟 — zf_common_clock

**文件**: [SeekFree/zf_common/zf_common_clock.h](SeekFree/zf_common/zf_common_clock.h)

```c
#define BOARD_XTAL_FREQ      ( 40000000 )        // 板载晶振 40MHz
#define XTAL_STARTUP_TIMEOUT ( 0x0F00 )           // 起振超时

typedef enum {
    SYSTEM_CLOCK_80M = 80000000,                 // 80MHz（唯一支持的选项）
} system_clock_enum;

extern uint32 system_clock;                      // 当前 AHB 系统时钟 (Hz) — 默认 80M
extern uint32 bus_clock;                         // 当前 APB 总线时钟 (Hz) — 默认 40M

void clock_init(uint32 clock);                   // 初始化系统时钟（参数填 SYSTEM_CLOCK_80M）
```

**实现细节**: `clock_init` 内部调用 `clock_reset()`（重置并上电所有外设）、`SYSCFG_DL_init()`（TI SDK 时钟树）和 `interrupt_init()`。`clock` 参数实际被忽略，所有时钟配置由 SysConfig 工具决定。

**使用**:
```c
clock_init(SYSTEM_CLOCK_80M);  // 系统启动时最先调用
```

---

### 3.2 调试输出 — zf_common_debug

**文件**: [SeekFree/zf_common/zf_common_debug.h](SeekFree/zf_common/zf_common_debug.h)

```c
// 调试串口（按 EC_APP_PROFILE 自动切换）
// Line-car: UART2 (蓝牙) | 其他: UART1 (PA8/PA9)
#define DEBUG_UART_BAUDRATE  ( 115200 )
#define DEBUG_RING_BUFFER_LEN ( 64 )

// 断言 — 条件为假时输出文件和行号
#define zf_assert(x)  ( debug_assert_handler((x), __FILE__, __LINE__) )

// 日志 — 条件为假时输出消息
#define zf_log(x, str)  ( debug_log_handler((x), (str), __FILE__, __LINE__) )

// 调试输出结构体（支持 UART + 屏幕双输出）
typedef struct {
    uint16 type_index;
    uint16 display_x_max, display_y_max;
    uint8  font_x_size, font_y_size;
    void (*output_uart)(const char *str);
    void (*output_screen)(uint16 x, uint16 y, const char *str);
    void (*output_screen_clear)(void);
} debug_output_struct;

void   debug_init(void);
void   debug_assert_enable(void);
void   debug_assert_disable(void);
void   debug_assert_handler(uint8 pass, char *file, int line);
void   debug_log_handler(uint8 pass, char *str, char *file, int line);
void   debug_output_struct_init(debug_output_struct *info);
void   debug_output_init(debug_output_struct *info);
uint32 debug_send_buffer(const uint8 *buff, uint32 len);
uint32 debug_read_ring_buffer(uint8 *buff, uint32 len);
void   debug_interrupr_handler(void);    // UART RX ISR — 写入环形缓冲区

// 标准 IO 重定向
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
```

**使用**:
```c
debug_init();
zf_assert(ptr != NULL);                         // 断言失败 → 循环打印错误
zf_log(ret != 0, "UART init failed");           // 条件为假 → 输出日志到调试串口
printf("Value = %d\r\n", value);                // printf 重定向到调试 UART
```

---

### 3.3 FIFO 队列 — zf_common_fifo

**文件**: [SeekFree/zf_common/zf_common_fifo.h](SeekFree/zf_common/zf_common_fifo.h)

线程安全的环形 FIFO，通过 PRIMASK 中断禁用做临界区保护。

```c
// 状态码
typedef enum {
    FIFO_SUCCESS, FIFO_RESET_UNDO, FIFO_CLEAR_UNDO, FIFO_BUFFER_NULL,
    FIFO_WRITE_UNDO, FIFO_SPACE_NO_ENOUGH,
    FIFO_READ_UNDO, FIFO_DATA_NO_ENOUGH,
} fifo_state_enum;

// 执行锁（防中断嵌套）
typedef enum {
    FIFO_IDLE = 0x00, FIFO_RESET = 0x01, FIFO_CLEAR = 0x02,
    FIFO_WRITE = 0x04, FIFO_READ  = 0x08,
} fifo_execution_enum;

// 读模式
typedef enum {
    FIFO_READ_AND_CLEAN,     // 读后释放空间
    FIFO_READ_ONLY,          // 只读不释放（peek）
} fifo_operation_enum;

// 位宽
typedef enum {
    FIFO_DATA_8BIT, FIFO_DATA_16BIT, FIFO_DATA_32BIT,
} fifo_data_type_enum;

// 核心结构体
typedef struct __attribute__((packed)) {
    volatile uint8      execution;    // 当前操作锁
    fifo_data_type_enum type;         // 元素位宽
    void                *buffer;      // 数据区指针
    volatile uint32     head;         // 写指针（生产者）
    volatile uint32     end;          // 读指针（消费者）
    volatile uint32     size;         // 剩余可写空间
    uint32              max;          // 总容量（元素个数）
} fifo_struct;

// API
fifo_state_enum fifo_init(fifo_struct *fifo, fifo_data_type_enum type,
                          void *buffer_addr, uint32 size);
fifo_state_enum fifo_clear(fifo_struct *fifo);
uint32          fifo_used(fifo_struct *fifo);
fifo_state_enum fifo_write_element(fifo_struct *fifo, uint32 dat);
fifo_state_enum fifo_write_buffer(fifo_struct *fifo, void *dat, uint32 length);
fifo_state_enum fifo_read_element(fifo_struct *fifo, void *dat, fifo_operation_enum flag);
fifo_state_enum fifo_read_buffer(fifo_struct *fifo, void *dat, uint32 *length, fifo_operation_enum flag);
fifo_state_enum fifo_read_tail_buffer(fifo_struct *fifo, void *dat, uint32 *length, fifo_operation_enum flag);
```

---

### 3.4 字体颜色 — zf_common_font

**文件**: [SeekFree/zf_common/zf_common_font.h](SeekFree/zf_common/zf_common_font.h)

```c
// RGB565 常用色
typedef enum {
    RGB565_WHITE   = 0xFFFF,  RGB565_BLACK   = 0x0000,
    RGB565_BLUE    = 0x001F,  RGB565_PURPLE  = 0xF81F,
    RGB565_PINK    = 0xFE19,  RGB565_RED     = 0xF800,
    RGB565_MAGENTA = 0xF81F,  RGB565_GREEN   = 0x07E0,
    RGB565_CYAN    = 0x07FF,  RGB565_YELLOW  = 0xFFE0,
    RGB565_BROWN   = 0xBC40,  RGB565_GRAY    = 0x8430,
} rgb565_color_enum;

// 字库数据
extern const uint8 ascii_font_8x16[][16];       // 8×16 ASCII（95字符，0x20-0x7E）
extern const uint8 ascii_font_6x8[][6];          // 6×8  ASCII
extern const uint8 oled_16x16_chinese[][16];     // 16×16 中文（"逐飞科技"）
extern const uint8 gImage_seekfree_logo[38400];  // 逐飞 LOGO 240×80 RGB565
```

---

### 3.5 实用函数 — zf_common_function

**文件**: [SeekFree/zf_common/zf_common_function.h](SeekFree/zf_common/zf_common_function.h)

```c
// 宏函数（无调用开销）
#define func_abs(x)            ((x) >= 0 ? (x) : -(x))                   // 绝对值
#define func_limit(x, y)       ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x))) // 双边限幅
#define func_limit_ab(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))    // 区间限幅

// 正弦表
void   func_get_sin_amplitude_table(uint32 *buf, uint32 sample_max,
                                    uint32 amplitude_max, uint32 offset_deg);
uint32 func_get_greatest_common_divisor(uint32 num1, uint32 num2);
void   func_soft_delay(volatile long t);           // 忙等待延时

// 字符串转换
int32  func_str_to_int(char *str);
void   func_int_to_str(char *str, int32 number);
uint32 func_str_to_uint(char *str);
void   func_uint_to_str(char *str, uint32 number);
float  func_str_to_float(char *str);
void   func_float_to_str(char *str, float number, uint8 point_bit);
double func_str_to_double(char *str);
void   func_double_to_str(char *str, double number, uint8 point_bit);
uint32 func_str_to_hex(char *str);
void   func_hex_to_str(char *str, uint32 number);

// 嵌入式 printf
uint32 zf_sprintf(int8 *buff, const int8 *format, ...); // 支持 %c %d %u %x %s %f %p %%
```

---

### 3.6 中断控制 — zf_common_interrupt

**文件**: [SeekFree/zf_common/zf_common_interrupt.h](SeekFree/zf_common/zf_common_interrupt.h)

```c
void   interrupt_global_enable(uint32 primask);   // 恢复全局中断（传入 disable 的返回值）
uint32 interrupt_global_disable(void);            // 关闭全局中断，返回 PRIMASK（支持嵌套）
void   interrupt_enable(IRQn_Type irqn);          // 使能指定 NVIC 中断
void   interrupt_disable(IRQn_Type irqn);         // 禁止指定 NVIC 中断
void   interrupt_set_priority(IRQn_Type irqn, uint8 priority); // 优先级 0-7（值越小越高）
void   interrupt_init(void);                      // NVIC 优先级分组 = 4（16 组, 0 子优先级）
```

**使用**:
```c
uint32 primask = interrupt_global_disable();    // 进入临界区
/* ... 不可中断代码 ... */
interrupt_global_enable(primask);               // 退出临界区（嵌套安全）
```

---

## 4. 外设驱动层 — zf_driver

> 驱动层所有函数直接操作 MSPM0G3507 外设寄存器。参数中的 `pin` 枚举值通过位域编码了寄存器索引和位偏移。

### 4.01 GPIO — zf_driver_gpio

**文件**: [SeekFree/zf_driver/zf_driver_gpio.h](SeekFree/zf_driver/zf_driver_gpio.h)

#### 引脚枚举

```c
// gpio_pin_enum: A0=0, A1=1 ... A31=31, B0=32, B1=33 ... B27=59
typedef enum {
    A0 = 0,  A1,  A2,  A3,  A4,  A5,  A6,  A7,
    A8,  A9,  A10, A11, A12, A13, A14, A15,
    A16, A17, A18, A19, A20, A21, A22, A23,
    A24, A25, A26, A27, A28, A29, A30, A31,
    B0 = 32, B1,  B2,  B3,  B4,  B5,  B6,  B7,
    B8,  B9,  B10, B11, B12, B13, B14, B15,
    B16, B17, B18, B19, B20, B21, B22, B23,
    B24, B25, B26, B27,
    GPIO_MAX,
} gpio_pin_enum;
```

#### 模式和电平

```c
typedef enum {
    GPI = 0x00,   // 输入
    GPO = 0x10,   // 输出
} gpio_dir_enum;

typedef enum {
    GPI_ANAOG_IN      = 0x01,  // 模拟输入 (ADC)
    GPI_FLOATING_IN   = 0x02,  // 浮空输入
    GPI_PULL_DOWN     = 0x03,  // 下拉输入
    GPI_PULL_UP       = 0x04,  // 上拉输入
    GPO_PUSH_PULL     = 0x11,  // 推挽输出
    GPO_OPEN_DTAIN    = 0x12,  // 开漏输出
    GPO_AF_PUSH_PULL  = 0x13,  // 复用推挽输出 (UART/SPI/I2C)
    GPO_AF_OPEN_DTAIN = 0x14,  // 复用开漏输出
} gpio_mode_enum;

typedef enum {
    GPIO_AF0  = 0,  GPIO_AF1,  GPIO_AF2,  GPIO_AF3,
    GPIO_AF4,  GPIO_AF5,  GPIO_AF6,  GPIO_AF7,
    GPIO_AF8,  GPIO_AF9,  GPIO_AF10, GPIO_AF11,
    GPIO_AF12, GPIO_AF13, GPIO_AF14, GPIO_AF15,
} gpio_af_enum;

typedef enum {
    GPIO_LOW  = 0x00,
    GPIO_HIGH = 0x01,
} gpio_level_enum;
```

#### API

```c
// 宏 — 零开销引脚翻转（直接写寄存器）
#define gpio_high(pin)  (gpio_group[...]->DOUTSET31_0 |= (1 << (pin & 0x1F)))
#define gpio_low(pin)   (gpio_group[...]->DOUTCLR31_0 |= (1 << (pin & 0x1F)))

// 函数
void  gpio_set_level(gpio_pin_enum pin, const uint8 dat);   // gpio_high/gpio_low 的函数式等效
uint8 gpio_get_level(gpio_pin_enum pin);                    // 读取输入电平
void  gpio_toggle_level(gpio_pin_enum pin);                 // 翻转输出
void  gpio_set_dir(gpio_pin_enum pin, gpio_dir_enum dir, gpio_mode_enum mode);

// 初始化
void  gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, const uint8 dat, gpio_mode_enum mode);
void  afio_init(gpio_pin_enum pin, gpio_dir_enum dir, gpio_af_enum af, gpio_mode_enum mode);
```

**使用**:
```c
gpio_init(B6, GPI, 0, GPI_PULL_UP);            // PB6 上拉输入（按键）
uint8 key = gpio_get_level(B6);                 // 读按键

gpio_init(A31, GPO, GPIO_LOW, GPO_PUSH_PULL);  // PA31 推挽输出，初始低
gpio_high(A31);                                 // 输出高（宏，无函数调用开销）
gpio_toggle_level(A31);                         // 翻转
```

**寄存器操作**: `DOUTSET31_0`/`DOUTCLR31_0` 原子写（无 RMW）、`DOESET31_0`/`DOECLR31_0` 方向控制、`IOMUX->SECCFG.PINCM[]` 引脚配置。

---

### 4.02 PWM — zf_driver_pwm

**文件**: [SeekFree/zf_driver/zf_driver_pwm.h](SeekFree/zf_driver/zf_driver_pwm.h)

```c
#define PWM_DUTY_MAX  ( 8000 )     // 占空比上限 0~8000

// 定时器索引
typedef enum {
    PWM_TIM_A0,       // TIMA0 — 4通道，支持互补输出
    PWM_TIM_A1,       // TIMA1 — 2通道
    PWM_TIM_G0,       // TIMG0 — 2通道
    PWM_TIM_G6,       // TIMG6 — 2通道
    PWM_TIM_G7,       // TIMG7 — 2通道
    PWM_TIM_G8,       // TIMG8 — 2通道
    PWM_TIM_G12,      // TIMG12— 2通道
} pwm_index_enum;

// PWM 引脚枚举 — 每条枚举值编码了 (定时器, 通道, AF功能, GPIO 引脚)
// 命名规则: PWM_TIM{A|G}{n}_CH{m}_{port}{pin}
// 如 PWM_TIM_A0_CH0_A8 = TIMA0 CH0, PA8, AF5
typedef enum {
    // TIMA0 CH0:  A0/AF4, A8/AF5, A21/AF5, B8/AF4, B14/AF7
    PWM_TIM_A0_CH0_A8,  PWM_TIM_A0_CH0_A21, PWM_TIM_A0_CH0_B8, ...
    // TIMA0 CH1:  A9/AF5, B20/AF7, ...
    PWM_TIM_A0_CH1_A9,  PWM_TIM_A0_CH1_B20, ...
    // TIMA0 CH2:  A15/AF8, ...
    // TIMA0 CH3:  A17/AF5, ...
    // TIMA1/TIMG0~G12 各通道同理
} pwm_channel_enum;

// API
void pwm_set_duty(pwm_channel_enum pin, const uint32 duty);   // 占空比 0~8000
void pwm_force_low(pwm_channel_enum pin);                      // 强制拉低输出
void pwm_init(pwm_channel_enum pin, const uint32 freq, const uint32 duty);
```

**使用**:
```c
// 四路电机 PWM（TIMA0 四个通道）
pwm_init(PWM_TIM_A0_CH0_A8,  20000, 0);    // CH0: PA8,  20kHz
pwm_init(PWM_TIM_A0_CH1_B20, 20000, 0);    // CH1: PB20, 20kHz
pwm_init(PWM_TIM_A0_CH2_A15, 20000, 0);    // CH2: PA15, 20kHz
pwm_init(PWM_TIM_A0_CH3_A17, 20000, 0);    // CH3: PA17, 20kHz

pwm_set_duty(PWM_TIM_A0_CH0_A8, 5000);     // 62.5% 占空比
pwm_set_duty(PWM_TIM_A0_CH1_B20, 0);       // 0%
```

---

### 4.03 UART — zf_driver_uart

**文件**: [SeekFree/zf_driver/zf_driver_uart.h](SeekFree/zf_driver/zf_driver_uart.h)

```c
#define UART_NUM  ( 4 )    // UART0~UART3

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

typedef enum {
    UART_INTERRUPT_STATE_NONE = 0x00,
    UART_INTERRUPT_STATE_RX   = 0x01,
    UART_INTERRUPT_STATE_TX   = 0x02,
    UART_INTERRUPT_STATE_ALL  = 0x03,
} uart_interrupt_state_enum;

// API
void  uart_write_byte(uart_index_enum idx, const uint8 dat);
void  uart_write_buffer(uart_index_enum idx, const uint8 *buff, uint32 len);
void  uart_write_string(uart_index_enum idx, const char *str);
uint8 uart_try_write_byte(uart_index_enum idx, const uint8 data);     // 非阻塞

uint8 uart_read_byte(uart_index_enum idx, uint8 *data);               // 阻塞（带超时）
uint8 uart_query_byte(uart_index_enum idx, uint8 *data);              // 非阻塞查询

void  uart_set_callback(uart_index_enum idx, void_callback_uint32_ptr cb, void *ptr);
void  uart_set_interrupt_config(uart_index_enum idx, uart_interrupt_config_enum cfg);
void  uart_init(uart_index_enum idx, uint32 baud, uart_tx_pin_enum tx, uart_rx_pin_enum rx);
```

**模板 — RX 中断 + 环形缓冲区**:
```c
// ISR 中只 push，主循环 pop 解析
static void my_isr(uint32_t state, void *ptr) {
    uint8_t byte;
    if ((state & UART_INTERRUPT_STATE_RX) == 0) return;
    while (uart_query_byte(UART_2, &byte) == ZF_TRUE)
        serial_rx_buffer_push(&rx_ring, byte);     // 见用户层 API 手册
}

uart_init(UART_2, 115200, UART2_TX_B15, UART2_RX_B16);
uart_set_callback(UART_2, my_isr, NULL);
uart_set_interrupt_config(UART_2, UART_INTERRUPT_CONFIG_RX_ENABLE);
```

---

### 4.04 ADC — zf_driver_adc

**文件**: [SeekFree/zf_driver/zf_driver_adc.h](SeekFree/zf_driver/zf_driver_adc.h)

```c
#define ADC_NUM  ( 2 )     // ADC0, ADC1

typedef enum { ADC_0, ADC_1 } adc_index_enum;
typedef enum {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7,
} adc_channel_enum;

// ADC 引脚: {ADC0, ADC1} × 8通道 — 编码了 (ADC索引, 通道号, GPIO)
typedef enum {
    ADC0_CH0_A27, ADC0_CH1_A26, ADC0_CH2_A25, ADC0_CH3_A24,
    ADC0_CH4_B25, ADC0_CH5_B24, ADC0_CH6_B20, ADC0_CH7_A22,
    ADC1_CH0_A15, ADC1_CH1_A16, ADC1_CH2_A17, ADC1_CH3_A18,
    ADC1_CH4_B17, ADC1_CH5_B18, ADC1_CH6_B19, ADC1_CH7_A21,
} adc_pin_enum;

typedef enum { ADC_12BIT, ADC_10BIT, ADC_8BIT } adc_resolution_enum;

// API
uint16 adc_convert(adc_pin_enum adc_pin);                                // 单次转换
uint8  adc_convert_checked(adc_pin_enum adc_pin, uint16 *result);       // 带超时保护
uint16 adc_mean_filter_convert(adc_pin_enum adc_pin, const uint8 count); // 均值滤波
void   adc_init(adc_pin_enum adc_pin, adc_resolution_enum resolution);
```

**使用**:
```c
adc_init(ADC0_CH2_A25, ADC_12BIT);
uint16 raw = adc_mean_filter_convert(ADC0_CH2_A25, 8);   // 8次平均
uint32 mv = (uint32)raw * 3300 / 4095;                   // 12位: 3300mV参考
```

---

### 4.05 延时 — zf_driver_delay

**文件**: [SeekFree/zf_driver/zf_driver_delay.h](SeekFree/zf_driver/zf_driver_delay.h)

```c
void system_delay_ms(uint32 time);    // 毫秒延时（阻塞，SysTick 实现）
void system_delay_us(uint32 time);    // 微秒延时（阻塞，SysTick 实现）
```

---

### 4.06 定时器 — zf_driver_timer

**文件**: [SeekFree/zf_driver/zf_driver_timer.h](SeekFree/zf_driver/zf_driver_timer.h)

```c
typedef enum {
    TIM_A0, TIM_A1, TIM_G0, TIM_G6, TIM_G7, TIM_G8, TIM_G12, TIM_MAX
} timer_index_enum;

typedef enum {
    TIMER_SYSTEM_CLOCK,    // 系统时钟计数（最大 0xFFFF）
    TIMER_US,              // 微秒计数
    TIMER_MS,              // 毫秒计数
} timer_mode_enum;

typedef enum {
    TIMER_FUNCTION_INIT = 0,
    TIMER_FUNCTION_TIMER,     // 用作计时器
    TIMER_FUNCTION_PIT,       // 用作 PIT（不可同时）
    TIMER_FUNCTION_PWM,       // 用作 PWM（不可同时）
    TIMER_FUNCTION_ENCODER,   // 用作编码器（不可同时）
} timer_function_enum;        // 功能互斥检查

extern GPTIMER_Regs * const timer_reg[TIM_MAX];

// API
uint8  timer_funciton_check(timer_index_enum index, timer_function_enum mode); // 检查/注册功能
void   timer_clock_enable(timer_index_enum index);
void   timer_start(timer_index_enum index);
void   timer_stop(timer_index_enum index);
uint16 timer_get(timer_index_enum index);       // MS模式下自动/50
void   timer_clear(timer_index_enum index);
void   timer_init(timer_index_enum index, timer_mode_enum mode);
```

---

### 4.07 PIT 周期中断 — zf_driver_pit

**文件**: [SeekFree/zf_driver/zf_driver_pit.h](SeekFree/zf_driver/zf_driver_pit.h)

```c
typedef enum {
    PIT_TIM_A0, PIT_TIM_A1, PIT_TIM_G0, PIT_TIM_G6,
    PIT_TIM_G7, PIT_TIM_G8, PIT_TIM_G12,
} pit_index_enum;

#define PIT_NUM ( 7 )

extern void_callback_uint32_ptr pit_callback_list[PIT_NUM]; // 回调表
extern void *pit_callback_ptr_list[PIT_NUM];                 // 回调参数表

void pit_enable(pit_index_enum pit_n);
void pit_disable(pit_index_enum pit_n);
void pit_init(pit_index_enum pit_n, uint32 period, void_callback_uint32_ptr cb, void *ptr);
void pit_us_init(pit_index_enum pit_n, uint32 period, void_callback_uint32_ptr cb, void *ptr);
void pit_ms_init(pit_index_enum pit_n, uint32 period, void_callback_uint32_ptr cb, void *ptr);
```

**使用**:
```c
void my_tick(uint32 event, void *ptr) { /* 每1ms调用一次 */ }
pit_ms_init(PIT_TIM_G0, 1, my_tick, NULL);    // 1ms 周期中断
pit_enable(PIT_TIM_G0);
```

---

### 4.08 SPI — zf_driver_spi

**文件**: [SeekFree/zf_driver/zf_driver_spi.h](SeekFree/zf_driver/zf_driver_spi.h)

```c
#define SPI_NUM ( 2 )
typedef enum { SPI_0, SPI_1 } spi_index_enum;
typedef enum { SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3 } spi_mode_enum;

// 不接硬件 MISO/CS 时填 SPI_MISO_NULL / SPI_CS_NULL (0xFFFFF)

// 写
void   spi_write_8bit(spi_index_enum idx, const uint8 data);
void   spi_write_8bit_array(spi_index_enum idx, const uint8 *data, uint32 len);
void   spi_write_16bit(spi_index_enum idx, const uint16 data);
void   spi_write_16bit_array(spi_index_enum idx, const uint16 *data, uint32 len);
void   spi_write_8bit_register(spi_index_enum idx, const uint8 reg, const uint8 data);
void   spi_write_8bit_registers(spi_index_enum idx, const uint8 reg, const uint8 *data, uint32 len);
void   spi_write_16bit_register(spi_index_enum idx, const uint16 reg, const uint16 data);
void   spi_write_16bit_registers(spi_index_enum idx, const uint16 reg, const uint16 *data, uint32 len);

// 读
uint8  spi_read_8bit(spi_index_enum idx);
void   spi_read_8bit_array(spi_index_enum idx, uint8 *data, uint32 len);
uint16 spi_read_16bit(spi_index_enum idx);
void   spi_read_16bit_array(spi_index_enum idx, uint16 *data, uint32 len);
uint8  spi_read_8bit_register(spi_index_enum idx, const uint8 reg);
void   spi_read_8bit_registers(spi_index_enum idx, const uint8 reg, uint8 *data, uint32 len);
uint16 spi_read_16bit_register(spi_index_enum idx, const uint16 reg);
void   spi_read_16bit_registers(spi_index_enum idx, const uint16 reg, uint16 *data, uint32 len);

// 全双工
void   spi_transfer_8bit(spi_index_enum idx, const uint8 *wb, uint8 *rb, uint32 len);
void   spi_transfer_16bit(spi_index_enum idx, const uint16 *wb, uint16 *rb, uint32 len);

void   spi_init(spi_index_enum idx, spi_mode_enum mode, uint32 baud,
                spi_sck_pin_enum sck, spi_mosi_pin_enum mosi,
                spi_miso_pin_enum miso, spi_cs_pin_enum cs);
```

---

### 4.09 软件 I2C — zf_driver_soft_iic

**文件**: [SeekFree/zf_driver/zf_driver_soft_iic.h](SeekFree/zf_driver/zf_driver_soft_iic.h)

> 完全用 GPIO 位操作模拟 I2C 协议，不占用硬件 I2C 外设。支持时钟拉伸、总线恢复、SCCB 协议。

```c
#define SOFT_IIC_CLOCK_STRETCH_TIMEOUT  ( 10000u )

typedef enum {
    SOFT_IIC_STATUS_OK = 0,
    SOFT_IIC_STATUS_NACK,
    SOFT_IIC_STATUS_INVALID_PARAM,
    SOFT_IIC_STATUS_CLOCK_STRETCH_TIMEOUT,
    SOFT_IIC_STATUS_BUS_STUCK,
} soft_iic_status_enum;

typedef struct {
    uint32              scl_pin;             // SCL GPIO 引脚
    uint32              sda_pin;             // SDA GPIO 引脚
    uint8               addr;                // 7位设备地址
    uint32              delay;               // 时钟延时（越小越快）
    soft_iic_status_enum last_error;          // 最后一次错误
    uint8               transaction_active;  // 事务中标志
} soft_iic_info_struct;

// === 基础原语 ===
void   soft_iic_start(soft_iic_info_struct *obj);
void   soft_iic_stop(soft_iic_info_struct *obj);
uint8  soft_iic_send_data(soft_iic_info_struct *obj, const uint8 data);   // 返回ACK状态
uint8  soft_iic_read_data(soft_iic_info_struct *obj, uint8 ack);          // ack=0发ACK, ack=1发NACK

// === 无寄存器地址的读写 ===
void   soft_iic_write_8bit(soft_iic_info_struct *obj, const uint8 data);
void   soft_iic_write_8bit_array(soft_iic_info_struct *obj, const uint8 *data, uint32 len);
void   soft_iic_write_16bit(soft_iic_info_struct *obj, const uint16 data);
void   soft_iic_write_16bit_array(soft_iic_info_struct *obj, const uint16 *data, uint32 len);
uint8  soft_iic_read_8bit(soft_iic_info_struct *obj);
void   soft_iic_read_8bit_array(soft_iic_info_struct *obj, uint8 *data, uint32 len);
uint16 soft_iic_read_16bit(soft_iic_info_struct *obj);
void   soft_iic_read_16bit_array(soft_iic_info_struct *obj, uint16 *data, uint32 len);

// === 带寄存器地址的读写（最常用） ===
void   soft_iic_write_8bit_register(soft_iic_info_struct *obj, const uint8 reg, const uint8 data);
void   soft_iic_write_8bit_registers(soft_iic_info_struct *obj, const uint8 reg, const uint8 *data, uint32 len);
soft_iic_status_enum soft_iic_write_8bit_register_checked(soft_iic_info_struct *obj, const uint8 reg, const uint8 data);
soft_iic_status_enum soft_iic_write_8bit_registers_checked(soft_iic_info_struct *obj, const uint8 reg, const uint8 *data, uint32 len);
void   soft_iic_write_16bit_register(soft_iic_info_struct *obj, const uint16 reg, const uint16 data);
void   soft_iic_write_16bit_registers(soft_iic_info_struct *obj, const uint16 reg, const uint16 *data, uint32 len);
uint8  soft_iic_read_8bit_register(soft_iic_info_struct *obj, const uint8 reg);
void   soft_iic_read_8bit_registers(soft_iic_info_struct *obj, const uint8 reg, uint8 *data, uint32 len);
soft_iic_status_enum soft_iic_read_8bit_register_checked(soft_iic_info_struct *obj, const uint8 reg, uint8 *data);
soft_iic_status_enum soft_iic_read_8bit_registers_checked(soft_iic_info_struct *obj, const uint8 reg, uint8 *data, uint32 len);
uint16 soft_iic_read_16bit_register(soft_iic_info_struct *obj, const uint16 reg);
void   soft_iic_read_16bit_registers(soft_iic_info_struct *obj, const uint16 reg, uint16 *data, uint32 len);

// === 高级操作 ===
void   soft_iic_transfer_8bit_array(soft_iic_info_struct *obj, const uint8 *wdata, uint32 wlen, uint8 *rdata, uint32 rlen);
void   soft_iic_transfer_16bit_array(soft_iic_info_struct *obj, const uint16 *wdata, uint32 wlen, uint16 *rdata, uint32 rlen);
void   soft_iic_sccb_write_register(soft_iic_info_struct *obj, const uint8 reg, uint8 data);   // SCCB(OminiVision)
uint8  soft_iic_sccb_read_register(soft_iic_info_struct *obj, const uint8 reg);
void   soft_iic_write_splicing_array(soft_iic_info_struct *obj, const uint8 *p1, uint32 l1, const uint8 *p2, uint32 l2);
soft_iic_status_enum soft_iic_write_splicing_array_checked(soft_iic_info_struct *obj, const uint8 *p1, uint32 l1, const uint8 *p2, uint32 l2);
soft_iic_status_enum soft_iic_get_last_error(const soft_iic_info_struct *obj);

void   soft_iic_init(soft_iic_info_struct *obj, uint8 addr, uint32 delay,
                     gpio_pin_enum scl_pin, gpio_pin_enum sda_pin);
```

**使用**:
```c
soft_iic_info_struct i2c;
soft_iic_init(&i2c, 0x68, 10, B9, B8);    // 地址 0x68, SCL=B9, SDA=B8, delay=10
soft_iic_write_8bit_register(&i2c, 0x6B, 0x80);
uint8 val = soft_iic_read_8bit_register(&i2c, 0x75);
```

---

### 4.10 外部中断 EXTI — zf_driver_exti

**文件**: [SeekFree/zf_driver/zf_driver_exti.h](SeekFree/zf_driver/zf_driver_exti.h)

```c
typedef enum {
    EXTI_TRIGGER_RISING  = 1,    // 上升沿
    EXTI_TRIGGER_FALLING,        // 下降沿
    EXTI_TRIGGER_BOTH,           // 双边沿
} exti_trigger_enum;

extern void_callback_uint32_ptr exti_callback_list[64];   // A0-A31 + B0-B31
extern void *exti_callback_ptr_list[64];

void exti_enable(gpio_pin_enum pin);
void exti_disable(gpio_pin_enum pin);
void exti_init(gpio_pin_enum pin, exti_trigger_enum trigger,
               void_callback_uint32_ptr callback, void *ptr);
```

**使用**:
```c
void on_edge(uint32 state, void *ptr) { /* 处理边沿 */ }
exti_init(A26, EXTI_TRIGGER_BOTH, on_edge, NULL);   // A26 双边沿中断
```

---

### 4.11 Flash — zf_driver_flash

**文件**: [SeekFree/zf_driver/zf_driver_flash.h](SeekFree/zf_driver/zf_driver_flash.h)

```c
#define FLASH_BASE_ADDR        ( 0x00016000 )     // 前32KB（10万次擦写）
#define FLASH_PAGE_SIZE        ( 0x00000400 )     // 1KB/页
#define FLASH_SECTION_SIZE     ( 0x00000800 )     // 2KB/扇区 (2页)
#define FLASH_DATA_BUFFER_SIZE ( 256 )             // 256 个 32bit 元素

// 数据类型联合体
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

// API — 直接操作
uint8 flash_check(uint32 sector_num, uint32 page_num);             // 检查是否为空
uint8 flash_erase_page(uint32 sector_num, uint32 page_num);        // 擦除
void  flash_read_page(uint32 sector_num, uint32 page_num, uint32 *buf, uint16 len);
uint8 flash_write_page(uint32 sector_num, uint32 page_num, const uint32 *buf, uint16 len);

// API — 缓冲区模式（使用 flash_union_buffer）
void  flash_read_page_to_buffer(uint32 sector, uint32 page);
uint8 flash_write_page_from_buffer(uint32 sector, uint32 page);
void  flash_buffer_clear(void);                                     // 填0xFF
```

---

### 4.12 编码器 — zf_driver_encoder

**文件**: [SeekFree/zf_driver/zf_driver_encoder.h](SeekFree/zf_driver/zf_driver_encoder.h)

```c
// 编码器通道枚举 — 编码 (定时器索引, AF, GPIO)
// encoder_channel1_enum (A相), encoder_channel2_enum (B相)
// 例如: TIMG8_ENCODER1_CH1_A26 = (TIM_G8, AF4, A26)

// API
int16 encoder_get_count(timer_index_enum index);    // 获取当前计数值（带方向符号）
int16 encoder_get_delta(timer_index_enum index);    // 获取自上次调用的增量
void  encoder_clear_count(timer_index_enum index);  // 清零硬件计数器

// 初始化
void  encoder_quad_init(timer_index_enum index,
                        encoder_channel1_enum ch1_pin,
                        encoder_channel2_enum ch2_pin);         // 正交解码 (A/B相)
void  encoder_dir_init(timer_index_enum index,
                       encoder_channel1_enum lsb_pin,
                       gpio_pin_enum dir_pin);                  // 方向+脉冲模式
```

**注意**: `encoder_quad_init` 仅支持 TIM_G8（TIMG12 不支持硬件 QEI）。

---

## 5. 设备层 — zf_device

### 5.1 IMU660RA 六轴传感器

**文件**: [SeekFree/zf_device/zf_device_imu660ra.h](SeekFree/zf_device/zf_device_imu660ra.h)

> 3轴加速度 + 3轴陀螺仪。默认硬件 SPI (SPI1, PB23/PB22/PB21, CS=PB19)。也支持 I2C。

```c
#define IMU660RA_USE_IIC       ( 0 )      // 0=SPI, 1=I2C
#define IMU660RA_DEV_ADDR      ( 0x69 )   // SA0上拉

// 量程枚举
typedef enum {
    IMU660RA_ACC_SAMPLE_SGN_2G,      // ±2g
    IMU660RA_ACC_SAMPLE_SGN_4G,      // ±4g
    IMU660RA_ACC_SAMPLE_SGN_8G,      // ±8g  (默认)
    IMU660RA_ACC_SAMPLE_SGN_16G,     // ±16g
} imu660ra_acc_sample_config;

typedef enum {
    IMU660RA_GYRO_SAMPLE_SGN_125DPS,  // ...2000DPS (默认)
} imu660ra_gyro_sample_config;

// 全局原始数据
extern int16 imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z;
extern int16 imu660ra_acc_x,  imu660ra_acc_y,  imu660ra_acc_z;
extern float imu660ra_transition_factor[2];  // [0]=acc转换因子 [1]=gyro转换因子

// API
uint8 imu660ra_init(void);             // 返回 ZF_TRUE(1)=成功
void  imu660ra_get_acc(void);
void  imu660ra_get_gyro(void);

// 单位转换宏
#define imu660ra_acc_transition(v)   ((float)(v) / imu660ra_transition_factor[0])  // → g
#define imu660ra_gyro_transition(v)  ((float)(v) / imu660ra_transition_factor[1])  // → °/s
```

### 5.2 IMU660RB 六轴传感器

**文件**: [SeekFree/zf_device/zf_device_imu660rb.h](SeekFree/zf_device/zf_device_imu660rb.h)

> 与 660RA 类似，但陀螺仪量程多了 ±4000dps。磁力计 2G/8G 可选。芯片 ID=0x0F。

```c
#define IMU660RB_DEV_ADDR  ( 0x6B )
// 额外的量程
typedef enum {
    IMU660RB_GYRO_SAMPLE_SGN_4000DPS,
} imu660rb_gyro_sample_config;

// 转换宏（使用常量表代替运行时 factor 数组）
#define imu660rb_acc_transition(v)  ((float)(v) / imu660rb_acc_transition_factor[...])
#define imu660rb_gyro_transition(v) ((float)(v) / imu660rb_gyro_transition_factor[...])

// API 同 660RA
void  imu660rb_get_acc(void);
void  imu660rb_get_gyro(void);
uint8 imu660rb_init(void);
```

### 5.3 IMU660RC 六轴传感器（含四元数）

**文件**: [SeekFree/zf_device/zf_device_imu660rc.h](SeekFree/zf_device/zf_device_imu660rc.h)

> 660系列增强版，内置姿态解算引擎（可输出四元数和欧拉角）。

```c
#define IMU660RC_DEV_ADDR  ( 0x6B )

// 四元数输出频率
typedef enum {
    IMU660RC_QUARTERNION_15HZ, ..., IMU660RC_QUARTERNION_480HZ, IMU660RC_QUARTERNION_DISABLE,
} imu660rc_quarternion_rate_config;

// 额外全局变量
extern float imu660rc_roll, imu660rc_pitch, imu660rc_yaw;       // 欧拉角 (°)
extern float imu660rc_quarternion[4];                            // 四元数 [w,x,y,z]

uint8 imu660rc_init(imu660rc_quarternion_rate_config rate);     // 初始化时可指定四元数频率
void  imu660rc_get_quarternion(void);                            // 读四元数+欧拉角
```

### 5.4 IMU963RA 九轴传感器

**文件**: [SeekFree/zf_device/zf_device_imu963ra.h](SeekFree/zf_device/zf_device_imu963ra.h)

> 3轴加速度 + 3轴陀螺仪 + 3轴磁力计。支持 ±4000dps 陀螺仪。

```c
#define IMU963RA_DEV_ADDR  ( 0x6B )

// 磁力计量程
typedef enum { IMU963RA_MAG_SAMPLE_2G, IMU963RA_MAG_SAMPLE_8G } imu963ra_mag_sample_config;

// 全局变量
extern int16 imu963ra_mag_x, imu963ra_mag_y, imu963ra_mag_z;
extern float imu963ra_transition_factor[3];  // [0]=acc [1]=gyro [2]=mag

// API
uint8 imu963ra_init(void);
void  imu963ra_get_acc(void);
void  imu963ra_get_gyro(void);
void  imu963ra_get_mag(void);
#define imu963ra_mag_transition(v)  ((float)(v) / imu963ra_transition_factor[2])  // → Gs
```

---

### 5.5 OLED SPI 屏

**文件**: [SeekFree/zf_device/zf_device_oled.h](SeekFree/zf_device/zf_device_oled.h)

> SSD1306, 128×64, 单色, 硬件SPI (SPI0)

```c
#define OLED_X_MAX  ( 128 )
#define OLED_Y_MAX  ( 64 )

typedef enum { OLED_PORTAIT = 0, OLED_PORTAIT_180 = 1 } oled_dir_enum;
typedef enum { OLED_6X8_FONT = 0, OLED_8X16_FONT = 1, OLED_16X16_FONT = 2 } oled_font_size_enum;

// API
void oled_init(void);
void oled_clear(void);
void oled_full(const uint8 color);                       // 全屏填充 0x00/0xFF
void oled_set_dir(oled_dir_enum dir);
void oled_set_font(oled_font_size_enum font);
void oled_draw_point(uint16 x, uint16 y, const uint8 color);
void oled_show_string(uint16 x, uint16 y, const char ch[]);
void oled_show_int(uint16 x, uint16 y, const int32 dat, uint8 num);
void oled_show_uint(uint16 x, uint16 y, const uint32 dat, uint8 num);
void oled_show_float(uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum);
void oled_show_binary_image(uint16 x, uint16 y, const uint8 *img, uint16 w, uint16 h, uint16 dw, uint16 dh);
void oled_show_gray_image(uint16 x, uint16 y, const uint8 *img, uint16 w, uint16 h, uint16 dw, uint16 dh, uint8 th);
void oled_show_wave(uint16 x, uint16 y, const uint16 *wave, uint16 w, uint16 vmax, uint16 dw, uint16 dvmax);
void oled_show_chinese(uint16 x, uint16 y, uint8 size, const uint8 *chs, uint8 num);
```

---

### 5.6 TFT180 彩屏

**文件**: [SeekFree/zf_device/zf_device_tft180.h](SeekFree/zf_device/zf_device_tft180.h)

> 1.8寸, 160×128, RGB565, 硬件SPI (SPI0)

```c
typedef enum {
    TFT180_PORTAIT = 0, TFT180_PORTAIT_180, TFT180_CROSSWISE, TFT180_CROSSWISE_180,
} tft180_dir_enum;

// API — 与 OLED 风格一致，但增加 RGB565 颜色
void tft180_init(void);
void tft180_clear(void);
void tft180_full(const uint16 color);
void tft180_set_dir(tft180_dir_enum dir);
void tft180_set_font(tft180_font_size_enum font);
void tft180_set_color(const uint16 pen, const uint16 bgcolor);
void tft180_draw_point(uint16 x, uint16 y, const uint16 color);
void tft180_draw_line(uint16 xs, uint16 ys, uint16 xe, uint16 ye, const uint16 color);
void tft180_show_string(uint16 x, uint16 y, const char dat[]);
void tft180_show_int(uint16 x, uint16 y, const int32 dat, uint8 num);
void tft180_show_uint(uint16 x, uint16 y, const uint32 dat, uint8 num);
void tft180_show_float(uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum);
void tft180_show_binary_image(...);
void tft180_show_gray_image(...);
void tft180_show_rgb565_image(uint16 x, uint16 y, const uint16 *img, uint16 w, uint16 h, uint16 dw, uint16 dh, uint8 color_mode);
void tft180_show_wave(...);
void tft180_show_chinese(...);
```

---

### 5.7 IPS114 彩屏 / 5.8 IPS200 彩屏

**文件**: [SeekFree/zf_device/zf_device_ips114.h](SeekFree/zf_device/zf_device_ips114.h) / [ips200.h](SeekFree/zf_device/zf_device_ips200.h)

> IPS114: 1.14寸 SPI | IPS200: 2.0寸 320×240 SPI/并口

IPS200 增加了 `ips200_type_enum` (SPI/PARALLEL8) 选择，初始化签名不同：
```c
void ips200_init(ips200_type_enum type_select);
void ips114_init(void);  // 仅 SPI
```

其余 API 命名与 TFT180 一致（`tft180_` → `ips200_`/`ips114_`）。

---

### 5.9 IPS200PRO 智能屏

**文件**: [SeekFree/zf_device/zf_device_ips200pro.h](SeekFree/zf_device/zf_device_ips200pro.h)

> 带 GUI 引擎的串口屏，支持控件（标签/表格/仪表/时钟/日历/波形/图像/进度条）。

```c
// 字体大小: FONT_SIZE_12 ~ FONT_SIZE_40
// 颜色类型: COLOR_FOREGROUND, COLOR_BACKGROUND, COLOR_BORDER, ...
// 显示方向: IPS200PRO_PORTRAIT / PORTRAIT_180 / CROSSWISE / CROSSWISE_180
// 控件: page, label, table, meter, clock, progress_bar, calendar, waveform, image, container

uint16 ips200pro_init(char *str, ips200pro_title_position_enum pos, uint8 title_size);
uint16 ips200pro_page_create(char *str);
uint8  ips200pro_page_switch(uint16 page_id, ips200pro_page_animations_enum anim);
uint16 ips200pro_label_create(int16 x, int16 y, uint16 w, uint16 h);
uint8  ips200pro_label_printf(uint16 label_id, const char *format, ...);
uint16 ips200pro_table_create(int16 x, int16 y, uint16 rows, uint16 cols);
uint8  ips200pro_table_cell_printf(uint16 table_id, uint8 row, uint8 col, char *fmt, ...);
uint16 ips200pro_meter_create(int16 x, int16 y, uint16 size, ips200pro_meter_style_enum style);
uint16 ips200pro_clock_create(int16 x, int16 y, uint16 size, ips200pro_clock_style_enum type);
uint16 ips200pro_progress_bar_create(int16 x, int16 y, uint16 w, uint16 h);
uint16 ips200pro_waveform_create(int16 x, int16 y, uint16 w, uint16 h);
uint8  ips200pro_waveform_add_value(uint16 id, uint8 line, const uint16 *data, uint16 len, uint16 color);
uint16 ips200pro_image_create(int16 x, int16 y, uint16 w, uint16 h);
uint8  ips200pro_image_display(uint16 id, const void *img, uint16 w, uint16 h, ips200pro_image_type_enum type, uint8 threshold);
// ... 更多 widget 接口
```

---

### 5.10 TSL1401 线阵 CCD

**文件**: [SeekFree/zf_device/zf_device_tsl1401.h](SeekFree/zf_device/zf_device_tsl1401.h)

> 128像素线性CCD，PIT驱动自动曝光采集。

```c
#define TSL1401_DATA_LEN      ( 128 )
#define TSL1401_EXPOSURE_TIME ( 10 )            // 曝光时间 ms
#define TSL1401_PIT_INDEX     ( PIT_TIM_G8 )    // 曝光定时器
#define TSL1401_AD_RESOLUTION ( ADC_12BIT )
#define TSL1401_AO_PIN_MAX    ( 2 )

extern uint16 tsl1401_data[TSL1401_AO_PIN_MAX][TSL1401_DATA_LEN];  // 曝光数据
extern vuint8  tsl1401_finish_flag;                                  // 采集完成标志

void tsl1401_collect_pit_handler(uint32 event, void *ptr);  // 在 PIT ISR 中调用
void tsl1401_send_data(uart_index_enum uart_n, uint8 index);  // 发送图像至上位机
void tsl1401_init(uint8 index);                               // 初始化第 index 路 AO
```

---

### 5.11 DL1A / DL1B 激光测距

**文件**: [SeekFree/zf_device/zf_device_dl1a.h](SeekFree/zf_device/zf_device_dl1a.h) / [dl1b.h](SeekFree/zf_device/zf_device_dl1b.h)

> VL53L1X ToF, 最大4m。DL1A 含完整寄存器定义；DL1B 仅保留必要接口。

```c
// DL1B 精简接口
#define DL1B_DEV_ADDR  ( 0x52 >> 1 )    // 0x29
extern uint8  dl1b_finsh_flag;          // 测距完成
extern uint16 dl1b_distance_mm;         // 距离 (mm)
uint8 dl1b_init(void);
void  dl1b_get_distance(void);          // 触发一次测距
void  dl1b_int_handler(void);           // INT 引脚中断

// DL1A 接口相同，额外提供完整的寄存器宏和校准 API
```

---

### 5.12 GS08RA 灰度传感器

**文件**: [SeekFree/zf_device/zf_device_gs08ra.h](SeekFree/zf_device/zf_device_gs08ra.h)

> 8路模拟灰度，通过 S0/S1/S2 GPIO 选通模拟开关，ADC 采集。

```c
#define GS08RA_S0_PIN   ( A16 )
#define GS08RA_S1_PIN   ( A17 )
#define GS08RA_S2_PIN   ( B17 )
#define GS08RA_OUT_PIN  ( ADC0_CH4_B25 )
#define GS08A_CHANNEL_NUM ( 8 )

// 全局数据
extern uint8 gs08ra_threshold;         // 二值化阈值
extern uint8 gs08ra_max_val[8];        // 8路白值
extern uint8 gs08ra_min_val[8];        // 8路黑值
extern uint8 gs08ra_raw_val[8];        // 原始ADC
extern uint8 gs08ra_deal_val[8];       // 归一化数据
extern uint8 gs08ra_bin_val[8];        // 二值化结果

void gs08ra_set_max(void);             // 记录当前值为最大值（放白色上）
void gs08ra_set_min(void);             // 记录当前值为最小值（放黑色上）
void gs08ra_set_threshold(uint8 threshold);
void gs08ra_scan_read(void);           // 扫描所有8通道
void gs08ra_init(void);
```

---

### 5.13 按键管理

**文件**: [SeekFree/zf_device/zf_device_key.h](SeekFree/zf_device/zf_device_key.h)

> 支持4路按键，自动消抖(10ms)和长按检测(1000ms)。

```c
#define KEY_LIST              { A30, A31, B0, B1 }   // 按键引脚列表
#define KEY_RELEASE_LEVEL     ( GPIO_HIGH )
#define KEY_MAX_SHOCK_PERIOD  ( 10 )            // 消抖 ms
#define KEY_LONG_PRESS_PERIOD ( 1000 )          // 长按阈值 ms

typedef enum { KEY_1, KEY_2, KEY_3, KEY_4, KEY_NUMBER } key_index_enum;
typedef enum { KEY_RELEASE, KEY_SHORT_PRESS, KEY_LONG_PRESS } key_state_enum;

void           key_scanner(void);                      // 扫描—需在 PIT 中周期调用
key_state_enum key_get_state(key_index_enum key_n);
void           key_clear_state(key_index_enum key_n);
void           key_clear_all_state(void);
void           key_init(uint32 period);                // period = 扫描间隔 ms
```

---

### 5.14 绝对式编码器

**文件**: [SeekFree/zf_device/zf_device_absolute_encoder.h](SeekFree/zf_device/zf_device_absolute_encoder.h)

> 多圈绝对位置编码器，SPI1 总线，最多4个设备。

```c
#define ABSOLUTE_ENCODER_SPI      ( SPI_1 )
#define ABSOLUTE_ENCODER_CS_PIN_MAX ( 4 )
#define ABSOLUTE_ENCODER_CS_PIN_LIST { B17, B18, B26, B27 }
#define ABSOLUTE_ENCODER_CS(index, state)  ((state) ? gpio_high(cs_list[index]) : gpio_low(cs_list[index]))

int16 absolute_encoder_get_location(uint8 index);  // 读取角度位置
int16 absolute_encoder_get_offset(uint8 index);    // 读取零点偏移
uint8 absolute_encoder_init(uint8 index);          // 初始化第 index 个设备
```

---

### 5.15 无线串口模块

**文件**: [SeekFree/zf_device/zf_device_wireless_uart.h](SeekFree/zf_device/zf_device_wireless_uart.h)

> 逐飞配套无线转串口模块，UART 通信，支持 RTS 流控和自动波特率。

```c
#define WIRELESS_UART_INDEX     ( UART_1 )
#define WIRELESS_UART_BUAD_RATE ( 115200 )

uint32 wireless_uart_send_byte(const uint8 data);
uint32 wireless_uart_send_buffer(const uint8 *buff, uint32 len);
uint32 wireless_uart_send_string(const char *str);
uint32 wireless_uart_read_buffer(uint8 *buff, uint32 len);
void   wireless_uart_callback(void);    // RX ISR
uint8  wireless_uart_init(void);
```

---

### 5.16 WiFi SPI / UART 模块

**文件**: [SeekFree/zf_device/zf_device_wifi_spi.h](SeekFree/zf_device/zf_device_wifi_spi.h) / [wifi_uart.h](SeekFree/zf_device/zf_device_wifi_uart.h)

#### WiFi SPI

```c
// 透传 TCP/UDP，SPI 高速传输
uint8  wifi_spi_wifi_connect(char *ssid, char *pwd);
uint8  wifi_spi_socket_connect(char *type, char *ip, char *port, char *local_port);
uint32 wifi_spi_send_buffer(const uint8 *buff, uint32 length);
void   wifi_spi_send_string(const char *string);
uint32 wifi_spi_read_buffer(uint8 *buffer, uint32 length);
uint8  wifi_spi_get_time(wifi_spi_time_enum fmt, char *buf, uint8 size);
uint8  wifi_spi_init(char *ssid, char *pwd);
```

#### WiFi UART

```c
// AT 指令集 Wi-Fi 模块，UART 接口
uint8  wifi_uart_connect_tcp_servers(char *ip, char *port, wifi_uart_transfer_mode_enum mode);
uint8  wifi_uart_connect_udp_client(char *ip, char *port, char *local_port, wifi_uart_transfer_mode_enum mode);
uint32 wifi_uart_send_buffer(const uint8 *buff, uint32 len);
uint32 wifi_uart_read_buffer(uint8 *buff, uint32 len);
uint8  wifi_uart_get_gmt_time(char *gmt_time);
uint8  wifi_uart_init(char *ssid, char *pwd, wifi_uart_mode_enum mode);
```

---

### 5.17 设备类型管理

**文件**: [SeekFree/zf_device/zf_device_type.h](SeekFree/zf_device/zf_device_type.h)

> 统一管理摄像头/无线/ToF 等外设类型的注册和中断分发。

```c
typedef enum {
    NO_CAMERE = 0, CAMERA_BIN_IIC, CAMERA_BIN_UART, CAMERA_GRAYSCALE, CAMERA_COLOR,
} camera_type_enum;

typedef enum {
    NO_WIRELESS = 0, WIRELESS_UART, BLUETOOTH_CH9141, WIFI_UART, WIFI_SPI,
} wireless_type_enum;

typedef enum { NO_TOF = 0, TOF_DL1A, TOF_DL1B } tof_type_enum;
typedef void (*callback_function)(void);

// 全局类型和回调指针
extern wireless_type_enum wireless_type;
extern callback_function wireless_module_uart_handler;
extern callback_function wireless_module_spi_handler;
extern camera_type_enum camera_type;
extern callback_function camera_dma_handler;
extern callback_function camera_vsync_handler;
extern callback_function camera_uart_handler;
extern tof_type_enum tof_type;
extern callback_function tof_module_exti_handler;

void set_camera_type(camera_type_enum type, callback_function vsync,
                     callback_function dma, callback_function uart);
void set_wireless_type(wireless_type_enum type, callback_function cb);
void set_tof_type(tof_type_enum type, callback_function exti_cb);
```

---

## 6. 组件层 — zf_components

### 6.1 逐飞助手上位机协议

**文件**: [SeekFree/zf_components/seekfree_assistant.h](SeekFree/zf_components/seekfree_assistant.h)

> 与逐飞官方上位机软件通信：发送 CCD 图像、摄像头图像、虚拟示波器、调参。

```c
#define SEEKFREE_ASSISTANT_OSCILLOSCOPE_MAX  ( 16 )    // 示波器最多16通道
#define SEEKFREE_ASSISTANT_DEBUG_PARAM_MAX   ( 8 )     // 调参最多8个参数

// CCD 图像
void seekfree_assistant_ccd_config(seekfree_assistant_ccd_struct *obj, ...);
void seekfree_assistant_ccd_send(seekfree_assistant_ccd_struct *obj);

// 摄像头图像
void seekfree_assistant_camera_config(seekfree_assistant_camera_struct *obj, ...);
void seekfree_assistant_camera_send(seekfree_assistant_camera_struct *obj);

// 边界框
void seekfree_assistant_camera_boundary_config(...);
void seekfree_assistant_camera_boundary_send(...);
void seekfree_assistant_camera_rectangular_send(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color);

// 虚拟示波器
void seekfree_assistant_oscilloscope_config(seekfree_assistant_oscilloscope_struct *obj, uint8 ch, void *buf);
void seekfree_assistant_oscilloscope_send(seekfree_assistant_oscilloscope_struct *obj);

// 在线调参（从主机接收参数）
void seekfree_assistant_debug_param_analysis(seekfree_assistant_debug_param_struct *obj);
```

### 6.2 上位机接口适配

**文件**: [SeekFree/zf_components/seekfree_assistant_interface.h](SeekFree/zf_components/seekfree_assistant_interface.h)

```c
typedef enum {
    SEEKFREE_ASSISTANT_DEBUG_UART,      // 通过调试串口
    SEEKFREE_ASSISTANT_WIRELESS_UART,   // 通过无线串口模块
    SEEKFREE_ASSISTANT_CH9141,          // CH9141 蓝牙
    SEEKFREE_ASSISTANT_WIFI_UART,       // WiFi UART 模块
    SEEKFREE_ASSISTANT_WIFI_SPI,        // WiFi SPI 模块
    SEEKFREE_ASSISTANT_CUSTOM,          // 自定义传输
} seekfree_assistant_transfer_device_enum;

void seekfree_assistant_interface_init(seekfree_assistant_transfer_device_enum device);
// 调用后自动绑定 send/receive 函数到对应外设
```

---

## 7. 资源速查表

### 7.1 GPIO 引脚索引

| Port | 索引范围 | 说明 |
|------|----------|------|
| A | A0=0 ~ A31=31 | GPIOA |
| B | B0=32 ~ B27=59 | GPIOB |
| 最大值 | GPIO_MAX=60 | |

`gpio_high(pin)` 内部: `group = pin >> 5`（0=A, 1=B），`bit = pin & 0x1F`

### 7.2 PWM 通道索引

| 定时器 | 通道数 | 常用引脚 |
|--------|--------|---------|
| TIMA0 | 4 (CH0~3) | CH0: PA0/PA8 | CH1: PA9/PB20 | CH2: PA15 | CH3: PA17 |
| TIMA1 | 2 (CH0~1) | CH0: PA10/PB0/PB4 | CH1: PA11/PB1/PB5 |
| TIMG0 | 2 | CH0: PA5/PA12/PB10 | CH1: PA6/PA13/PB11 |
| TIMG6 | 2 | CH0: PA5/PA29/PB2/PB6/PB10 | CH1: PA6/PA30/PB3/PB7/PB11 |
| TIMG7 | 2 | CH0: PA3/PA17/PA23/PA26/PA28/PB15 | CH1: PA2/PA4/PA7/PA18/PA24 |
| TIMG8 | 2 | CH0: PA1/PA3/PA5/PA7/PA21/PA23/PA26/PA29/PB6/PB10/PB15/PB21 | 同上 |
| TIMG12 | 2 | CH0: PA10/PA14/PB13/PB20 | CH1: PA25/PA31/PB14/PB24 |

### 7.3 UART 引脚索引

| UART | TX 可选 | RX 可选 |
|------|---------|---------|
| UART0 | PA0, PA10, PA28, PB0 | PA1, PA11, PA31, PB1 |
| UART1 | PA8, PA17, PB4, PB6 | PA9, PA18, PB5, PB7 |
| UART2 | PA21, PA23, PB15, PB17 | PA22, PA24, PB16, PB18 |
| UART3 | PA14, PA26, PB2, PB12 | PA13, PA25, PB3, PB13 |

### 7.4 定时器资源索引

| 定时器 | PIT 索引 | PWM 索引 | 本工程典型用途 |
|--------|----------|----------|--------------|
| TIMA0 | PIT_TIM_A0 | PWM_TIM_A0 | 电机4路PWM |
| TIMA1 | PIT_TIM_A1 | PWM_TIM_A1 | 备用 |
| TIMG0 | PIT_TIM_G0 | PWM_TIM_G0 | 备用 |
| TIMG6 | PIT_TIM_G6 | PWM_TIM_G6 | 传感器PIT |
| TIMG7 | PIT_TIM_G7 | PWM_TIM_G7 | 备用 |
| TIMG8 | PIT_TIM_G8 | PWM_TIM_G8 | 编码器QEI / 蜂鸣器PWM |
| TIMG12 | PIT_TIM_G12 | PWM_TIM_G12 | 编码器GPIO |

> 每个定时器只能分配一种功能（PIT/PWM/ENCODER 互斥），`timer_funciton_check()` 负责冲突检测。

---

## 8. 与用户层的边界约定

| 边界 | 逐飞底层 (本手册) | 用户层 (见 [ZF_USER_API_MANUAL](ZF_USER_API_MANUAL.md)) |
|------|-----------------|------------------------------------------------------|
| **头文件入口** | `#include "zf_common_headfile.h"` | 再 `#include "config.h"` `#include "pin_mapping.h"` |
| **初始化顺序** | 提供底层外设 init | 负责组织 init 顺序（clock → interrupt → 各外设） |
| **中断回调** | 提供空回调数组（`exti_callback_list` 等） | 填充具体回调函数 |
| **数据结构** | 提供通用 FIFO / 环形缓冲区 | 根据协议定义帧格式和解析逻辑 |
| **PID/调度** | 不提供 | 提供 `PidController`, `ec_scheduler` 等 |
| **竞赛逻辑** | 不涉及 | `line_car`, `h2024_tasks`, `gimbal` 等 |

---

> **版本**: v2.0 (解耦版) | **生成日期**: 2026-07-20  
> **配套文档**: [ZF_USER_API_MANUAL.md](ZF_USER_API_MANUAL.md) — 用户层 API 手册
