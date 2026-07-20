# 逐飞 MSPM0G3507 开源库 — API 速查手册

> 总览: **42 个头文件** | **~350 个函数** | **~100 个宏/枚举**

---

## 一、zf_common — 公共层 (8 个模块)

| 模块 | 头文件 | 核心 API |
|------|--------|---------|
| **类型定义** | `zf_common_typedef.h` | `uint8/16/32`, `ZF_ENABLE/DISABLE`, 函数指针类型 |
| **系统时钟** | `zf_common_clock.h` | `clock_init(SYSTEM_CLOCK_80M)` — 初始化时钟 80MHz |
| **调试输出** | `zf_common_debug.h` | `debug_init()`, `debug_printf(...)` — 串口调试打印 |
| **FIFO** | `zf_common_fifo.h` | `fifo_init/read/write/clear` — 环形缓冲区 |
| **字库** | `zf_common_font.h` | `RGB565_RED/GREEN...` 颜色枚举, 中英文字库 |
| **工具函数** | `zf_common_function.h` | `func_abs/limit`, 字符串转换, 正弦表生成 |
| **中断管理** | `zf_common_interrupt.h` | `interrupt_global_enable/disable`, `interrupt_set_priority` |
| **总头文件** | `zf_common_headfile.h` | 一键包含所有公共层头文件 |

## 二、zf_driver — 外设驱动层 (12 个模块)

| 模块 | 头文件 | 核心 API |
|------|--------|---------|
| **GPIO** | `zf_driver_gpio.h` | `gpio_init(pin, dir, level, mode)`, `gpio_set/get/toggle` |
| **UART** | `zf_driver_uart.h` | `uart_init(n, baud, tx, rx)`, `uart_write_byte/string`, `uart_read_byte` |
| **ADC** | `zf_driver_adc.h` | `adc_init(pin, res)`, `adc_convert(pin)`, `adc_mean_filter_convert` |
| **PWM** | `zf_driver_pwm.h` | `pwm_init(pin, freq, duty)`, `pwm_set_duty` |
| **PIT** | `zf_driver_pit.h` | `pit_init/us_init/ms_init(n, period, callback)`, `pit_enable/disable` |
| **TIMER** | `zf_driver_timer.h` | `timer_init/start/stop/get/clear` — 通用定时器 |
| **SPI** | `zf_driver_spi.h` | `spi_init`, `spi_write/read_8/16bit`, `spi_transfer`, 寄存器读写 |
| **Soft IIC** | `zf_driver_soft_iic.h` | `soft_iic_init`, 全系列读写/寄存器/SCCB 操作 |
| **FLASH** | `zf_driver_flash.h` | `flash_erase_page`, `flash_read/write_page` — 内部 Flash |
| **ENCODER** | `zf_driver_encoder.h` | `encoder_quad_init/dir_init`, `encoder_get/clear_count` |
| **EXTI** | `zf_driver_exti.h` | `exti_init(pin, trigger, callback)`, `exti_enable/disable` |
| **DELAY** | `zf_driver_delay.h` | `system_delay_ms(time)`, `system_delay_us(time)` |

## 三、zf_device — 外接设备驱动层 (14 个模块)

| 模块 | 头文件 | 核心 API |
|------|--------|---------|
| **OLED 0.96"** | `zf_device_oled.h` | `oled_init/show_string/int/float/image/wave/chinese` |
| **TFT 1.8"** | `zf_device_tft180.h` | `tft180_init/show_string/draw_line/show_rgb565_image` |
| **IPS 1.14"** | `zf_device_ips114.h` | 同 TFT180 接口, `ips114_` 前缀 |
| **IPS 2.0"** | `zf_device_ips200.h` | `ips200_init(type)`, 支持 SPI/并口切换 |
| **IPS Pro 2.0"** | `zf_device_ips200pro.h` | 智能屏, 50+ 函数：页面/标签/表格/仪表/波形/图像组件 |
| **IMU 660RA** | `zf_device_imu660ra.h` | `imu660ra_init/get_acc/get_gyro` + 物理值转换 |
| **IMU 660RB** | `zf_device_imu660rb.h` | 同上接口, 不同传感器参数 |
| **IMU 660RC** | `zf_device_imu660rc.h` | 增加四元数: `get_quarternion()` |
| **IMU 963RA** | `zf_device_imu963ra.h` | 九轴: `get_acc/gyro/mag` |
| **按键** | `zf_device_key.h` | `key_init/scanner`, `key_get_state` (短按/长按) |
| **8路灰度** | `zf_device_gs08ra.h` | `gs08ra_init/set_max/set_min/scan_read` |
| **CCD 线阵** | `zf_device_tsl1401.h` | `tsl1401_init/collect/send_data` |
| **ToF 测距** | `zf_device_dl1a/b.h` | `dl1a_init/get_distance/int_handler` |
| **绝对值编码器** | `zf_device_absolute_encoder.h` | `absolute_encoder_init/get_location/get_offset` |
| **无线串口** | `zf_device_wireless_uart.h` | `wireless_uart_init/send/read_buffer` |
| **UART WiFi** | `zf_device_wifi_uart.h` | `wifi_uart_init/connect_tcp/udp/send/read` |
| **SPI WiFi** | `zf_device_wifi_spi.h` | `wifi_spi_init/scan/connect/socket/send/read` |
| **设备类型** | `zf_device_type.h` | `set_camera_type/set_wireless_type/set_tof_type` |

## 四、zf_components — 组件层 (2 个模块)

| 模块 | 头文件 | 核心 API |
|------|--------|---------|
| **逐飞助手** | `seekfree_assistant.h` | CCD图像发送、摄像头图传、虚拟示波器(16通道)、参数调试(8通道) |
| **助手接口** | `seekfree_assistant_interface.h` | `seekfree_assistant_interface_init(device)` — 选择通信通道 |

---

## 五、最常用 API 速查卡片

```c
#include "zf_common_headfile.h"    // 一键包含全部公共层

// ===== 系统 =====
clock_init(SYSTEM_CLOCK_80M);      // 初始化 80MHz
debug_init();                       // 初始化调试串口
debug_printf("val=%d\r\n", val);   // 打印到上位机

// ===== GPIO =====
gpio_init(D0, GPO, GPIO_PIN_OUTPUT_LOW, GPIO_PIN_OUTPUT_LOW);
gpio_set(D0, 1);                   // 高电平
gpio_toggle(D0);                   // 翻转

// ===== UART =====
uart_init(UART_0, 115200, UART0_TX_C15, UART0_RX_C14);
uart_write_string(UART_0, "hello\r\n");

// ===== ADC =====
adc_init(ADC0_CH0_A27, ADC_12BIT);
uint16 val = adc_convert(ADC0_CH0_A27);

// ===== PWM =====
pwm_init(PWM_TIM_A0_CH0_D0, 10000, 5000);  // 10KHz, 50%占空比
pwm_set_duty(PWM_TIM_A0_CH0_D0, 7500);     // 改占空比

// ===== PIT =====
pit_ms_init(PIT_TIM_G0, 100, my_callback, NULL);  // 每100ms触发

// ===== 编码器 =====
encoder_quad_init(TIM_G8, ENCODER_CH1_B0, ENCODER_CH2_B1);
int16 count = encoder_get_count(TIM_G8);

// ===== 延时 =====
system_delay_ms(500);               // 裸机延时
vTaskDelay(pdMS_TO_TICKS(500));     // FreeRTOS 延时(推荐)

// ===== FLASH =====
flash_erase_page(0, 0);
flash_write_page(0, 0, data, len);

// ===== IMU =====
imu660ra_init();
imu660ra_get_acc();  float ax = imu660ra_acc_transition(imu660ra_acc_x);
imu660ra_get_gyro(); float gz = imu660ra_gyro_transition(imu660ra_gyro_z);

// ===== OLED =====
oled_init();
oled_show_string(0, 0, "Hello");
oled_show_float(0, 2, 3.14, 2, 2);

// ===== 按键 =====
key_init(10);
key_state_enum state = key_get_state(KEY_1);  // KEY_RELEASE / SHORT / LONG

// ===== 逐飞助手上位机 =====
// 虚拟示波器: seekfree_assistant_oscilloscope_config(&obj, 4, buffer);
//             seekfree_assistant_oscilloscope_send(&obj);
// 参数调试:   seekfree_assistant_debug_param_analysis(&obj);
```
