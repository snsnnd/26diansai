# Line-Car Hardware Revision 2026

本文档描述当前 line-car 硬件版本的 MCU 引脚、外设复用和固件配置。
配置源以 `user/inc/config.h` 和 `user/inc/pin_mapping.h` 为准。

## 引脚表

| 功能 | MCU 引脚 | 外设复用 | 固件标识 |
| --- | --- | --- | --- |
| 按键 1 | PB6 | GPIO，低电平按下 | `KEY1_PIN` |
| 按键 2 | PB7 | GPIO，低电平按下 | `KEY2_PIN` |
| 按键 3 | PB23 | GPIO，低电平按下 | `KEY3_PIN` |
| 蓝牙 TX | PB15 | UART2 TX | `BLUETOOTH_TX_PIN` |
| 蓝牙 RX | PB16 | UART2 RX | `BLUETOOTH_RX_PIN` |
| 左电机 IN1 | PA8 | TIMA0 CCP0 | `MOTOR_L_IN1` |
| 左电机 IN2 | PB20 | TIMA0 CCP1 | `MOTOR_L_IN2` |
| 右电机 IN1 | PA17 | TIMA0 CCP3 | `MOTOR_R_IN1` |
| 右电机 IN2 | PA15 | TIMA0 CCP2 | `MOTOR_R_IN2` |
| 左编码器 A | PA26 | TIMG8 CCP0/QEI A | `ENCODER1_CHANNEL_A` |
| 左编码器 B | PA27 | TIMG8 CCP1/QEI B | `ENCODER1_CHANNEL_B` |
| 右编码器 A | PA14 | GPIO 双边沿输入 | `ENCODER2_A_PIN` |
| 右编码器 B | PB24 | GPIO 双边沿输入 | `ENCODER2_B_PIN` |
| LED1 | PA30 | GPIO 输出 | `PIN_LED1` |
| LED2 | PA7 | GPIO 输出 | `PIN_LED2` |
| LED3 | PB27 | GPIO 输出 | `PIN_LED3` |
| RGB 红 | PB0 | GPIO 输出 | `PIN_RGB_R` |
| RGB 绿 | PB1 | GPIO 输出 | `PIN_RGB_G` |
| RGB 蓝 | PA29 | GPIO 输出 | `PIN_RGB_B` |
| T8 SCL | PB2 | 软件 I2C | `TRACE_SCL` |
| T8 SDA | PB3 | 软件 I2C | `TRACE_SDA` |
| M0 陀螺仪 TX | PA10 | UART0 TX，接模块 RX | `GYRO_Z_TX_PIN` |
| M0 陀螺仪 RX | PA11 | UART0 RX，接模块 TX | `GYRO_Z_RX_PIN` |
| MPU6050 SCL | PA1 | 软件 I2C | `MPU6050_SCL` |
| MPU6050 SDA | PA0 | 软件 I2C | `MPU6050_SDA` |
| 蜂鸣器 | PA31 | GPIO 输出 | `BUZZER_PIN` |
| 继电器 | PA28 | GPIO 输出 | `RELAY_PIN` |
| 电池 ADC | PA25 | ADC0 channel 2 | `BAT_ADC` |

## 蓝牙与调试串口

line-car Profile 将 UART2 PB15/PB16 同时作为蓝牙透明串口、debug、VOFA 和
在线调参端口，默认波特率为 115200 8N1。蓝牙模块与 MCU 必须交叉连接：

```text
MCU PB15 / UART2 TX -> Bluetooth RX
MCU PB16 / UART2 RX <- Bluetooth TX
MCU GND              -- Bluetooth GND
```

当前固件把蓝牙视为透明串口，不控制 HC-05 EN/KEY 引脚，`EC_ENABLE_HC05`
应保持为 0。hardware-test Profile 的 EMM 仍使用 UART2 PB15/PB16，因此两个
Profile 不可在同一固件中同时占用 UART2；编译期 Profile 隔离保证了这一点。

M0 单轴陀螺仪使用 UART0：MCU TX PA10 接模块 RX，MCU RX PA11 接模块 TX。
hardware-test 固定使用 M0；line-car 默认也使用 M0，且可通过
`CAR_GYRO_SOURCE` 切换为 MPU6050。hardware-test debug 仍使用 UART1 PA8/PA9。

## 航向传感器选择

统一接口位于 `driver/dt_heading.h`。只需修改 `user/inc/config.h`：

```c
#define CAR_GYRO_SOURCE CAR_GYRO_SOURCE_M0
```

或：

```c
#define CAR_GYRO_SOURCE CAR_GYRO_SOURCE_MPU6050
```

M0 使用 UART0 PA10/PA11、115200；MPU6050 使用软件 I2C PA1/PA0。
控制、故障保护、OLED `GYRO TEST` 和遥测均通过同一个航向接口读取数据。

## 电机 PWM

四路电机输入全部由 TIMA0 输出 20 kHz PWM。逐飞库枚举使用从 0 开始的
`CH0..CH3` 名称，分别对应当前接线的 PA8、PB20、PA15、PA17。电机方向还受
`MOTOR_LEFT_OUTPUT_SIGN`、`MOTOR_RIGHT_OUTPUT_SIGN` 和
`CAR_DEFAULT_MOTOR_DIRECTION` 影响。

2026-07-19 实机 `MOTOR R RAW` 测试确认电机2可运行，PA17/PA15、驱动器
第二通道、电机供电和电机本体链路有效。右编码器故障应作为独立问题处理。

首次上电必须架空车轮，通过 raw PWM 测试确认左右轮归属和前进方向，再落地测试。

## AB 正交编码器

左轮使用 TIMG8 硬件 QEI。右轮接线 PA14/PB24 对应 TIMG12 引脚，但
MSPM0G3507 的 TIMG12 不支持 QEI；固件因此将右轮配置为 GPIO 双边沿正交解码，
避免把 TIMG12 自由运行计数误判为编码器脉冲。GPIO pending bit 不是事件 FIFO，
右轮高速运行时可能合并边沿；最终硬件版本应将右编码器改接到受支持的 QEI 定时器。

当前标称参数为 13 PPR、30:1 减速箱、AB x4，因此：

```text
ENCODER_CPR = 13 * 30 * 4 = 1560
```

不同批次电机的 PPR 或减速比可能不同。应在输出轴旋转 10 圈后，用实测总计数
重新计算 `ENCODER_CPR`。若正向运行时某轮有符号计数为负，修改对应的
`ENCODER1_DIRECTION_SIGN` 或 `ENCODER2_DIRECTION_SIGN`。

## LED、RGB 和继电器

三颗独立 LED、RGB 和继电器均为高电平有效，在 line-car 初始化时输出低电平
关闭。当前角色分配：

- LED1：H 题到点提示。
- LED2：编码器活动指示。
- LED3：预留。
- RGB：预留给状态或故障显示。
- 继电器：默认关闭，当前业务逻辑不主动吸合。

RGB 有效电平由 `PIN_RGB_ON_LEVEL` 和 `PIN_RGB_OFF_LEVEL` 定义；不要在业务
代码中绕过这些宏直接写有效电平。

当前硬件上电测试阶段启用了 `LINE_CAR_LED_RGB_TEST_AUTOSTART`。line-car 固件
启动后电机保持关闭，KEY1/KEY2/KEY3 分别翻转 LED1/LED2/LED3，并把 RGB
切换为红/绿/蓝。每次只点亮一个 RGB 通道，复位后重新开始测试。OLED 的
`RGBIO` 按 R/G/B 顺序显示引脚实测电平；选择绿色时应为 `010`。

## 上电检查顺序

1. 断开电机功率或架空车轮，确认三个按键空闲为高、按下为低。
2. 连接蓝牙串口，确认能收到启动和传感器帧。
3. 手转左轮和右轮，确认对应编码器计数分别变化且方向正确。
4. 使用 raw PWM 分别驱动左右轮，确认电机归属和方向。
5. 重新执行 `MOTOR DEADZONE`，更新 `MOTOR_MIN_RUN_PWM_L/R`。
6. 核对 PA25 电池 ADC 分压，禁止电池电压直接接入 MCU。
7. 最后连接 T8、蜂鸣器和继电器并进行整车测试。
