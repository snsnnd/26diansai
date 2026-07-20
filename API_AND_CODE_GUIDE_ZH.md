# MSPM0G3507 固件代码理解、API 与使用手册

本文档对应当前工作区中的实际实现，目标是回答三个问题：

1. 代码从哪里开始、每一层负责什么、数据怎样流动。
2. `user/inc` 中每个项目自有公共 API 应该怎样调用。
3. 怎样构建、烧录、测试以及安全地扩展这套代码。

## 1. 文档范围

### 1.1 本文完整覆盖的代码

- `user/inc/**`：项目自有公共类型、宏和函数声明。
- `user/src/**`：上述 API 的实现、启动入口和中断入口。
- `config.h`、`pin_mapping.h`：编译 profile、功能开关和板级引脚。

### 1.2 单独维护的依赖 API

- `SeekFree/**` 是逐飞底层库，不属于本项目业务 API。常用接口见
  [zf_api_reference.md](zf_api_reference.md)。该速查表是通用逐飞库资料，其中
  `D0`、C 口 UART 和 `vTaskDelay()` 等示例不适用于本工程；本板只使用 PA/PB，
  当前固件也没有 FreeRTOS，不能直接复制这些通用示例。
- `SeekFree/ti_config/**`、`Debug/ti_msp_dl_config.*` 和外部 MSPM0 SDK
  startup/DriverLib 属于生成代码或第三方代码，不应按业务模块修改。
- `tmp/**` 是资料、参考驱动和工具，不参与当前正式构建。

### 1.3 头文件仍是签名的最终依据

本文描述调用语义和约束；如果以后代码与文档发生差异，以对应的
`user/inc/*.h` 和实现为准，同时应更新本文档。

## 2. 先用一张图理解工程

这是一个运行在 TI MSPM0G3507 Cortex-M0+ 上的无 RTOS 裸机固件：

```text
Reset_Handler / TI C runtime
        |
        v
user/src/main.c:main
        |
        +-- clock_init(80 MHz)
        +-- debug_init(UART2, PB15/PB16, Bluetooth)
        +-- ec_app_init()
        |      |
        |      +-- ec_time_init()       1 ms 中断时基
        |      +-- ec_scheduler_init()  协作式调度器
        |      +-- 按 EC_APP_PROFILE 组装应用
        |
        `-- for (;;)
               +-- ec_app_run()
               |      `-- 执行到期任务
               `-- __WFI() 等待下一次中断
```

工程可以按下面的层次阅读：

```text
user/src/app        业务：循迹车、H1-H4、硬件测试、电池保护
user/src/gimbal     云台领域：EMM 电机、视觉协议、位置/速度控制
user/src/device     外部设备协议：T8 灰度传感器
user/src/driver     项目板级驱动：电机、编码器、MPU6050、OLED 等
user/src/protocol   上位机协议：VOFA JustFloat
user/src/lib        通用算法与容器：PID、串口环形缓冲区
user/src/framework  时基、调度、模式、菜单、按键、profile 组装
SeekFree            GPIO/UART/PWM/ADC/PIT/软件 I2C 等底层依赖
TI DriverLib        芯片寄存器和启动层
```

推荐阅读顺序：

1. `user/src/main.c`
2. `user/src/framework/ec_app.c`
3. `user/src/framework/ec_time.c` 和 `ec_scheduler.c`
4. 默认 profile 的 `user/src/app/line_car.c`
5. `user/src/app/h2024_tasks.c`
6. 对照需要的 `driver`、`device`、`lib` 模块
7. 需要云台时再读 `hardware_test.c` 和 `gimbal/**`
8. 最后读 `user/src/isr.c`，理解中断如何把数据交给主循环

## 3. 构建 profile 与固件选择

### 3.1 Profile

`user/inc/config.h` 定义三个互斥 profile：

| 宏 | 值 | 用途 |
| --- | ---: | --- |
| `EC_APP_PROFILE_HARDWARE_TEST` | 1 | 单轴串口陀螺仪、EMM 云台、MaixCam2 视觉 |
| `EC_APP_PROFILE_LINE_CAR` | 2 | 循迹车、可选 M0/MPU6050 航向源、T8、编码器、电机、OLED；默认 |
| `EC_APP_PROFILE_EMPTY` | 3 | 只有时基和框架，供新应用起步 |

默认 CCS `Debug` 配置通过编译宏使用：

```text
EC_APP_PROFILE=2
```

不要在 `config.h` 内硬编码当前 profile。profile 的意义之一是隔离冲突引脚，
必须由构建配置定义且一次只构建一个。

### 3.2 功能开关

| 宏 | 默认值 | 作用 |
| --- | ---: | --- |
| `EC_ENABLE_VOFA` | 1 | line-car 通过蓝牙 UART2 PB15/PB16 输出 VOFA 数据 |
| `EC_ENABLE_HC05` | 0 | 启用 HC-05 AT 配置状态机 |
| `GIMBAL_ENABLE_CALIBRATION` | 1 | 编译云台校准逻辑 |
| `GIMBAL_DEFAULT_CONTROL_MODE` | `GIMBAL_CONTROL_POSITION` | 云台默认视觉控制策略 |
| `HW_TEST_GIMBAL_ACCEPT_STARTUP_REFERENCE` | 0 | 是否声明上电机械位置就是已知参考点 |
| `HW_TEST_GIMBAL_STARTUP_YAW_DEG` | 0.0 | 接受启动参考时的已知 yaw |
| `HW_TEST_GIMBAL_STARTUP_PITCH_DEG` | 0.0 | 接受启动参考时的已知 pitch |

布尔开关只允许 `0` 或 `1`，非法值会触发编译错误。

### 3.3 构建和产物

- 已验证工具版本为 MSPM0 SDK `2.10.0.04`、SysConfig `1.26.2` 和 TI Arm
  Clang `5.1.1.LTS`。在 CCS/Theia 中导入工程根目录的 `.project/.cproject`，
  并确认这些产品版本可用。
- 在 CCS/Theia 中构建 `Debug`，得到 `Debug/MSPM0G3507_ZF.out`。
- 在 Windows 工程目录运行 `build_profiles.bat`，会分别生成：

```text
BuildProfiles/MSPM0G3507_ZF_line_car.out
BuildProfiles/MSPM0G3507_ZF_hardware_test.out
```

脚本内 `CCS`、`GMAKE` 当前硬编码到 `D:\Ti_M0\CCS\...`；安装位置不同必须先
修改这两个路径。脚本最后恢复默认 line-car 的 `Debug` 构建。生成的 Makefile
还含本机 Windows 绝对路径，不能直接当成跨平台 Makefile 使用。

烧录时在 CCS 选择 `targetConfigs/MSPM0G3507.ccxml` 对应的 MSPM0G3507/XDS110
目标，启动 debug session 后 Load Program，选择需要的 `.out`，再复位运行。
天猛星板若没有板载 XDS110，需要外接 3.3 V SWD 调试器：PA20/SWCLK、
PA19/SWDIO、GND 必须连接，目标板供电方式必须明确且所有设备共地。

### 3.4 当前硬件资源

| 功能 | 引脚/外设 | Profile |
| --- | --- | --- |
| 调试、调参与 VOFA | 蓝牙 UART2，TX PB15，RX PB16，115200 8N1 | line-car |
| LED1/2/3 | PA30、PA7、PB27 | line-car |
| RGB G/R/B | PB1、PB0、PA29 | line-car |
| 左电机 PWM | PA8、PB20，TIMA0，20 kHz | line-car |
| 右电机 PWM | PA17、PA15，TIMA0，20 kHz | line-car |
| 左编码器 AB | PA26、PA27，TIMG8 QEI | line-car |
| 右编码器 AB | PA14、PB24，GPIO 双边沿正交解码 | line-car |
| KEY1/2/3 | PB6、PB7、PB23，低电平按下 | line-car |
| 有源蜂鸣器 | PA31 | line-car |
| 继电器 | PA28 | line-car |
| MPU6050 | 软件 I2C，SCL PA1、SDA PA0 | line-car 可选航向源 |
| SSD1306 OLED | 软件 I2C，SCL PB9、SDA PB8 | line-car |
| T8 灰度 | 软件 I2C，SCL PB2、SDA PB3 | line-car |
| 电池采样 | ADC0 channel 2，PA25 | line-car |
| M0 单轴陀螺仪 | UART0，TX PA10、RX PA11，115200 | line-car 默认航向源、hardware-test |
| EMM 两轴电机 | UART2，PB15/PB16，半双工 | hardware-test |
| MaixCam2 | UART3，PB2/PB3，115200 | hardware-test |

UART2 PB15/PB16 在 line-car 中用于蓝牙 debug，在 hardware-test 中用于 EMM；
两者由编译 Profile 隔离。PB2/PB3 在 line-car 中用于 T8，在 hardware-test 中
用于 MaixCam2，同样不能跨 Profile 同时初始化。

本工程引脚表以立创·天猛星 MSPM0G3507 板的 `pin_mapping.h` 为准；工程中保留的
LaunchPad/SysConfig 描述不能替代实际原理图。I2C 只能上拉到 3.3 V。电池、
电机驱动、传感器、USB-UART、EMM 和 MCU 必须共地；电机功率级不得由 GPIO
直接驱动。EMM 的两个地址共享 UART2 半双工总线。

电池绝对不能直接接 PA25。必须使用外部约 4.47:1 电阻分压，使最高电池电压
在 ADC 允许的 0-3.3 V 范围内，并核对电阻误差、共地和输入保护后再上电。

### 3.5 公共板级宏索引

以下是 `config.h` 和 `pin_mapping.h` 中可供代码直接使用的实际标识符：

| 分类 | 宏 |
| --- | --- |
| 电机 | `MOTOR_L_IN1`、`MOTOR_L_IN2`、`MOTOR_R_IN1`、`MOTOR_R_IN2`、`MOTOR_PWM_FREQ`、`MOTOR_PWM_DUTY_MAX`、`MOTOR_LEFT_OUTPUT_SIGN`、`MOTOR_RIGHT_OUTPUT_SIGN` |
| 编码器 | `ENCODER1_A_PIN`、`ENCODER1_B_PIN`、`ENCODER1_DIRECTION_SIGN`、`ENCODER2_A_PIN`、`ENCODER2_B_PIN`、`ENCODER2_DIRECTION_SIGN`、`ENCODER_CPR`、`WHEEL_DIAMETER_MM` |
| 人机与传感器 | `KEY1_PIN`、`KEY2_PIN`、`KEY3_PIN`、`BUZZER_PIN`、`RELAY_PIN`、`MPU6050_SCL`、`MPU6050_SDA`、`OLED_SCL`、`OLED_SDA`、`BAT_ADC`、`TRACE_SCL`、`TRACE_SDA`、`HC05_EN_PIN` |
| 单轴陀螺仪 | `GYRO_Z_UART`、`GYRO_Z_TX_PIN`、`GYRO_Z_RX_PIN`、`GYRO_Z_BAUD` |
| LED/RGB/调试 | `PIN_LED1`、`PIN_LED2`、`PIN_LED3`、`PIN_RGB_G`、`PIN_RGB_R`、`PIN_RGB_B`、`PIN_LED`、`PIN_ENCODER_ACTIVITY_LED`、`BOARD_DEBUG_UART`、`BOARD_DEBUG_UART_TX`、`BOARD_DEBUG_UART_RX` |
| EMM | `BOARD_EMM_UART`、`BOARD_EMM_UART_TX`、`BOARD_EMM_UART_RX` |
| UART1 预留映射 | `BOARD_T8_UART`、`BOARD_T8_UART_TX`、`BOARD_T8_UART_RX` |
| MaixCam | `BOARD_MAIXCAM_UART`、`BOARD_MAIXCAM_UART_TX`、`BOARD_MAIXCAM_UART_RX`、`BOARD_MAIXCAM_BAUDRATE` |

`BOARD_T8_UART` 是保留的 UART1 板名，但当前 line-car 的 T8 实际走
`TRACE_SCL/TRACE_SDA` 软件 I2C；hardware-test 的 UART1 则用于单轴陀螺仪。

## 4. 运行模型和必须遵守的约束

### 4.1 不是多线程，也不是抢占式调度

`ec_scheduler` 是固定顺序的协作式调度器。一个任务未返回时，后面的任务都
不能运行。每次调度扫描中，一个到期任务最多运行一次；错过的周期只计入
`missed_deadlines`，不会补跑多次。

因此周期任务必须：

- 快速返回，不使用无限循环。
- 用 `now_ms` 和状态机表达等待，不在控制循环中调用长延时。
- 把串口解析、显示刷新、控制计算拆成不同任务。
- 不假设 `period_ms` 就等于精确执行间隔；必要时用时间戳计算真实 `dt`。

### 4.2 中断只采集，主循环做解析

`user/src/isr.c` 集中定义定时器、UART 和 GPIO ISR。典型 UART 数据路径是：

```text
UART RX ISR -> SerialRxBuffer.push -> 主循环 update() -> 协议解析/控制
```

ISR 中允许计数、入队和紧急关闭 PWM，不应打印、延时、做浮点控制或阻塞读写。

### 4.3 时间回绕

毫秒计数是 `uint32_t`，约 49.7 天回绕。比较周期应使用
`ec_time_elapsed()` 或 `(int32_t)(now - deadline)` 形式，不直接比较
`now > deadline`。

### 4.4 对象和存储所有权

多数 API 不动态分配内存：

- scheduler、mode manager、PID 等由调用者分配对象。
- transport 的 `context/user_data` 只是借用指针。
- 蜂鸣器 sequence、参数菜单 item 数组在使用期间必须一直有效。
- 串口环形缓冲区的字节和时间戳数组由调用者持有。
- 单例 API 包括 `dt_gyro_z`、`dt_hc05` 和 `maixcam2`。

## 5. 当前两个应用的调用链

### 5.1 Line-car

`ec_app_init()` 为 line-car 注册以下任务：

| 周期 | 任务 | 职责 |
| ---: | --- | --- |
| 1 ms | `line_car_input_task` | 按键、急停、故障确认、主循环看门狗 |
| 10 ms | `line_car_gyro_task` | `dt_heading` 统一航向更新 |
| 10 ms | `line_car_line_sensor_task` | T8 读取和线状态去抖 |
| 50 ms | `line_car_sensor_task` | ADC、电池、编码器 RPM/里程 |
| 10 ms | `line_car_control_task` | 当前模式和 H 题状态机 |
| 200 ms | `line_car_menu_task` | 菜单渲染 |
| 5 ms | `line_car_buzzer_task` | 非阻塞蜂鸣器和点位 LED |
| 20 ms | `line_car_oled_task` | OLED 脏页刷新 |
| 20 ms | `line_car_telemetry_task` | VOFA 遥测 |

主要数据流：

```text
T8 bits -> 线位置/进出线计数 ------+
M0/MPU6050 -> dt_heading -> yaw/wz -+-> 模式/H1-H4 -> 左右轮目标 PWM
编码器 -> rpm/里程 ----------------+                   |
电池 ADC -> 电压/补偿/欠压保护 -----+                   v
                                                    dt_motor
```

`LINE FOLLOW`、`SPEED TEST`、`TUNING`、`GYRO TEST` 和 `2024 H1-H4`
都由同一个 `ec_mode_manager` 管理。完整比赛行为见
[COMPETITION_TASKS.md](COMPETITION_TASKS.md)，按键见 [UI_MENU.md](UI_MENU.md)。

### 5.2 Hardware-test / gimbal

```text
UART3 ISR -> MaixCam ring -> maixcam2_update -> MaixVisionTarget
                                                |
                          position/speed controller
                                                |
                                     Gimbal 高层软限位
                                                |
                                    EMM 协议 -> UART2
```

此 profile 上电时默认急停锁存且未 armed。只有建立可信机械参考、清除安全锁存、
使能并验证电机后，`hardware_test_rearm()` 才成功。默认
`HW_TEST_GIMBAL_ACCEPT_STARTUP_REFERENCE=0` 时，当前交互流程不会凭空建立参考，
这是有意的安全设计，不应为了“上电自动转”而绕过。

## 6. API 通用约定

- `bool`：`true` 表示成功或条件成立，`false` 表示失败。
- `uint8_t` 驱动返回值：多数 MPU 接口以 `1` 表示成功、`0` 表示失败。
- `T8Status`、`EmmStatus`、`GimbalStatus`：`0` 成功，负数失败。
- `now_ms`：必须来自同一个单调毫秒时基，通常传 `ec_time_ms()`。
- scheduler 回调：签名为 `void fn(uint32_t now_ms, void *context)`。
- UART transport：回调返回实际处理的字节数；I2C transport 返回布尔成功。
- 所有硬件驱动都应先初始化，再读写。

## 7. Framework API

### 7.1 应用组装 `framework/ec_app.h`

```c
void ec_app_init(void);
void ec_app_run(void);
int ec_app_emergency_stop(void);
```

- `ec_app_init()`：初始化时基、scheduler，并按 `EC_APP_PROFILE` 初始化和注册任务。
- `ec_app_run()`：执行一次到期任务扫描，然后 `__WFI()`；由 `main()` 永久调用。
- `ec_app_emergency_stop()`：转发给当前 profile。line-car 返回 0；hardware-test
  返回转换成 `int` 的 `GimbalStatus`；empty profile 返回 0。

### 7.2 单调时基 `framework/ec_time.h`

```c
typedef void (*ec_time_tick_hook_t)(uint32_t now_ms, void *context);

void ec_time_init(void);
uint32_t ec_time_ms(void);
bool ec_time_elapsed(uint32_t now_ms, uint32_t since_ms, uint32_t period_ms);
void ec_time_set_tick_hook(ec_time_tick_hook_t hook, void *context);
```

- `ec_time_init()`：占用 `PIT_TIM_G0`，建立 1 ms 时基并清除旧 hook。
- `ec_time_ms()`：返回当前毫秒计数。
- `ec_time_elapsed()`：回绕安全地判断从 `since_ms` 起是否经过 `period_ms`。
- `ec_time_set_tick_hook()`：原子替换唯一 tick hook；传 `NULL` 删除。

tick hook 直接从 TIMG0 ISR 调用，必须短、不可阻塞、不可使用普通串口打印。
line-car 用它实现电机命令超时后的 ISR 级 PWM 关闭。

### 7.3 协作调度器 `framework/ec_scheduler.h`

```c
#define EC_SCHEDULER_MAX_TASKS 12u
typedef void (*ec_task_fn)(uint32_t now_ms, void *context);

void ec_scheduler_init(ec_scheduler_t *scheduler);
bool ec_scheduler_add(ec_scheduler_t *scheduler, ec_task_fn run,
    void *context, uint32_t period_ms, uint32_t start_ms);
void ec_scheduler_run(ec_scheduler_t *scheduler, uint32_t now_ms);
```

`ec_task_t` 公开保存 `run/context/period_ms/next_run_ms/run_count/
missed_deadlines/enabled`；`ec_scheduler_t` 最多保存 12 个任务。

- `ec_scheduler_init()`：逻辑重置任务表，将 count、各槽 run/enabled 清除；未使用
  槽的其他公开字段不会全部归零，调用者不应读取它们；`NULL` 时忽略。
- `ec_scheduler_add()`：添加任务。callback 为空、周期为 0 或表满时返回 `false`。
  `start_ms` 是第一次运行的绝对时间，不是延迟量。
- `ec_scheduler_run()`：按注册顺序执行到期且 enabled 的任务，并在任务间重新读取时间。

示例：

```c
static void sample_task(uint32_t now_ms, void *context)
{
    (void)now_ms;
    (void)context;
    /* 快速采样，不阻塞。 */
}

ec_scheduler_t scheduler;
ec_scheduler_init(&scheduler);
if (!ec_scheduler_add(&scheduler, sample_task, NULL, 10u, ec_time_ms()))
{
    /* 处理注册失败。 */
}
```

### 7.4 模式管理 `framework/ec_mode_manager.h`

```c
#define EC_MODE_MAX_COUNT 16u
typedef bool (*ec_mode_init_fn)(void *context);
typedef void (*ec_mode_fn)(uint32_t now_ms, void *context);

typedef enum {
    EC_MODE_STOPPED,
    EC_MODE_RUNNING,
    EC_MODE_FAULT
} ec_mode_state_t;

void ec_mode_manager_init(ec_mode_manager_t *manager);
bool ec_mode_manager_add(ec_mode_manager_t *manager, const ec_mode_t *mode);
void ec_mode_manager_select_next(ec_mode_manager_t *manager);
void ec_mode_manager_select_previous(ec_mode_manager_t *manager);
bool ec_mode_manager_start(ec_mode_manager_t *manager, uint32_t now_ms);
void ec_mode_manager_stop(ec_mode_manager_t *manager, uint32_t now_ms);
void ec_mode_manager_run(ec_mode_manager_t *manager, uint32_t now_ms);
const char *ec_mode_manager_selected_name(const ec_mode_manager_t *manager);
const char *ec_mode_manager_active_name(const ec_mode_manager_t *manager);
```

`ec_mode_t` 包含 `name/init/start/run/stop/context`。manager 按值复制描述符，
但不拥有 `name` 和 `context` 指向的存储。

- `add()` 在复制前立即调用可选 `init(context)`；失败则不添加。
- 只有 STOPPED 状态能切换选择。
- `start()` 设置 RUNNING 后调用 start callback；callback 可以把状态改为 FAULT。
- `stop()` 调用 active stop 后强制进入 STOPPED。
- `run()` 只在 RUNNING 时调用 active run。
- 名称查询在没有任务时返回静态字符串 `"NO TASK"`。

新增模式的最小示例：

```c
static void demo_start(uint32_t now_ms, void *context) { (void)now_ms; (void)context; }
static void demo_run(uint32_t now_ms, void *context)   { (void)now_ms; (void)context; }
static void demo_stop(uint32_t now_ms, void *context)  { (void)now_ms; (void)context; }

const ec_mode_t mode = {
    .name = "DEMO",
    .init = NULL,
    .start = demo_start,
    .run = demo_run,
    .stop = demo_stop,
    .context = NULL,
};
ec_mode_manager_add(&manager, &mode);
```

stop callback 必须让执行器进入安全状态。

### 7.5 菜单 `framework/ec_menu.h`

```c
typedef enum {
    EC_MENU_KEY_PREVIOUS = 1,
    EC_MENU_KEY_NEXT = 2,
    EC_MENU_KEY_CONFIRM = 3
} ec_menu_key_t;

typedef void (*ec_menu_render_fn)(const ec_mode_manager_t *manager,
    uint32_t now_ms, void *context);

void ec_menu_init(ec_menu_t *menu, ec_mode_manager_t *manager,
    ec_menu_render_fn render, void *render_context, uint32_t render_period_ms);
void ec_menu_handle_key(ec_menu_t *menu, ec_menu_key_t key, uint32_t now_ms);
void ec_menu_update(ec_menu_t *menu, uint32_t now_ms);
```

- `init()`：绑定 manager 和 renderer；0 周期会改为 100 ms。
- `handle_key()`：前后键选择；RUNNING 时确认键 stop、STOPPED 时确认键 start；
  `EC_MODE_FAULT` 时 generic menu 无法自行确认故障，因为 manager 会拒绝 start，
  应用必须先显式 stop/清 fault。处理后标记界面 dirty。
- `update()`：dirty 或周期到达时同步调用 renderer。框架不依赖 OLED。

### 7.6 按键 `framework/ec_keys.h`

```c
typedef struct {
    gpio_pin_enum key1_pin;
    gpio_pin_enum key2_pin;
    gpio_pin_enum key3_pin;
    uint32_t debounce_ms;
    uint32_t startup_lock_ms;
} ec_keys_config_t;

void ec_keys_init(const ec_keys_config_t *config);
void ec_keys_poll(void);
bool ec_keys_pop(uint8_t *key);
```

- 三个输入都按“上拉、低电平按下”配置，并注册下降沿 EXTI。
- `poll()` 是共享中断的轮询后备路径，必须在 init 后调用。
- `pop()` 在关中断保护下取一个键值 1/2/3。
- 队列数组大小为 8，环形实现实际可保存 7 个事件；满时静默丢弃新事件。

### 7.7 参数菜单 `framework/ec_parameter_menu.h`

公开类型：

```c
typedef enum {
    EC_PARAM_INT8,
    EC_PARAM_INT16,
    EC_PARAM_UINT16,
    EC_PARAM_FLOAT,
    EC_PARAM_BOOL,
    EC_PARAM_ACTION
} ec_parameter_type_t;

typedef void (*ec_parameter_action_fn)(void *context);
```

`ec_parameter_item_t` 包含 `name/type/value/min_value/max_value/step/action/context`；
`value` 必须指向与 `type` 完全匹配的存储。ACTION 使用 `action(context)`。

```c
void ec_parameter_menu_init(ec_parameter_menu_t *menu,
    ec_parameter_item_t *items, uint8_t count);
void ec_parameter_menu_handle_key(ec_parameter_menu_t *menu, ec_menu_key_t key);
const ec_parameter_item_t *ec_parameter_menu_current(const ec_parameter_menu_t *menu);
void ec_parameter_menu_format_value(const ec_parameter_item_t *item,
    char *buffer, size_t capacity);
```

- 未编辑时前后键选择，确认键进入编辑或执行 ACTION。
- 编辑时前后键按 `step` 修改并钳位到 min/max；BOOL 两个方向都执行翻转。
- `current()` 返回借用指针，无有效条目时返回 `NULL`。
- `format_value()` 格式化 ACTION、BOOL、浮点和整数文本。

## 8. 通用库与协议 API

### 8.1 PID `lib/pid_controller.h`

`PidController` 的公开字段包括 PID 增益、积分和上次误差、输出/积分限幅、
deadband、导数低通系数以及初始化状态。

```c
void pid_init(PidController *pid);
void pid_reset(PidController *pid);
void pid_set_gain(PidController *pid, float kp, float ki, float kd);
void pid_set_limits(PidController *pid, float output_min, float output_max,
    float integral_min, float integral_max);
void pid_set_deadband(PidController *pid, float deadband);
void pid_set_derivative_lpf(PidController *pid, float alpha);
float pid_update(PidController *pid, float error, float dt_s);
```

- `init()`：清状态，默认限幅为 +/-1e6，导数不过滤。
- `reset()`：只清动态状态，保留增益和限幅。
- `set_gain()`：`ki=0` 时同时清积分。
- `set_deadband()`：取绝对值。
- `set_derivative_lpf()`：alpha 钳位到 `[0,1]`；0 保持旧导数，1 不滤波。
- `update()`：`pid==NULL` 或 `dt_s<=0` 返回 0；包含积分限幅、输出限幅和
  条件积分 anti-windup。调用者仍需保证数值有限且 min <= max。

### 8.2 串口 SPSC 环形缓冲区 `lib/serial_rx_buffer.h`

```c
void serial_rx_buffer_init(SerialRxBuffer *buffer, uint8_t *storage, size_t capacity);
void serial_rx_buffer_init_timed(SerialRxBuffer *buffer, uint8_t *storage,
    uint32_t *timestamp_storage, size_t capacity);
void serial_rx_buffer_clear(SerialRxBuffer *buffer);
bool serial_rx_buffer_push(SerialRxBuffer *buffer, uint8_t byte);
bool serial_rx_buffer_push_timed(SerialRxBuffer *buffer, uint8_t byte, uint32_t rx_time_ms);
bool serial_rx_buffer_pop(SerialRxBuffer *buffer, uint8_t *byte);
bool serial_rx_buffer_pop_timed(SerialRxBuffer *buffer, uint8_t *byte, uint32_t *rx_time_ms);
bool serial_rx_buffer_peek(const SerialRxBuffer *buffer, size_t offset, uint8_t *byte);
size_t serial_rx_buffer_available(const SerialRxBuffer *buffer);
size_t serial_rx_buffer_overflow_count(const SerialRxBuffer *buffer);
void serial_rx_buffer_drop(SerialRxBuffer *buffer, size_t length);
```

设计目标是“一个 ISR producer + 一个主循环 consumer”。容量必须至少为 2，
实际可用容量是 `capacity - 1`。满时 push 返回 `false` 并增加 overflow；
`clear()` 丢弃未读数据但不清 overflow 统计；`drop()` 最多丢弃现有数据。

UART 接入模板：

```c
static uint8_t rx_storage[256];
static SerialRxBuffer rx_ring;

static void uart_rx_callback(uint32_t state, void *context)
{
    uint8_t byte;
    (void)context;
    if ((state & UART_INTERRUPT_STATE_RX) == 0u) return;
    while (uart_query_byte(MY_UART, &byte) == ZF_TRUE)
    {
        (void)serial_rx_buffer_push(&rx_ring, byte);
    }
}
```

### 8.3 VOFA JustFloat `protocol/vofa.h`

```c
#define VOFA_JUSTFLOAT_MAX_CHANNELS 15u
typedef bool (*VofaWriteCallback)(const uint8_t *data, size_t length, void *context);
typedef struct { VofaWriteCallback write; void *context; } VofaTransport;

bool vofa_send(const VofaTransport *transport, const float *data, uint8_t count);
```

`vofa_send()` 按 MCU 本地的小端 32 位 float 编码，末尾追加
`00 00 80 7F`。无效参数返回 `false`；超过 15 路会静默截断；write callback
同步执行，接收到的栈缓冲区只在 callback 返回前有效。

重要：当前 `line_car_telemetry_task()` 准备了 27 个值，但此 API 最多编码前
15 个。因此现状下文档或注释中提到的第 16-27 路没有真正发出。若需要这些
通道，应先增大 API 上限并核对栈占用/带宽，或拆成明确的多个 frame。

## 9. Board Driver API

### 9.1 直流电机 `driver/dt_motor.h`

```c
#define DT_MOTOR_DUTY_MAX 10000
typedef struct {
    pwm_channel_enum in1_pin;
    pwm_channel_enum in2_pin;
    uint32_t pwm_freq;
} dt_motor_config_t;

void dt_motor_init(dt_motor_config_t *cfg);
void dt_motor_set_speed(dt_motor_config_t *cfg, int16_t speed);
void dt_motor_stop(dt_motor_config_t *cfg);
void dt_motor_brake(dt_motor_config_t *cfg);
```

- `init()`：两个 PWM 通道初始化为 0；频率必须非 0。
- `set_speed()`：钳位到 `[-10000,10000]`；正值驱动 IN1，负值驱动 IN2。
- `stop()`：两个通道为 0，属于滑行/停驱动。
- `brake()`：两个通道为 10000，具体制动行为仍取决于 H 桥硬件。

```c
dt_motor_config_t motor = { MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_PWM_FREQ };
dt_motor_init(&motor);
dt_motor_set_speed(&motor, 2500);
dt_motor_stop(&motor);
```

### 9.2 GPIO 编码器 `driver/dt_encoder.h`

`dt_encoder_t` 支持单相和 AB 正交两种模式。当前 line-car 使用旧版已验证的
硬件 AB 正交模式：左 TIMG8 PA26/PA27、右 TIMG12 PA14/PB24，标称 CPR 1560。

```c
void dt_encoder_init(dt_encoder_t *enc);
void dt_encoder_reset_odometry(dt_encoder_t *enc);
uint32_t dt_encoder_get_edges(dt_encoder_t *enc);
int32_t dt_encoder_get_signed_edges(dt_encoder_t *enc);
uint32_t dt_encoder_get_invalid_transitions(dt_encoder_t *enc);
uint32_t dt_encoder_get_delta(dt_encoder_t *enc);
float dt_encoder_compute_rpm(dt_encoder_t *enc, uint32_t dt_ms);
float dt_encoder_get_travel_mm(dt_encoder_t *enc);
float dt_encoder_get_distance_mm(dt_encoder_t *enc);
```

- `init()`：单相模式只配置 `a_pin`；正交模式才配置 `a_pin/b_pin`。
- `reset_odometry()`：原子清边沿、位移、RPM、QERR；只在轮子静止时调用。
- `get_edges()`：无方向总有效边沿。
- `get_signed_edges()`：应用 `direction_sign` 后的有符号位移边沿。
- `get_invalid_transitions()`：正交模式的非法 AB 跳变诊断值。
- `get_delta()`：自上次调用后的无方向边沿增量，并更新内部基准。
- `compute_rpm()`：使用 delta、CPR、`dt_ms` 和一阶低通得到绝对 RPM。
- `get_travel_mm()`：无方向累计路程。
- `get_distance_mm()`：有方向位移。

GPIO pending 位不是边沿 FIFO，高速或长关中断时会丢边。当前 `CPR=780`、
轮径 65 mm 都是待实测标定值，详见
[DRIVER_AUDIT.md](DRIVER_AUDIT.md)。

### 9.3 蜂鸣器 `driver/dt_buzzer.h`

```c
typedef struct { bool on; uint32_t duration_ms; } dt_buzzer_step_t;

void dt_buzzer_init(dt_buzzer_config_t *cfg);
void dt_buzzer_on(dt_buzzer_config_t *cfg);
void dt_buzzer_off(dt_buzzer_config_t *cfg);
void dt_buzzer_beep(dt_buzzer_config_t *cfg, uint32_t duration_ms);
void dt_buzzer_beep_async(dt_buzzer_config_t *cfg, uint32_t duration_ms, uint32_t now_ms);
void dt_buzzer_play_sequence(dt_buzzer_config_t *cfg,
    const dt_buzzer_step_t *sequence, uint8_t length, uint32_t now_ms);
void dt_buzzer_service(dt_buzzer_config_t *cfg, uint32_t now_ms);
void dt_buzzer_service_task(uint32_t now_ms, void *context);
```

`beep()` 是阻塞兼容接口；运行期应使用 `beep_async()` 或 sequence，并周期调用
`service()`。sequence 指针在播放结束前必须有效。`service_task()` 可直接注册到
scheduler，`context` 传 `dt_buzzer_config_t *`。

### 9.4 MPU6050 底层 `driver/dt_mpu6050.h`

公开量程宏：

- 加速度：`DT_MPU6050_ACCEL_FS_2G/4G/8G/16G`。
- 陀螺仪：`DT_MPU6050_GYRO_FS_250/500/1000/2000`。
- 默认 7 位地址：`DT_MPU6050_DEFAULT_ADDR=0x68`。
- 寄存器宏：`DT_MPU6050_REG_SMPLRT_DIV`、`DT_MPU6050_REG_CONFIG`、
  `DT_MPU6050_REG_GYRO_CONFIG`、`DT_MPU6050_REG_ACCEL_CONFIG`、
  `DT_MPU6050_REG_ACCEL_XOUT_H`、`DT_MPU6050_REG_PWR_MGMT_1`、
  `DT_MPU6050_REG_WHO_AM_I`。

`dt_mpu6050_config_t` 保存逐飞软件 I2C 对象和量程；`dt_mpu6050_data_t`
提供 `ax/ay/az`（g）、`gx/gy/gz`（degree/s）、`temp`（摄氏度）。

```c
uint8_t dt_mpu6050_hal_init(soft_iic_info_struct *iic, uint8_t sample_rate_div,
    uint8_t dlpf_cfg, uint8_t gyro_fs, uint8_t accel_fs);
uint8_t dt_mpu6050_hal_write_reg(soft_iic_info_struct *iic, uint8_t reg, uint8_t value);
uint8_t dt_mpu6050_hal_write_regs(soft_iic_info_struct *iic, uint8_t reg,
    const uint8_t *data, uint16_t len);
uint8_t dt_mpu6050_hal_read_reg(soft_iic_info_struct *iic, uint8_t reg, uint8_t *value);
uint8_t dt_mpu6050_hal_read_regs(soft_iic_info_struct *iic, uint8_t reg,
    uint8_t *data, uint16_t len);
uint8_t dt_mpu6050_init(dt_mpu6050_config_t *cfg);
uint8_t dt_mpu6050_read_all(dt_mpu6050_config_t *cfg, dt_mpu6050_data_t *data);
```

返回 1 成功、0 失败。调用 `dt_mpu6050_init()` 前必须先初始化 `cfg->iic`。
HAL init 会复位、延时并检查 WHO_AM_I；DLPF 必须 <=7，量程编号必须 <=3。

### 9.5 MPU6050 航向 `driver/dt_mpu6050_heading.h`

`dt_mpu6050_heading_status_t` 状态：`UNINITIALIZED/BUS_ERROR/ID_ERROR/
CONFIG_ERROR/CALIBRATION_ERROR/READY`。`dt_mpu6050_heading_t` 公开最近六轴样本、相对
`yaw_deg`、去偏置 `wz_dps`、Z 零偏、校准方差、计数和诊断信息。

```c
bool dt_mpu6050_heading_init(dt_mpu6050_heading_t *heading,
    gpio_pin_enum scl, gpio_pin_enum sda);
bool dt_mpu6050_heading_read_sample(dt_mpu6050_heading_t *heading);
bool dt_mpu6050_heading_update(dt_mpu6050_heading_t *heading, uint32_t now_ms);
void dt_mpu6050_heading_zero(dt_mpu6050_heading_t *heading);
```

- `init()`：自动探测 0x68/0x69，配置 +/-500 dps、100 Hz，等待 500 ms 并
  丢弃 32 个预热读数后，以 2 ms 间隔采 1000 次校准；至少 900 次成功且
  Z 方差 <=0.25 `(degree/s)^2`。
- `read_sample()`：只更新最近六轴样本和计数。
- `update()`：读样本并在 1-50 ms 的有效 dt 内积分，yaw wrap 到 [-180,180]。
- `zero()`：清相对 yaw，不重新估计零偏。

当前软件 I2C 配置下初始化约 7 秒，期间必须保持静止。完整接线和故障码见
[MPU6050_USAGE_ZH.md](MPU6050_USAGE_ZH.md)。

### 9.6 Mahony 姿态 `driver/dt_imu_mahony.h`

```c
uint8_t dt_imu_mahony_init(dt_imu_mahony_t *imu,
    gpio_pin_enum scl, gpio_pin_enum sda);
void dt_imu_mahony_update(dt_imu_mahony_t *imu, float dt_s);
void dt_imu_mahony_zero_yaw(dt_imu_mahony_t *imu);
```

`dt_imu_mahony_t` 公开 MPU、四元数、陀螺零偏、积分误差、加速度低通、
offset 和 roll/pitch/yaw。init 返回 1/0；update 仅接受 `0 < dt_s <= 0.1`。
此算法没有磁力计，roll/pitch 可由重力约束，yaw 仍会漂移。当前 line-car 不使用它。
当前实现还私有地固定启用了针对某一设备测得的陀螺零偏
`{-1.69, 0.32, 0.13}`；因此此模块属于实验性/设备相关实现，换 MPU6050 后必须
重新标定并修改实现，不能直接把这些偏置当作通用值。

### 9.7 外置单轴 Z 陀螺仪 `driver/dt_gyro_z.h`

公开波特率枚举 `DT_GYRO_Z_BAUD_2400` 到 `...230400`，输出率枚举
`DT_GYRO_Z_RATE_0_1HZ` 到 `...1000HZ`。`dt_gyro_z_config_t` 指定 UART、
波特率、TX/RX；`dt_gyro_z_data_t` 提供 yaw、wz、raw、更新标志和错误统计。

```c
void dt_gyro_z_init(const dt_gyro_z_config_t *config);
uint8_t dt_gyro_z_update(void);
const dt_gyro_z_data_t *dt_gyro_z_get_data(void);
float dt_gyro_z_get_yaw(void);
float dt_gyro_z_get_wz(void);
uint32_t dt_gyro_z_get_rx_overflow(void);

void dt_gyro_z_unlock(void);
void dt_gyro_z_save(void);
void dt_gyro_z_restart(void);
void dt_gyro_z_restore_default(void);
void dt_gyro_z_zero_yaw(void);
void dt_gyro_z_start_bias_cal(void);
void dt_gyro_z_start_scale_factor_cal(void);
void dt_gyro_z_finish_scale_factor_cal(void);
void dt_gyro_z_set_baud(dt_gyro_z_baud_t baud);
void dt_gyro_z_set_rate(dt_gyro_z_rate_t rate);
void dt_gyro_z_request_bias_status(void);
```

这是单例驱动。ISR 只入 512 字节 ring，必须在主循环调用 `update()` 解析；
返回值是本次新有效帧数。配置命令可能延时、保存参数或重启设备，只适合维护流程。
`get_yaw()/get_wz()` 会无限期返回最近缓存值，没有内置时间戳。用于控制时必须监控
`update()` 返回值或 `frame_count` 的变化，并在外层实现数据 freshness timeout。
`DT_GYRO_Z_BAUD_230400` 的值来自手册歧义后的推测，尚未验证。`set_baud()` 只
修改并保存传感器设置，不会重配 MCU UART；切换后必须按新速率重新初始化 UART/
驱动并重启通信，否则双方会失步。

### 9.8 HC-05 `driver/dt_hc05.h`

```c
#define HC05_NAME "MSPM0_Car"
#define HC05_PIN  "1234"
#define HC05_BAUD 115200

bool dt_hc05_begin(gpio_pin_enum en_pin, uint32_t now_ms);
void dt_hc05_update(uint32_t now_ms);
dt_hc05_status_t dt_hc05_get_status(void);
void dt_hc05_init(gpio_pin_enum en_pin);
```

状态为 `DISABLED/IDLE/BUSY/READY/ERROR_RESPONSE/ERROR_TIMEOUT`。启用时 begin
会启动非阻塞 AT 配置，update 推进状态机，并临时接管 debug UART。当前
`EC_ENABLE_HC05=0`，begin 固定失败、状态为 DISABLED。兼容 `init()` 只是以
时间 0 启动，之后仍必须周期调用 update。

### 9.9 SSD1306 OLED `driver/dt_oled.h`

```c
#define DT_OLED_DEFAULT_ADDR 0x3C
#define DT_OLED_WIDTH 128
#define DT_OLED_HEIGHT 64
#define DT_OLED_PAGE_COUNT 8

void dt_oled_init(dt_oled_config_t *cfg);
void dt_oled_clear(dt_oled_config_t *cfg);
void dt_oled_fill(dt_oled_config_t *cfg, uint8_t data);
void dt_oled_set_pos(dt_oled_config_t *cfg, uint8_t x, uint8_t y);
void dt_oled_show_char(dt_oled_config_t *cfg, uint8_t x, uint8_t y, char ch);
void dt_oled_show_string(dt_oled_config_t *cfg, uint8_t x, uint8_t y, const char *str);
void dt_oled_show_num(dt_oled_config_t *cfg, uint8_t x, uint8_t y, int32_t num, uint8_t len);
void dt_oled_show_hex(dt_oled_config_t *cfg, uint8_t x, uint8_t y, uint32_t num, uint8_t len);
void dt_oled_show_float(dt_oled_config_t *cfg, uint8_t x, uint8_t y,
    float num, uint8_t int_len, uint8_t dec_len);
void dt_oled_mark_page_dirty(dt_oled_config_t *cfg, uint8_t page);
void dt_oled_mark_line_dirty(dt_oled_config_t *cfg, uint8_t line);
void dt_oled_refresh_page(dt_oled_config_t *cfg, uint8_t page);
void dt_oled_refresh_line(dt_oled_config_t *cfg, uint8_t line);
void dt_oled_refresh_dirty(dt_oled_config_t *cfg);
void dt_oled_refresh_task(uint32_t now_ms, void *context);
```

调用者先 `soft_iic_init(&cfg->iic, 0x3C, ...)`，再 `dt_oled_init()`。
`clear/fill/show_*/mark_*` 只改 128x64 framebuffer/脏标记，不立即发送；
`dt_oled_set_pos()` 会立即同步写页列命令，refresh 系列也同步写 I2C。
`y/line/page` 是 0-7 的 8 像素页，字符为 6x8 ASCII。`refresh_task` 的 context
传 `dt_oled_config_t *`。

## 10. T8 灰度传感器 API

头文件：`device/t8_gray_sensor.h`。支持 UART 和 I2C 两种同步 transport。

### 10.1 常量、状态和对象

- `T8_SENSOR_COUNT=8`，`T8_UART_BAUDRATE=115200`。
- 默认 I2C 地址 `T8_DEFAULT_I2C_ADDRESS=0x40`，范围
  `T8_MIN_I2C_ADDRESS=0x10` 到 `T8_MAX_I2C_ADDRESS=0x7F`。
- `T8_MAX_FRAME_SIZE=32`。
- 状态：`T8_OK`，以及 ARG/IO/TIMEOUT/BAD_FRAME/CHECKSUM/UNKNOWN_COMMAND。
- `T8Command` 完整定义 8 位灰度、黑白标定、数字位、16 位 ADC/标定、停止、
  I2C 地址和版本命令，具体命令值见头文件。
- `T8UartTransport`：write/read/flush_input/flush_output/delay_ms/user_data。
- `T8I2cTransport`：write/read/user_data。
- `T8UartDevice`：transport、timeout、retry。
- `T8I2cDevice`：transport、地址和 timeout。
- `T8Packet`：command、length、最多 32 字节 data。

Transport callback 类型为 `T8UartWriteFn`、`T8UartReadFn`、`T8UartFlushFn`、
`T8DelayFn`、`T8I2cWriteFn` 和 `T8I2cReadFn`；UART 回调返回字节数，I2C
回调返回布尔成功。

### 10.2 初始化和 packet API

```c
void t8_uart_init(T8UartDevice *device, const T8UartTransport *transport);
void t8_i2c_init(T8I2cDevice *device, const T8I2cTransport *transport, uint8_t address);
uint8_t t8_checksum(const uint8_t *data, size_t length);

T8Status t8_uart_read_packet(T8UartDevice *device, uint8_t command, T8Packet *packet);
T8Status t8_uart_receive_packet(T8UartDevice *device, T8Packet *packet);
T8Status t8_uart_write_command(T8UartDevice *device, uint8_t command,
    const uint8_t *data, uint8_t length, T8Packet *packet);
T8Status t8_i2c_read_packet(T8I2cDevice *device, uint8_t command, T8Packet *packet);
```

UART 默认 timeout 100 ms、最多重试 3 次；I2C 默认 timeout 100 ms。
`t8_checksum()` 是 8 位加法和，NULL 数据返回 0。
UART packet 读取在查找帧头和读取字段时可能多次调用同步 read，因此最坏阻塞时间
可能是多个 timeout，而不是整笔交易只阻塞 100 ms。不要从 ISR 调用；若要放进
严格 10 ms 控制任务，应改成非阻塞 parser，而不是依赖同步 UART transport。

### 10.3 UART 功能 API

```c
T8Status t8_uart_start_continuous(T8UartDevice *device, uint8_t command,
    uint8_t period_units_10ms);
T8Status t8_uart_stop_continuous(T8UartDevice *device);
T8Status t8_uart_set_i2c_address(T8UartDevice *device, uint8_t address,
    uint8_t *effective_address);
T8Status t8_uart_get_i2c_address(T8UartDevice *device, uint8_t *address);
T8Status t8_uart_get_version(T8UartDevice *device, uint8_t *version);
T8Status t8_uart_get_gray8(T8UartDevice *device, uint8_t channel, uint8_t *value);
T8Status t8_uart_get_gray8_all(T8UartDevice *device, uint8_t values[8]);
T8Status t8_uart_get_black8_all(T8UartDevice *device, uint8_t values[8]);
T8Status t8_uart_get_white8_all(T8UartDevice *device, uint8_t values[8]);
T8Status t8_uart_get_digital(T8UartDevice *device, uint8_t *bits);
T8Status t8_uart_get_adc16(T8UartDevice *device, uint8_t channel, uint16_t *value);
T8Status t8_uart_get_adc16_all(T8UartDevice *device, uint16_t values[8]);
T8Status t8_uart_get_black16_all(T8UartDevice *device, uint16_t values[8]);
T8Status t8_uart_get_white16_all(T8UartDevice *device, uint16_t values[8]);
```

channel 是 1-8。continuous start/stop 只发命令，不等待 ACK。

### 10.4 I2C 功能 API

```c
T8Status t8_i2c_get_gray8(T8I2cDevice *device, uint8_t channel, uint8_t *value);
T8Status t8_i2c_get_gray8_all(T8I2cDevice *device, uint8_t values[8]);
T8Status t8_i2c_get_black8_all(T8I2cDevice *device, uint8_t values[8]);
T8Status t8_i2c_get_white8_all(T8I2cDevice *device, uint8_t values[8]);
T8Status t8_i2c_get_digital(T8I2cDevice *device, uint8_t *bits);
T8Status t8_i2c_get_adc16(T8I2cDevice *device, uint8_t channel, uint16_t *value);
T8Status t8_i2c_get_adc16_all(T8I2cDevice *device, uint16_t values[8]);
T8Status t8_i2c_get_black16_all(T8I2cDevice *device, uint16_t values[8]);
T8Status t8_i2c_get_white16_all(T8I2cDevice *device, uint16_t values[8]);
```

每次操作先写一个命令字节，再同步读取并校验长度和 checksum。line-car 使用
`t8_i2c_get_digital()`，并把传感器位按低有效黑线处理。

## 11. 应用领域 API

### 11.1 电池补偿 `app/battery_compensation.h`

`battery_compensation_status_t` 状态：`BATTERY_COMP_STARTUP/OK/INVALID/
UNDERVOLTAGE`。
config 包含参考、有效范围、欠压、恢复电压、补偿限幅、滤波和去抖样本数；
runtime 对象保存过滤电压、factor、状态和恢复条件。

```c
void battery_compensation_init(battery_compensation_t *compensation,
    const battery_compensation_config_t *config);
void battery_compensation_update(battery_compensation_t *compensation,
    uint16_t sample_mv, bool sample_valid);
bool battery_compensation_acknowledge(battery_compensation_t *compensation);
bool battery_compensation_can_run(const battery_compensation_t *compensation);
float battery_compensation_factor(const battery_compensation_t *compensation);
float battery_compensation_voltage_mv(const battery_compensation_t *compensation);
float battery_compensation_apply(const battery_compensation_t *compensation,
    float reference_pwm, float maximum_pwm);
```

`update()` 对无效样本和运行态欠压进行去抖；STARTUP 收到第一个低于 cutoff 的
有效样本会立即进入欠压。补偿因子是限制后的
`reference_mv/filtered_mv`。故障恢复只置 `recovery_ready`，必须显式
`acknowledge()`。`apply()` 在不可运行、reference PWM 为 NaN 或 maximum PWM
不大于 0 时返回 0；调用者还应保证所有输入和 config 都是有限值。

调用者必须保证 `valid_min_mv <= cutoff_mv < recovery_mv <= valid_max_mv`，当前
init 不会替你验证这组关系。

### 11.2 车辆调参 `app/car_tuning.h`

`car_tuning_t` 保存 target RPM、base/feedforward、speed PID、line PD、heading
PD、最大转向、左右增益、方向符号和 speed-loop 开关。

```c
void car_tuning_defaults(car_tuning_t *tuning);
int16_t car_tuning_feedforward_pwm(const car_tuning_t *tuning);
int16_t car_tuning_feedforward_pwm_for_rpm(const car_tuning_t *tuning,
    float target_rpm);
```

feedforward 计算 `base_pwm + target_rpm * gain` 并钳位到统一的
`MOTOR_PWM_DUTY_MAX`；NULL、NaN 或非正结果返回 0。base PWM 是物理 PWM
前馈截距，输出端不会再次叠加一层死区。默认值和调参范围集中在
`user/inc/config.h`，仍必须按实车调试。

### 11.3 线事件去抖 `app/line_event_detector.h`

```c
void line_event_detector_init(line_event_detector_t *detector,
    uint8_t debounce_samples);
void line_event_detector_update(line_event_detector_t *detector,
    bool raw_on_line, uint32_t now_ms);
```

第一次 update 只建立基线，不产生 enter/exit；状态变化连续达到样本数后，更新
`stable_on_line`、`enter_count/exit_count` 和 `last_transition_ms`。0 样本会
改为 1。

### 11.4 H1-H4 车辆端口 `app/h2024_tasks.h`

任务 ID：`H2024_TASK_1` 到 `H2024_TASK_4`。
`h2024_vehicle_state_t` 提供 heading、line enter/exit count 和 on-line 状态。

`h2024_vehicle_port_t` 是业务与底盘之间的依赖反转接口：

```c
bool (*prepare)(h2024_task_id_t task, uint32_t now_ms, void *context);
void (*reset_odometry)(void *context);
bool (*read_state)(h2024_vehicle_state_t *state, uint32_t now_ms, void *context);
void (*drive_heading)(float heading_deg, uint32_t now_ms, void *context);
void (*follow_line)(uint32_t now_ms, void *context);
bool (*align_heading_forward)(float heading_deg, uint32_t now_ms, void *context);
void (*signal_point)(uint32_t now_ms, void *context);
void (*stop)(uint32_t now_ms, void *context);
```

```c
bool h2024_tasks_register(ec_mode_manager_t *manager,
    const h2024_vehicle_port_t *vehicle, void *vehicle_context);
bool h2024_tasks_is_active(const ec_mode_manager_t *manager);
h2024_task_state_t h2024_tasks_active_state(const ec_mode_manager_t *manager);
const char *h2024_tasks_active_status(const ec_mode_manager_t *manager);
```

register 要求 prepare/read/drive/follow/align/stop 齐全，并向 manager 添加四个
mode。任何准备/读取失败或路线 timeout 都会 stop 并进入 `EC_MODE_FAULT`。
弧线出口还要求弧线状态至少持续 800 ms 且相对入口航向变化至少 130 度。第二条
直线必须持续至少 800 ms 且达到第一条直线实测用时的 60%，入口附近的短暂丢线或
重获不会再被误判为后续赛道点。普通循迹和 H 弧线共用可调的 `LINE KP/KD`，使用
实际控制周期计算误差 D 项并进行低通。正常循迹先降低内轮；若内轮将低于
`MOTOR_MIN_RUN_PWM_L/R`，则通过受斜率限制的公共 PWM 补偿保持内轮转动并逐渐提高
外轮，补偿上限和斜率由 `CAR_STEER_BOOST_MAX_PWM`、
`CAR_STEER_BOOST_SLEW_PWM_PER_S` 控制，当前分别为 800 PWM 和 6000 PWM/s。
基础 PWM 为 4900，高于最低运行值 4500，因此误差 1 的中小转向完全不需要
补偿也不会加速。T8 位置误差按线性关系输入位置 PID，避免边缘误差档位产生二次增长。
位置 PID 的有符号修正量还受 `CAR_LINE_STEER_SLEW_PWM_PER_S=6000 PWM/s` 限制，
误差换向时必须平滑经过零点，不能瞬间交换左右轮补偿；默认 `LINE KD=0`，避免
离散位置跳变产生微分尖峰。
巡线启动时先以固定物理 PWM 5500 驱动两轮 20 ms，再切换到 4500 基础 PWM
并启用位置纠偏。主动寻线使用双轮反向、固定 4500 PWM 低速原地旋转，不设自动
超时，直到检测到的线进入中央区域后才恢复正常前进；定点对齐仍允许零速。
`active_state()` 返回公开状态枚举，status 返回静态借用
文本，如 `H ARC1`、`H DONE`、`H TIMEOUT`。
模块内部使用一组静态全局 context，只应注册一次；调用前 manager 至少要有四个
空槽。四个 mode 是逐个添加且失败时不回滚，若 register 返回 false，应重新初始化
manager 后重新组装模式，不能继续使用半注册状态。

### 11.5 Line-car profile `app/line_car.h`

车辆、电机、编码器、循迹调参和电池配置统一位于 `user/inc/config.h`。电池宏为
`BAT_REFERENCE_MV`、`BAT_DIVIDER`、`BAT_ADC_REF_MV`、
`BAT_VALID_MIN_MV`、`BAT_VALID_MAX_MV`、`BAT_UNDERVOLTAGE_MV`、
`BAT_RECOVERY_MV`、`BAT_COMP_MIN_FACTOR` 和 `BAT_COMP_MAX_FACTOR`；当前定义
14.8 V 参考、4.47 分压、13.0 V 欠压、14.0 V 恢复以及 0.85-1.30 补偿范围。

```c
void line_car_init(void);
void line_car_run(void);
void line_car_emergency_stop(void);
void line_car_input_task(uint32_t now_ms, void *context);
void line_car_gyro_task(uint32_t now_ms, void *context);
void line_car_line_sensor_task(uint32_t now_ms, void *context);
void line_car_sensor_task(uint32_t now_ms, void *context);
void line_car_control_task(uint32_t now_ms, void *context);
void line_car_menu_task(uint32_t now_ms, void *context);
void line_car_buzzer_task(uint32_t now_ms, void *context);
void line_car_oled_task(uint32_t now_ms, void *context);
void line_car_telemetry_task(uint32_t now_ms, void *context);
```

- `init()`：初始化所有 line-car 硬件、保护、模式和菜单；当前软件 I2C 配置下
  MPU 校准会阻塞约 7 秒。
- `run()`：兼容式聚合入口，当前 scheduler profile 不调用它。
- `emergency_stop()`：立即锁存急停并进入 fault。
- 九个 task 用途见第 5.1 节；当前都忽略 context。

电机输出同时受电池、陀螺仪、命令 watchdog、T8/丢线和 emergency latch 保护。
故障不会因电压恢复而自动重新开车，必须满足恢复条件并由按键确认。

### 11.6 Hardware-test profile `app/hardware_test.h`

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

- `init()`：初始化 KEY3、UART1 gyro、EMM gimbal、MaixCam 和控制器；默认未 armed。
- `run()`：1 ms 调用，处理急停、串口、视觉控制和周期诊断。
- `set_gimbal_control_mode()`：只接受 position/speed；若当前已 armed 则先停止，
  随后重置两个 controller 并切换。
- `emergency_stop()`：disarm、锁存、停止并 disable。
- `rearm()`：要求初始化成功、已 homed/position valid、KEY3 已释放；清 safety、
  enable 并验证后 armed。

## 12. MaixCam 与视觉控制 API

### 12.1 MaixCam2 `gimbal/maixcam2_protocol.h`

协议宏为 `MAIXCAM2_FRAME_HEAD1=0xAA`、`MAIXCAM2_FRAME_HEAD2=0x55`、
`MAIXCAM2_PROTOCOL_VER=1`、`MAIXCAM2_MAX_PAYLOAD=64`、
`MAIXCAM2_TARGET_PAYLOAD_LEN=36`、`MAIXCAM2_INTERBYTE_TIMEOUT_MS=20` 和
`MAIXCAM2_MAX_ABS_ERROR_PX=500`。
`VisionState` 为 IDLE/SEARCHING/CANDIDATE/LOCKED/TRACKING/LOST。

`MaixVisionTarget`：`target_valid/error_x/error_y/vision_state`。
`MaixProtocolStats`：有效帧、CRC、ring overflow、格式、语义和字节间超时统计。

```c
void maixcam2_init(void);
void maixcam2_update(uint32_t now_ms);
bool maixcam2_get_latest_target(MaixVisionTarget *target, uint32_t *rx_time_ms);
bool maixcam2_target_semantically_valid(const MaixVisionTarget *target);
const MaixProtocolStats *maixcam2_get_stats(void);
uint16_t maixcam2_crc16_modbus(const uint8_t *data, uint16_t length);
```

这是 UART3 单例。update 必须在主循环调用。默认误差绝对值上限为 500 px；
overflow 会让当前 target 失效并重置 parser。stats 返回内部借用指针。
`maixcam2_get_latest_target()` 返回 true 只表示存在缓存过的语义有效 target，不
表示它仍然新鲜；调用者必须比较 `now_ms-rx_time_ms` 与自己的 freshness timeout。

### 12.2 公共状态 `gimbal/vision_gimbal_control.h`

- `VisionControlState`：IDLE、SEARCH、COARSE_TRACK、FINE_TRACK、LOST_HOLD、FAILSAFE。
- `GimbalControlMode`：POSITION 或 SPEED。

### 12.3 位置控制 `gimbal/position_control.h`

`PositionControlConfig` 包括 PID、每次最大角度增量、最大输出角速度、deadband、
大误差门限、视觉 timeout、lost hold 和控制周期。`PositionController` 保存两个
PID、状态、计时、计数和 failsafe/motion 标志。

```c
void position_control_init(PositionController *ctrl);
void position_control_update(PositionController *ctrl, Gimbal *gimbal, uint32_t now_ms);
VisionControlState position_control_get_state(const PositionController *ctrl);
```

update 按配置周期读取最新 target，限制每次位置增量并调用
`gimbal_move_relative()`。视觉 timeout、命令失败或 stall 恢复失败会停止并锁存
failsafe；但当前实现遇到未 homed/position invalid 时只把 state 设为 FAILSAFE
并返回，不会额外 stop 或设置 latch。正常 profile 用 rearm 前置条件避免此路径，
独立调用者必须自行保证执行器已停。NULL 查询返回 IDLE。

### 12.4 速度控制 `gimbal/speed_control.h`

`SpeedControlConfig` 包括 PID、输出角速度、静摩擦补偿、电机 RPM 上限、视觉和
反馈周期/超时、command horizon。controller 保存两个 PID、反馈时间和上次 RPM。

```c
void speed_control_init(SpeedController *ctrl);
void speed_control_update(SpeedController *ctrl, Gimbal *gimbal, uint32_t now_ms);
void speed_control_stop(SpeedController *ctrl, Gimbal *gimbal);
VisionControlState speed_control_get_state(const SpeedController *ctrl);
```

当前实现把输出轴 degree/s 按齿轮比换算为 motor RPM，并发送有时间边界的相对
位置段，不允许无限 jog。反馈 stale、视觉 timeout 或命令失败进入 failsafe。

## 13. EMM 步进电机协议 API

头文件：`gimbal/emm_stepper.h`。这是 transport 无关的同步协议层。

### 13.1 状态、模式和结构

- `EmmStatus`：OK、INVALID_ARG、IO、TIMEOUT、BAD_RESPONSE、CHECKSUM、PARAM、
  FORMAT、OVERFLOW、NO_RESPONSE。
- 模式枚举：`EmmChecksumMode`、`EmmDirection`、`EmmSyncFlag`、`EmmStoreFlag`、
  `EmmMotionMode`、`EmmHomingMode`、`EmmControlMode`、`EmmMotorType`、
  `EmmFirmwareType`、`EmmBaudRate`、`EmmCanRate`、`EmmResponseMode`、
  `EmmStallProtect`、`EmmPulsePortMode`、`EmmSerialPortMode`、`EmmEnableLevel`、
  `EmmDirLevel`。枚举数值与设备协议一致，见头文件定义。
- `EmmTransport`：同步 write/read/flush_input/flush_output/delay_ms/user_data。
- `EmmDevice`：地址、checksum、timeout/retry、response mode、校验/flush 策略和
  内部 256 字节 parser ring。
- 参数/响应结构：`EmmJogParams`、`EmmPositionParams`、`EmmHomingParams`、
  `EmmVersionParams`、`EmmMotorRHParams`、`EmmPIDParams`、`EmmHomingStatus`、
  `EmmMotorStatus`、`EmmSystemStatusParams`、`EmmConfigParams`、
  `EmmAutoRunParams`、`EmmRxFrame`。

Transport callback 类型为 `EmmWriteFn`、`EmmReadFn`、`EmmFlushFn` 和
`EmmDelayFn`。协议常量包括 `EMM_STEPPER_DEFAULT_BAUDRATE`、
`EMM_STEPPER_DEFAULT_ADDRESS`、`EMM_STEPPER_BROADCAST_ADDRESS`、
`EMM_STEPPER_MAX_RETRIES`、`EMM_STEPPER_MAX_FRAME_SIZE`、
`EMM_STEPPER_RX_BUFFER_SIZE`、`EMM_STEPPER_RX_READ_CHUNK`、
`EMM_STEPPER_DEFAULT_TIMEOUT_MS`、`EMM_STEPPER_REACHED_TIMEOUT_MS` 和
`EMM_STEPPER_POLL_ATTEMPTS`。

MODBUS 和 DMX512 checksum 虽然出现在枚举中，但当前
`emm_calculate_checksum()` 没有实现对应算法，会退回固定值 `0x6B`。不能在未
补实现和测试前把它们当作可用模式。

### 13.2 本地对象和接收 API

```c
void emm_init(EmmDevice *device, const EmmTransport *transport, uint8_t address);
void emm_select_address(EmmDevice *device, uint8_t address);
void emm_set_checksum_mode(EmmDevice *device, EmmChecksumMode mode);
void emm_set_response_mode_local(EmmDevice *device, EmmResponseMode mode);
void emm_set_timeouts(EmmDevice *device, uint32_t command_timeout_ms,
    uint32_t reached_timeout_ms);
void emm_set_strict_frame_check(EmmDevice *device, bool enable);
void emm_set_auto_flush_before_write(EmmDevice *device, bool enable);
void emm_rx_clear(EmmDevice *device);
size_t emm_rx_available(const EmmDevice *device);
size_t emm_rx_overflow_count(const EmmDevice *device);
uint8_t emm_calculate_checksum(const uint8_t *data, size_t length, EmmChecksumMode mode);
EmmStatus emm_poll(EmmDevice *device, uint32_t timeout_ms);
```

这些 set 函数只改本地软件对象，不会配置电机本体。默认写命令是
`EMM_RESPONSE_NONE`。读取时建议同一个物理 UART bus 只使用一个承担解析的
`EmmDevice`，避免两个对象竞争同一字节流。
默认 `EMM_RESPONSE_NONE` 下，写命令返回 `EMM_OK` 通常只表示字节已经发给
transport，不表示电机接受或执行成功。配置和安全关键命令必须通过 forced status/
config readback 验证；尤其 `emm_set_id()` 会在未收到 ACK 时也更新本地 address。

### 13.3 帧和 raw 命令

```c
EmmStatus emm_read_fixed_frame(EmmDevice *device, uint8_t expected_address,
    uint8_t expected_code, uint8_t *response, size_t response_length,
    uint32_t timeout_ms);
EmmStatus emm_read_dynamic_frame(EmmDevice *device, uint8_t expected_address,
    uint8_t expected_code, uint8_t *response, size_t response_capacity,
    size_t *response_length, uint32_t timeout_ms);
EmmStatus emm_read_any_frame(EmmDevice *device, EmmRxFrame *frame, uint32_t timeout_ms);
EmmStatus emm_wait_reached(EmmDevice *device, uint8_t expected_code,
    uint8_t *response, size_t response_length, uint32_t timeout_ms);
EmmStatus emm_send_raw_no_response(EmmDevice *device,
    const uint8_t *body, size_t body_length);
EmmStatus emm_send_raw(EmmDevice *device, const uint8_t *body, size_t body_length,
    uint8_t *response, size_t response_length);
EmmStatus emm_send_raw_dynamic(EmmDevice *device, const uint8_t *body,
    size_t body_length, uint8_t *response, size_t response_capacity,
    size_t *response_length);
```

`EMM_STEPPER_MATCH_ANY=0xFF` 可作为期望地址/命令的 wildcard。send 函数会追加
当前 checksum；body 不应包含 checksum。

### 13.4 运动控制

```c
EmmStatus emm_calibrate_encoder(EmmDevice *device);
EmmStatus emm_restart(EmmDevice *device);
EmmStatus emm_zero_position(EmmDevice *device);
EmmStatus emm_zero_position_verified(EmmDevice *device);
EmmStatus emm_clear_protection(EmmDevice *device);
EmmStatus emm_factory_reset(EmmDevice *device);
EmmStatus emm_enable(EmmDevice *device, bool enable, EmmSyncFlag sync_flag);
EmmStatus emm_disable(EmmDevice *device, EmmSyncFlag sync_flag);
EmmStatus emm_jog(EmmDevice *device, const EmmJogParams *params);
EmmStatus emm_move_pulses(EmmDevice *device, const EmmPositionParams *params);
EmmStatus emm_move_degrees(EmmDevice *device, float degrees, uint16_t speed_rpm,
    uint8_t acceleration, EmmMotionMode motion_mode, uint16_t microstep,
    EmmSyncFlag sync_flag);
EmmStatus emm_move_revolutions(EmmDevice *device, float revolutions,
    uint16_t speed_rpm, uint8_t acceleration, EmmMotionMode motion_mode,
    uint16_t microstep, EmmSyncFlag sync_flag);
EmmStatus emm_stop(EmmDevice *device, EmmSyncFlag sync_flag);
EmmStatus emm_stop_verified(EmmDevice *device, EmmSyncFlag sync_flag);
EmmStatus emm_sync_move(EmmDevice *device);
```

速度上限 3000 RPM，microstep 为 1-256。`stop_verified()` 发送无响应 stop，
随后 forced 读取系统状态并验证速度在约 +/-1 RPM 内；任何失败会 fallback
disable，并返回非 OK。它验证的是停机结果，不是 stop 命令本身的 ACK。

### 13.5 回零

```c
EmmStatus emm_set_home_zero(EmmDevice *device, EmmStoreFlag store);
EmmStatus emm_home(EmmDevice *device, EmmHomingMode mode, EmmSyncFlag sync_flag);
EmmStatus emm_stop_home(EmmDevice *device);
EmmStatus emm_get_homing_status(EmmDevice *device, EmmHomingStatus *status);
EmmStatus emm_get_homing_params(EmmDevice *device, EmmHomingParams *params);
EmmStatus emm_set_homing_params(EmmDevice *device,
    const EmmHomingParams *params, EmmStoreFlag store);
```

### 13.6 读取状态

```c
EmmStatus emm_get_version(EmmDevice *device, EmmVersionParams *version);
EmmStatus emm_get_motor_rh(EmmDevice *device, EmmMotorRHParams *params);
EmmStatus emm_get_bus_voltage(EmmDevice *device, uint16_t *voltage_mv);
EmmStatus emm_get_bus_current(EmmDevice *device, uint16_t *current_ma);
EmmStatus emm_get_phase_current(EmmDevice *device, uint16_t *current_ma);
EmmStatus emm_get_encoder(EmmDevice *device, uint16_t *encoder);
EmmStatus emm_get_encoder_degrees(EmmDevice *device, float *degrees);
EmmStatus emm_get_pulse_count(EmmDevice *device, int32_t *pulse_count);
EmmStatus emm_get_target_position(EmmDevice *device, float *degrees);
EmmStatus emm_get_realtime_speed(EmmDevice *device, int16_t *speed_rpm);
EmmStatus emm_get_realtime_position(EmmDevice *device, float *degrees);
EmmStatus emm_get_position_error(EmmDevice *device, float *degrees);
EmmStatus emm_get_temperature(EmmDevice *device, int16_t *temperature_c);
EmmStatus emm_get_motor_status(EmmDevice *device, EmmMotorStatus *status);
EmmStatus emm_get_pid(EmmDevice *device, EmmPIDParams *params);
EmmStatus emm_get_config(EmmDevice *device, EmmConfigParams *params);
EmmStatus emm_get_system_status(EmmDevice *device, EmmSystemStatusParams *params);
```

这些普通 getter 只允许在电机本体和本地 `EmmDevice` 都已配置为产生响应时使用。
在 `EMM_RESPONSE_NONE` 下禁止调用：底层 raw send 可能成功返回但没有填充响应，
随后 getter 会解码未初始化数据。该模式必须使用下面的 forced getter。

### 13.7 参数配置

```c
EmmStatus emm_set_id(EmmDevice *device, uint8_t new_id, EmmStoreFlag store);
EmmStatus emm_set_microstep(EmmDevice *device, uint16_t microstep, EmmStoreFlag store);
EmmStatus emm_set_loop_mode(EmmDevice *device, EmmControlMode mode, EmmStoreFlag store);
EmmStatus emm_set_open_loop_current(EmmDevice *device, uint16_t current_ma,
    EmmStoreFlag store);
EmmStatus emm_set_closed_loop_current(EmmDevice *device, uint16_t current_ma,
    EmmStoreFlag store);
EmmStatus emm_set_pid(EmmDevice *device, const EmmPIDParams *params, EmmStoreFlag store);
EmmStatus emm_set_motor_direction(EmmDevice *device, EmmDirection direction,
    EmmStoreFlag store);
EmmStatus emm_set_position_window(EmmDevice *device, float window_deg,
    EmmStoreFlag store);
EmmStatus emm_set_heartbeat_time(EmmDevice *device, uint32_t time_ms,
    EmmStoreFlag store);
EmmStatus emm_set_auto_run(EmmDevice *device, const EmmAutoRunParams *params);
EmmStatus emm_set_config(EmmDevice *device, const EmmConfigParams *params,
    EmmStoreFlag store);
EmmStatus emm_set_scale_input(EmmDevice *device, bool enable, EmmStoreFlag store);
EmmStatus emm_set_lock_button(EmmDevice *device, bool lock, EmmStoreFlag store);
EmmStatus emm_broadcast_get_id(EmmDevice *device, uint8_t *motor_id);
```

ID 不能为 0，电流最大 5000 mA，position window 为 0-6553.5 degree。
`EMM_STORE_YES` 会写设备持久参数，不应在高频循环反复调用。

### 13.8 Forced read 和恢复

```c
EmmStatus emm_get_realtime_position_forced(EmmDevice *device, float *degrees);
EmmStatus emm_get_encoder_forced(EmmDevice *device, uint16_t *encoder);
EmmStatus emm_get_motor_status_forced(EmmDevice *device, EmmMotorStatus *status);
EmmStatus emm_get_system_status_forced(EmmDevice *device, EmmSystemStatusParams *params);
EmmStatus emm_get_pulse_count_forced(EmmDevice *device, int32_t *pulse_count);
EmmStatus emm_jog_no_response(EmmDevice *device, const EmmJogParams *params);
EmmStatus emm_clear_stall_and_recover(EmmDevice *device);
```

forced read 会临时切换到 RECEIVE、处理半双工 TX echo，并恢复旧模式。调用是阻塞
且不能并发访问同一 UART。`clear_stall_and_recover()` 会验证 stall、停止、清保护、
重新使能并再次验证。

## 14. Gimbal 高层 API

### 14.1 类型和配置 `gimbal/gimbal.h`

`GimbalStatus`：OK、通用错误、MOTOR、SENSOR、CALIB、NOT_HOMED、
SAFETY_LATCHED。

主要编译宏为 `GIMBAL_EMM_UART`、`GIMBAL_EMM_UART_TX_PIN`、
`GIMBAL_EMM_UART_RX_PIN`、`GIMBAL_PITCH_MOTOR_ADDRESS`、
`GIMBAL_YAW_MOTOR_ADDRESS`、`GIMBAL_DEFAULT_MICROSTEP`、
`GIMBAL_DEFAULT_SPEED_RPM`、`GIMBAL_DEFAULT_ACCELERATION`、
`GIMBAL_MAX_COMMAND_STEP_DEG`、`GIMBAL_MAX_OUTPUT_SPEED_DPS`、
`GIMBAL_USE_PRECALIB_PITCH`、`GIMBAL_PITCH_RATIO`、`GIMBAL_YAW_RATIO`、
`GIMBAL_PITCH_BACK_ANGLE`、`GIMBAL_PITCH_ENC_LIMIT`、
`GIMBAL_PITCH_ENC_HORIZONTAL` 和 `GIMBAL_YAW_ENC_CENTER`。当前默认电机地址
pitch=1/yaw=2、microstep=16、speed=300 RPM、acceleration=50、单次输出轴
命令最大 3 degree、最大输出速度 120 degree/s、pitch ratio=4、yaw ratio=8。
预标定值必须和实际机械结构一致。

hardware-test profile 在 `hardware_test_init()` 中会把请求 speed 配置为 1200
RPM、acceleration=255、两轴 EMM command timeout=5 ms、poll attempts=2，并把
yaw 软限位设为 +/-90 degree。1200 RPM 是配置上限，不是控制器一定发出的实际
速度；位置/速度控制仍受 `GIMBAL_MAX_OUTPUT_SPEED_DPS=120` 限制，按当前齿轮比
换算约为 pitch 80 RPM、yaw 160 RPM 的电机命令上限。
`GIMBAL_USE_PRECALIB_PITCH` 默认 1，相关 encoder 常量具有机械装置特异性。
必须在低速台架上核对实际电机、负载、齿轮比和限位。

公开校准类型：

- `GimbalGearedCalib`：encoder zero/max、输出轴 max、gear ratio、calibrated。
- `GimbalCalibConfig`：探索速度/加速度/次数、检测间隔和 timeout。
- `GimbalAxisCalib`：min/max/range/mid/calibrated。

`Gimbal` 保存 yaw/pitch 两个 `EmmDevice`、实际/命令角、encoder zero、软限位、
运动参数、校准数据以及 homed/position/feedback/safety/manual 状态。全局实例为：

```c
extern Gimbal g_gimbal;
```

### 14.2 初始化、安全和运动

```c
GimbalStatus gimbal_init(Gimbal *gimbal);
GimbalStatus gimbal_enable(Gimbal *gimbal, bool enable);
GimbalStatus gimbal_stop(Gimbal *gimbal);
GimbalStatus gimbal_zero_position(Gimbal *gimbal);
GimbalStatus gimbal_accept_known_reference(Gimbal *gimbal,
    float known_yaw_deg, float known_pitch_deg);
GimbalStatus gimbal_clear_safety_fault(Gimbal *gimbal);
void gimbal_latch_safety_fault(Gimbal *gimbal);
GimbalStatus gimbal_move_relative(Gimbal *gimbal,
    float yaw_delta_deg, float pitch_delta_deg);
GimbalStatus gimbal_move_to(Gimbal *gimbal, float yaw_deg, float pitch_deg);
void gimbal_debug_probe_emm_uart(void);
```

- `init()`：初始化 transport、电机对象、microstep 和默认值，但不会假装已 homed。
- `enable(true)`：safety latch 时拒绝；使能后读回验证，失败则停机、disable、锁 fault。
- `stop()`：验证两轴停止；失败锁 fault 并 disable。
- `zero_position()`：停止、把两轴电机当前位置设零并建立 0 degree 参考。
- `accept_known_reference()`：仅当给定有限且在软限位内时，读取单圈 encoder 并
  建立软件零点，随后标记 homed/position/feedback valid。
- `clear_safety_fault()`：只有验证停止并 disable 后才清锁存。
- `move_relative()/move_to()`：要求已 homed、位置有效、无 safety、非 manual；
  先读反馈，钳位软限位和每次 +/-3 degree，再按齿轮比转换。
- `debug_probe_emm_uart()`：阻塞诊断发送和约 500 ms 接收 dump，只用于维护。

调用 `gimbal_zero_position()` 前，机械结构必须已经固定在经过确认的物理零位；该
函数只是把“当前姿态”声明为零，不会寻找真实零点。在任意姿态调用会建立错误 home
和错误软限位。

EMM 0x31 encoder 是单圈值。软件只会选择最接近当前跟踪目标的圈，不能替代上电
回零；断电、丢步、stall 或手动移动后，安全上必须重新建立可信参考。注意当前
代码不会在 enter/exit manual mode 时强制作废 `homed/position_valid`，调用者不能
把 `gimbal_exit_manual_mode()` 成功误认为参考已重新校准。

### 14.3 位置、校准、限位和手动模式

```c
GimbalStatus gimbal_auto_calibrate(Gimbal *gimbal);
GimbalStatus gimbal_calibrate_geared(Gimbal *gimbal);
GimbalStatus gimbal_calibrate_axis(Gimbal *gimbal, EmmDevice *motor,
    const char *axis_name, GimbalAxisCalib *result);
GimbalStatus gimbal_read_actual_position(Gimbal *gimbal,
    float *yaw_deg, float *pitch_deg);
void gimbal_set_limits_from_calib(Gimbal *gimbal);
GimbalStatus gimbal_enter_manual_mode(Gimbal *gimbal);
GimbalStatus gimbal_exit_manual_mode(Gimbal *gimbal);
```

- 校准是会真实移动机械并碰撞/探索限位的长时间阻塞维护操作。
- 阻塞校准期间 `hardware_test_run()` 无法轮询 KEY3，因此 KEY3 不能作为该阶段的
  可靠急停。只能在清空机械运动范围、低风险台架和独立断电/物理急停可用时校准。
- `calibrate_axis()` 的 explore attempts 必须为 1-5。
- `read_actual_position()` 要求 homed/valid，读取单圈 encoder 并做近邻 unwrap。
- `enter_manual_mode()` 停止并 disable，使反馈失效。
- `exit_manual_mode()` 的代码只要求旧 `position_valid` 和无 safety fault，随后
  重新 enable 并用旧零点同步；它不会验证手动移动后参考仍可信。
- 关闭 `GIMBAL_ENABLE_CALIBRATION` 后，校准状态 API 仍可链接，但返回
  `GIMBAL_ERROR_CALIB`，设置校准限位成为 no-op。

### 14.4 逐飞 UART transport `gimbal/gimbal_transport_zf.h`

```c
GimbalStatus gimbal_transport_zf_init(Gimbal *gimbal);
```

它初始化 UART2 半双工 transport、RX ISR ring、两轴地址、80 ms timeout 和
echo-aware forced read。发送时会在复用输出与浮空输入间切换 TX pin。

## 15. 中断 ABI 和兼容 API

`user/src/isr.c` 中的 `TIMA0/1_IRQHandler`、`TIMG0/6/7/8/12_IRQHandler`、
`UART0/1/2/3_IRQHandler` 和 `GROUP1_IRQHandler` 是启动向量调用的 ABI，
不是普通业务 API。不要从应用直接调用它们。

GPIOA/GPIOB 共用 GROUP1；实现分别读取各端口 IIDX，并优先服务包含 KEY1 急停的
GPIOB。UART ISR 有有界 drain 循环，回调仍应快速。

`zf_ccs_compat.h` 只声明：

```c
void DL_Timer_Count_CCP(GPTIMER_Regs *gptimer);
```

实现位于 `SeekFree/dl_timer.c`，用于补足旧逐飞库与 MSPM0 SDK 2.10 的 CCP
计数 API 差异。普通业务不应直接依赖它。

## 16. 常见扩展方式

### 16.1 新增一个周期任务

1. 写 scheduler 签名的非阻塞函数。
2. 在对应 profile 的 `ec_app_init()` 分支注册。
3. 检查总任务数不超过 12。
4. 明确周期、首次 start time 和最坏执行时间。
5. 通过 `run_count/missed_deadlines` 或日志验证是否超期。

### 16.2 新增一个菜单模式

1. 实现 `start/run/stop`，必要时实现一次性 `init`。
2. 创建 `ec_mode_t` 并 `ec_mode_manager_add()`。
3. run 使用状态机，不调用长 delay。
4. stop 无条件关闭本模式涉及的执行器。
5. 如果是独立比赛题，参考 `h2024_tasks`，通过 port callback 隔离底盘细节。

### 16.3 新增一个 UART 设备

1. 先检查 `pin_mapping.h` 和 profile 引脚冲突。
2. 用 `SerialRxBuffer` 分配静态 ring。
3. ISR callback 只 drain FIFO 并 push。
4. 主循环 task 解析完整协议帧。
5. 统计 overflow、checksum、timeout 和 frame count。
6. 避免覆盖同一 UART 已注册 callback；共享总线时必须设计唯一仲裁者。

### 16.4 新增一个应用 profile

1. 在 `config.h` 增加 profile ID 和范围检查。
2. 在 `.cproject` 或独立 build configuration 定义编译宏。
3. 在 `ec_app.c` 增加 init、task 注册和 emergency-stop 分支。
4. 为该 profile 单独列出所有引脚，确认不存在复用冲突。
5. 不要初始化 profile 不需要的硬件。

## 17. 安全使用顺序

### 17.1 Line-car 首次上电

1. 电机断电或架空车轮。
2. 烧录 line-car image，UART1 解析器应显示 `启动 ... profile=2 ... MPU=开`。
3. 上电后保持 MPU6050 静止，直到解析器输出 `MPU初始化`（当前约 7 秒）。
4. 先进入 `GYRO TEST`，确认 READY、样本增长、错误不增长。
5. 手转车轮验证左右 encoder 方向、CPR 和 QERR。
6. 以低 PWM 单独验证左右电机方向。
7. 验证电池换算电压和欠压停机。
8. 调整开环 feedforward、line PD 和 heading PD；H1-H4 不使用 speed PID。
9. 最后在真实赛道测试 H1-H4 timeout 和急停。

### 17.2 Gimbal 首次上电

1. 保持默认 safety latch，不自动接受未知参考。
2. 低速验证两个 EMM 地址、方向、microstep 和 gearbox ratio。
3. 使用 verified stop 验证停止和读回链路。
4. 通过机械回零、已知参考或受控校准建立 homed 状态。
5. 先测试 +/-5 degree 命令和软限位。
6. 再验证 MaixCam frame、CRC、语义和 timeout。
7. 最后 rearm 并启用视觉控制。

## 18. 当前实现的重要限制

- 无 RTOS；任何阻塞任务都会拖慢全部控制与服务。
- 无独立硬件 watchdog。line-car 有 100 ms 电机命令 timeout，并在 1 ms ISR
  直接关 PWM，但它不能替代 MCU 硬件 watchdog。
- 软件 GPIO encoder 可能丢边，不是安全级绝对里程源。
- MPU6050 无磁力计，yaw 是相对积分值，会随时间和温度漂移。
- EMM 0x31 encoder 是单圈反馈，不能在断电后自动知道多圈机械位置。
- OLED 脏页写入仍是同步软件 I2C，应保持为低优先级任务。
- `system_delay_ms()` 会重配/占用 SysTick，只适合启动和维护，不适合周期控制。
- VOFA 当前只能发送 line-car 准备数据的前 15 路。
- `line_car_run()` 和 `ec_app_emergency_stop()` 当前没有内部常规调用者，保留为
  聚合/外部入口。
- `dt_imu_mahony`、HC-05、部分校准 API 当前已编译但不在默认 line-car 活跃路径。

## 19. 相关专项文档

- [README.md](README.md)：工程入口和产物速览。
- [zf_api_reference.md](zf_api_reference.md)：逐飞底层库速查。
- [UI_MENU.md](UI_MENU.md)：按键、菜单和调参页面。
- [COMPETITION_TASKS.md](COMPETITION_TASKS.md)：H1-H4 路线和编码器/航向约束。
- [MOTOR_TEST_TUNING.md](MOTOR_TEST_TUNING.md)：电机与速度环调试。
- [MPU6050_USAGE_ZH.md](MPU6050_USAGE_ZH.md)：MPU6050 接线、测试和排障。
- [DRIVER_AUDIT.md](DRIVER_AUDIT.md)：驱动问题、硬件冲突和剩余限制。
- [CALIB_DATA.md](CALIB_DATA.md)：校准数据记录。
