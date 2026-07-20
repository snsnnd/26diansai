# MPU6050 完整使用、测试与故障排查方案

## 1. 当前工程中的正确定位

本工程通过 `driver/dt_heading` 统一封装两个航向传感器：

| 固件 | 惯性传感器 | 接口 | 用途 |
| --- | --- | --- | --- |
| line-car 默认 | 外置 M0 单轴陀螺仪 | UART0，TX PA10、RX PA11 | H1-H4 航向控制 |
| line-car 可选 | MPU6050 | 软件 I2C，SCL PA1、SDA PA0 | H1-H4 航向控制和六轴诊断 |
| hardware-test | 外置 M0 单轴陀螺仪 | UART0，TX PA10、RX PA11 | 云台硬件测试 |

在 `user/inc/config.h` 中选择后端：

```c
#define CAR_GYRO_SOURCE CAR_GYRO_SOURCE_M0
/* 或 CAR_GYRO_SOURCE_MPU6050 */
```

修改后必须重新 clean build。`hardware-test` 不受该开关影响，始终测试 M0。

旧版 line-car 只保留 `yaw` 和 Z 轴角速度，底层读到的
`ax/ay/az/gx/gy/gz` 是局部变量，OLED 没有显示六轴值。因此在别处看到
XYZ 为 0，不能据此判断 MPU6050 没有工作。当前版本已经保存最近一次六轴
样本。当前统一的安全测试任务名为 `GYRO TEST`。

## 2. 接线

| MPU6050 模块引脚 | MSPM0G3507 | 说明 |
| --- | --- | --- |
| VCC | 3.3 V | 推荐使用 3.3 V |
| GND | GND | 必须共地 |
| SCL | PA1 | `MPU6050_SCL` |
| SDA | PA0 | `MPU6050_SDA` |
| AD0 | GND 或 3.3 V | GND 地址 0x68，3.3 V 地址 0x69；代码自动探测 |
| INT | 不接 | 当前代码不使用中断脚 |
| XDA/XCL | 不接 | 当前代码不使用辅助 I2C |

注意事项：

1. MCU GPIO 是 3.3 V 逻辑，不要把 SCL/SDA 上拉到 5 V。
2. 常见 GY-521 模块自带 I2C 上拉。若裸模块没有上拉，在 SCL、SDA 上分别接
   4.7 kΩ 到 3.3 V。
3. 不要把 MPU6050 接到 OLED 的 PB9/PB8，也不要接到 T8 的 PB2/PB3。
4. 接线或修改 AD0 后必须复位。地址探测和零偏校准只在启动时执行。
5. 电机地、传感器地和 MCU 地必须连接，但排障阶段应断开电机驱动电源或架空车轮。

## 3. 构建和烧录

在工程目录运行：

```bat
build_profiles.bat
```

脚本依次生成：

```text
BuildProfiles/MSPM0G3507_ZF_line_car.out
BuildProfiles/MSPM0G3507_ZF_hardware_test.out
```

脚本结束时会恢复默认 `Debug/MSPM0G3507_ZF.out` 为 line-car 固件。

也可以在 CCS 中直接构建 `Debug`。`.cproject` 的默认宏必须是：

```text
EC_APP_PROFILE=2
```

烧录后，`tmp/debug_parser.py` 应首先解析出：

```text
启动 ... profile=2 ... GYRO=M0/MPU
```

如果 `profile` 不是 `2`，说明烧错了固件。

## 4. 调试串口

| 项目 | 配置 |
| --- | --- |
| UART | UART2（蓝牙透明串口） |
| MCU TX | PB15，接蓝牙 RX |
| MCU RX | PB16，接蓝牙 TX；只看日志时可不接 |
| 波特率 | 115200 |
| 数据格式 | 8N1 |
| 电平 | 3.3 V TTL |

蓝牙模块必须与 MCU 共地。不要使用 RS-232 电平，也不要把 5 V TTL 直接接入
MCU RX。选择 M0 时 line-car 使用 UART0 PA10/PA11；选择 MPU6050 时使用
PA1/PA0 软件 I2C。

## 5. 第一次安全测试

1. 断开电机驱动电源，或者将车轮完全架空。
2. 正确连接 MPU6050、OLED 和调试串口。
3. 烧录 line-car 固件。
4. 按复位后保持车体完全静止，直到解析器输出 `MPU初始化`；当前软件 I2C
   配置通常需要约 7 秒。
5. 启动默认选中 `2024 H1`。按一次 KEY1，选择前一个任务 `GYRO TEST`。
6. 按 KEY3 启动 `GYRO TEST`。
7. 先静置观察，再分别绕 X、Y、Z 轴缓慢旋转模块。
8. 再沿 X、Y、Z 方向短促移动，观察加速度变化。
9. 按 KEY3 退出。`GYRO TEST` 始终调用停车函数，不会输出电机驱动。

外接按键接法：

| 按键 | MCU 引脚 | 动作 |
| --- | --- | --- |
| KEY1 | PB6 短接 GND | 上一个任务；运行时急停 |
| KEY2 | PB7 短接 GND | 下一个任务 |
| KEY3 | PB23 短接 GND | 启动或停止 |

开发板自带 PB21 `KEY` 不属于这三个菜单按键。

## 6. GYRO TEST OLED 含义

典型页面：

```text
HEADING SAFE TEST
SRC:MPU ST:READY
Y:    0 WZ:    1
N: 1234 E:    0
BUS:0 CHK:0
A:    5   -12   998
ID:68 GZ:    1
K3 EXIT MOTORS OFF
```

| 字段 | 含义 | 缩放 |
| --- | --- | --- |
| `SRC` | 当前后端 | M0/MPU |
| `ST` | 初始化状态 | INIT/WAIT/READY/BUS/ID/CFG/CAL |
| `A:x y z` | X/Y/Z 加速度 | mg，1000 表示 1 g |
| `Y` | 积分航向 | 0.01° |
| `WZ` | 去零偏后的 Z 角速度 | 0.01 °/s |
| `N` | 运行期成功样本数 | 应持续增加 |
| `E` | 读失败次数 | 正常应保持不变 |
| `BUS` | 最近一次软件 I2C 状态 | 0 表示成功 |
| `CHK` | 校验错误数 | MPU 后端应为 0 |
| `ID` | WHO_AM_I | 通常为 68 或 69 |
| `GZ` | MPU6050 原始 Z 角速度 | 0.01 °/s |

静止平放时，某一个加速度轴通常约为 `+1000` 或 `-1000` mg，另外两轴接近
0。哪个轴接近 1 g 取决于模块安装方向。

平移并不一定让陀螺仪 XYZ 改变。陀螺仪只测旋转速度；匀速直线移动时
`gx/gy/gz` 接近 0 是正常现象。测试陀螺仪必须绕对应轴旋转模块。

## 7. 串口诊断输出

进入 `GYRO TEST` 后，VOFA 输出会临时暂停，每 500 ms 发送一帧航向诊断数据：

```text
[CAR][MPU6050] st=READY addr=0x68 who=0x68 i2c=0 scl=1 sda=1 n=1200 err=0 A_mg=(3,-8,1001) G_mdps=(15,-23,40) T_centiC=2865 yaw_cdeg=12
```

| 字段 | 单位 |
| --- | --- |
| `A_mg` | mg |
| `G_mdps` | 0.001 °/s |
| `T_centiC` | 0.01 °C |
| `yaw_cdeg` | 0.01° |

启动成功时还会输出地址、WHO_AM_I、Z 轴零偏、有效校准样本数、方差以及
正式校准窗口内的 Z 轴角速度范围：

```text
[CAR][MPU6050] st=READY addr=0x68 who=0x68 bias_z_x100=... cal_n=1000 var_x1000=...
```

## 8. 状态码解释

MPU 状态：

| `ST` | 含义 | 处理 |
| --- | --- | --- |
| `READY` | 初始化、配置和零偏校准成功 | 可以继续方向测试 |
| `BUS` | 0x68 和 0x69 都没有得到有效响应 | 查电源、共地、SCL/SDA、上拉 |
| `ID` | 总线上有器件响应，但 WHO_AM_I 不是 MPU6050 | 查型号、地址冲突、假芯片或接错器件 |
| `CFG` | 找到 MPU6050，但复位或寄存器配置失败 | 查供电稳定性、I2C 波形和模块质量 |
| `CAL` | 能读取数据，但启动零偏方差过大或有效样本不足 | 复位并保持车体静止，检查振动和接触不良 |
| `INIT` | 尚未初始化 | 检查是否运行正确固件 |

软件 I2C 数字状态：

| `I2C` | 含义 |
| --- | --- |
| 0 | 正常 |
| 1 | NACK，无器件应答 |
| 2 | 参数或地址无效 |
| 3 | SCL 时钟延展超时，常见于 SCL 被拉低 |
| 4 | 总线卡死，SCL 或 SDA 无法释放为高电平 |

## 9. XYZ 全为 0 的逐项判断

| 现象 | 最可能原因 | 检查方法 |
| --- | --- | --- |
| 没有 `[APP] profile=line_car` | 烧错固件 | 烧录 line-car `.out` |
| `ST:BUS`、`I2C:1`、`C:1 D:1` | 地址无应答 | 查 VCC/GND、SCL/SDA 是否接反 |
| `ST:BUS`、`C:0` | SCL 被短路或没有释放 | 断电测阻值，检查焊桥和引脚映射 |
| `ST:BUS`、`D:0` | SDA 被短路或从设备卡死 | 断电重接，检查上拉和模块 |
| `ST:ID` | 器件不是 MPU6050 | 读取串口 `who=0x..`，核对芯片型号 |
| `ST:CAL` | 上电时移动或振动 | 静置后复位，保持不动直到出现 `MPU初始化`（约 7 秒） |
| `ST:READY`、`N` 增加、A 全 0 | 数据异常 | 检查是否是真 MPU6050，抓取串口完整一行 |
| `ST:READY`、G 接近 0 | 只做了平移，没有旋转 | 绕各轴转动模块 |
| `ST:READY`、A 一轴约 1000 | 加速度计正常 | 继续旋转测试陀螺仪 |
| `N` 不增、`E` 增加 | 运行中掉线 | 查接插件、电机干扰、供电去耦和线长 |

建议排障顺序：

1. 万用表确认 MPU VCC 对 GND 为 3.3 V。
2. 断电确认 MCU 与 MPU 共地。
3. 通电空闲时测 SCL、SDA，二者都应接近 3.3 V。
4. 进入 `GYRO TEST`，先看 `SRC`、`ST`、`BUS`，不要先看 XYZ。
5. 确认 `ID` 为 `68` 或 `69`，`N` 持续增加且 `E` 不增加。
6. 用逻辑分析仪检查地址帧应为 0x68 或 0x69 的 7 位地址，不要把 0xD0
   当成配置地址；0xD0 只是 0x68 左移后的写地址字节。
7. 电机启动后才掉线时，在 MPU 模块 VCC/GND 附近增加 0.1 µF 和 10 µF 去耦，
   缩短 I2C 线并让传感器线远离电机线。

## 10. 当前驱动代码结构

| 文件 | 作用 |
| --- | --- |
| `user/src/driver/dt_mpu6050.c` | 寄存器初始化和 14 字节六轴读取 |
| `user/inc/driver/dt_mpu6050.h` | MPU6050 寄存器、量程和原始数据接口 |
| `user/src/driver/dt_mpu6050_heading.c` | 地址探测、Z 零偏、100 Hz 航向积分 |
| `user/inc/driver/dt_mpu6050_heading.h` | 航向状态、最近六轴样本和诊断字段 |
| `user/src/driver/dt_heading.c` | M0/MPU6050 统一初始化、更新和诊断接口 |
| `user/src/app/line_car.c` | 调度、GYRO TEST、OLED、串口和 H 题接入 |
| `user/inc/config.h` | PA1/PA0 引脚配置 |

初始化流程：

1. 先用标准 7 位地址 0x68 读取 `WHO_AM_I`。
2. 如果失败，再尝试 0x69。
3. 接受 MPU6050 标识 0x68/0x69 的有效身份位。
4. 复位芯片并解除睡眠。
5. 设置加速度 ±2 g、陀螺仪 ±500 °/s、DLPF 44 Hz、输出 100 Hz。
6. 等待 500 ms，再读取并丢弃 32 个启动预热样本。
7. 静止读取 1000 次 Z 轴角速度，至少要求 900 次成功。
8. 方差不得超过 0.25 `(°/s)^2`。
9. 运行期每 10 ms 读取一次，积分 `yaw += (gz-bias)*dt`。

## 11. 在其他模块中使用航向驱动

推荐使用高层接口，因为它包含地址探测、校准、错误计数和时间保护：

```c
#include "config.h"
#include "driver/dt_mpu6050_heading.h"

static dt_mpu6050_heading_t g_heading;

bool my_mpu_init(void)
{
    if (!dt_mpu6050_heading_init(&g_heading, MPU6050_SCL, MPU6050_SDA))
    {
        printf("MPU init failed: status=%u i2c=%u addr=0x%02X who=0x%02X\r\n",
            (unsigned)g_heading.status,
            (unsigned)soft_iic_get_last_error(&g_heading.mpu.iic),
            (unsigned)g_heading.address,
            (unsigned)g_heading.who_am_i);
        return false;
    }

    dt_mpu6050_heading_zero(&g_heading);
    return true;
}

void my_mpu_10ms_task(uint32_t now_ms)
{
    const dt_mpu6050_data_t *data;

    if (!dt_mpu6050_heading_update(&g_heading, now_ms))
    {
        return;
    }

    data = &g_heading.last_sample;
    /* data->ax/ay/az: g; data->gx/gy/gz: degree/s; data->temp: degree C. */
    /* g_heading.yaw_deg: integrated relative yaw; g_heading.wz_dps: corrected Z rate. */
}
```

调用要求：

1. `my_mpu_init()` 只能在系统时基和 GPIO 可用后调用。
2. 初始化期间必须保持静止。
3. `my_mpu_10ms_task()` 每 10 ms 调用一次，不要在中断中执行软件 I2C。
4. 每次必须检查布尔返回值，不能在读取失败后继续使用未更新数据。
5. 可用 `sample_count` 判断成功数据是否持续更新，用 `read_error_count` 判断掉线。

## 12. 只读取六轴，不需要航向积分

若独立程序只需要六轴数据，可以使用底层接口：

```c
#include "driver/dt_mpu6050.h"

static dt_mpu6050_config_t g_mpu;

bool raw_mpu_init(void)
{
    soft_iic_init(&g_mpu.iic, 0x68u, 100u, A11, A10);
    g_mpu.accel_fs = DT_MPU6050_ACCEL_FS_2G;
    g_mpu.gyro_fs = DT_MPU6050_GYRO_FS_500;
    return dt_mpu6050_init(&g_mpu) != 0u;
}

bool raw_mpu_read(dt_mpu6050_data_t *data)
{
    return dt_mpu6050_read_all(&g_mpu, data) != 0u;
}
```

如果 AD0 接高，应把示例地址改成 `0x69u`。工程的航向高层接口已经自动探测，
通常不需要手动使用这个低层示例。

## 13. H1-H4 中的使用方式

H 题不使用加速度 XYZ 计算绝对航向，只使用去零偏后的 `gz` 积分：

```text
wz = gz - gyro_bias_z
yaw = wrap(yaw + wz * dt)
```

原因是 MPU6050 没有磁力计。加速度计可以约束横滚和俯仰，但不能从重力方向
推算平面内的绝对偏航角。因此：

1. H 任务启动瞬间的当前方向作为相对零方向。
2. 直线段使用 `yaw` 和 `wz` 做航向 PD。
3. 黑线圆弧段使用 T8 循迹。
4. 超过 250 ms 没有成功 MPU 样本时进入 `GYRO FAULT` 并停车。
5. 每次温度明显变化或重新摆放车辆后应复位并重新校准。

方向符号校准：

1. 电机断电或架空车轮。
2. 进入 `GYRO TEST`，手动让车体向右转。
3. 确认原始 Z 角速度和 `Y` 有明显变化。
4. 进入 `TUNING` 调整 `HEADING SIGN`。
5. 目标是物理右转时路线航向增加；若相反，将 `HEADING SIGN` 在 +1/-1 间切换。

## 14. 不能从 MPU6050 获得的量

MPU6050 能直接提供三轴加速度、三轴角速度和温度，不能直接提供：

1. 绝对指南针方向。
2. 不随时间漂移的 yaw。
3. 仅靠一次采样得到的位移或速度。
4. 静止状态下可靠的平面朝向。

若比赛要求长时间绝对航向，需要磁力计、视觉、编码器/赛道事件或其他外部参考。
