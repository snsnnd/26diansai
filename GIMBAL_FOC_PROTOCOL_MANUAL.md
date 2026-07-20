# STM32F103 2804 双轴无刷云台 FOC 控制 — 调用协议手册

> **项目**: 2804_FOC_HAL_DUAL  
> **MCU**: STM32F103C8T6 @ 72MHz (HSE 8MHz × PLL9)  
> **电机**: 2× 2804 无刷云台电机 (7极对)  
> **驱动**: 2× SimpleFOCmini (12V)  
> **编码器**: 2× AS5600 磁编码器 (I2C)  
> **生成日期**: 2026-07-20  

---

## 目录

1. [系统架构总览](#1-系统架构总览)
2. [二进制协议 (重点)](#2-二进制协议-重点)
   - [2.1 帧格式](#21-帧格式)
   - [2.2 CRC8 算法](#22-crc8-算法)
   - [2.3 应答规则](#23-应答规则)
   - [2.4 命令全表](#24-命令全表)
   - [2.5 每条命令详细说明](#25-每条命令详细说明)
3. [ASCII 文本命令](#3-ascii-文本命令)
   - [3.1 电机角度控制](#31-电机角度控制)
   - [3.2 PID 在线调参](#32-pid-在线调参)
   - [3.3 模式控制](#33-模式控制)
   - [3.4 VOFA / 测试 / 调试](#34-vofa--测试--调试)
4. [VOFA JustFloat 遥测](#4-vofa-justfloat-遥测)
5. [S-Curve 轨迹规划器](#5-s-curve-轨迹规划器)
6. [云台管理层 API (gimbal_*)](#6-云台管理层-api-gimbal_)
7. [SimpleFOC 电机层 API](#7-simplefoc-电机层-api)
8. [错误码参考](#8-错误码参考)
9. [MSPM0G3507 侧对接指南](#9-mspm0g3507-侧对接指南)

---

## 1. 系统架构总览

```
┌──────────────────────────────────────────────────────────────────┐
│                    UART1 (115200 8N1)                            │
│  ← 命令 (ASCII / Binary)                                        │
│  → 应答 (Binary) 或 遥测 (VOFA JustFloat)                       │
└──────────────────────┬───────────────────────────────────────────┘
                       │
┌──────────────────────┴───────────────────────────────────────────┐
│  commander_run() — 收发复用器                                    │
│  ├── protocol_feed() — 二进制帧接收状态机                        │
│  ├── protocol_process() — 命令分发                               │
│  └── ascii_process() — 文本命令解析                              │
└──────────────────────┬───────────────────────────────────────────┘
                       │
┌──────────────────────┴───────────────────────────────────────────┐
│  云台管理层 (gimbal_* functions)                                 │
│  ├── gimbal_set_angle_target() — 绝对角度目标 (度)               │
│  ├── gimbal_set_dual_angle() — 双轴同步目标                      │
│  ├── gimbal_hold() — 当前位置保持                                │
│  ├── gimbal_set_run_mode() — 跟踪/转向/自动 模式                 │
│  ├── gimbal_set_profile() — 轨迹参数配置                         │
│  ├── gimbal_set_feedforward() — 前馈增益                         │
│  └── gimbal_get_status() — 完整状态快照                          │
└──────────────────────┬───────────────────────────────────────────┘
                       │
┌──────────────────────┴───────────────────────────────────────────┐
│  S-Curve 轨迹规划器 (traj_update / Trajectory)                   │
│  输入: target(rad)  │  输出: pos_sp, vel_sp, acc                 │
│  参数: response_wn, max_vel, max_acc, max_jerk                   │
└──────────────────────┬───────────────────────────────────────────┘
                       │
┌──────────────────────┴───────────────────────────────────────────┐
│  级联 PID + 前馈                                                 │
│  pos_sp → [位置P] → vel_sp → [速度PI] → voltage_feedback         │
│  vel_sp × Kv ──────────────────────────→ voltage_feedforward     │
│  acc × Ka ─────────────────────────────→                        │
│  voltage.q = feedback + Kv×vel + Ka×acc                          │
└──────────────────────┬───────────────────────────────────────────┘
                       │
┌──────────────────────┴───────────────────────────────────────────┐
│  SimpleFOC 底层                                                  │
│  ├── FOC 电流环 (SVPWM, Clark/Park)                              │
│  ├── AS5600 磁编码器 (I2C, 12bit)                                │
│  └── TIM2/TIM3 6路PWM (M0:PA0/PA1/PA2, M1:PA6/PA7/PB0)         │
└──────────────────────────────────────────────────────────────────┘
```

**关键参数**:
| 参数 | 默认值 | 说明 |
|------|--------|------|
| 控制周期 | 2000 μs (500Hz) | `CONTROL_PERIOD_US` |
| 电源电压 | 12V | `voltage_power_supply` |
| 电压上限 | 5V | `voltage_limit` |
| 速度上限 | 5 rad/s (~286°/s) | `velocity_limit` |
| 软限位 | ±30° | `GIMBAL_SOFT_LIMIT_DEG` |

---

## 2. 二进制协议 (重点)

### 2.1 帧格式

```
┌──────┬──────┬──────┬─────────────────┬──────┐
│ SYNC │ CMD  │ LEN  │ DATA (0~60字节) │ CRC8 │
│ 0xAA │ 1B   │ 1B   │ LEN bytes       │ 1B   │
└──────┴──────┴──────┴─────────────────┴──────┘
总长 = 4 + LEN 字节
```

- **SYNC**: 固定 `0xAA`
- **CMD**: 命令码, 应答时错误帧用 `CMD | 0x80`
- **LEN**: DATA 长度 (0~60), `PROTO_MAX_DATA = 60`
- **DATA**: 多字节整数/浮点数一律**小端序 (LE)**
- **CRC8**: 覆盖 CMD + LEN + DATA, 多项式 `0x07`

### 2.2 CRC8 算法

```c
// 多项式: x^8 + x^2 + x + 1  (0x07)
// 覆盖范围: CMD[1] + LEN[1] + DATA[LEN]

static uint8_t crc8_compute(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) crc = crc8_table[crc ^ *data++];
    return crc;
}
```

CRC8 查表 (poly 0x07):
```
00,07,0E,09,1C,1B,12,15,38,3F,36,31,24,23,2A,2D,
70,77,7E,79,6C,6B,62,65,48,4F,46,41,54,53,5A,5D,
E0,E7,EE,E9,FC,FB,F2,F5,D8,DF,D6,D1,C4,C3,CA,CD,
...
```

### 2.3 应答规则

1. **每条命令必须回复**（除 `CMD_SESSION=0` 退出会话为单向）
2. **成功**: 回复 CMD + 数据
3. **失败**: 回复 `CMD | 0x80` + 1字节错误码
4. **TX 忙时** (DMA 发送中): 最多缓存 1 个待处理包 (`pending_packet`)
5. **帧间超时**: 100ms 未收到完整帧则状态机复位

**错误响应格式**:
```
SYNC(0xAA) | CMD|0x80 | LEN=1 | ERR_CODE | CRC8
```

### 2.4 命令全表

| 命令码 | 名称 | 请求长度 | 应答长度 | 功能 |
|--------|------|----------|----------|------|
| `0x01` | PING | 0 | 1 | 心跳, 返回在线电机数 |
| `0x02` | SET_TARGET | 5 | 5 | 设置目标 (角度° / 速度 rad/s / 电压 V) |
| `0x03` | GET_ANGLE | 1 | 9 | 读当前角度+速度 |
| `0x04` | SET_MODE | 2 | 2 | 切换控制模式 |
| `0x05` | SET_PID | 13 | 13 | 设置位置环 PID |
| `0x06` | SET_VEL_PID | 13 | 13 | 设置速度环 PID |
| `0x07` | SET_VOLTAGE | 5 | 5 | 力矩模式: 直接电压 |
| `0x08` | SET_LIMITS | 9 | 9 | 设置电压/速度限制 |
| `0x0F` | RESET | 0 | 0 | 软件复位 |
| `0x10` | GET_INFO | 0 | 7 | 版本 + 在线位图 + 能力位图 |
| `0x11` | SET_RUN_MODE | 2 | 3 | 运行模式 (跟踪/转向/自动) |
| `0x12` | SET_ANGLE | 5 | 5 | 设置 home-relative 角度 (度) |
| `0x13` | GET_STATUS | 1 | 28 | 单个电机完整状态 |
| `0x14` | GET_ALL_STATUS | 0 | 56 | 双电机原子状态快照 |
| `0x15` | SET_DUAL_ANGLE | 8 | 8 | 双轴原子角度设置 |
| `0x16` | HOLD | 1 | 1 | 保持当前位置 |
| `0x17` | SET_AUTO_THRESH | 9 | 9 | 自动模式进出阈值 |
| `0x18` | GET_AUTO_THRESH | 1 | 9 | 查询自动模式阈值 |
| `0x19` | SET_FEEDFORWARD | 9 | 9 | 速度+加速度前馈增益 |
| `0x1A` | GET_FEEDFORWARD | 1 | 9 | 查询前馈增益 |
| `0x1B` | SESSION | 1 | 1 | 进入/退出二进制会话 |
| `0x1C` | SET_PROFILE | 18 | 18 | 轨迹参数配置 |
| `0x1D` | GET_PROFILE | 2 | 18 | 查询轨迹参数 |

---

### 2.5 每条命令详细说明

#### CMD_PING (0x01) — 心跳

```
请求: AA 01 00 CRC
应答: AA 01 01 [count] CRC
```
- `count`: 在线电机数 (0/1/2)

---

#### CMD_SET_TARGET (0x02) — 通用目标设置

```
请求: AA 02 05 [motor_id:1B] [target:float_LE 4B] CRC
应答: AA 02 05 [motor_id] [target_echo] CRC  (成功)
       AA 82 01 [err] CRC                      (失败)
```
- `motor_id`: 0=M0, 1=M1
- `target` 含义取决于当前控制模式:
  - **角度模式 (Type_angle)**: 目标角度 (°), 相对上电零位, 范围 ±30°
  - **速度模式**: 目标速度 (rad/s)
  - **力矩模式**: 目标电压 (V)

---

#### CMD_GET_ANGLE (0x03) — 读角度

```
请求: AA 03 01 [motor_id] CRC
应答: AA 03 09 [motor_id] [angle_deg:float_LE] [velocity_dps:float_LE] CRC
```
- `angle_deg`: 当前角度 (°), 相对上电零位
- `velocity_dps`: 当前角速度 (°/s)

---

#### CMD_SET_MODE (0x04) — 切换控制模式

```
请求: AA 04 02 [motor_id] [mode] CRC
```
- `mode`:
  - `0` = Torque (力矩/电压)
  - `1` = Velocity (速度闭环)
  - `2` = Angle (角度位置闭环)
  - `3` = Velocity Openloop (开环速度)
  - `4` = Angle Openloop (开环角度)

---

#### CMD_SET_PID (0x05) — 位置环 PID

```
请求: AA 05 0D [motor_id] [P:float_LE] [I:float_LE] [D:float_LE] CRC
```
- 范围: 0.0 ~ 1000.0
- 设置后自动 `PID_reset()`, 中断自动测试和压力测试

---

#### CMD_SET_VEL_PID (0x06) — 速度环 PID

```
请求: AA 06 0D [motor_id] [P:float_LE] [I:float_LE] [D:float_LE] CRC
```
- 范围: 0.0 ~ 1000.0

---

#### CMD_SET_VOLTAGE (0x07) — 直接电压

```
请求: AA 07 05 [motor_id] [voltage:float_LE] CRC
```
- 自动切换到 `Type_torque` 模式
- 电压范围: ±voltage_limit (默认 ±5V)

---

#### CMD_SET_LIMITS (0x08) — 电压/速度限制

```
请求: AA 08 09 [motor_id] [voltage_limit:float_LE] [velocity_limit:float_LE] CRC
```
- `voltage_limit`: 0.1 ~ power_supply×0.577 (约 6.9V@12V)
- `velocity_limit`: 0.1 ~ 100.0 (rad/s)

---

#### CMD_RESET (0x0F) — 软件复位

```
请求: AA 0F 00 CRC
应答: AA 0F 00 CRC  (然后延时10ms, NVIC_SystemReset)
```

---

#### CMD_GET_INFO (0x10) — 设备信息

```
请求: AA 10 00 CRC
应答: AA 10 07 [ver_major] [ver_minor] [online_bitmap] [capabilities:uint32_LE] CRC
```
- `ver_major/minor`: 协议版本 (当前 2.0)
- `online_bitmap`: bit0=M0在线, bit1=M1在线
- `capabilities`: 能力位图
  - `0x01` = AUTO_MODE
  - `0x02` = DUAL_TARGET
  - `0x04` = FULL_STATUS
  - `0x08` = BINARY_SESSION
  - `0x10` = PROFILE_CONFIG
  - `0x20` = LEGACY_COMMANDS

---

#### CMD_SET_RUN_MODE (0x11) — 运行模式

```
请求: AA 11 02 [motor_id] [run_mode] CRC
应答: AA 11 03 [motor_id] [run_mode] [active_profile] CRC
```
- `run_mode`:
  - `0` = TRACK (跟踪模式, profile=0, 高精度)
  - `1` = TURN (转向模式, profile=1, 高速)
  - `2` = AUTO (自动切换: 大误差→TURN, 接近目标→TRACK)

---

#### CMD_SET_ANGLE (0x12) — 绝对角度目标

```
请求: AA 12 05 [motor_id] [angle_deg:float_LE] CRC
```
- 这是最常用的命令。角度相对上电时记录的 home 零位。
- 自动确保角度控制模式, 停止所有测试。
- 范围: ±30° (软限位 `GIMBAL_SOFT_LIMIT_DEG`)

---

#### CMD_GET_STATUS (0x13) — 单电机完整状态 (28字节)

```
请求: AA 13 01 [motor_id] CRC
应答: AA 13 1C [status:28B] CRC
```

**status 结构体 (28 bytes, 全部小端)**:
```
Offset  Size  Name             说明
 0      1     motor_id         0=M0, 1=M1
 1      1     run_mode         0=TRACK, 1=TURN, 2=AUTO
 2      1     active_profile   当前生效的轨迹编号
 3      1     flags            状态位掩码 (见下)
 4      4     target_deg       目标角度 (°, float)
 8      4     trajectory_deg   轨迹规划位置 (°, float)
12      4     angle_deg        实际角度 (°, float)
16      4     velocity_deg_s   实际角速度 (°/s, float)
20      4     error_deg        跟踪误差 (°, float)
24      4     applied_voltage  施加电压 (V, float)
```
**flags 位掩码**:
```
0x01 = STATUS_ONLINE        电机在线
0x02 = STATUS_ANGLE_CONTROL 处于角度控制模式
0x04 = STATUS_IN_POSITION   已到达目标 (误差<0.18°)
0x08 = STATUS_SENSOR_VALID  传感器数据有效
0x10 = STATUS_TURN_PROFILE  当前使用转向轨迹参数
0x20 = STATUS_TEST_ACTIVE   测试运行中
```

---

#### CMD_GET_ALL_STATUS (0x14) — 双电机原子快照 (56字节)

```
请求: AA 14 00 CRC
应答: AA 14 38 [m0_status:28B] [m1_status:28B] CRC
```
- M0 在前 28 字节, M1 在后 28 字节
- 格式与 CMD_GET_STATUS 完全相同
- 一次性获取两个电机的同步状态

---

#### CMD_SET_DUAL_ANGLE (0x15) — 双轴原子角度

```
请求: AA 15 08 [m0_deg:float_LE] [m1_deg:float_LE] CRC
```
- 同时设置两个电机角度, 避免先后命令的时间差

---

#### CMD_HOLD (0x16) — 保持当前位置

```
请求: AA 16 01 [motor_id] CRC
```
- `motor_id = 0xFF` 表示同时保持两个电机
- 功能: 将轨迹目标设为当前角度, 复位 PID, 电机停在原位

---

#### CMD_SET_AUTO_THRESH (0x17) — 自动模式阈值

```
请求: AA 17 09 [motor_id] [enter_turn_deg:float_LE] [enter_track_deg:float_LE] CRC
```
- `enter_turn_deg`: 误差超过此值 → 切换到 TURN 模式 (默认 8°)
- `enter_track_deg`: 误差低于此值 → 切换回 TRACK 模式 (默认 3°)
- 约束: enter_track ≥ 0.1°, enter_turn > enter_track

---

#### CMD_SET_FEEDFORWARD (0x19) — 前馈增益

```
请求: AA 19 09 [motor_id] [velocity_gain:float_LE] [acceleration_gain:float_LE] CRC
```
- `velocity_gain`: V/(rad/s), 范围 0~5, 默认 0.15
- `acceleration_gain`: V/(rad/s²), 范围 0~0.1, 默认 0.005
- 作用于: `voltage_feedforward = vel_sp × Kv + acc × Ka`

---

#### CMD_SESSION (0x1B) — 二进制会话

```
请求: AA 1B 01 [enter:1/0] CRC
```
- `1` = 进入纯净二进制会话 (抑制所有文本输出)
- `0` = 退出二进制会话
- TX 完成后生效

---

#### CMD_SET_PROFILE (0x1C) — 轨迹参数配置

```
请求: AA 1C 12 [motor_id] [profile_id] [response_wn:float_LE] [max_vel:float_LE] [max_acc:float_LE] [max_jerk:float_LE] CRC
```
- `profile_id`: 0=TRACK (跟踪), 1=TURN (转向)
- `response_wn`: 闭环带宽 (rad/s), 1~20, 默认 6~8
- `max_vel`: 最大速度 (rad/s), 0.1~10, 默认 3~5
- `max_acc`: 最大加速度 (rad/s²), 1~100, 默认 15~30
- `max_jerk`: 最大加加速度 (rad/s³), 10~2000, 默认 150~300

---

## 3. ASCII 文本命令

> 波特率 115200 8N1, 所有命令以 `\r` 或 `\n` 结尾。  
> **注意**: 在二进制会话期间 (`protocol_binary_session=1`) 文本命令被忽略。  
> `debug_text_enabled` 控制 printf 文本输出；`D1` 开启, `D0` 关闭 (上电默认关闭)。

### 3.1 电机角度控制

| 命令 | 效果 | 示例 |
|------|------|------|
| `M0:30` | M0 相对零位转到 30° | 范围 ±30° |
| `M1:-15` | M1 转到 -15° | 范围 ±30° |
| `M0:90` | M0 转到 90° (超限位, 仅测试用) | 范围 ±720° 校验 |

### 3.2 PID 在线调参

| 命令 | 参数 | 说明 |
|------|------|------|
| `A0:18.0` | 位置环 P (M0) | 范围 0~200 |
| `A1:18.0` | 位置环 P (M1) | |
| `P0:0.19` | 速度环 P (M0) | 范围 0~20 |
| `P1:0.19` | 速度环 P (M1) | |
| `I0:0.10` | 速度环 I (M0) | 范围 0~100 |
| `I1:0.10` | 速度环 I (M1) | |
| `F0:0.15` | 速度前馈增益 (M0) | V/(rad/s), 0~5 |
| `F1:0.15` | 速度前馈增益 (M1) | |
| `G0:0.005` | 加速度前馈增益 (M0) | V/(rad/s²), 0~1 |
| `G1:0.005` | 加速度前馈增益 (M1) | |

### 3.3 模式控制

| 命令 | 效果 |
|------|------|
| `R0:0` | M0 → TRACK 模式 |
| `R0:1` | M0 → TURN 模式 |
| `R0:2` | M0 → AUTO 模式 |
| `R1:N` | M1 同上 |

### 3.4 VOFA / 测试 / 调试

| 命令 | 效果 |
|------|------|
| `H` | 通信测试 → `OK\r\n` |
| `D1` / `D0` | 开启/关闭 ASCII 调试文本输出 |
| `V0` | 开启 M0 VOFA 波形 (16通道 JustFloat) |
| `V1` | 开启 M1 VOFA 波形 |
| `V` | 切换 VOFA 开关 (保持当前电机选择) |
| `T0` / `T1` | 切换自动往返测试 (M0/M1, ±30° 往复) |
| `S0` / `S1` | 切换压力测试 (M0/M1, 4级递增) |
| `Q0` / `Q1` | 快速测试 (复位默认PID, 开VOFA, 起自动测试) |
| `O0` / `O1` | 驱动测试 (离线电机, 扫频电压) |
| `X` | 停止一切测试 |

---

## 4. VOFA JustFloat 遥测

当 `V0`/`V1` 命令开启时, 每 ~10ms (100Hz) 发送一帧 JustFloat 数据。

**帧格式**:
```
[16×float_LE = 64B] [00 00 80 7F = 4B 帧尾] = 68 bytes total
```

**16通道定义**:

| 通道 | 名称 | 说明 |
|------|------|------|
| 0 | command_pos | 最终目标角度 (°) |
| 1 | trajectory_pos | 轨迹规划给定位置 (°) |
| 2 | actual_pos | 实际编码器角度 (°) |
| 3 | position_error | 位置跟踪误差 (°) |
| 4 | trajectory_vel | 轨迹规划速度 (°/s) |
| 5 | velocity_sp | 速度环给定 (°/s) |
| 6 | actual_vel | 实际速度 (°/s) |
| 7 | velocity_error | 速度误差 (°/s) |
| 8 | trajectory_acc | 轨迹加速度 (°/s²) |
| 9 | feedback_v | PID 反馈电压 (V) |
| 10 | feedforward_v | 前馈电压 (V) |
| 11 | applied_v | 最终 q轴电压 (V) |
| 12 | stress_status | 0=空闲, 1~4=级别, <0=中止, 10=完成 |
| 13 | stress_peak_vel | 压力测试峰值速度 (°/s) |
| 14 | stress_peak_err | 压力测试峰值误差 (°) |
| 15 | saturation_pct | 当前级电压饱和占比 (%) |

---

## 5. S-Curve 轨迹规划器

**文件**: main.c `traj_*` 函数 + trajectory.h/c

### 数据结构

```c
typedef struct {
    float pos;           // 当前位置 (rad)
    float vel;           // 当前速度 (rad/s)
    float acc;           // 当前加速度 (rad/s²)
    float target;        // 目标位置 (rad)
    float max_vel;       // 最大速度 (rad/s)
    float max_acc;       // 最大加速度 (rad/s²)
    float max_jerk;      // 最大加加速度 (rad/s³)
    float response_wn;   // 闭环带宽 (rad/s) — 临界阻尼二阶跟踪滤波器
    uint8_t online;      // 1=使能
} Trajectory;

typedef struct {
    float response_wn;
    float max_vel, max_acc, max_jerk;
} GimbalProfileConfig;
```

### API

```c
void traj_init(Trajectory *t);                                    // 初始化为默认值
void traj_reset(Trajectory *t, float pos_rad);                   // 重置到当前位置
void traj_set_target_deg(Trajectory *t, float origin_rad, float deg); // 设置角度目标
void traj_update(Trajectory *t, float dt, float *pos_out, float *vel_out); // 更新一步
```

### 算法说明

1. **二阶临界阻尼跟踪器**: `a_des = ωn² × err - 2×ωn × vel`
2. **加速度钳制**: `a_des ∈ [-max_acc, +max_acc]`
3. **Jerk 限制**: `Δa ∈ [-jerk×dt, +jerk×dt]`
4. **子步积分**: dt 超过 1ms 时自动拆分成 1ms 子步, 保证 jerk 精度
5. **临近目标**: 误差 < 0.0005rad 且速度/加速度接近零 → 精确锁定

**默认参数**:
```c
{ response_wn = 8.0, max_vel = 5.0, max_acc = 30.0, max_jerk = 300.0 }  // 默认
{ response_wn = 6.0, max_vel = 3.0, max_acc = 15.0, max_jerk = 150.0 }  // TRACK
{ response_wn = 8.0, max_vel = 5.0, max_acc = 30.0, max_jerk = 300.0 }  // TURN
```

---

## 6. 云台管理层 API (gimbal_*)

> 这些函数在 main.c 中实现, 声明在 main.h。它们封装了轨迹规划器 + PID + 前馈的完整调用链。

```c
// 运行模式 (0=TRACK, 1=TURN, 2=AUTO)
uint8_t gimbal_set_run_mode(uint8_t motor_id, uint8_t run_mode);

// 角度目标 — 最常用
// 自动确保角度控制模式, 停止测试, 设置轨迹目标
uint8_t gimbal_set_angle_target(uint8_t motor_id, float target_deg);

// 双轴同步角度
uint8_t gimbal_set_dual_angle(float m0_deg, float m1_deg);

// 当前位置保持 (轨迹目标=当前角度, PID复位)
uint8_t gimbal_hold(uint8_t motor_id);  // motor_id=0xFF → 同时保持双轴

// 完整状态快照
uint8_t gimbal_get_status(uint8_t motor_id, GimbalStatus *status);

// 自动模式阈值: 误差超过 enter_turn_deg→TURN, 低于 enter_track_deg→TRACK
uint8_t gimbal_set_auto_thresholds(uint8_t motor_id, float enter_turn_deg, float enter_track_deg);
uint8_t gimbal_get_auto_thresholds(uint8_t motor_id, float *enter_turn_deg, float *enter_track_deg);

// 前馈增益 (V/(rad/s), V/(rad/s²))
uint8_t gimbal_set_feedforward(uint8_t motor_id, float velocity_gain, float acceleration_gain);
uint8_t gimbal_get_feedforward(uint8_t motor_id, float *velocity_gain, float *acceleration_gain);

// 轨迹参数配置 (profile: 0=TRACK, 1=TURN)
uint8_t gimbal_set_profile(uint8_t motor_id, uint8_t profile, const GimbalProfileConfig *config);
uint8_t gimbal_get_profile(uint8_t motor_id, uint8_t profile, GimbalProfileConfig *config);

// 速度上限 (rad/s) — 覆盖轨迹 max_vel
uint8_t gimbal_set_velocity_ceiling(uint8_t motor_id, float velocity_limit_rad_s);
```

### 内部辅助函数

```c
static void gimbal_apply_profile(uint8_t motor_id, uint8_t profile);      // 应用轨迹参数
static void gimbal_ensure_angle_control(uint8_t motor_id);                // 确保角度控制模式
static void gimbal_runtime_update(uint8_t motor_id);                      // 自动模式逻辑
static void gimbal_enforce_soft_limit(uint8_t motor_id);                  // 软限位 (±30°)
static void gimbal_sensor_fault_update(uint8_t motor_id);                 // 传感器故障检测
static void gimbal_stop_tests(uint8_t motor_id);                          // 停止测试
static uint8_t gimbal_motor_online(uint8_t motor_id);                     // 在线检查
```

---

## 7. SimpleFOC 电机层 API

> 基于 SimpleFOC 开源库, 双电机实例 (M0/M1 各自有独立的结构体副本)。

### 通用类型

```c
typedef enum { CW = 1, CCW = -1, UNKNOWN = 0 } Direction;
typedef enum {
    Type_torque, Type_velocity, Type_angle,
    Type_velocity_openloop, Type_angle_openloop
} MotionControlType;

typedef enum { Type_voltage, Type_dc_current, Type_foc_current } TorqueControlType;
```

### 全局变量 (M0)

```c
extern MotionControlType controller;        // 当前控制模式
extern TorqueControlType torque_controller; // 力矩控制类型
extern PIDController P_angle;               // 位置环 PID
extern PIDController PID_velocity;          // 速度环 PID
extern float voltage_limit;                 // 电压上限 (V)
extern float velocity_limit;                // 速度上限 (rad/s)
extern float shaft_angle;                   // 当前机械角度 (rad)
extern float shaft_velocity;                // 当前机械角速度 (rad/s)
extern float shaft_velocity_sp;             // 速度给定 (rad/s)
extern float voltage_feedback;              // PID 输出电压 (V)
extern float voltage_feedforward;           // 前馈电压 (V)
extern float voltage_applied;               // 总输出电压 (V)
extern long  sensor_direction;              // 编码器方向 (CW/CCW/UNKNOWN)
```

M1 有对应的 `_m1` 后缀版本 (`controller_m1`, `shaft_angle_m1` 等)。

### M0 函数

```c
// 初始化与校准
void Motor_init(void);
void Motor_initFOC(float zero_electric_offset, Direction _sensor_direction);
void Motor_syncSensor(void);             // 同步编码器零点
void Motor_updateSensor(void);           // 更新传感器读数

// 主循环
void loopFOC(void);                      // FOC 电流环

// 运动控制
void move(float new_target);             // 通用目标 (含义取决于 controller)
void moveAngleWithFeedforward(float position_target, float velocity_target,
                              float velocity_ff_gain, float acceleration_ff);
// 开环
float velocityOpenloop(float target_velocity);
float angleOpenloop(float target_angle);

// 直接电压
void setPhaseVoltage(float Uq, float Ud, float angle_el);
```

### M1 函数 (带 `_M1` 后缀)

```c
void Motor_init_M1(void);
void Motor_initFOC_M1(float zero_electric_offset, Direction dir);
void Motor_syncSensor_M1(void);
void Motor_updateSensor_M1(void);
void loopFOC_M1(void);
void move_M1(float new_target);
void moveAngleWithFeedforward_M1(float pos, float vel, float v_gain, float a_gain);
void setPhaseVoltage_M1(float Uq, float Ud, float angle_el);
float velocityOpenloop_M1(float target_velocity);
float angleOpenloop_M1(float target_angle);
```

### PID 控制器结构体

```c
typedef struct {
    float P, I, D;           // 增益
    float limit;             // 输出限幅
    float integral;          // 积分累加
    float prev_error;        // 上次误差
    float output_ramp;       // 输出斜率限制 (本项目设为0)
    // ... 更多字段
} PIDController;

void PID_init(PIDController *pid);
void PID_reset(PIDController *pid);
```

---

## 8. 错误码参考

| 错误码 | 宏名 | 说明 |
|--------|------|------|
| `0x00` | ERR_NONE | 无错误 |
| `0x01` | ERR_BAD_CRC | CRC 校验失败 |
| `0x02` | ERR_BAD_CMD | 未知命令 |
| `0x03` | ERR_BAD_LEN | 数据长度不匹配 |
| `0x04` | ERR_BAD_MOTOR_ID | 无效电机ID (只支持0/1) |
| `0x05` | ERR_MOTOR_OFFLINE | 目标电机不在线 |
| `0x06` | ERR_BAD_MODE | 无效控制模式 |
| `0x07` | ERR_TIMEOUT | 超时 |
| `0x08` | ERR_BAD_VALUE | 参数值超出范围或 NaN |
| `0x09` | ERR_BAD_STATE | 当前状态不允许此操作 |

---

## 9. MSPM0G3507 侧对接指南

> 本手册是为 MSPM0G3507 主控通过 UART 命令此云台驱动板而编写。以下是一个完整的对接模板。

### 9.1 硬件连接

```
MSPM0G3507          STM32F103 云台驱动板
  UART TX ────────────→ PA10 (USART1 RX)
  UART RX ←──────────── PA9  (USART1 TX)
  GND ──────────────── GND
```

### 9.2 上电初始化序列

```
1. MSPM0G3507 上电后等待 3 秒 (云台 FOC 校准需要约2秒)
2. 发送 PING (0x01) 确认双电机在线
3. 发送 SESSION (0x1B, enter=1) 进入二进制会话
4. 可选: 发送 GET_INFO (0x10) 确认版本和能力
5. 可选: 配置轨迹参数 SET_PROFILE (0x1C)
6. 发送 SET_RUN_MODE (0x11) 设置控制模式
```

### 9.3 典型控制循环

```
以 100Hz (10ms) 循环:
  1. 发送 SET_DUAL_ANGLE (0x15) — 同时设置 M0/M1 角度
     [AA 15 08 m0_deg m1_deg CRC]
  2. 等待应答或直接发送下一帧 (非阻塞)
  3. 可选: 每 100ms 发送 GET_ALL_STATUS (0x14) 获取完整状态
```

### 9.4 MSPM0G3507 C 语言实现参考

```c
#include "zf_common_headfile.h"

// ============ CRC8 查表 (poly 0x07) ============
static const uint8_t crc8_tab[256] = {
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    // ... 完整256字节表见 protocol.c
};

static uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t c = 0;
    while (len--) c = crc8_tab[c ^ *data++];
    return c;
}

// ============ 辅助函数 ============
static void put_f32_le(uint8_t *buf, float val) {
    union { float f; uint32_t u; } v = { .f = val };
    buf[0] = v.u; buf[1] = v.u >> 8; buf[2] = v.u >> 16; buf[3] = v.u >> 24;
}

static float get_f32_le(const uint8_t *buf) {
    union { float f; uint32_t u; } v;
    v.u = buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    return v.f;
}

// ============ 发送二进制帧 ============
static void send_frame(uint8_t cmd, const uint8_t *data, uint8_t len) {
    uint8_t buf[64];
    uint8_t idx = 0;
    buf[idx++] = 0xAA;         // SYNC
    buf[idx++] = cmd;           // CMD
    buf[idx++] = len;           // LEN
    if (len) memcpy(&buf[idx], data, len);
    idx += len;
    buf[idx] = crc8(&buf[1], idx - 1);  // CRC over CMD+LEN+DATA
    idx++;
    uart_write_buffer(GIMBAL_UART, buf, idx);
}

// ============ 常用命令封装 ============

// 设置单轴角度 (最常用)
void gimbal_set_angle(uint8_t uart_idx, uint8_t motor_id, float deg) {
    uint8_t data[5];
    data[0] = motor_id;
    put_f32_le(&data[1], deg);
    send_frame(uart_idx, 0x12, data, 5);
}

// 设置双轴角度 (推荐: 100Hz)
void gimbal_set_dual(uint8_t uart_idx, float m0_deg, float m1_deg) {
    uint8_t data[8];
    put_f32_le(&data[0], m0_deg);
    put_f32_le(&data[4], m1_deg);
    send_frame(uart_idx, 0x15, data, 8);
}

// 读取单轴状态
void gimbal_req_status(uint8_t uart_idx, uint8_t motor_id) {
    send_frame(uart_idx, 0x13, &motor_id, 1);
}

// 保持位置
void gimbal_cmd_hold(uint8_t uart_idx, uint8_t motor_id) {
    send_frame(uart_idx, 0x16, &motor_id, 1);
}

// 设置运行模式
void gimbal_set_mode(uint8_t uart_idx, uint8_t motor_id, uint8_t run_mode) {
    uint8_t data[2] = { motor_id, run_mode };
    send_frame(uart_idx, 0x11, data, 2);
}

// ============ 解析应答 ============
// 在 UART RX ISR 中调用, 收到完整帧后解析
typedef struct {
    float angle_deg, velocity_deg_s;
    float target_deg, trajectory_deg, error_deg, applied_voltage;
    uint8_t motor_id, run_mode, active_profile, flags;
    bool valid;
} gimbal_status_t;

bool parse_status_response(const uint8_t *frame, gimbal_status_t *s) {
    // frame = { SYNC, CMD, LEN, DATA[28], CRC }
    if (frame[0] != 0xAA || frame[1] != 0x13 || frame[2] != 28) return false;
    // 校验 CRC
    uint8_t expected = crc8(&frame[1], 30);
    if (frame[31] != expected) return false;
    // 解析 (小端)
    const uint8_t *d = &frame[3];
    s->motor_id       = d[0];
    s->run_mode       = d[1];
    s->active_profile = d[2];
    s->flags          = d[3];
    s->target_deg     = get_f32_le(&d[4]);
    s->trajectory_deg = get_f32_le(&d[8]);
    s->angle_deg      = get_f32_le(&d[12]);
    s->velocity_deg_s = get_f32_le(&d[16]);
    s->error_deg      = get_f32_le(&d[20]);
    s->applied_voltage= get_f32_le(&d[24]);
    s->valid = true;
    return true;
}
```

### 9.5 对接注意事项

1. **上电等待**: FOC 校准需要约 2 秒, 期间不要发送控制命令。通过 PING 确认在线后再开始控制。
2. **二进制会话**: 发送 `CMD_SESSION(1)` 后, 云台抑制所有 printf 文本, 确保 UART 流纯净。
3. **角度范围**: 软限位 ±30°, 超出范围会拒绝 (`ERR_BAD_VALUE`)。
4. **控制频率**: 建议 50~200Hz。500Hz 是云台内部控制周期, 外部命令不需要这么快。
5. **传感器故障**: 磁编码器连续 3 次读取失败会自动下电对应电机。MSPM0G3507 可通过 `CMD_GET_STATUS.flags & STATUS_SENSOR_VALID` 检测。
6. **双轴同步**: 使用 `CMD_SET_DUAL_ANGLE (0x15)` 而非先后两条 `CMD_SET_ANGLE`。

---

> **版本**: v1.0 | **生成日期**: 2026-07-20  
> **适用于**: MSPM0G3507 主控 → STM32F103 2804 云台驱动板 的串口命令对接
