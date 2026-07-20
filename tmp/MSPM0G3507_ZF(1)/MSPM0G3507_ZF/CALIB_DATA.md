# MSPM0G3507 云台标定数据

## 硬件配置

| 参数 | 值 |
|------|-----|
| MCU | MSPM0G3507 |
| PITCH 电机 | ZDT_X42S, 地址 1, UART2 |
| YAW 电机 | ZDT_X42S, 地址 2, UART2 |
| 通信 | 半双工 UART, 115200bps, 单线 (PB15, AF2) |
| PITCH 齿轮比 | 4:1 (电机4圈 = 云台1圈) |
| YAW 齿轮比 | 8:1 (电机8圈 = 云台1圈) |
| 微步细分 | 16 |
| 默认速度 | 300 RPM |
| 默认加速度 | 50 |

## 编码器 (0x31) 绝对位置数据

编码器是绝对值传感器，每转 65536 计数 → 360°。同一物理位置每次上电读数相同。

### PITCH

| 位置 | 编码器值 | 云台角度 | 备注 |
|------|---------|---------|------|
| CW 限位 (最上) | ~136.5° | 85° (计算) | Jog CW 堵转检测 |
| CCW 限位 (最下) | ~156.2° | ~0° | 反向探索滞停 |
| 水平位 (正前方) | 326.4° | 0° | 从限位回退 85°×4 = 340° 电机度 |
| 电机总行程 | ~340.3° | — | 编码器经过一次 360→0 回绕 |

### YAW

| 位置 | 编码器值 | 备注 |
|------|---------|------|
| 中心 (正前方) | ~0.0° | 360° 无限旋转, 无机械限位 |

## 标定方法

1. **PITCH**: Jog CW (30RPM, accel=20) → 堵转 → 清除保护 → 编码器回退 `back_angle × ratio` 电机度到水平位 → 记录编码器值
2. **YAW**: 手动摆正 → 记录编码器值

标定后 `gimbal_move_to(yaw_deg, pitch_deg)` 直接控制云台角度。

## gimbal.h 常量

```c
#define GIMBAL_USE_PRECALIB_PITCH  1       // 1=使用预标定值, 0=上电自动标定

#define GIMBAL_PITCH_RATIO         4.0f    // 电机:云台 齿轮比
#define GIMBAL_YAW_RATIO           8.0f
#define GIMBAL_PITCH_BACK_ANGLE   -85.0f   // 从CW限位回退到水平的角度(云台度)
#define GIMBAL_PITCH_ENC_LIMIT     136.5f  // CW限位编码器值
#define GIMBAL_PITCH_ENC_HORIZONTAL 326.4f // 水平位编码器值
#define GIMBAL_YAW_ENC_CENTER      0.0f    // YAW中心编码器值
```

## 位置换算公式

```
编码器 → 云台角度:  angle = (encoder - HORIZONTAL_ENC) / RATIO
云台角度 → 编码器:  encoder = HORIZONTAL_ENC + angle * RATIO
```

## 标定日志 (参考)

```
[GEARED] PITCH CW limit enc: 136.5 deg
[GEARED] PITCH horizontal enc: 326.4 deg
[GEARED] YAW center enc: 0.0 deg
[GEARED] PITCH limits: -85.0 ~ 0.0 gimbal deg
[GEARED] YAW limits: -179.0 ~ 179.0 gimbal deg
```

## 移动测试

```
[MOVE] pitch cur=162.8 tgt=326.4 d=163.5 pul=1454
```
PITCH 从编码器 162.8° 移动到水平位 326.4°，delta=163.5°（1454 脉冲），执行成功。

## 已知问题与注意事项

1. **printf 不支持 %f**: MSPM0 嵌入式库默认不带浮点格式化，所有角度用整数 ×10 打印
2. **半双工总线**: PITCH/YAW 共用 UART2 单线总线，读取编码器时需短暂延迟避免冲突
3. **预标定模式**: `GIMBAL_USE_PRECALIB_PITCH=1` 跳过校准直接使用预存值，上电 `gimbal_move_to(0,0)` 回中
4. **全自动标定**: `GIMBAL_USE_PRECALIB_PITCH=0` 上电自动 Jog 堵转标定
