#ifndef _LINE_CAR_H_
#define _LINE_CAR_H_

/**
 * @file line_car.h
 * @brief 智能车巡线主控模块
 *
 * 本模块是智能车巡线程序的核心调度层，定义了所有周期性任务的接口。
 * 通过EC调度器(ec_scheduler)管理多个任务的执行频率和优先级。
 * 任务包括传感器读取、控制计算、调试输出等。
 */

#include "zf_common_headfile.h"
#include "app/h2024_tasks.h"
#include "framework/ec_mode_manager.h"

/**
 * @brief 智能车初始化
 *
 * 初始化所有硬件模块(电机、编码器、陀螺仪、OLED、按键、LED、蜂鸣器、
 * 灰度传感器等)和软件模块(PID控制器、模式管理器、菜单系统、参数调优等)。
 * 应在系统启动时调用一次。
 */
void line_car_init(void);

/* ==================== 通用车辆控制接口 ==================== */

bool car_prepare(uint32_t now_ms);
void car_reset_odometry(void);
bool car_read_state(h2024_vehicle_state_t *state, uint32_t now_ms);
void car_drive_heading(float heading_deg, uint32_t now_ms);
void car_follow_line(uint32_t now_ms);
bool car_align_heading(float heading_deg, uint32_t now_ms);
void car_signal_point(uint32_t now_ms);
void car_stop(uint32_t now_ms);

/**
 * @brief 紧急停止
 *
 * 触发车辆急停：锁定紧急状态，停止电机并设置故障模式。
 * 可由紧急ISR或按键事件触发。
 */
void line_car_emergency_stop(void);

ec_mode_manager_t *line_car_get_mode_manager(void);
uint8_t car_get_line_bits(void);

void car_get_imu_accel(float *ax, float *ay, float *az);
float car_get_heading_deg(void);
float car_get_wz_dps(void);

/**
 * @brief 输入采集任务
 * 采集编码器输入，检查按键事件，处理电机看门狗和服务故障
 */
void line_car_input_task(uint32_t now_ms, void *context);

/**
 * @brief 陀螺仪更新任务
 * 更新 CAR_GYRO_SOURCE 选择的航向传感器
 */
void line_car_gyro_task(uint32_t now_ms, void *context);

/**
 * @brief 巡线传感器更新任务
 * 读取T8灰度传感器，更新线位置和线状态
 */
void line_car_line_sensor_task(uint32_t now_ms, void *context);

/**
 * @brief 传感器综合任务
 * 读取电池ADC、计算RPM、更新编码器数据
 */
void line_car_sensor_task(uint32_t now_ms, void *context);

/**
 * @brief 控制任务(主控制环路)
 * 执行模式管理器(含PID控制计算)、远程命令服务和电机看门狗
 */
void line_car_control_task(uint32_t now_ms, void *context);

/**
 * @brief 菜单/显示更新任务
 * 根据当前模式更新OLED显示内容(主菜单/参数调优/死区测试等)
 */
void line_car_menu_task(uint32_t now_ms, void *context);

/**
 * @brief 蜂鸣器服务任务
 * 处理蜂鸣器异步鸣叫序列和路径点LED指示
 */
void line_car_buzzer_task(uint32_t now_ms, void *context);

/**
 * @brief OLED刷新任务
 * 增量刷新OLED显示(只更新脏区域)
 */
void line_car_oled_task(uint32_t now_ms, void *context);

/**
 * @brief 遥测数据发送任务
 * 通过VOFA协议向上位机发送实时数据(用于在线调试)
 */
void line_car_telemetry_task(uint32_t now_ms, void *context);

/**
 * @brief 调试数据发送任务
 * 发送运行帧、传感器帧、定时帧等二进制调试数据到上位机
 */
void line_car_debug_task(uint32_t now_ms, void *context);

/**
 * @brief 调参/遥控接收任务
 * 解析来自串口的调参命令和远程控制命令(CC协议和遥控协议)
 */
void line_car_tune_task(uint32_t now_ms, void *context);

#endif
