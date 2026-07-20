/**
 * @file dt_imu_mahony.h
 * @brief 基于Mahony互补滤波算法的六轴IMU姿态解算驱动头文件
 *        使用MPU6050的加速度计和陀螺仪数据，通过Mahony算法
 *        解算出载体当前的roll（横滚角）、pitch（俯仰角）、yaw（偏航角）。
 */

#ifndef _DT_IMU_MAHONY_H_
#define _DT_IMU_MAHONY_H_

#include "zf_common_headfile.h"
#include "driver/dt_mpu6050.h"

/**
 * @brief Mahony姿态解算滤波器实例结构体
 *        包含MPU6050配置、四元数、陀螺仪零偏、积分误差项、
 *        加速度低通滤波值、角度偏移量以及解算出的欧拉角
 */
typedef struct {
    dt_mpu6050_config_t mpu;          /**< MPU6050传感器配置（IIC接口、量程） */
    uint8_t             ready;         /**< 初始化完成标志（1=就绪，0=未就绪） */
    /* 四元数（描述刚体姿态的单位四元数，q0为标量部分） */
    float               q0, q1, q2, q3;
    float               gbias_x, gbias_y, gbias_z;  /**< 陀螺仪零偏（°/s） */
    float               Iex, Iey, Iez;              /**< Mahony算法PI控制器的积分项 */
    float               acc_lpf_x, acc_lpf_y, acc_lpf_z; /**< 加速度低通滤波值 */
    float               offs_roll, offs_pitch, offs_yaw; /**< 角度偏移量（安装偏差补偿） */
    float               roll, pitch, yaw;            /**< 解算后的欧拉角（度） */
} dt_imu_mahony_t;

/**
 * @brief 初始化Mahony姿态解算模块
 *        初始化MPU6050（IIC、量程配置），执行陀螺仪零偏校准，
 *        初始化四元数及加速度低通滤波值。
 * @param imu Mahony结构体指针
 * @param scl IIC时钟线引脚
 * @param sda IIC数据线引脚
 * @return 1=初始化成功，0=初始化失败（传感器未响应或校准失败）
 */
uint8_t dt_imu_mahony_init(dt_imu_mahony_t *imu, gpio_pin_enum scl, gpio_pin_enum sda);

/**
 * @brief Mahony姿态解算更新函数
 *        读取MPU6050最新数据，执行加速度低通滤波、
 *        Mahony互补滤波算法更新四元数、计算欧拉角。
 * @param imu Mahony结构体指针
 * @param dt_s 距离上次更新的时间间隔（秒）
 * @note dt_s 建议在0.001~0.1范围内；过大或过小会影响算法稳定性
 */
void   dt_imu_mahony_update(dt_imu_mahony_t *imu, float dt_s);

/**
 * @brief 将当前偏航角清零（用于重新设定方向基准）
 * @param imu Mahony结构体指针
 */
void   dt_imu_mahony_zero_yaw(dt_imu_mahony_t *imu);

#endif
