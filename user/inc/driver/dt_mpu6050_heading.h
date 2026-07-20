/**
 * @file dt_mpu6050_heading.h
 * @brief 基于MPU6050的偏航角推算模块头文件
 *        仅使用Z轴陀螺仪进行偏航角积分，适用于无磁力计的场合。
 *        提供初始化、零偏校准、方差分析、角度积分和归零功能。
 */

#ifndef DT_MPU6050_HEADING_H
#define DT_MPU6050_HEADING_H

#include "driver/dt_mpu6050.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 模块状态枚举
 */
typedef enum
{
    DT_MPU6050_HEADING_UNINITIALIZED = 0,  /**< 未初始化 */
    DT_MPU6050_HEADING_BUS_ERROR,           /**< IIC总线通信错误 */
    DT_MPU6050_HEADING_ID_ERROR,            /**< 芯片ID验证失败 */
    DT_MPU6050_HEADING_CONFIG_ERROR,        /**< 传感器配置失败 */
    DT_MPU6050_HEADING_CALIBRATION_ERROR,   /**< 零偏校准失败（方差过大或样本不足） */
    DT_MPU6050_HEADING_READY                /**< 就绪，可正常使用 */
} dt_mpu6050_heading_status_t;

/**
 * @brief 偏航角推算模块结构体
 *        维护MPU6050配置、最新样本、偏航角、Z轴角速度、校准数据及状态
 */
typedef struct
{
    dt_mpu6050_config_t mpu;                      /**< MPU6050配置 */
    dt_mpu6050_data_t last_sample;                /**< 最近一次读取的原始样本 */
    float yaw_deg;                                /**< 积分后的偏航角（度），范围-180~180 */
    float wz_dps;                                 /**< 去零偏后的Z轴角速度（度/秒） */
    float gyro_bias_z;                            /**< Z轴陀螺仪零偏（度/秒） */
    float calibration_variance_dps2;              /**< 校准时的零偏方差（用于评估校准质量） */
    float calibration_min_gz_dps;                 /**< 校准过程中的最小Z轴角速度 */
    float calibration_max_gz_dps;                 /**< 校准过程中的最大Z轴角速度 */
    uint32_t last_update_ms;                      /**< 上次更新的时间戳（毫秒） */
    uint32_t sample_count;                        /**< 成功读取的样本总数 */
    uint32_t read_error_count;                    /**< 读取失败累计次数 */
    uint32_t calibration_valid_samples;           /**< 校准中有效样本数 */
    uint8_t address;                              /**< 检测到的MPU6050 I2C地址 */
    uint8_t who_am_i;                             /**< WHO_AM_I寄存器读取值 */
    dt_mpu6050_heading_status_t status;           /**< 当前状态 */
    bool ready;                                   /**< 初始化完成标志 */
} dt_mpu6050_heading_t;

/**
 * @brief 初始化偏航角推算模块
 *        探测I2C地址、初始化传感器、预热、执行零偏校准
 * @param heading 偏航角模块结构体指针
 * @param scl IIC SCL引脚
 * @param sda IIC SDA引脚
 * @return true=成功，false=失败（可通过status查看具体原因）
 */
bool dt_mpu6050_heading_init(dt_mpu6050_heading_t *heading,
    gpio_pin_enum scl, gpio_pin_enum sda);

/**
 * @brief 读取一次MPU6050样本到last_sample中
 * @param heading 偏航角模块结构体指针
 * @return true=成功，false=失败
 */
bool dt_mpu6050_heading_read_sample(dt_mpu6050_heading_t *heading);

/**
 * @brief 更新偏航角积分
 *        读取最新样本，计算去零偏后的Z轴角速度，
 *        根据时间间隔进行角度积分并自动归一到[-180,180]范围。
 * @param heading 偏航角模块结构体指针
 * @param now_ms 当前系统时间戳（毫秒）
 * @return true=成功，false=模块未就绪或读取失败
 */
bool dt_mpu6050_heading_update(dt_mpu6050_heading_t *heading,
    uint32_t now_ms);

/**
 * @brief 将当前偏航角清零
 * @param heading 偏航角模块结构体指针
 */
void dt_mpu6050_heading_zero(dt_mpu6050_heading_t *heading);

#endif
