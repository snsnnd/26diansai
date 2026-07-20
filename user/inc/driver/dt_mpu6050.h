/**
 * @file dt_mpu6050.h
 * @brief MPU6050 六轴惯性传感器驱动头文件
 *        支持通过IIC接口配置和读取加速度计、陀螺仪和温度传感器数据。
 *        提供硬件抽象层（HAL）和高级初始化/读取接口。
 */

#ifndef _DT_MPU6050_H_
#define _DT_MPU6050_H_

#include "zf_common_headfile.h"

/** @brief MPU6050默认I2C地址（AD0引脚接GND时为0x68，接VCC时为0x69） */
#define DT_MPU6050_DEFAULT_ADDR    0x68

/* ===== 加速度计量程配置 ===== */
#define DT_MPU6050_ACCEL_FS_2G     0   /**< ±2g  (灵敏度 16384 LSB/g) */
#define DT_MPU6050_ACCEL_FS_4G     1   /**< ±4g  (灵敏度 8192 LSB/g) */
#define DT_MPU6050_ACCEL_FS_8G     2   /**< ±8g  (灵敏度 4096 LSB/g) */
#define DT_MPU6050_ACCEL_FS_16G    3   /**< ±16g (灵敏度 2048 LSB/g) */

/* ===== 陀螺仪量程配置 ===== */
#define DT_MPU6050_GYRO_FS_250     0   /**< ±250 °/s  (灵敏度 131 LSB/(°/s)) */
#define DT_MPU6050_GYRO_FS_500     1   /**< ±500 °/s  (灵敏度 65.5 LSB/(°/s)) */
#define DT_MPU6050_GYRO_FS_1000    2   /**< ±1000 °/s (灵敏度 32.8 LSB/(°/s)) */
#define DT_MPU6050_GYRO_FS_2000    3   /**< ±2000 °/s (灵敏度 16.4 LSB/(°/s)) */

/* ===== MPU6050寄存器地址 ===== */
#define DT_MPU6050_REG_SMPLRT_DIV    0x19   /**< 采样率分频寄存器 */
#define DT_MPU6050_REG_CONFIG        0x1A   /**< 数字低通滤波器(DLPF)配置 */
#define DT_MPU6050_REG_GYRO_CONFIG   0x1B   /**< 陀螺仪量程配置 */
#define DT_MPU6050_REG_ACCEL_CONFIG  0x1C   /**< 加速度计量程配置 */
#define DT_MPU6050_REG_ACCEL_XOUT_H  0x3B   /**< 加速度计X轴数据高字节（数据起始地址） */
#define DT_MPU6050_REG_PWR_MGMT_1    0x6B   /**< 电源管理寄存器1 */
#define DT_MPU6050_REG_WHO_AM_I      0x75   /**< 芯片ID寄存器（应返回0x68或0x69） */

/**
 * @brief MPU6050配置结构体
 *        包含IIC接口信息和量程选择
 */
typedef struct {
    soft_iic_info_struct  iic;      /**< 软件IIC接口结构体 */
    uint8_t               accel_fs; /**< 加速度计量程配置值 */
    uint8_t               gyro_fs;  /**< 陀螺仪量程配置值 */
} dt_mpu6050_config_t;

/**
 * @brief MPU6050读取数据输出结构体
 *        包含经过量程换算后的物理量值
 */
typedef struct {
    float ax, ay, az;    /**< 三轴加速度（单位：g） */
    float gx, gy, gz;    /**< 三轴角速度（单位：°/s） */
    float temp;          /**< 温度（单位：°C） */
} dt_mpu6050_data_t;

/* ===== 硬件抽象层（HAL）函数 ===== */

/**
 * @brief 初始化MPU6050传感器（低层HAL接口）
 *        执行完整的初始化序列：复位、唤醒、配置采样率/DLPF/量程
 * @param iic IIC接口结构体指针
 * @param sample_rate_div 采样率分频系数（采样率=8kHz/(1+div)）
 * @param dlpf_cfg DLPF配置（0~6对应不同截止频率）
 * @param gyro_fs 陀螺仪量程（0~3）
 * @param accel_fs 加速度计量程（0~3）
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_hal_init(soft_iic_info_struct *iic, uint8_t sample_rate_div,
                            uint8_t dlpf_cfg, uint8_t gyro_fs, uint8_t accel_fs);

/**
 * @brief 向MPU6050指定寄存器写入一个字节
 * @param iic IIC接口结构体指针
 * @param reg 寄存器地址
 * @param value 要写入的值
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_hal_write_reg(soft_iic_info_struct *iic, uint8_t reg, uint8_t value);

/**
 * @brief 向MPU6050指定寄存器写入多字节数据
 * @param iic IIC接口结构体指针
 * @param reg 起始寄存器地址
 * @param data 待写入数据缓冲区指针
 * @param len 数据长度（字节）
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_hal_write_regs(soft_iic_info_struct *iic, uint8_t reg,
                                  const uint8_t *data, uint16_t len);

/**
 * @brief 从MPU6050指定寄存器读取一个字节
 * @param iic IIC接口结构体指针
 * @param reg 寄存器地址
 * @param value 读取值输出指针
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_hal_read_reg(soft_iic_info_struct *iic, uint8_t reg, uint8_t *value);

/**
 * @brief 从MPU6050指定寄存器连续读取多字节数据
 * @param iic IIC接口结构体指针
 * @param reg 起始寄存器地址
 * @param data 读取数据缓冲区指针
 * @param len 读取长度（字节）
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_hal_read_regs(soft_iic_info_struct *iic, uint8_t reg,
                                 uint8_t *data, uint16_t len);

/* ===== 高级接口 ===== */

/**
 * @brief 初始化MPU6050（高级接口）
 *        使用配置结构体中的量程设置，采样率分频=7，DLPF=6
 * @param cfg MPU6050配置结构体指针
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_init(dt_mpu6050_config_t *cfg);

/**
 * @brief 读取MPU6050全部数据（加速度+温度+陀螺仪）
 *        从寄存器0x3B开始连续读取14字节
 * @param cfg MPU6050配置结构体指针
 * @param data 数据输出结构体指针
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_read_all(dt_mpu6050_config_t *cfg, dt_mpu6050_data_t *data);

#endif
