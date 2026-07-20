/**
 * @file dt_mpu6050.c
 * @brief MPU6050 六轴惯性传感器驱动实现
 *        提供基于软件IIC的寄存器读写操作、传感器初始化以及
 *        加速度/角速度/温度数据的读取和物理量换算。
 */

#include "driver/dt_mpu6050.h"

/**
 * @brief 加速度计量程灵敏度查找表
 *        索引对应 DT_MPU6050_ACCEL_FS_xG 宏定义值
 *        值 = 每g对应的ADC读数
 */
static const float accel_sensitivity[4] = {
    16384.0f,  /* ±2g  */
    8192.0f,   /* ±4g  */
    4096.0f,   /* ±8g  */
    2048.0f    /* ±16g */
};

/**
 * @brief 陀螺仪量程灵敏度查找表
 *        索引对应 DT_MPU6050_GYRO_FS_xxx 宏定义值
 *        值 = 每(°/s)对应的ADC读数
 */
static const float gyro_sensitivity[4] = {
    131.0f,   /* ±250 °/s   */
    65.5f,    /* ±500 °/s   */
    32.8f,    /* ±1000 °/s  */
    16.4f     /* ±2000 °/s  */
};

/* ===== HAL层：寄存器读写 ===== */

/**
 * @brief 向MPU6050指定寄存器写入一个字节
 * @param iic IIC接口指针
 * @param reg 寄存器地址
 * @param value 要写入的值
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_hal_write_reg(soft_iic_info_struct *iic, uint8_t reg, uint8_t value)
{
    return soft_iic_write_8bit_register_checked(iic, reg, value) == SOFT_IIC_STATUS_OK;
}

/**
 * @brief 向MPU6050写入多字节数据
 * @param iic IIC接口指针
 * @param reg 起始寄存器地址
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_hal_write_regs(soft_iic_info_struct *iic, uint8_t reg,
                                  const uint8_t *data, uint16_t len)
{
    return soft_iic_write_8bit_registers_checked(iic, reg, data, len) == SOFT_IIC_STATUS_OK;
}

/**
 * @brief 从MPU6050指定寄存器读取一个字节
 * @param iic IIC接口指针
 * @param reg 寄存器地址
 * @param value 输出读取的值
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_hal_read_reg(soft_iic_info_struct *iic, uint8_t reg, uint8_t *value)
{
    return soft_iic_read_8bit_register_checked(iic, reg, value) == SOFT_IIC_STATUS_OK;
}

/**
 * @brief 从MPU6050连续读取多字节数据
 * @param iic IIC接口指针
 * @param reg 起始寄存器地址
 * @param data 输出缓冲区
 * @param len 读取字节数
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_hal_read_regs(soft_iic_info_struct *iic, uint8_t reg,
                                 uint8_t *data, uint16_t len)
{
    return soft_iic_read_8bit_registers_checked(iic, reg, data, len) == SOFT_IIC_STATUS_OK;
}

/**
 * @brief MPU6050完整初始化序列（HAL接口）
 *
 * 初始化步骤：
 *   1. 写0x80到PWR_MGMT_1 -> 复位整个芯片
 *   2. 等待100ms复位完成
 *   3. 写0x00到PWR_MGMT_1 -> 唤醒芯片（使用内部振荡器）
 *   4. 等待10ms唤醒稳定
 *   5. 读取WHO_AM_I寄存器验证芯片ID（高7位应为0x68）
 *   6. 配置采样率分频、DLPF、陀螺仪量程、加速度计量程
 *
 * @param iic IIC接口指针
 * @param sample_rate_div 采样率分频器
 * @param dlpf_cfg 数字低通滤波器配置
 * @param gyro_fs 陀螺仪量程
 * @param accel_fs 加速度计量程
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_hal_init(soft_iic_info_struct *iic, uint8_t sample_rate_div,
                            uint8_t dlpf_cfg, uint8_t gyro_fs, uint8_t accel_fs)
{
    uint8_t whoami;

    /* 参数合法性检查 */
    if((NULL == iic) || (dlpf_cfg > 7) || (gyro_fs > 3) || (accel_fs > 3))
    {
        return 0;
    }

    /* 步骤1：复位芯片（DEVICE_RESET=1） */
    if(!dt_mpu6050_hal_write_reg(iic, DT_MPU6050_REG_PWR_MGMT_1, 0x80))
    {
        return 0;
    }
    system_delay_ms(100); /* 等待复位完成 */

    /* 步骤2：唤醒芯片（清除复位位，使用内部振荡器作为时钟源） */
    if(!dt_mpu6050_hal_write_reg(iic, DT_MPU6050_REG_PWR_MGMT_1, 0x00))
    {
        return 0;
    }
    system_delay_ms(10); /* 等待唤醒稳定 */

    /* 步骤3：验证芯片ID */
    if(!dt_mpu6050_hal_read_reg(iic, DT_MPU6050_REG_WHO_AM_I, &whoami)
        || ((whoami & 0x7Eu) != 0x68u))
    {
        return 0; /* ID不匹配，通信失败或芯片不正确 */
    }

    /* 步骤4：配置传感器参数
     * SMPLRT_DIV = sample_rate_div（采样率=8kHz/(1+div)）
     * CONFIG = dlpf_cfg（数字低通滤波器带宽）
     * GYRO_CONFIG = gyro_fs << 3（量程选择位在bit3~bit4）
     * ACCEL_CONFIG = accel_fs << 3（量程选择位在bit3~bit4） */
    return dt_mpu6050_hal_write_reg(iic, DT_MPU6050_REG_SMPLRT_DIV, sample_rate_div)
        && dt_mpu6050_hal_write_reg(iic, DT_MPU6050_REG_CONFIG, dlpf_cfg)
        && dt_mpu6050_hal_write_reg(iic, DT_MPU6050_REG_GYRO_CONFIG, (uint8_t)(gyro_fs << 3))
        && dt_mpu6050_hal_write_reg(iic, DT_MPU6050_REG_ACCEL_CONFIG, (uint8_t)(accel_fs << 3));
}

/**
 * @brief 初始化MPU6050（高级接口）
 *        使用8kHz内部采样率，分频7得1kHz输出，DLPF=6（约5Hz带宽，适合姿态解算）
 * @param cfg MPU6050配置结构体指针
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_init(dt_mpu6050_config_t *cfg)
{
    if(NULL == cfg) return 0;
    /* 量程参数钳位到有效范围 */
    if (cfg->accel_fs > 3) cfg->accel_fs = 0;
    if (cfg->gyro_fs  > 3) cfg->gyro_fs  = 0;

    /* 采样率分频=7 -> 1kHz输出率，DLPF=6 -> 5Hz截止频率（适用于姿态解算） */
    return dt_mpu6050_hal_init(&cfg->iic, 0x07, 0x06, cfg->gyro_fs, cfg->accel_fs);
}

/**
 * @brief 读取MPU6050全部传感器数据
 *        从ACCEL_XOUT_H(0x3B)开始连续读取14字节：
 *        ACCEL_X(2) + ACCEL_Y(2) + ACCEL_Z(2) + TEMP(2) + GYRO_X(2) + GYRO_Y(2) + GYRO_Z(2)
 *        原始16位值除以对应的灵敏度系数得到物理量
 * @param cfg MPU6050配置结构体指针
 * @param data 输出数据结构体指针
 * @return 1=成功，0=失败
 */
uint8_t dt_mpu6050_read_all(dt_mpu6050_config_t *cfg, dt_mpu6050_data_t *data)
{
    uint8_t buf[14];  /* 14 = 6轴 * 2字节 + 温度 * 2字节 */
    int16_t raw;

    /* 参数检查 */
    if((NULL == cfg) || (NULL == data) || (cfg->accel_fs > 3) || (cfg->gyro_fs > 3))
    {
        return 0;
    }

    /* 从ACCEL_XOUT_H开始连续读取14字节 */
    if(!dt_mpu6050_hal_read_regs(&cfg->iic, DT_MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf)))
    {
        return 0;
    }

    /* === 加速度数据（大端格式：高字节在前） === */
    raw = (int16_t)((buf[0] << 8) | buf[1]);
    data->ax = (float)raw / accel_sensitivity[cfg->accel_fs]; /* 单位：g */
    raw = (int16_t)((buf[2] << 8) | buf[3]);
    data->ay = (float)raw / accel_sensitivity[cfg->accel_fs];
    raw = (int16_t)((buf[4] << 8) | buf[5]);
    data->az = (float)raw / accel_sensitivity[cfg->accel_fs];

    /* === 温度数据 ===
     * 公式：Temperature(C) = (raw/340) + 36.53 */
    raw = (int16_t)((buf[6] << 8) | buf[7]);
    data->temp = ((float)raw / 340.0f) + 36.53f;

    /* === 陀螺仪数据 === */
    raw = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gx = (float)raw / gyro_sensitivity[cfg->gyro_fs]; /* 单位：°/s */
    raw = (int16_t)((buf[10] << 8) | buf[11]);
    data->gy = (float)raw / gyro_sensitivity[cfg->gyro_fs];
    raw = (int16_t)((buf[12] << 8) | buf[13]);
    data->gz = (float)raw / gyro_sensitivity[cfg->gyro_fs];

    return 1;
}
