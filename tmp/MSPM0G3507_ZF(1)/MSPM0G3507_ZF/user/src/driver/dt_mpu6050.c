#include "driver/dt_mpu6050.h"

#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_TEMP_OUT_H    0x41
#define MPU6050_REG_GYRO_XOUT_H   0x43

static const float accel_sensitivity[4] = {
    16384.0f, 8192.0f, 4096.0f, 2048.0f
};

static const float gyro_sensitivity[4] = {
    131.0f, 65.5f, 32.8f, 16.4f
};

uint8_t dt_mpu6050_init(dt_mpu6050_config_t *cfg)
{
    uint8_t whoami;

    if (cfg->accel_fs > 3) cfg->accel_fs = 0;
    if (cfg->gyro_fs  > 3) cfg->gyro_fs  = 0;

    soft_iic_write_8bit_register(&cfg->iic, MPU6050_REG_PWR_MGMT_1, 0x80);
    system_delay_ms(100);
    soft_iic_write_8bit_register(&cfg->iic, MPU6050_REG_PWR_MGMT_1, 0x00);
    system_delay_ms(10);

    whoami = soft_iic_read_8bit_register(&cfg->iic, MPU6050_REG_WHO_AM_I);
    if (whoami != 0x68)
    {
        return 0;
    }

    soft_iic_write_8bit_register(&cfg->iic, MPU6050_REG_SMPLRT_DIV, 0x07);
    soft_iic_write_8bit_register(&cfg->iic, MPU6050_REG_CONFIG, 0x06);
    soft_iic_write_8bit_register(&cfg->iic, MPU6050_REG_GYRO_CONFIG,  (cfg->gyro_fs  << 3));
    soft_iic_write_8bit_register(&cfg->iic, MPU6050_REG_ACCEL_CONFIG, (cfg->accel_fs << 3));

    return 1;
}

uint8_t dt_mpu6050_read_all(dt_mpu6050_config_t *cfg, dt_mpu6050_data_t *data)
{
    uint8_t buf[14];
    int16_t raw;

    soft_iic_read_8bit_registers(&cfg->iic, MPU6050_REG_ACCEL_XOUT_H, buf, 14);

    raw = (int16_t)((buf[0] << 8) | buf[1]);
    data->ax = (float)raw / accel_sensitivity[cfg->accel_fs];
    raw = (int16_t)((buf[2] << 8) | buf[3]);
    data->ay = (float)raw / accel_sensitivity[cfg->accel_fs];
    raw = (int16_t)((buf[4] << 8) | buf[5]);
    data->az = (float)raw / accel_sensitivity[cfg->accel_fs];

    raw = (int16_t)((buf[6] << 8) | buf[7]);
    data->temp = ((float)raw / 340.0f) + 36.53f;

    raw = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gx = (float)raw / gyro_sensitivity[cfg->gyro_fs];
    raw = (int16_t)((buf[10] << 8) | buf[11]);
    data->gy = (float)raw / gyro_sensitivity[cfg->gyro_fs];
    raw = (int16_t)((buf[12] << 8) | buf[13]);
    data->gz = (float)raw / gyro_sensitivity[cfg->gyro_fs];

    return 1;
}
