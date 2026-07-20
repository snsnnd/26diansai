#ifndef _DT_MPU6050_H_
#define _DT_MPU6050_H_

#include "zf_common_headfile.h"

#define DT_MPU6050_DEFAULT_ADDR    0x68

#define DT_MPU6050_ACCEL_FS_2G     0
#define DT_MPU6050_ACCEL_FS_4G     1
#define DT_MPU6050_ACCEL_FS_8G     2
#define DT_MPU6050_ACCEL_FS_16G    3

#define DT_MPU6050_GYRO_FS_250     0
#define DT_MPU6050_GYRO_FS_500     1
#define DT_MPU6050_GYRO_FS_1000    2
#define DT_MPU6050_GYRO_FS_2000    3

typedef struct {
    soft_iic_info_struct  iic;
    uint8_t               accel_fs;
    uint8_t               gyro_fs;
} dt_mpu6050_config_t;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
} dt_mpu6050_data_t;

uint8_t dt_mpu6050_init(dt_mpu6050_config_t *cfg);
uint8_t dt_mpu6050_read_all(dt_mpu6050_config_t *cfg, dt_mpu6050_data_t *data);

#endif
