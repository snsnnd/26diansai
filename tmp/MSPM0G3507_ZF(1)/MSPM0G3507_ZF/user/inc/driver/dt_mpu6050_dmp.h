#ifndef _DT_MPU6050_DMP_H_
#define _DT_MPU6050_DMP_H_

#include "zf_common_headfile.h"

typedef struct {
    soft_iic_info_struct  iic;
} dt_mpu6050_dmp_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} dt_dmp_data_t;

uint8_t dt_mpu6050_dmp_init(dt_mpu6050_dmp_t *dmp);
uint8_t dt_mpu6050_dmp_read(dt_mpu6050_dmp_t *dmp, dt_dmp_data_t *out);

#endif
