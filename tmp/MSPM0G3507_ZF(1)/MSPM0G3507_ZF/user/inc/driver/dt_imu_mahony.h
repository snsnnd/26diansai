#ifndef _DT_IMU_MAHONY_H_
#define _DT_IMU_MAHONY_H_

#include "zf_common_headfile.h"
#include "driver/dt_mpu6050.h"

typedef struct {
    dt_mpu6050_config_t mpu;
    uint8_t             ready;
    float               q0, q1, q2, q3;
    float               gbias_x, gbias_y, gbias_z;
    float               Iex, Iey, Iez;
    float               acc_lpf_x, acc_lpf_y, acc_lpf_z;
    float               offs_roll, offs_pitch, offs_yaw;
    float               roll, pitch, yaw;
} dt_imu_mahony_t;

uint8_t dt_imu_mahony_init(dt_imu_mahony_t *imu, gpio_pin_enum scl, gpio_pin_enum sda);
void   dt_imu_mahony_update(dt_imu_mahony_t *imu, float dt_s);
void   dt_imu_mahony_zero_yaw(dt_imu_mahony_t *imu);

#endif
