#include "driver/dt_imu_mahony.h"
#include <math.h>

#define PI         3.1415926f
#define KP         0.5f
#define KI         0.01f
#define ACC_ALPHA  0.3f

#define IMU_USE_HARDCODED_BIAS 1

#if IMU_USE_HARDCODED_BIAS
static const float g_bias_x = -1.69f;
static const float g_bias_y = 0.32f;
static const float g_bias_z = 0.13f;
#endif

static float inv_sqrt(float num)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5f;
    x2 = num * 0.5f;
    y = num;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y));
    y = y * (threehalfs - (x2 * y * y));
    return y;
}

static void imu_calibrate(dt_imu_mahony_t *imu)
{
#if IMU_USE_HARDCODED_BIAS
    imu->gbias_x = g_bias_x;
    imu->gbias_y = g_bias_y;
    imu->gbias_z = g_bias_z;
    printf("[IMU] bias: hardcoded\r\n");
#else
    dt_mpu6050_data_t raw;
    float gx = 0, gy = 0, gz = 0;

    printf("[IMU] calibrating 5000 samples...\r\n");
    for (int i = 0; i < 5000; i++)
    {
        dt_mpu6050_read_all(&imu->mpu, &raw);
        gx += raw.gx; gy += raw.gy; gz += raw.gz;
        system_delay_ms(2);
    }
    imu->gbias_x = gx / 5000.0f;
    imu->gbias_y = gy / 5000.0f;
    imu->gbias_z = gz / 5000.0f;

    printf("[IMU] bias5000: gx=%d gy=%d gz=%d (x100)\r\n",
           (int)(imu->gbias_x * 100), (int)(imu->gbias_y * 100), (int)(imu->gbias_z * 100));
#endif
}

uint8_t dt_imu_mahony_init(dt_imu_mahony_t *imu, gpio_pin_enum scl, gpio_pin_enum sda)
{
    dt_mpu6050_data_t raw;

    memset(imu, 0, sizeof(*imu));
    imu->q0 = 1.0f;

    soft_iic_init(&imu->mpu.iic, DT_MPU6050_DEFAULT_ADDR, 100, scl, sda);
    imu->mpu.accel_fs = DT_MPU6050_ACCEL_FS_2G;
    imu->mpu.gyro_fs  = DT_MPU6050_GYRO_FS_250;
    if (!dt_mpu6050_init(&imu->mpu)) return 0;

    imu_calibrate(imu);

    dt_mpu6050_read_all(&imu->mpu, &raw);
    float ax = raw.ax, ay = raw.ay, az = raw.az;
    float norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= norm; ay *= norm; az *= norm;
    imu->offs_roll  = atan2f(2.0f * (imu->q0 * imu->q1 + imu->q2 * imu->q3),
                             1.0f - 2.0f * (imu->q1 * imu->q1 + imu->q2 * imu->q2)) * 180.0f / PI;

    imu->ready = 1;
    return 1;
}

void dt_imu_mahony_update(dt_imu_mahony_t *imu, float dt)
{
    dt_mpu6050_data_t raw;
    float halfT = 0.5f * dt;

    if (!imu->ready) return;
    dt_mpu6050_read_all(&imu->mpu, &raw);

    imu->acc_lpf_x = raw.ax * ACC_ALPHA + imu->acc_lpf_x * (1.0f - ACC_ALPHA);
    imu->acc_lpf_y = raw.ay * ACC_ALPHA + imu->acc_lpf_y * (1.0f - ACC_ALPHA);
    imu->acc_lpf_z = raw.az * ACC_ALPHA + imu->acc_lpf_z * (1.0f - ACC_ALPHA);

    float gx = (raw.gx - imu->gbias_x) * PI / 180.0f;
    float gy = (raw.gy - imu->gbias_y) * PI / 180.0f;
    float gz = (raw.gz - imu->gbias_z) * PI / 180.0f;

    float ax = imu->acc_lpf_x, ay = imu->acc_lpf_y, az = imu->acc_lpf_z;
    float norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= norm; ay *= norm; az *= norm;

    float q0 = imu->q0, q1 = imu->q1, q2 = imu->q2, q3 = imu->q3;
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    imu->Iex += halfT * ex;
    imu->Iey += halfT * ey;
    imu->Iez += halfT * ez;
    gx += KP * ex + KI * imu->Iex;
    gy += KP * ey + KI * imu->Iey;
    gz += KP * ez + KI * imu->Iez;

    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    imu->q0 = q0 * norm; imu->q1 = q1 * norm;
    imu->q2 = q2 * norm; imu->q3 = q3 * norm;

    imu->roll  = -asinf(-2.0f * imu->q1 * imu->q3 + 2.0f * imu->q0 * imu->q2) * 180.0f / PI;
    imu->pitch = -atan2f(2.0f * imu->q2 * imu->q3 + 2.0f * imu->q0 * imu->q1,
                        -2.0f * imu->q1 * imu->q1 - 2.0f * imu->q2 * imu->q2 + 1.0f) * 180.0f / PI;
    imu->yaw   = -atan2f(2.0f * imu->q1 * imu->q2 + 2.0f * imu->q0 * imu->q3,
                        -2.0f * imu->q2 * imu->q2 - 2.0f * imu->q3 * imu->q3 + 1.0f) * 180.0f / PI;

    imu->roll  -= imu->offs_roll;
    imu->pitch -= imu->offs_pitch;
    imu->yaw   -= imu->offs_yaw;
}

void dt_imu_mahony_zero_yaw(dt_imu_mahony_t *imu)
{
    imu->offs_yaw = imu->yaw + imu->offs_yaw;
}
