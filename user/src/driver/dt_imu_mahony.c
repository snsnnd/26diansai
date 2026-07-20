/**
 * @file dt_imu_mahony.c
 * @brief Mahony互补滤波姿态解算算法实现
 *        融合MPU6050加速度计和陀螺仪数据，通过四元数形式的
 *        Mahony算法（非线性互补滤波器）估算载体姿态。
 *
 * 算法原理：
 *   1. 加速度计提供重力参考方向（长期稳定，动态响应慢）
 *   2. 陀螺仪提供角速度积分（短期精确，长期漂移）
 *   3. Mahony滤波器用加速度计估计的误差修正陀螺仪的积分漂移
 *   4. PI控制器（KP, KI）控制修正速度和稳态误差
 *
 * 坐标系：右-前-上 (x-right, y-forward, z-up)
 * 欧拉角定义：roll(绕x), pitch(绕y), yaw(绕z)
 */

#include "driver/dt_imu_mahony.h"
#include <math.h>

#define PI         3.1415926f   /* 圆周率 */
#define KP         0.5f         /* Mahony滤波器比例增益——控制收敛速度 */
#define KI         0.01f        /* Mahony滤波器积分增益——消除稳态漂移 */
#define ACC_ALPHA  0.3f         /* 加速度低通滤波系数（值越小滤波越强） */

/* 是否使用硬编码的陀螺仪零偏值（1=使用，0=上电实时校准） */
#define IMU_USE_HARDCODED_BIAS 1

#if IMU_USE_HARDCODED_BIAS
/* 通过多次实验测量得到的陀螺仪零偏值（单位：原始ADC值） */
static const float g_bias_x = -1.69f;
static const float g_bias_y = 0.32f;
static const float g_bias_z = 0.13f;
#endif

/**
 * @brief 计算平方根的倒数（带零值保护）
 * @param num 输入浮点数
 * @return 1/sqrt(num)，若num接近0则返回0
 */
static float inv_sqrt(float num)
{
    return (num > 1.0e-12f) ? (1.0f / sqrtf(num)) : 0.0f;
}

/**
 * @brief 将数值钳位到[-1, 1]范围内（用于反三角函数输入保护）
 * @param value 输入浮点数
 * @return 钳位后的值
 */
static float clamp_unit(float value)
{
    if (value > 1.0f) return 1.0f;
    if (value < -1.0f) return -1.0f;
    return value;
}

/**
 * @brief 陀螺仪零偏校准
 *        使用硬编码值时直接赋值；
 *        使用实时校准时采集5000个静态样本求平均值。
 * @param imu Mahony结构体指针
 */
static void imu_calibrate(dt_imu_mahony_t *imu)
{
#if IMU_USE_HARDCODED_BIAS
    /* 使用预测量好的零偏值，省去每次上电校准的时间 */
    imu->gbias_x = g_bias_x;
    imu->gbias_y = g_bias_y;
    imu->gbias_z = g_bias_z;
    printf("[IMU] bias: hardcoded\r\n");
#else
    /* 动态校准：采集5000个样本（约10秒），求平均值作为零偏 */
    dt_mpu6050_data_t raw;
    float gx = 0, gy = 0, gz = 0;

    printf("[IMU] calibrating 5000 samples...\r\n");
    for (int i = 0; i < 5000; i++)
    {
        dt_mpu6050_read_all(&imu->mpu, &raw);
        gx += raw.gx; gy += raw.gy; gz += raw.gz;
        system_delay_ms(2); /* 2ms间隔对应约500Hz采样率 */
    }
    imu->gbias_x = gx / 5000.0f;
    imu->gbias_y = gy / 5000.0f;
    imu->gbias_z = gz / 5000.0f;

    printf("[IMU] bias5000: gx=%d gy=%d gz=%d (x100)\r\n",
           (int)(imu->gbias_x * 100), (int)(imu->gbias_y * 100), (int)(imu->gbias_z * 100));
#endif
}

/**
 * @brief 初始化Mahony姿态解算模块
 *        步骤：
 *        1. 清除结构体，设置初始四元数 q0=1（单位四元数，表示无旋转）
 *        2. 初始化软件IIC及MPU6050（2g加速度量程，250dps陀螺量程）
 *        3. 执行陀螺仪零偏校准
 *        4. 读取一次加速度计数据，初始化低通滤波器
 *        5. 计算初始roll偏移量
 * @param imu Mahony结构体指针
 * @param scl IIC SCL引脚
 * @param sda IIC SDA引脚
 * @return 1=成功，0=失败
 */
uint8_t dt_imu_mahony_init(dt_imu_mahony_t *imu, gpio_pin_enum scl, gpio_pin_enum sda)
{
    dt_mpu6050_data_t raw;

    memset(imu, 0, sizeof(*imu));
    imu->q0 = 1.0f;  /* 初始四元数：无旋转 */

    /* 初始化IIC和MPU6050配置 */
    soft_iic_init(&imu->mpu.iic, DT_MPU6050_DEFAULT_ADDR, 100, scl, sda);
    imu->mpu.accel_fs = DT_MPU6050_ACCEL_FS_2G;   /* 加速度量程 ±2g */
    imu->mpu.gyro_fs  = DT_MPU6050_GYRO_FS_250;   /* 陀螺仪量程 ±250°/s */
    if (!dt_mpu6050_init(&imu->mpu)) return 0;     /* 初始化传感器 */

    imu_calibrate(imu);  /* 陀螺仪零偏校准 */

    /* 读取第一帧加速度数据，初始化低通滤波器状态 */
    if (!dt_mpu6050_read_all(&imu->mpu, &raw)) return 0;
    imu->acc_lpf_x = raw.ax;
    imu->acc_lpf_y = raw.ay;
    imu->acc_lpf_z = raw.az;

    /* 计算初始roll偏移量（用于补偿安装偏差） */
    imu->offs_roll  = atan2f(2.0f * (imu->q0 * imu->q1 + imu->q2 * imu->q3),
                             1.0f - 2.0f * (imu->q1 * imu->q1 + imu->q2 * imu->q2)) * 180.0f / PI;

    imu->ready = 1;   /* 标记初始化完成 */
    return 1;
}

/**
 * @brief Mahony互补滤波姿态解算更新
 *
 * 算法流程：
 *   1. 读取MPU6050原始数据
 *   2. 加速度一阶低通滤波（滤除高频振动噪声）
 *   3. 陀螺仪减零偏并转换为弧度/秒
 *   4. 加速度归一化，估计重力方向在机体坐标系中的投影
 *   5. 叉积计算加速度计估计方向与实际方向的误差
 *   6. PI控制器修正陀螺仪角速度（消除漂移）
 *   7. 四元数微分方程更新
 *   8. 四元数归一化
 *   9. 四元数 -> 欧拉角转换
 *
 * @param imu Mahony结构体指针
 * @param dt  距离上次更新的时间间隔（秒）
 * @note dt 应控制在0.001~0.1s范围内。过大则精度下降，过小则积分噪声增大。
 *       使用-180~180度范围的角度表示。
 */
void dt_imu_mahony_update(dt_imu_mahony_t *imu, float dt)
{
    dt_mpu6050_data_t raw;
    float halfT = 0.5f * dt;  /* 四元数微分步长 = dt/2 */

    /* 参数有效性检查 */
    if (!imu->ready || dt <= 0.0f || dt > 0.1f) return;
    if (!dt_mpu6050_read_all(&imu->mpu, &raw)) return;

    /* ========== 1. 加速度一阶低通滤波 ==========
     * 公式：filtered = raw * alpha + filtered_prev * (1-alpha)
     * 用于抑制电机振动等高频噪声对加速度计的影响 */
    imu->acc_lpf_x = raw.ax * ACC_ALPHA + imu->acc_lpf_x * (1.0f - ACC_ALPHA);
    imu->acc_lpf_y = raw.ay * ACC_ALPHA + imu->acc_lpf_y * (1.0f - ACC_ALPHA);
    imu->acc_lpf_z = raw.az * ACC_ALPHA + imu->acc_lpf_z * (1.0f - ACC_ALPHA);

    /* ========== 2. 陀螺仪去零偏并转弧度 ==========
     * 原始陀螺仪值：(raw - bias) * PI/180 转换为弧度/秒 */
    float gx = (raw.gx - imu->gbias_x) * PI / 180.0f;
    float gy = (raw.gy - imu->gbias_y) * PI / 180.0f;
    float gz = (raw.gz - imu->gbias_z) * PI / 180.0f;

    /* ========== 3. 加速度归一化 ==========
     * 归一化后的加速度向量表示重力方向在机体坐标系中的观测值 */
    float ax = imu->acc_lpf_x, ay = imu->acc_lpf_y, az = imu->acc_lpf_z;
    float norm = inv_sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;  /* 防止除零 */
    ax *= norm; ay *= norm; az *= norm;

    /* ========== 4. 估计重力方向 ==========
     * 从当前四元数估计重力向量在机体坐标系中的投影（vx, vy, vz）
     * 旋转矩阵第三列：将世界坐标系Z轴(0,0,1)转换到机体坐标系 */
    float q0 = imu->q0, q1 = imu->q1, q2 = imu->q2, q3 = imu->q3;
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* ========== 5. 计算误差 ==========
     * 加速度计观测到的重力方向(ax,ay,az)与四元数估计的重力方向(vx,vy,vz)的叉积
     * 叉积大小正比于两个向量间的角度误差，用于修正陀螺仪漂移 */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    /* ========== 6. PI控制器 ==========
     * 积分项累积误差（消除稳态漂移），P项响应瞬时误差
     * 修正后的角速度 = 原始角速度 + KP*误差 + KI*积分误差 */
    imu->Iex += halfT * ex;
    imu->Iey += halfT * ey;
    imu->Iez += halfT * ez;
    gx += KP * ex + KI * imu->Iex;
    gy += KP * ey + KI * imu->Iey;
    gz += KP * ez + KI * imu->Iez;

    /* ========== 7. 四元数微分方程更新 ==========
     * dq/dt = 0.5 * Omega * q，其中Omega是由角速度构成的斜对称矩阵
     * 一阶龙格-库塔数值积分 */
    {
        float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
        float dq1 = ( q0 * gx + q2 * gz - q3 * gy) * halfT;
        float dq2 = ( q0 * gy - q1 * gz + q3 * gx) * halfT;
        float dq3 = ( q0 * gz + q1 * gy - q2 * gx) * halfT;
        q0 += dq0;
        q1 += dq1;
        q2 += dq2;
        q3 += dq3;
    }

    /* ========== 8. 四元数归一化 ==========
     * 数值积分会导致四元数失去单位模长，必须归一化以保持正交性 */
    norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm == 0.0f) return;
    imu->q0 = q0 * norm; imu->q1 = q1 * norm;
    imu->q2 = q2 * norm; imu->q3 = q3 * norm;

    /* ========== 9. 四元数 -> 欧拉角 ==========
     * 使用Z-Y-X旋转顺序（偏航-俯仰-横滚）的转换公式
     * 符号取反是因为坐标系定义差异 */
    imu->roll  = -asinf(clamp_unit(-2.0f * imu->q1 * imu->q3 + 2.0f * imu->q0 * imu->q2)) * 180.0f / PI;
    imu->pitch = -atan2f(2.0f * imu->q2 * imu->q3 + 2.0f * imu->q0 * imu->q1,
                        -2.0f * imu->q1 * imu->q1 - 2.0f * imu->q2 * imu->q2 + 1.0f) * 180.0f / PI;
    imu->yaw   = -atan2f(2.0f * imu->q1 * imu->q2 + 2.0f * imu->q0 * imu->q3,
                        -2.0f * imu->q2 * imu->q2 - 2.0f * imu->q3 * imu->q3 + 1.0f) * 180.0f / PI;

    /* 减去偏移量（安装偏差补偿和偏航清零） */
    imu->roll  -= imu->offs_roll;
    imu->pitch -= imu->offs_pitch;
    imu->yaw   -= imu->offs_yaw;
}

/**
 * @brief 将偏航角当前值设为新的零偏航基准
 *        实质是将当前yaw值累加到offs_yaw中
 * @param imu Mahony结构体指针
 */
void dt_imu_mahony_zero_yaw(dt_imu_mahony_t *imu)
{
    /* offs_yaw = yaw当前值 + 原来的offs_yaw
     * 这样yaw - offs_yaw 就等于0了 */
    imu->offs_yaw = imu->yaw + imu->offs_yaw;
}
