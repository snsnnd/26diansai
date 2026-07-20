/**
 * @file dt_mpu6050_heading.c
 * @brief 基于MPU6050的Z轴陀螺仪偏航角推算实现
 *        使用MPU6050内置温度计和陀螺仪，通过积分Z轴角速度获取偏航角。
 *        特点：
 *        - 独立的零偏校准流程（含方差评估）
 *        - 预热处理使传感器稳定后再校准
 *        - 角度积分自动归一化到[-180,180]
 *        - 完整的状态报告机制（便于故障诊断）
 *
 *        局限性：纯陀螺仪积分存在累积漂移，长时间使用精度会下降。
 *        适用于短时（几分钟级别）的航向保持。
 */

#include "driver/dt_mpu6050_heading.h"

#include <stddef.h>
#include <string.h>

#define MPU6050_HEADING_CAL_ATTEMPTS       1000u   /**< 校准采样次数 */
#define MPU6050_HEADING_CAL_MIN_SAMPLES     900u   /**< 有效样本最低要求 */
#define MPU6050_HEADING_CAL_DELAY_MS          2u   /**< 校准采样间隔（ms） */
#define MPU6050_HEADING_SETTLE_DELAY_MS      500u   /**< 传感器稳定等待时间 */
#define MPU6050_HEADING_WARMUP_SAMPLES        32u   /**< 预热样本数 */
#define MPU6050_HEADING_MAX_VARIANCE_DPS2   0.25f  /**< 最大允许的零偏方差（dps^2） */
#define MPU6050_HEADING_MAX_DT_MS             50u   /**< 两次更新允许的最大时间间隔（ms） */

/**
 * @brief 探测MPU6050传感器在指定I2C地址上是否存在
 *        初始化IIC、读取WHO_AM_I寄存器，验证芯片ID的高7位是否为0x68
 * @param heading 偏航角模块结构体指针
 * @param scl IIC SCL引脚
 * @param sda IIC SDA引脚
 * @param address 要探测的I2C地址
 * @return true=探测成功（芯片存在且ID正确），false=探测失败
 */
static bool mpu6050_heading_probe(dt_mpu6050_heading_t *heading,
    gpio_pin_enum scl, gpio_pin_enum sda, uint8_t address)
{
    uint8_t who_am_i = 0u;

    soft_iic_init(&heading->mpu.iic, address, 100u, scl, sda);
    heading->address = address;
    if (!dt_mpu6050_hal_read_reg(&heading->mpu.iic,
        DT_MPU6050_REG_WHO_AM_I, &who_am_i))
    {
        return false; /* IIC通信失败 */
    }
    heading->who_am_i = who_am_i;
    /* 芯片ID的高7位应固定为0x68（即0x68或0x69） */
    return (who_am_i & 0x7Eu) == 0x68u;
}

/**
 * @brief 将角度归一化到[-180, 180]范围
 * @param heading_deg 输入角度（度）
 * @return 归一化后的角度
 */
static float mpu6050_heading_wrap(float heading_deg)
{
    while (heading_deg > 180.0f) heading_deg -= 360.0f;
    while (heading_deg < -180.0f) heading_deg += 360.0f;
    return heading_deg;
}

/**
 * @brief 传感器预热
 *        上电后传感器输出需要一段时间才能稳定。
 *        等待500ms后连续读取32个样本，使传感器进入稳定工作状态。
 * @param heading 偏航角模块结构体指针
 */
static void mpu6050_heading_warm_up(dt_mpu6050_heading_t *heading)
{
    dt_mpu6050_data_t data;
    uint32_t i;

    system_delay_ms(MPU6050_HEADING_SETTLE_DELAY_MS); /* 等待传感器稳定 */
    for (i = 0u; i < MPU6050_HEADING_WARMUP_SAMPLES; i++)
    {
        if (dt_mpu6050_read_all(&heading->mpu, &data))
        {
            heading->last_sample = data; /* 更新最新样本 */
        }
        else
        {
            heading->read_error_count++; /* 记录读取失败 */
        }
        system_delay_ms(MPU6050_HEADING_CAL_DELAY_MS);
    }
}

/**
 * @brief Z轴陀螺仪零偏校准
 *        采集约2秒（1000次*2ms）的静态数据，使用Welford在线算法
 *        计算均值和方差，同时记录最小/最大值用于诊断。
 *        校准要求：
 *        - 有效样本数 >= 900
 *        - 方差 <= 0.25 dps^2（传感器必须绝对静止）
 * @param heading 偏航角模块结构体指针
 * @return true=校准成功，false=校准失败（样本不足或方差过大）
 * @note Welford算法优势：单次遍历，数值稳定性好，不需要存储所有数据
 */
static bool mpu6050_heading_calibrate(dt_mpu6050_heading_t *heading)
{
    dt_mpu6050_data_t data;
    float mean = 0.0f;               /* 运行均值 */
    float sum_squared_deviation = 0.0f; /* 平方偏差和 */
    uint32_t valid_samples = 0u;     /* 有效样本计数 */
    uint32_t i;

    for (i = 0u; i < MPU6050_HEADING_CAL_ATTEMPTS; i++)
    {
        if (dt_mpu6050_read_all(&heading->mpu, &data))
        {
            float delta;

            heading->last_sample = data;
            valid_samples++;

            /* Welford在线方差计算：
             * delta = x - mean_old
             * mean_new = mean_old + delta / n
             * sum_sq = sum_sq + delta * (x - mean_new) */
            delta = data.gz - mean;
            mean += delta / (float)valid_samples;
            sum_squared_deviation += delta * (data.gz - mean);

            /* 记录校准过程中的最小/最大值 */
            if (valid_samples == 1u || data.gz < heading->calibration_min_gz_dps)
            {
                heading->calibration_min_gz_dps = data.gz;
            }
            if (valid_samples == 1u || data.gz > heading->calibration_max_gz_dps)
            {
                heading->calibration_max_gz_dps = data.gz;
            }
        }
        else
        {
            heading->read_error_count++; /* IIC读取失败计数 */
        }
        system_delay_ms(MPU6050_HEADING_CAL_DELAY_MS);
    }

    heading->calibration_valid_samples = valid_samples;
    if (valid_samples == 0u)
    {
        return false; /* 没有有效样本，校准失败 */
    }

    /* 保存校准结果 */
    heading->gyro_bias_z = mean;                      /* Z轴零偏值 */
    heading->calibration_variance_dps2 =
        sum_squared_deviation / (float)valid_samples; /* 方差 */
    if (heading->calibration_variance_dps2 < 0.0f)
    {
        heading->calibration_variance_dps2 = 0.0f;   /* 防止浮点误差导致负值 */
    }

    /* 判断校准是否合格 */
    return valid_samples >= MPU6050_HEADING_CAL_MIN_SAMPLES &&
        heading->calibration_variance_dps2 <=
            MPU6050_HEADING_MAX_VARIANCE_DPS2;
}

/**
 * @brief 初始化偏航角推算模块
 *
 * 完整流程：
 *   1. 清空结构体
 *   2. 尝试默认地址(0x68)探测传感器
 *   3. 若默认地址无响应，尝试备用地址(0x69)
 *   4. 初始化传感器（500dps陀螺量程，采样率=8kHz/(1+9)=800Hz，DLPF=3≈44Hz带宽）
 *   5. 传感器预热
 *   6. 执行零偏校准
 *   7. 标记就绪
 *
 * @param heading 偏航角模块结构体指针
 * @param scl IIC SCL引脚
 * @param sda IIC SDA引脚
 * @return true=初始化成功，false=失败（通过heading->status查看原因）
 */
bool dt_mpu6050_heading_init(dt_mpu6050_heading_t *heading,
    gpio_pin_enum scl, gpio_pin_enum sda)
{
    if (heading == NULL)
    {
        return false;
    }

    memset(heading, 0, sizeof(*heading));
    heading->status = DT_MPU6050_HEADING_BUS_ERROR;

    /* 探测传感器：先试默认地址0x68，再试0x69 */
    if (!mpu6050_heading_probe(heading, scl, sda, DT_MPU6050_DEFAULT_ADDR))
    {
        uint8_t first_who_am_i = heading->who_am_i;

        /* 尝试备用地址（AD0接VCC时为0x69） */
        if (!mpu6050_heading_probe(heading, scl, sda,
            (uint8_t)(DT_MPU6050_DEFAULT_ADDR + 1u)))
        {
            /* 两个地址都探测失败，且至少有一个地址有WHO_AM_I响应
             * 说明IIC通信正常但ID不对，可能是其他设备 */
            if (first_who_am_i != 0u || heading->who_am_i != 0u)
            {
                heading->status = DT_MPU6050_HEADING_ID_ERROR;
            }
            return false;
        }
    }

    /* 配置传感器参数 */
    heading->mpu.accel_fs = DT_MPU6050_ACCEL_FS_2G;  /* ±2g（加速度计在本模块中未使用） */
    heading->mpu.gyro_fs = DT_MPU6050_GYRO_FS_500;   /* ±500°/s */

    /* DLPF=3 提供44Hz陀螺仪带宽；分频9使输出速率约为100Hz
     * (实际采样率=内部8kHz/(1+9)=800Hz，但DLPF限制了带宽) */
    if (!dt_mpu6050_hal_init(&heading->mpu.iic, 9u, 3u,
        heading->mpu.gyro_fs, heading->mpu.accel_fs))
    {
        heading->status = DT_MPU6050_HEADING_CONFIG_ERROR;
        return false;
    }

    mpu6050_heading_warm_up(heading);  /* 传感器预热 */
    if (!mpu6050_heading_calibrate(heading))  /* 零偏校准 */
    {
        heading->status = DT_MPU6050_HEADING_CALIBRATION_ERROR;
        return false;
    }

    heading->ready = true;
    heading->status = DT_MPU6050_HEADING_READY;
    return true;
}

/**
 * @brief 读取一次MPU6050样本并保存到last_sample
 * @param heading 偏航角模块结构体指针
 * @return true=读取成功，false=读取失败
 */
bool dt_mpu6050_heading_read_sample(dt_mpu6050_heading_t *heading)
{
    dt_mpu6050_data_t data;

    if (heading == NULL)
    {
        return false;
    }
    if (!dt_mpu6050_read_all(&heading->mpu, &data))
    {
        heading->read_error_count++; /* IIC读取失败计数 */
        return false;
    }

    heading->last_sample = data;   /* 保存最新样本 */
    heading->sample_count++;       /* 总样本数递增 */
    return true;
}

/**
 * @brief 更新偏航角积分
 *        读取最新样本，计算去零偏后的Z轴角速度，
 *        通过角速度*时间间隔积分得到偏航角变化量，累加到当前偏航角上。
 *        首次调用时仅记录时间戳，不进行积分。
 * @param heading 偏航角模块结构体指针
 * @param now_ms 当前系统时间戳（毫秒）
 * @return true=成功，false=模块未就绪或读取失败
 * @note 如果两次更新的间隔超过MPU6050_HEADING_MAX_DT_MS，
 *       跳过本次积分以免长时间间隔导致积分误差过大。
 */
bool dt_mpu6050_heading_update(dt_mpu6050_heading_t *heading,
    uint32_t now_ms)
{
    uint32_t dt_ms;

    if (heading == NULL || !heading->ready)
    {
        return false;
    }
    if (!dt_mpu6050_heading_read_sample(heading)) return false;

    /* 去零偏后的Z轴角速度（度/秒） */
    heading->wz_dps = heading->last_sample.gz - heading->gyro_bias_z;

    /* 首次更新：只记录时间戳，不积分 */
    if (heading->last_update_ms == 0u)
    {
        heading->last_update_ms = now_ms;
        return true;
    }

    /* 计算时间间隔并积分 */
    dt_ms = now_ms - heading->last_update_ms;
    heading->last_update_ms = now_ms;

    /* 检查时间间隔是否合理（防止长间隔产生的积分误差） */
    if (dt_ms > 0u && dt_ms <= MPU6050_HEADING_MAX_DT_MS)
    {
        /* 角度 = 角速度 * 时间(秒)，再归一化到[-180,180] */
        heading->yaw_deg = mpu6050_heading_wrap(heading->yaw_deg +
            heading->wz_dps * (float)dt_ms / 1000.0f);
    }
    /* 如果dt_ms > MAX_DT_MS，跳过本次积分（可能是长时间暂停后恢复） */
    return true;
}

/**
 * @brief 偏航角归零
 *        将当前偏航角设为0°，但陀螺仪积分漂移仍然存在，
 *        后续角度将从新的零点开始累计。
 * @param heading 偏航角模块结构体指针
 */
void dt_mpu6050_heading_zero(dt_mpu6050_heading_t *heading)
{
    if (heading != NULL)
    {
        heading->yaw_deg = 0.0f; /* 直接将偏航角清零 */
    }
}
