#include "driver/dt_heading.h"

#include "config.h"
#include "driver/dt_gyro_z.h"

#include <stddef.h>
#include <string.h>

#define DT_HEADING_M0_SETTLE_MS 200u

#if CAR_GYRO_SOURCE == CAR_GYRO_SOURCE_MPU6050
static dt_heading_status_t dt_heading_mpu_status(
    dt_mpu6050_heading_status_t status)
{
    switch (status)
    {
        case DT_MPU6050_HEADING_BUS_ERROR:
            return DT_HEADING_STATUS_BUS_ERROR;
        case DT_MPU6050_HEADING_ID_ERROR:
            return DT_HEADING_STATUS_ID_ERROR;
        case DT_MPU6050_HEADING_CONFIG_ERROR:
            return DT_HEADING_STATUS_CONFIG_ERROR;
        case DT_MPU6050_HEADING_CALIBRATION_ERROR:
            return DT_HEADING_STATUS_CALIBRATION_ERROR;
        case DT_MPU6050_HEADING_READY:
            return DT_HEADING_STATUS_READY;
        case DT_MPU6050_HEADING_UNINITIALIZED:
        default:
            return DT_HEADING_STATUS_UNINITIALIZED;
    }
}

static void dt_heading_sync_mpu(dt_heading_t *heading)
{
    const dt_mpu6050_heading_t *mpu = &heading->mpu6050;

    heading->status = dt_heading_mpu_status(mpu->status);
    heading->yaw_deg = mpu->yaw_deg;
    heading->wz_dps = mpu->wz_dps;
    heading->sample_count = mpu->sample_count;
    heading->read_error_count = mpu->read_error_count;
    heading->last_update_ms = mpu->last_update_ms;
    heading->address = mpu->address;
    heading->device_id = mpu->who_am_i;
    heading->bus_status = (uint8_t)soft_iic_get_last_error(&mpu->mpu.iic);
    heading->calibration_valid_samples = mpu->calibration_valid_samples;
    heading->gyro_bias_z = mpu->gyro_bias_z;
    heading->calibration_variance_dps2 = mpu->calibration_variance_dps2;
    heading->calibration_min_gz_dps = mpu->calibration_min_gz_dps;
    heading->calibration_max_gz_dps = mpu->calibration_max_gz_dps;
    heading->ax = mpu->last_sample.ax;
    heading->ay = mpu->last_sample.ay;
    heading->az = mpu->last_sample.az;
    heading->gx = mpu->last_sample.gx;
    heading->gy = mpu->last_sample.gy;
    heading->gz = mpu->last_sample.gz;
    heading->temperature = mpu->last_sample.temp;
}
#endif

bool dt_heading_init(dt_heading_t *heading)
{
    if (heading == NULL)
    {
        return false;
    }

    memset(heading, 0, sizeof(*heading));
    heading->source = (dt_heading_source_t)CAR_GYRO_SOURCE;

#if CAR_GYRO_SOURCE == CAR_GYRO_SOURCE_M0
    {
        dt_gyro_z_config_t config;

#if GYRO_Z_TRANSPORT == 1
        config.uart   = GYRO_Z_UART;
        config.baud   = GYRO_Z_BAUD;
        config.tx_pin = GYRO_Z_TX_PIN;
        config.rx_pin = GYRO_Z_RX_PIN;
#else
        config.uart   = 0;
        config.baud   = 0;
        config.tx_pin = 0;
        config.rx_pin = 0;
#endif

        dt_gyro_z_init(&config);
        system_delay_ms(DT_HEADING_M0_SETTLE_MS);
#if GYRO_Z_TRANSPORT == 1
        dt_gyro_z_zero_yaw();
        system_delay_ms(DT_HEADING_M0_SETTLE_MS);
#endif
        heading->initialized = true;
        heading->status = DT_HEADING_STATUS_WAITING_DATA;
        return true;
    }
#else
    heading->initialized = dt_mpu6050_heading_init(&heading->mpu6050,
        MPU6050_SCL, MPU6050_SDA);
    if (heading->initialized)
    {
        dt_mpu6050_heading_zero(&heading->mpu6050);
    }
    dt_heading_sync_mpu(heading);
    return heading->initialized;
#endif
}

bool dt_heading_update(dt_heading_t *heading, uint32_t now_ms)
{
    if (heading == NULL || !heading->initialized)
    {
        return false;
    }

#if CAR_GYRO_SOURCE == CAR_GYRO_SOURCE_M0
    {
        const dt_gyro_z_data_t *data;
        bool updated;

        (void)dt_gyro_z_update(now_ms);
        data = dt_gyro_z_get_data();
        updated = data->yaw_updated != 0u || data->wz_updated != 0u;
        if (data->yaw_updated != 0u)
        {
            heading->yaw_deg = data->yaw_deg;
            heading->last_update_ms = now_ms;
            heading->ready = true;
            heading->status = DT_HEADING_STATUS_READY;
        }
        if (data->wz_updated != 0u)
        {
            heading->wz_dps = data->wz_dps;
        }
        heading->yaw_raw = data->yaw_raw;
        heading->wz_raw = data->wz_raw;
        heading->sample_count = data->frame_count;
        heading->checksum_error_count = data->checksum_error_count;
        heading->read_error_count = data->checksum_error_count;
        heading->rx_overflow = data->rx_overflow;
        return updated;
    }
#else
    if (!dt_mpu6050_heading_update(&heading->mpu6050, now_ms))
    {
        dt_heading_sync_mpu(heading);
        return false;
    }
    dt_heading_sync_mpu(heading);
    heading->ready = true;
    return true;
#endif
}

void dt_heading_zero(dt_heading_t *heading)
{
    if (heading == NULL || !heading->initialized)
    {
        return;
    }

#if CAR_GYRO_SOURCE == CAR_GYRO_SOURCE_M0
    dt_gyro_z_zero_yaw();
#else
    dt_mpu6050_heading_zero(&heading->mpu6050);
#endif
    heading->yaw_deg = 0.0f;
}

bool dt_heading_is_fresh(const dt_heading_t *heading, uint32_t now_ms,
    uint32_t timeout_ms)
{
    return heading != NULL && heading->ready &&
        (uint32_t)(now_ms - heading->last_update_ms) < timeout_ms;
}

const char *dt_heading_source_name(dt_heading_source_t source)
{
    return source == DT_HEADING_SOURCE_M0 ? "M0" :
        (source == DT_HEADING_SOURCE_MPU6050 ? "MPU" : "?");
}

const char *dt_heading_status_name(dt_heading_status_t status)
{
    switch (status)
    {
        case DT_HEADING_STATUS_WAITING_DATA: return "WAIT";
        case DT_HEADING_STATUS_READY: return "READY";
        case DT_HEADING_STATUS_BUS_ERROR: return "BUS";
        case DT_HEADING_STATUS_ID_ERROR: return "ID";
        case DT_HEADING_STATUS_CONFIG_ERROR: return "CFG";
        case DT_HEADING_STATUS_CALIBRATION_ERROR: return "CAL";
        case DT_HEADING_STATUS_UNINITIALIZED:
        default: return "INIT";
    }
}
