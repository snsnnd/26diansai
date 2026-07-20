#ifndef DT_HEADING_H
#define DT_HEADING_H

#include "driver/dt_mpu6050_heading.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DT_HEADING_SOURCE_M0 = 1,
    DT_HEADING_SOURCE_MPU6050 = 2
} dt_heading_source_t;

typedef enum
{
    DT_HEADING_STATUS_UNINITIALIZED = 0,
    DT_HEADING_STATUS_WAITING_DATA,
    DT_HEADING_STATUS_READY,
    DT_HEADING_STATUS_BUS_ERROR,
    DT_HEADING_STATUS_ID_ERROR,
    DT_HEADING_STATUS_CONFIG_ERROR,
    DT_HEADING_STATUS_CALIBRATION_ERROR
} dt_heading_status_t;

typedef struct
{
    dt_heading_source_t source;
    dt_heading_status_t status;
    bool initialized;
    bool ready;
    float yaw_deg;
    float wz_dps;
    uint32_t last_update_ms;
    uint32_t sample_count;
    uint32_t read_error_count;
    uint32_t checksum_error_count;
    uint32_t rx_overflow;
    int16_t yaw_raw;
    int16_t wz_raw;

    uint8_t address;
    uint8_t device_id;
    uint8_t bus_status;
    uint32_t calibration_valid_samples;
    float gyro_bias_z;
    float calibration_variance_dps2;
    float calibration_min_gz_dps;
    float calibration_max_gz_dps;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float temperature;

    dt_mpu6050_heading_t mpu6050;
} dt_heading_t;

bool dt_heading_init(dt_heading_t *heading);
bool dt_heading_update(dt_heading_t *heading, uint32_t now_ms);
void dt_heading_zero(dt_heading_t *heading);
bool dt_heading_is_fresh(const dt_heading_t *heading, uint32_t now_ms,
    uint32_t timeout_ms);
const char *dt_heading_source_name(dt_heading_source_t source);
const char *dt_heading_status_name(dt_heading_status_t status);

#endif
