#ifndef E2025_TASK_H
#define E2025_TASK_H

#include "framework/ec_mode_manager.h"
#include "device/t8_road_detector.h"
#include "gimbal/foc_protocol.h"
#include "gimbal/maixcam2_protocol.h"

#include <stdbool.h>
#include <stdint.h>

/* ==================== 各任务超时时间 ==================== */
#define E2025_B1_TIMEOUT_MS 190000u
#define E2025_B2_TIMEOUT_MS 190000u
#define E2025_B3_TIMEOUT_MS 190000u
#define E2025_A1_TIMEOUT_MS 190000u
#define E2025_A2_TIMEOUT_MS 190000u
#define E2025_A3_TIMEOUT_MS 190000u

/* ==================== 默认圈数 ==================== */
#define E2025_B1_DEFAULT_LAPS  1u
#define E2025_A1_DEFAULT_LAPS  1u
#define E2025_A2_DEFAULT_LAPS  2u
#define E2025_A3_DEFAULT_LAPS  1u

/* ==================== 赛道参数 ==================== */
#define E2025_TRACK_PERIMETER_MM      4000u
#define E2025_HEADING_PER_LAP_DEG     360.0f

/* ==================== 目标靶参数 ==================== */
#define E2025_TARGET_DISTANCE_CM      50.0f
#define E2025_TARGET_CIRCLE_RADIUS_CM 6.0f

/* ==================== 瞄准控制参数 ==================== */
#define E2025_B2_AIM_TIMEOUT_MS       2000u
#define E2025_B3_AIM_TIMEOUT_MS       4000u
#define E2025_AIM_FRESHNESS_MS        150u
#define E2025_AIM_LOCK_SAMPLES        10u
#define E2025_AIM_ERROR_THRESHOLD_PX  8u
#define E2025_AIM_PIXEL_TO_DEG        0.05f
#define E2025_AIM_KP                  0.8f
#define E2025_AIM_KD                  0.1f

/* ==================== T8路段识别 ==================== */
#define E2025_TURN_CONFIRM_SAMPLES     3u
#define E2025_TURN_HALF_BLACK_MIN      3u
#define E2025_TURN_OTHER_HALF_MAX      1u
#define E2025_TURN_ANGLE_DEG           90.0f

/* ==================== 竞赛任务ID ==================== */
typedef enum
{
    E2025_BASE_TASK_1 = 0,
    E2025_BASE_TASK_2,
    E2025_BASE_TASK_3,
    E2025_ADV_TASK_1,
    E2025_ADV_TASK_2,
    E2025_ADV_TASK_3,
    E2025_TASK_COUNT
} e2025_task_id_t;

/* ==================== 任务状态机 ==================== */
typedef enum
{
    E2025_STATE_STOPPED = 0,
    E2025_STATE_INIT,
    E2025_STATE_LINE_FOLLOW,
    E2025_STATE_AIMING,
    E2025_STATE_LINE_AIMING,
    E2025_STATE_CIRCLE_SYNC,
    E2025_STATE_DONE,
    E2025_STATE_TIMEOUT,
    E2025_STATE_FAULT
} e2025_task_state_t;

/* ==================== 车体姿态快照 ==================== */
typedef struct
{
    uint32_t timestamp_ms;
    float    yaw_deg;
    float    pitch_deg;
    float    roll_deg;
    float    wz_dps;
    float    heading_accum_deg;
    bool     on_line;
    float    travel_mm;
    float    travel_global_mm;
    uint32_t line_enter_count;
    uint32_t line_exit_count;
    uint32_t lap_count_dist;
    uint32_t lap_count_heading;
    uint32_t lap_count;
} e2025_vehicle_state_t;

/* ==================== 云台电机类型 ==================== */
typedef enum
{
    E2025_GIMBAL_NONE = 0,
    E2025_GIMBAL_EMM_STEPPER,
    E2025_GIMBAL_FOC_BRUSHLESS,
} e2025_gimbal_type_t;

/* ==================== 公共API ==================== */
bool               e2025_tasks_register(ec_mode_manager_t *manager);
bool               e2025_tasks_is_active(const ec_mode_manager_t *manager);
e2025_task_state_t e2025_tasks_active_state(const ec_mode_manager_t *manager);
const char        *e2025_tasks_active_status(const ec_mode_manager_t *manager);

void ec_vision_lazy_init(void);
void ec_foc_gimbal_lazy_init(void);

#endif
