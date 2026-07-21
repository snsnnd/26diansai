#include "app/e2025_task.h"
#include "app/line_car.h"
#include "gimbal/foc_gimbal.h"
#include "gimbal/maixcam2_protocol.h"

#include <math.h>
#include <stddef.h>

typedef struct
{
    e2025_task_id_t    id;
    e2025_task_state_t state;
    ec_mode_manager_t *manager;
    uint32_t           deadline_ms;
    uint32_t           lap_target;
    uint32_t           lap_current;
    uint32_t           state_enter_ms;
    uint32_t           aim_start_ms;
    float              segment_start_travel_mm;
    float              travel_global_mm;
    float              circle_phase_deg;
    float              heading_accum_deg;
    float              prev_heading_deg;
    bool               prev_heading_valid;
    bool               performing_turn;
    float              turn_target_heading;
    t8_road_detector_t road_det;
    float              aim_yaw_deg;
    float              aim_pitch_deg;
    float              aim_prev_error_x;
    float              aim_prev_error_y;
    uint32_t           aim_locked_samples;
    bool               aim_locked;
    bool               is_circle_task;
} e2025_task_context_t;

static e2025_task_context_t g_e2025_tasks[E2025_TASK_COUNT];

static const char *const g_e2025_task_names[E2025_TASK_COUNT] = {
    "2025 EB1",
    "2025 EB2",
    "2025 EB3",
    "2025 EA1",
    "2025 EA2",
    "2025 EA3",
};

/* ==================== 辅助函数 ==================== */

static uint32_t e2025_task_timeout_ms(e2025_task_id_t id)
{
    switch (id)
    {
        case E2025_BASE_TASK_1: return E2025_B1_TIMEOUT_MS;
        case E2025_BASE_TASK_2: return E2025_B2_TIMEOUT_MS;
        case E2025_BASE_TASK_3: return E2025_B3_TIMEOUT_MS;
        case E2025_ADV_TASK_1:  return E2025_A1_TIMEOUT_MS;
        case E2025_ADV_TASK_2:  return E2025_A2_TIMEOUT_MS;
        case E2025_ADV_TASK_3:  return E2025_A3_TIMEOUT_MS;
        default:                return E2025_B1_TIMEOUT_MS;
    }
}

static uint32_t e2025_task_lap_target(e2025_task_id_t id)
{
    switch (id)
    {
        case E2025_BASE_TASK_1: return E2025_B1_DEFAULT_LAPS;
        case E2025_ADV_TASK_1:  return E2025_A1_DEFAULT_LAPS;
        case E2025_ADV_TASK_2:  return E2025_A2_DEFAULT_LAPS;
        case E2025_ADV_TASK_3:  return E2025_A3_DEFAULT_LAPS;
        default:                return 1u;
    }
}

static bool e2025_task_needs_gimbal(e2025_task_id_t id)
{
    switch (id)
    {
        case E2025_BASE_TASK_1: return false;
        default:                return true;
    }
}

static bool e2025_task_is_aim_only(e2025_task_id_t id)
{
    return (id == E2025_BASE_TASK_2 || id == E2025_BASE_TASK_3);
}

static bool e2025_task_is_circle_sync(e2025_task_id_t id)
{
    return (id == E2025_ADV_TASK_3);
}

static e2025_task_state_t e2025_get_running_state(e2025_task_id_t id)
{
    if (e2025_task_is_aim_only(id))
    {
        return E2025_STATE_AIMING;
    }
    if (e2025_task_is_circle_sync(id))
    {
        return E2025_STATE_CIRCLE_SYNC;
    }
    if (id == E2025_BASE_TASK_1)
    {
        return E2025_STATE_LINE_FOLLOW;
    }
    return E2025_STATE_LINE_AIMING;
}

static void e2025_set_fault(e2025_task_context_t *task,
    e2025_task_state_t state, uint32_t now_ms)
{
    car_stop(now_ms);
    task->state = state;
    if (task->manager != NULL)
    {
        task->manager->state = EC_MODE_FAULT;
    }
}

/* ==================== 航向积分 ==================== */

static float e2025_unwrap_heading_delta(float current, float previous)
{
    float delta = current - previous;

    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }
    return delta;
}

static void e2025_heading_accum_reset(e2025_task_context_t *task)
{
    task->heading_accum_deg  = 0.0f;
    task->prev_heading_deg   = 0.0f;
    task->prev_heading_valid = false;
}

static void e2025_heading_accum_update(e2025_task_context_t *task,
    float current_heading_deg)
{
    float delta;

    if (!task->prev_heading_valid)
    {
        task->prev_heading_deg   = current_heading_deg;
        task->heading_accum_deg  = 0.0f;
        task->prev_heading_valid = true;
        return;
    }

    delta = e2025_unwrap_heading_delta(current_heading_deg,
                                       task->prev_heading_deg);
    task->heading_accum_deg += delta;
    task->prev_heading_deg   = current_heading_deg;
}

/* ==================== 圈数追踪 (AND 逻辑) ==================== */

static void e2025_update_lap_count(e2025_task_context_t *task,
    const e2025_vehicle_state_t *state)
{
    uint32_t min_lap;

    if (state->lap_count_dist <= state->lap_count_heading)
    {
        min_lap = state->lap_count_dist;
    }
    else
    {
        min_lap = state->lap_count_heading;
    }

    if (min_lap > task->lap_current)
    {
        task->lap_current = min_lap;
    }
}

/* ==================== 车辆状态读取 (含姿态) ==================== */

static void e2025_read_vehicle_pose(e2025_vehicle_state_t *state,
    e2025_task_context_t *task, uint32_t now_ms)
{
    h2024_vehicle_state_t h_state;

    (void)car_read_state(&h_state, now_ms);

    state->timestamp_ms     = now_ms;
    state->yaw_deg          = h_state.heading_deg;
    state->pitch_deg        = 0.0f;
    state->roll_deg         = 0.0f;
    state->wz_dps           = 0.0f;
    state->on_line          = h_state.on_line;
    state->travel_mm        = h_state.travel_mm;
    state->travel_global_mm = task->travel_global_mm;
    state->line_enter_count = h_state.line_enter_count;
    state->line_exit_count  = h_state.line_exit_count;

    state->heading_accum_deg = task->heading_accum_deg;
    state->lap_count_dist    = (uint32_t)(state->travel_global_mm
                                / (float)E2025_TRACK_PERIMETER_MM);
    state->lap_count_heading = (uint32_t)(fabsf(task->heading_accum_deg)
                                / E2025_HEADING_PER_LAP_DEG);
    state->lap_count         = task->lap_current;
}

static bool e2025_read_vehicle_state(e2025_task_context_t *task,
    e2025_vehicle_state_t *state, uint32_t now_ms)
{
    h2024_vehicle_state_t h_state;
    float ax, ay, az;
    float az_abs;

    if (state == NULL)
    {
        e2025_set_fault(task, E2025_STATE_FAULT, now_ms);
        return false;
    }

    if (!car_read_state(&h_state, now_ms))
    {
        e2025_set_fault(task, E2025_STATE_FAULT, now_ms);
        return false;
    }

    e2025_heading_accum_update(task, h_state.heading_deg);

    car_get_imu_accel(&ax, &ay, &az);
    az_abs = fabsf(az);

    state->yaw_deg           = h_state.heading_deg;
    state->pitch_deg         = (az_abs > 0.01f)
                             ? atan2f(-ax, az) * 57.29578f : 0.0f;
    state->roll_deg          = (az_abs > 0.01f)
                             ? atan2f(ay, az) * 57.29578f : 0.0f;
    state->wz_dps            = car_get_wz_dps();
    state->heading_accum_deg = task->heading_accum_deg;
    state->on_line           = h_state.on_line;
    state->travel_mm         = h_state.travel_mm;
    state->travel_global_mm  = task->travel_global_mm;
    state->line_enter_count  = h_state.line_enter_count;
    state->line_exit_count   = h_state.line_exit_count;
    state->lap_count_dist    = (uint32_t)(state->travel_global_mm
                                / (float)E2025_TRACK_PERIMETER_MM);
    state->lap_count_heading = (uint32_t)(fabsf(task->heading_accum_deg)
                                / E2025_HEADING_PER_LAP_DEG);
    state->lap_count         = task->lap_current;

    return true;
}

static void e2025_complete(e2025_task_context_t *task, uint32_t now_ms)
{
    car_stop(now_ms);
    task->performing_turn = false;
    task->aim_locked = false;
    if (e2025_task_needs_gimbal(task->id))
    {
        (void)foc_gimbal_stop(&g_foc_gimbal);
    }
    task->state = E2025_STATE_DONE;
    (void)now_ms;
}

static void e2025_enter_state(e2025_task_context_t *task,
    e2025_task_state_t new_state, uint32_t now_ms)
{
    task->state = new_state;
    task->state_enter_ms = now_ms;
}

/* ==================== FOC 瞄准控制 (视觉闭环) ==================== */

static void e2025_aim_start(e2025_task_context_t *task, uint32_t now_ms)
{
    (void)foc_gimbal_enable(&g_foc_gimbal, true);
    (void)foc_gimbal_move_to(&g_foc_gimbal, 0.0f, 0.0f);
    task->aim_yaw_deg       = 0.0f;
    task->aim_pitch_deg     = 0.0f;
    task->aim_prev_error_x  = 0.0f;
    task->aim_prev_error_y  = 0.0f;
    task->aim_locked_samples = 0u;
    task->aim_locked        = false;
    task->aim_start_ms      = now_ms;
}

static bool e2025_aim_read_target(MaixVisionTarget *target,
    uint32_t *rx_time_ms)
{
    return maixcam2_get_latest_target(target, rx_time_ms);
}

static void e2025_aim_update(e2025_task_context_t *task, uint32_t now_ms)
{
    MaixVisionTarget target;
    uint32_t         rx_time;
    float            error_dx;
    float            error_dy;
    float            delta_yaw;
    float            delta_pitch;

    if (!e2025_aim_read_target(&target, &rx_time))
    {
        return;
    }

    if (now_ms - rx_time > E2025_AIM_FRESHNESS_MS)
    {
        return;
    }

    if (!target.target_valid
        || !maixcam2_target_semantically_valid(&target))
    {
        task->aim_locked_samples = 0u;
        task->aim_locked         = false;
        return;
    }

    error_dx   = (float)target.error_x;
    error_dy   = (float)target.error_y;
    delta_yaw  = error_dx * E2025_AIM_PIXEL_TO_DEG * E2025_AIM_KP
               + (error_dx - task->aim_prev_error_x) * E2025_AIM_PIXEL_TO_DEG
               * E2025_AIM_KD;
    delta_pitch = error_dy * E2025_AIM_PIXEL_TO_DEG * E2025_AIM_KP
                + (error_dy - task->aim_prev_error_y) * E2025_AIM_PIXEL_TO_DEG
                * E2025_AIM_KD;

    task->aim_yaw_deg   += delta_yaw;
    task->aim_pitch_deg += delta_pitch;

    task->aim_prev_error_x = error_dx;
    task->aim_prev_error_y = error_dy;

    (void)foc_gimbal_move_to_fast(&g_foc_gimbal, task->aim_yaw_deg,
                                  task->aim_pitch_deg);

    if ((uint32_t)fabsf(error_dx) < E2025_AIM_ERROR_THRESHOLD_PX
        && (uint32_t)fabsf(error_dy) < E2025_AIM_ERROR_THRESHOLD_PX)
    {
        task->aim_locked_samples++;
        if (task->aim_locked_samples >= E2025_AIM_LOCK_SAMPLES)
        {
            task->aim_locked = true;
        }
    }
    else
    {
        task->aim_locked_samples = 0u;
        task->aim_locked         = false;
    }
}

/* ==================== 画圆控制 (正确角度换算) ==================== */

static void e2025_circle_update(e2025_task_context_t *task,
    float progress, uint32_t now_ms)
{
    float angle_rad;
    float yaw_deg;
    float pitch_deg;

    angle_rad = progress * 2.0f * 3.14159f;
    yaw_deg   = atanf(E2025_TARGET_CIRCLE_RADIUS_CM * cosf(angle_rad)
                / E2025_TARGET_DISTANCE_CM) * 57.29578f;
    pitch_deg = atanf(E2025_TARGET_CIRCLE_RADIUS_CM * sinf(angle_rad)
                / E2025_TARGET_DISTANCE_CM) * 57.29578f;

    (void)foc_gimbal_move_to_fast(&g_foc_gimbal, yaw_deg, pitch_deg);
    task->circle_phase_deg = progress * 360.0f;
    (void)now_ms;
}

/* ==================== 巡线 + T8路段识别 + 原地转向 ==================== */

static void e2025_start_turn(e2025_task_context_t *task,
    float current_heading, t8_road_type_t turn_type, uint32_t now_ms)
{
    float target;

    if (turn_type == T8_ROAD_LEFT_TURN)
    {
        target = current_heading - E2025_TURN_ANGLE_DEG;
    }
    else
    {
        target = current_heading + E2025_TURN_ANGLE_DEG;
    }

    if (target > 180.0f)
    {
        target -= 360.0f;
    }
    else if (target < -180.0f)
    {
        target += 360.0f;
    }

    task->turn_target_heading = target;
    task->performing_turn     = true;

    car_stop(now_ms);
}

static void e2025_accum_global_travel(e2025_task_context_t *task,
    float current_travel_mm)
{
    float delta;

    delta = current_travel_mm - task->segment_start_travel_mm;
    if (delta > 0.0f)
    {
        task->travel_global_mm += delta;
    }
    task->segment_start_travel_mm = current_travel_mm;
}

static void e2025_line_follow_with_detection(e2025_task_context_t *task,
    const e2025_vehicle_state_t *state, uint32_t now_ms)
{
    uint8_t        line_bits;
    t8_road_type_t turn_type;

    if (task->performing_turn)
    {
        if (car_align_heading(task->turn_target_heading, now_ms))
        {
            float current_travel = state->travel_mm;

            task->performing_turn = false;
            t8_road_detector_reset(&task->road_det);

            if (!task->is_circle_task)
            {
                car_reset_odometry();
                task->travel_global_mm     += current_travel;
                task->segment_start_travel_mm = 0.0f;
            }
            else
            {
                e2025_accum_global_travel(task, current_travel);
            }
        }
        return;
    }

    line_bits = car_get_line_bits();

    if (t8_road_detector_confirm(&task->road_det, line_bits))
    {
        e2025_accum_global_travel(task, state->travel_mm);

        turn_type = t8_road_detector_type(&task->road_det);
        e2025_start_turn(task, state->yaw_deg, turn_type, now_ms);
        t8_road_detector_reset(&task->road_det);
        return;
    }

    car_follow_line(now_ms);
}

static void e2025_circle_progress(e2025_task_context_t *task,
    const e2025_vehicle_state_t *state, float *progress)
{
    float phase;

    phase = fmodf(state->travel_global_mm, (float)E2025_TRACK_PERIMETER_MM)
          / (float)E2025_TRACK_PERIMETER_MM;
    *progress = (float)(task->lap_current) + phase;
}

/* ==================== 各状态运行逻辑 ==================== */

static void e2025_run_line_follow(e2025_task_context_t *task,
    const e2025_vehicle_state_t *state, uint32_t now_ms)
{
    e2025_line_follow_with_detection(task, state, now_ms);

    if (task->lap_current >= task->lap_target)
    {
        e2025_complete(task, now_ms);
    }
}

static void e2025_run_aiming(e2025_task_context_t *task,
    const e2025_vehicle_state_t *state, uint32_t now_ms)
{
    e2025_aim_update(task, now_ms);

    if (task->aim_locked)
    {
        e2025_complete(task, now_ms);
        return;
    }

    if (task->id == E2025_BASE_TASK_2
        && (now_ms - task->aim_start_ms) >= E2025_B2_AIM_TIMEOUT_MS)
    {
        e2025_complete(task, now_ms);
        return;
    }
    if (task->id == E2025_BASE_TASK_3
        && (now_ms - task->aim_start_ms) >= E2025_B3_AIM_TIMEOUT_MS)
    {
        e2025_complete(task, now_ms);
        return;
    }
    (void)state;
}

static void e2025_run_line_aiming(e2025_task_context_t *task,
    const e2025_vehicle_state_t *state, uint32_t now_ms)
{
    e2025_line_follow_with_detection(task, state, now_ms);
    e2025_aim_update(task, now_ms);

    if (task->lap_current >= task->lap_target)
    {
        e2025_complete(task, now_ms);
    }
}

static void e2025_run_circle_sync(e2025_task_context_t *task,
    const e2025_vehicle_state_t *state, uint32_t now_ms)
{
    float progress;

    e2025_line_follow_with_detection(task, state, now_ms);
    e2025_circle_progress(task, state, &progress);
    e2025_circle_update(task, progress, now_ms);

    if (task->lap_current >= task->lap_target)
    {
        e2025_complete(task, now_ms);
    }
}

/* ==================== 生命周期回调 ==================== */

static void e2025_turn_state_reset(e2025_task_context_t *task)
{
    task->performing_turn     = false;
    task->turn_target_heading = 0.0f;
    t8_road_detector_reset(&task->road_det);
}

static void e2025_task_start(uint32_t now_ms, void *context)
{
    e2025_task_context_t *task = (e2025_task_context_t *)context;
    e2025_vehicle_state_t state;

    if (task == NULL)
    {
        return;
    }

    car_stop(now_ms);
    car_reset_odometry();

    if (!car_prepare(now_ms))
    {
        task->state = E2025_STATE_FAULT;
        if (task->manager != NULL)
        {
            task->manager->state = EC_MODE_FAULT;
        }
        return;
    }

    if (!e2025_read_vehicle_state(task, &state, now_ms))
    {
        return;
    }

    task->lap_target              = e2025_task_lap_target(task->id);
    task->lap_current             = 0u;
    task->deadline_ms             = now_ms + e2025_task_timeout_ms(task->id);
    task->state_enter_ms          = now_ms;
    task->segment_start_travel_mm = state.travel_mm;
    task->travel_global_mm        = 0.0f;
    task->circle_phase_deg        = 0.0f;
    task->aim_start_ms            = 0u;
    task->is_circle_task          = e2025_task_is_circle_sync(task->id);

    e2025_heading_accum_reset(task);
    e2025_turn_state_reset(task);

    if (e2025_task_is_aim_only(task->id))
    {
        e2025_aim_start(task, now_ms);
    }
    else if (e2025_task_needs_gimbal(task->id))
    {
        (void)foc_gimbal_enable(&g_foc_gimbal, true);
    }

    e2025_enter_state(task, e2025_get_running_state(task->id), now_ms);
}

static void e2025_task_run(uint32_t now_ms, void *context)
{
    e2025_task_context_t *task = (e2025_task_context_t *)context;
    e2025_vehicle_state_t state;

    if (task == NULL)
    {
        return;
    }

    if (now_ms > task->deadline_ms)
    {
        e2025_set_fault(task, E2025_STATE_TIMEOUT, now_ms);
        return;
    }

    if (!e2025_read_vehicle_state(task, &state, now_ms))
    {
        return;
    }

    e2025_update_lap_count(task, &state);
    state.lap_count = task->lap_current;

    switch (task->state)
    {
        case E2025_STATE_INIT:
            break;
        case E2025_STATE_LINE_FOLLOW:
            e2025_run_line_follow(task, &state, now_ms);
            break;
        case E2025_STATE_AIMING:
            e2025_run_aiming(task, &state, now_ms);
            break;
        case E2025_STATE_LINE_AIMING:
            e2025_run_line_aiming(task, &state, now_ms);
            break;
        case E2025_STATE_CIRCLE_SYNC:
            e2025_run_circle_sync(task, &state, now_ms);
            break;
        case E2025_STATE_DONE:
        case E2025_STATE_TIMEOUT:
        case E2025_STATE_FAULT:
            break;
        case E2025_STATE_STOPPED:
        default:
            break;
    }
}

static void e2025_task_stop(uint32_t now_ms, void *context)
{
    e2025_task_context_t *task = (e2025_task_context_t *)context;

    if (task == NULL)
    {
        return;
    }
    car_stop(now_ms);
    if (e2025_task_needs_gimbal(task->id))
    {
        (void)foc_gimbal_stop(&g_foc_gimbal);
    }
    task->state = E2025_STATE_STOPPED;
}

/* ==================== 公共API实现 ==================== */

bool e2025_tasks_register(ec_mode_manager_t *manager)
{
    uint8_t i;

    if (manager == NULL)
    {
        return false;
    }

    for (i = 0u; i < (uint8_t)E2025_TASK_COUNT; i++)
    {
        ec_mode_t             mode;
        e2025_task_context_t *task = &g_e2025_tasks[i];

        task->id          = (e2025_task_id_t)i;
        task->state       = E2025_STATE_STOPPED;
        task->manager     = manager;
        task->lap_target  = e2025_task_lap_target(task->id);
        task->lap_current = 0u;

        t8_road_detector_init(&task->road_det,
            E2025_TURN_CONFIRM_SAMPLES,
            E2025_TURN_HALF_BLACK_MIN,
            E2025_TURN_OTHER_HALF_MAX);

        mode.name    = g_e2025_task_names[i];
        mode.init    = NULL;
        mode.start   = e2025_task_start;
        mode.run     = e2025_task_run;
        mode.stop    = e2025_task_stop;
        mode.context = task;

        if (!ec_mode_manager_add(manager, &mode))
        {
            return false;
        }
    }
    return true;
}

static const e2025_task_context_t *e2025_tasks_active_context(
    const ec_mode_manager_t *manager)
{
    const void *active_context;
    uint8_t     i;

    if (manager == NULL || manager->state == EC_MODE_STOPPED
        || manager->active >= manager->count)
    {
        return NULL;
    }

    active_context = manager->modes[manager->active].context;
    for (i = 0u; i < (uint8_t)E2025_TASK_COUNT; i++)
    {
        if (active_context == &g_e2025_tasks[i])
        {
            return &g_e2025_tasks[i];
        }
    }
    return NULL;
}

bool e2025_tasks_is_active(const ec_mode_manager_t *manager)
{
    return e2025_tasks_active_context(manager) != NULL;
}

e2025_task_state_t e2025_tasks_active_state(const ec_mode_manager_t *manager)
{
    const e2025_task_context_t *task = e2025_tasks_active_context(manager);
    return (task == NULL) ? E2025_STATE_STOPPED : task->state;
}

const char *e2025_tasks_active_status(const ec_mode_manager_t *manager)
{
    const e2025_task_context_t *task = e2025_tasks_active_context(manager);

    if (task == NULL)
    {
        return "E STOP";
    }

    switch (task->state)
    {
        case E2025_STATE_INIT:        return "E INIT";
        case E2025_STATE_LINE_FOLLOW: return "E LINE";
        case E2025_STATE_AIMING:      return "E AIM";
        case E2025_STATE_LINE_AIMING: return "E LN+AM";
        case E2025_STATE_CIRCLE_SYNC: return "E CIRC";
        case E2025_STATE_DONE:        return "E DONE";
        case E2025_STATE_TIMEOUT:     return "E TIMEOUT";
        case E2025_STATE_FAULT:       return "E FAULT";
        case E2025_STATE_STOPPED:
        default:                      return "E STOP";
    }
}
