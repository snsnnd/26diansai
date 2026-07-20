#include "app/h2024_tasks.h"
#include "app/line_car.h"

#include <stddef.h>

/**
 * @brief H2024任务运行时上下文结构体
 *
 * 每个H2024任务实例拥有一个该结构体，用于维护任务执行过程中的
 * 所有状态变量、计时器和几何参数。注意这是内部实现结构，对外部不可见。
 */
typedef struct
{
    /* ========== 任务标识与状态 ========== */
    h2024_task_id_t id;                     /* 任务ID(H1~H4) */
    h2024_task_state_t state;               /* 当前状态机阶段 */

    /* ========== 系统接口 ========== */
    ec_mode_manager_t *manager;             /* 模式管理器引用 */

    /* ========== 航向和轨迹参数 ========== */
    float initial_heading_deg;              /* 任务开始时的初始航向角(度) */
    float straight_heading_deg;             /* 直线段的目标航向角(度) */
    float arc_entry_heading_deg;            /* 进入弧线时刻的航向角(度)，用于判断弧线完成 */

    /* ========== 线段计数和计时 ========== */
    uint32_t line_enter_count;              /* 上次记录的进入黑线计数(用于检测变化) */
    uint32_t line_exit_count;               /* 上次记录的离开黑线计数(用于检测变化) */
    uint32_t arc_started_ms;                /* 进入弧线段时的时间戳(毫秒) */
    uint32_t first_straight_started_ms;     /* 第一直线段开始时的时间戳 */
    uint32_t first_straight_duration_ms;    /* 第一直线段实际持续时间(毫秒) */
    uint32_t pre_arc_start_ms;               /* 弧前对准开始时间戳(毫秒) */
    float segment_start_travel_mm;            /* 当前线段开始时的行驶距离(毫米) */
    uint32_t second_straight_started_ms;    /* 第二直线段开始时的时间戳 */

    /* ========== 时间约束 ========== */
    uint32_t deadline_ms;                   /* 任务截止时间(当前时间+超时时间) */

    /* ========== 多圈任务 ========== */
    uint8_t lap;                            /* 当前圈数(H4需要4圈) */
} h2024_task_context_t;

/**
 * 所有H2024任务的全局上下文数组，每个任务一个独立实例
 * 使用静态全局变量而非动态分配，避免堆碎片化
 */
static h2024_task_context_t g_h2024_tasks[H2024_TASK_COUNT];

/**
 * 任务名称映射表，用于OLED显示和调试输出
 */
static const char *const g_h2024_task_names[H2024_TASK_COUNT] = {
    "2024 H1",   /* 最简单的入门任务 */
    "2024 H2",   /* 含一个弧线的任务 */
    "2024 H3",   /* 含两个弧线的任务 */
    "2024 H4"    /* 需要4圈的高阶任务 */
};

/**
 * @brief 根据任务ID获取对应的超时时间
 * @param id 任务ID
 * @return 超时时间(毫秒)
 */
static uint32_t h2024_task_timeout_ms(h2024_task_id_t id)
{
    switch (id)
    {
        case H2024_TASK_1: return H2024_H1_TIMEOUT_MS;   /* 15秒 */
        case H2024_TASK_2: return H2024_H2_TIMEOUT_MS;   /* 30秒 */
        case H2024_TASK_3: return H2024_H3_TIMEOUT_MS;   /* 40秒 */
        case H2024_TASK_4: return H2024_H4_TIMEOUT_MS;   /* 180秒(3分钟) */
        default: return H2024_H1_TIMEOUT_MS;
    }
}

/**
 * @brief 将航向角归一化到[-180, 180)度范围
 *
 * 航向角是连续的角度值，可能因多次旋转而累积超出±180度。
 * 此函数将其折叠回标准范围，便于比较角度差。
 * 例如：190度 -> -170度，-200度 -> 160度
 *
 * @param heading_deg 原始航向角(度)
 * @return 归一化后的航向角
 */
static float h2024_wrap_heading(float heading_deg)
{
    while (heading_deg > 180.0f) heading_deg -= 360.0f;
    while (heading_deg < -180.0f) heading_deg += 360.0f;
    return heading_deg;
}

/**
 * @brief 浮点数绝对值
 * @param value 输入值
 * @return 绝对值
 */
static float h2024_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 判断弧线段出口是否有效(是否真正完成了弧线行驶)
 *
 * 弧线出口有效的三个条件：
 * 1. 车辆已离开黑线(弧线结束)
 * 2. 弧线持续时间超过最小要求(防止噪声误判)
 * 3. 航向变化量达到最小转弯角度(确保物理上完成了弧线)
 *
 * @param task 任务上下文
 * @param state 车辆当前状态
 * @param now_ms 当前时间戳
 * @return true 弧线出口有效，可以切换到下一阶段
 */
static bool h2024_arc_exit_is_valid(const h2024_task_context_t *task,
    const h2024_vehicle_state_t *state, uint32_t now_ms)
{
    /* 计算进入弧线以来的航向变化量 */
    float heading_change_deg = h2024_absf(h2024_wrap_heading(
        state->heading_deg - task->arc_entry_heading_deg));

    /* 三个条件必须同时满足 */
    return !state->on_line &&                                          /* 条件1：离开黑线 */
        (uint32_t)(now_ms - task->arc_started_ms) >=
            H2024_ARC_MIN_DURATION_MS &&                               /* 条件2：最小持续时间 */
        heading_change_deg >= H2024_ARC_MIN_TURN_DEG &&                /* 条件3：最小转弯角度 */
        (state->travel_mm - task->segment_start_travel_mm) >=
            (float)H2024_ARC_MIN_MM;                                   /* 条件4：最小弧线距离 */
}


/**
 * @brief 设置故障状态
 *
 * 当任务执行过程中出现故障时调用。立即停止车辆运动，
 * 设置任务状态到指定的故障/超时状态，并通知模式管理器。
 *
 * @param task 任务上下文
 * @param state 要设置的故障状态(通常是H2024_STATE_FAULT或TIMEOUT)
 * @param now_ms 当前时间戳
 */
static void h2024_set_fault(h2024_task_context_t *task,
    h2024_task_state_t state, uint32_t now_ms)
{
    car_stop(now_ms);
    task->state = state;
    if (task->manager != NULL)
    {
        task->manager->state = EC_MODE_FAULT;
    }
}

/**
 * @brief 通过车辆接口读取当前状态，失败时自动触发故障
 * @param task 任务上下文
 * @param state 输出参数，接收车辆状态
 * @param now_ms 当前时间戳
 * @return true 读取成功；false 读取失败(已自动设置故障)
 */
static bool h2024_read_state(h2024_task_context_t *task,
    h2024_vehicle_state_t *state, uint32_t now_ms)
{
    if (state == NULL || !car_read_state(state, now_ms))
    {
        h2024_set_fault(task, H2024_STATE_FAULT, now_ms);
        return false;
    }
    return true;
}

/**
 * @brief 发出路径点到达信号(点亮LED并蜂鸣)
 * @param task 任务上下文
 * @param now_ms 当前时间戳
 */
static void h2024_signal_point(h2024_task_context_t *task, uint32_t now_ms)
{
    car_signal_point(now_ms);
    (void)task;
}

/**
 * @brief 完成当前任务
 *
 * 停止车辆运动，发出完成信号，将状态设置为DONE。
 * 这是正常任务完成的唯一路径。
 *
 * @param task 任务上下文
 * @param now_ms 当前时间戳
 */
static void h2024_complete(h2024_task_context_t *task, uint32_t now_ms)
{
    car_stop(now_ms);
    h2024_signal_point(task, now_ms);
    task->state = H2024_STATE_DONE;
}

/**
 * @brief 任务启动回调
 *
 * 当用户选择启动某个H2024任务时，由模式管理器调用此函数。
 * 主要工作：
 * 1. 停止车辆(安全复位)
 * 2. 重置里程计
 * 3. 调用prepare接口准备硬件
 * 4. 读取初始车辆状态
 * 5. 初始化任务上下文变量
 * 6. 根据是否在黑线上，设置初始状态为LEAVE_A或FIRST_STRAIGHT
 *
 * @param now_ms 当前时间戳
 * @param context 指向h2024_task_context_t的指针
 */
static void h2024_task_start(uint32_t now_ms, void *context)
{
    h2024_task_context_t *task = (h2024_task_context_t *)context;
    h2024_vehicle_state_t state;

    if (task == NULL)
    {
        return;
    }

    /* 安全停止车辆 */
    car_stop(now_ms);
    /* 重置编码器里程计 */
    car_reset_odometry();
    /* 准备硬件(如检查电池、陀螺仪等)，失败则报故障 */
    if (!car_prepare(now_ms))
    {
        task->state = H2024_STATE_FAULT;
        if (task->manager != NULL)
        {
            task->manager->state = EC_MODE_FAULT;
        }
        return;
    }

    /* 读取初始车辆状态 */
    if (!h2024_read_state(task, &state, now_ms))
    {
        return;
    }

    /* ========== 初始化任务变量 ========== */
    task->initial_heading_deg = state.heading_deg;        /* 记录初始航向(正放=朝B) */
    task->line_enter_count = state.line_enter_count;      /* 记录当前的出入线计数器 */
    task->line_exit_count = state.line_exit_count;
    task->first_straight_started_ms = now_ms;             /* 第一直线段开始计时 */
    task->first_straight_duration_ms = 0u;                /* 持续时间初始为0 */
    task->second_straight_started_ms = 0u;                /* 第二直线段尚未开始 */
    task->deadline_ms = now_ms + h2024_task_timeout_ms(task->id);  /* 设置截止时间 */
    task->lap = 1u;                                       /* 第一圈开始 */

    if (task->id >= H2024_TASK_3)
    {
        /* H3/H4：正放(朝B)，自动转向对角线后开始 */
        task->straight_heading_deg = h2024_wrap_heading(
            state.heading_deg + H2024_DIAGONAL_TURN_DEG);
        task->state = H2024_STATE_INIT_TURN;
    }
    else
    {
        /* H1/H2：正放即可，无需转向 */
        task->straight_heading_deg = state.heading_deg;
        task->state = state.on_line ? H2024_STATE_LEAVE_A :
            H2024_STATE_FIRST_STRAIGHT;
    }
}

/**
 * @brief 任务执行回调(主状态机)
 *
 * 这是H2024任务最核心的函数，实现了完整的竞赛流程状态机。
 * 状态转换图：
 *
 *   LEAVE_A --> FIRST_STRAIGHT --> (H1: DONE)
 *                               --> FIRST_ARC --> SECOND_STRAIGHT --> SECOND_ARC
 *                                    (H2: DONE after SECOND_ARC)
 *                                    (H3: DONE after SECOND_ARC, different heading)
 *                                    (H4: LAP_ALIGN --> LEAVE_A repeat for 4 laps --> DONE)
 *
 * 每个状态的切换条件依赖于对车辆状态的监测(通过line_enter/exit_count
 * 判断是否经过了特定的路径点)。
 *
 * @param now_ms 当前时间戳
 * @param context 指向h2024_task_context_t的指针
 */
static void h2024_task_run(uint32_t now_ms, void *context)
{
    h2024_task_context_t *task = (h2024_task_context_t *)context;
    h2024_vehicle_state_t state;

    /* 跳过已完成、已停止或空指针 */
    if (task == NULL || task->state == H2024_STATE_DONE ||
        task->state == H2024_STATE_STOPPED)
    {
        return;
    }
    /* 读取车辆当前状态 */
    if (!h2024_read_state(task, &state, now_ms))
    {
        return;
    }

    /* ========== 状态机主循环 ========== */
    switch (task->state)
    {
        /* ---------- 初始转向：H3/H4自动转向对角线 ---------- */
        case H2024_STATE_INIT_TURN:
            if (car_align_heading(task->straight_heading_deg,
                now_ms))
            {
                task->line_enter_count = state.line_enter_count;
                task->line_exit_count = state.line_exit_count;
                task->segment_start_travel_mm = state.travel_mm;
                task->state = state.on_line ? H2024_STATE_LEAVE_A :
                    H2024_STATE_FIRST_STRAIGHT;
            }
            break;

        /* ---------- 离开A点 ---------- */
        case H2024_STATE_LEAVE_A:
            /* 沿目标航向直行，离开起点区域 */
            car_drive_heading(task->straight_heading_deg, now_ms);
            /* 检测到车辆离开了黑线(起点线)，切换到第一直线段 */
            if (state.line_exit_count != task->line_exit_count)
            {
                task->line_exit_count = state.line_exit_count;
                task->line_enter_count = state.line_enter_count;
                task->first_straight_started_ms = now_ms;
                task->segment_start_travel_mm = state.travel_mm;
                task->state = H2024_STATE_FIRST_STRAIGHT;
            }
            break;

        /* ---------- 第一直线段：从A到B(或A到C) ---------- */
        case H2024_STATE_FIRST_STRAIGHT:
            car_drive_heading(task->straight_heading_deg, now_ms);
            if (state.line_enter_count != task->line_enter_count &&
                (state.travel_mm - task->segment_start_travel_mm) >=
                    (float)H2024_STRAIGHT_MIN_MM &&
                h2024_absf(h2024_wrap_heading(state.heading_deg -
                    task->straight_heading_deg)) < H2024_STRAIGHT_HEADING_DEG)
            {
                task->line_enter_count = state.line_enter_count;
                task->first_straight_duration_ms =
                    now_ms - task->first_straight_started_ms;
                if (task->id == H2024_TASK_1)
                {
                    h2024_complete(task, now_ms);
                }
                else
                {
                    h2024_signal_point(task, now_ms);
                    task->line_exit_count = state.line_exit_count;
                    task->pre_arc_start_ms = now_ms;
                    task->state = H2024_STATE_PRE_FIRST_ARC;
                }
            }
            break;

        /* ---------- 弧前对准：到达第一弧线入口，先对齐航向再入弧 ---------- */
        case H2024_STATE_PRE_FIRST_ARC:
            if (car_align_heading(task->straight_heading_deg, now_ms))
            {
                task->arc_entry_heading_deg = state.heading_deg;
                task->segment_start_travel_mm = state.travel_mm;
                task->arc_started_ms = now_ms;
                task->state = H2024_STATE_FIRST_ARC;
            }
            break;

        /* ---------- 第一弧线段：在B点循迹过弧 ---------- */
        case H2024_STATE_FIRST_ARC:
            /* 切换到循迹模式，沿黑线过弯 */
            car_follow_line(now_ms);
            if (state.line_enter_count != task->line_enter_count)
            {
                task->line_enter_count = state.line_enter_count;
            }
            /* 检测到离开黑线(弧线结束)，验证是否真正完成了弧线 */
            if (state.line_exit_count != task->line_exit_count)
            {
                task->line_exit_count = state.line_exit_count;
                if (h2024_arc_exit_is_valid(task, &state, now_ms))
                {
                    task->line_enter_count = state.line_enter_count;
                    h2024_signal_point(task, now_ms);
                    if (task->id == H2024_TASK_2)
                    {
                        task->straight_heading_deg = state.heading_deg;
                    }
                    else
                    {
                        /* H3/H4：右弧出口处左转对角线角度，切向 B→D */
                        task->straight_heading_deg = h2024_wrap_heading(
                            state.heading_deg -
                            H2024_DIAGONAL_TURN_DEG * 1.6f);
                    }
                    task->pre_arc_start_ms = now_ms;
                    task->state = H2024_STATE_PRE_SECOND_STRAIGHT;
                }
            }
            break;

        /* ---------- 弧后对准：弧线出口先对齐航向再直行 ---------- */
        case H2024_STATE_PRE_SECOND_STRAIGHT:
            if (car_align_heading(task->straight_heading_deg, now_ms))
            {
                task->second_straight_started_ms = now_ms;
                task->segment_start_travel_mm = state.travel_mm;
                task->state = H2024_STATE_SECOND_STRAIGHT;
            }
            break;

        /* ---------- 第二直线段 ---------- */
        case H2024_STATE_SECOND_STRAIGHT:
            car_drive_heading(task->straight_heading_deg, now_ms);
            if (state.line_enter_count != task->line_enter_count &&
                (state.travel_mm - task->segment_start_travel_mm) >=
                    (float)H2024_STRAIGHT_MIN_MM &&
                h2024_absf(h2024_wrap_heading(state.heading_deg -
                    task->straight_heading_deg)) < H2024_STRAIGHT_HEADING_DEG)
            {
                task->line_enter_count = state.line_enter_count;
                task->line_exit_count = state.line_exit_count;
                h2024_signal_point(task, now_ms);
                task->pre_arc_start_ms = now_ms;
                task->state = H2024_STATE_PRE_SECOND_ARC;
            }
            break;

        /* ---------- 弧前对准：到达第二弧线入口，先对齐航向再入弧 ---------- */
        case H2024_STATE_PRE_SECOND_ARC:
            if (car_align_heading(task->straight_heading_deg, now_ms))
            {
                task->arc_entry_heading_deg = state.heading_deg;
                task->segment_start_travel_mm = state.travel_mm;
                task->arc_started_ms = now_ms;
                task->state = H2024_STATE_SECOND_ARC;
            }
            break;

        /* ---------- 第二弧线段 ---------- */
        case H2024_STATE_SECOND_ARC:
            /* 循迹过第二个弧线 */
            car_follow_line(now_ms);
            if (state.line_enter_count != task->line_enter_count)
            {
                task->line_enter_count = state.line_enter_count;
            }
            /* 检测到离开黑线，验证弧线完成 */
            if (state.line_exit_count != task->line_exit_count)
            {
                task->line_exit_count = state.line_exit_count;
                if (h2024_arc_exit_is_valid(task, &state, now_ms))
                {
                    task->line_enter_count = state.line_enter_count;
                    /* H4需要跑4圈，否则直接完成 */
                    if (task->id != H2024_TASK_4 || task->lap >= 4u)
                    {
                        h2024_complete(task, now_ms);
                    }
                    else
                    {
                        /* H4多圈：左弧出口 = 入口 - 180°，
                         * 再加对角角恢复下一圈的 A->C 方向 */
                        h2024_signal_point(task, now_ms);
                        task->lap++;
                        task->straight_heading_deg = h2024_wrap_heading(
                            state.heading_deg +
                            H2024_DIAGONAL_TURN_DEG * 1.6f);
                        task->initial_heading_deg = task->straight_heading_deg;
                        task->lap++;
                        task->state = H2024_STATE_LAP_ALIGN;
                    }
                }
            }
            break;

        /* ---------- 圈间航向对准 ---------- */
        case H2024_STATE_LAP_ALIGN:
            /* 原地调整航向对准到初始方向，为下一圈做准备 */
            if (car_align_heading(task->initial_heading_deg,
                now_ms))
            {
                task->line_enter_count = state.line_enter_count;
                task->line_exit_count = state.line_exit_count;
                if (state.on_line)
                {
                    /* 如果在黑线上，需要先离开A点 */
                    task->state = H2024_STATE_LEAVE_A;
                }
                else
                {
                    /* 如果已经离开黑线，直接开始第一直线段 */
                    task->first_straight_started_ms = now_ms;
                    task->state = H2024_STATE_FIRST_STRAIGHT;
                }
            }
            break;

        /* ---------- 终端状态：停止、超时、故障 ---------- */
        case H2024_STATE_DONE:
        case H2024_STATE_STOPPED:
        case H2024_STATE_TIMEOUT:
        case H2024_STATE_FAULT:
        default:
            /* 安全停车 */
            car_stop(now_ms);
            break;
    }
}

/**
 * @brief 任务停止回调
 *
 * 当模式管理器要求停止当前任务时调用。安全停止车辆并将状态设为STOPPED。
 *
 * @param now_ms 当前时间戳
 * @param context 指向h2024_task_context_t的指针
 */
static void h2024_task_stop(uint32_t now_ms, void *context)
{
    h2024_task_context_t *task = (h2024_task_context_t *)context;

    if (task == NULL)
    {
        return;
    }
    car_stop(now_ms);
    task->state = H2024_STATE_STOPPED;
}

/**
 * @brief 注册H2024竞赛任务到模式管理器
 *
 * 遍历H2024_TASK_COUNT个任务，为每个任务初始化上下文并创建
 * 对应的模式(ec_mode_t)，然后通过ec_mode_manager_add添加到管理器。
 * 任务模式按照H1~H4的顺序添加到管理器已有的模式之后。
 *
 * @param manager 模式管理器实例(不能为NULL)
 * @return true 全部注册成功；false 参数无效或添加失败
 */
bool h2024_tasks_register(ec_mode_manager_t *manager)
{
    uint8_t i;

    if (manager == NULL)
    {
        return false;
    }

    for (i = 0u; i < (uint8_t)H2024_TASK_COUNT; i++)
    {
        ec_mode_t mode;
        h2024_task_context_t *task = &g_h2024_tasks[i];

        /* 初始化任务上下文 */
        task->id = (h2024_task_id_t)i;
        task->state = H2024_STATE_STOPPED;
        task->manager = manager;

        /* 创建模式并注册 */
        mode.name = g_h2024_task_names[i];
        mode.init = NULL;                   /* H2024任务不需要init回调 */
        mode.start = h2024_task_start;
        mode.run = h2024_task_run;
        mode.stop = h2024_task_stop;
        mode.context = task;                /* 将任务上下文作为模式上下文 */
        if (!ec_mode_manager_add(manager, &mode))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 获取当前正在运行的H2024任务上下文
 *
 * 在模式管理器中查找当前激活的模式是否属于H2024任务，
 * 如果是则返回对应的任务上下文指针。
 *
 * @param manager 模式管理器实例
 * @return 任务上下文指针，或NULL(无H2024任务运行)
 */
static const h2024_task_context_t *h2024_tasks_active_context(
    const ec_mode_manager_t *manager)
{
    const void *active_context;
    uint8_t i;

    /* 快速失败：管理器未运行或无激活模式 */
    if (manager == NULL || manager->state == EC_MODE_STOPPED ||
        manager->active >= manager->count)
    {
        return NULL;
    }

    /* 遍历所有H2024任务，通过指针比较判断当前激活的模式是否属于H2024 */
    active_context = manager->modes[manager->active].context;
    for (i = 0u; i < (uint8_t)H2024_TASK_COUNT; i++)
    {
        if (active_context == &g_h2024_tasks[i])
        {
            return &g_h2024_tasks[i];
        }
    }
    return NULL;
}

/**
 * @brief 检查是否有H2024任务正在运行
 * @param manager 模式管理器实例
 * @return true 有任务正在运行
 */
bool h2024_tasks_is_active(const ec_mode_manager_t *manager)
{
    return h2024_tasks_active_context(manager) != NULL;
}

/**
 * @brief 获取当前运行的H2024任务的状态
 * @param manager 模式管理器实例
 * @return 任务状态，无任务运行时返回H2024_STATE_STOPPED
 */
h2024_task_state_t h2024_tasks_active_state(
    const ec_mode_manager_t *manager)
{
    const h2024_task_context_t *task = h2024_tasks_active_context(manager);

    return (task == NULL) ? H2024_STATE_STOPPED : task->state;
}

/**
 * @brief 获取当前运行的H2024任务的状态描述字符串
 *
 * 返回简短的状态标识，用于OLED屏幕显示和上位机调试。
 * 字符串长度尽量压缩以适应小屏幕。
 *
 * @param manager 模式管理器实例
 * @return 状态描述字符串指针(静态内存，无需释放)
 */
const char *h2024_tasks_active_status(const ec_mode_manager_t *manager)
{
    const h2024_task_context_t *task = h2024_tasks_active_context(manager);

    if (task == NULL)
    {
        return "STOP";
    }
    switch (task->state)
    {
        case H2024_STATE_INIT_TURN: return "H INIT";
        case H2024_STATE_LEAVE_A: return "H LEAVE A";
        case H2024_STATE_FIRST_STRAIGHT: return "H STRAIGHT1";
        case H2024_STATE_PRE_FIRST_ARC: return "H PRE ARC1";
        case H2024_STATE_FIRST_ARC: return "H ARC1";
        case H2024_STATE_PRE_SECOND_STRAIGHT: return "H PRE STR2";
        case H2024_STATE_SECOND_STRAIGHT: return "H STRAIGHT2";
        case H2024_STATE_PRE_SECOND_ARC: return "H PRE ARC2";
        case H2024_STATE_SECOND_ARC: return "H ARC2";
        case H2024_STATE_LAP_ALIGN: return "H ALIGN";
        case H2024_STATE_DONE: return "H DONE";
        case H2024_STATE_TIMEOUT: return "H TIMEOUT";
        case H2024_STATE_FAULT: return "H FAULT";
        case H2024_STATE_STOPPED:
        default: return "H STOP";
    }
}
