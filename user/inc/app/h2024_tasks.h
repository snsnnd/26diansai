#ifndef H2024_TASKS_H
#define H2024_TASKS_H

/**
 * @file h2024_tasks.h
 * @brief 2024年全国大学生智能汽车竞赛(H2024)任务模块
 *
 * 本模块实现了竞赛要求的场地任务序列，包括直线行驶、弧线循迹、
 * 对角转向、多圈循迹等复杂赛道元素。通过状态机驱动车辆的
 * 航向控制、巡线控制和姿态对准。
 */

/* ==================== 各任务超时时间 ==================== */
#define H2024_H1_TIMEOUT_MS          25000u   /* H1任务最大执行时间(25秒) */
#define H2024_H2_TIMEOUT_MS          40000u   /* H2任务最大执行时间(40秒) */
#define H2024_H3_TIMEOUT_MS          50000u   /* H3任务最大执行时间(50秒) */
#define H2024_H4_TIMEOUT_MS         190000u   /* H4任务最大执行时间(190秒) */

/* ==================== 赛道几何参数 ==================== */
#define H2024_DIAGONAL_TURN_DEG        38.0f  /* B点对角转弯角度(度)，使车辆从弧线出口切向下一段直线 */
#define H2024_ARC_MIN_DURATION_MS      800u    /* 弧线最小持续时间，防止噪声误判 */
#define H2024_ARC_MIN_TURN_DEG         130.0f  /* 弧线最小转弯角度，验证是否真正完成了弧线 */
#define H2024_ARC_EXPECT_TURN_DEG      180.0f  /* 弧线预期转弯角度(半圆=180度) */

/* ==================== 直线行驶约束 ==================== */
#define H2024_STRAIGHT_MIN_MS           800u   /* 直线段最小持续时间(毫秒) */
#define H2024_STRAIGHT_TIME_RATIO        60u   /* 第二直线段最小时间 = 第一直线段时间 * 60/100 */
#define H2024_STRAIGHT_HEADING_DEG       10.0f /* 直线段到达路径点时允许的最大航向偏差(度) */

/* ==================== 航向对准参数 ==================== */
#define H2024_ALIGN_BASE_SCALE            1.00f /* 原地转向时的基础PWM缩放比 */
#define H2024_ALIGN_TOLERANCE_DEG          3.0f /* 航向对准容差(度)，在此范围内认为对准完成 */
#define H2024_ALIGN_CONFIRM_SAMPLES          5u /* 对准确认样本数，连续多次在容差内才认为对准 */

/* ==================== 信号指示 ==================== */
#define H2024_POINT_SIGNAL_MS              250u /* 到达路径点时的LED亮灯和蜂鸣器持续时间 */
#define H2024_PRE_ARC_ALIGN_MS             400u /* 进入弧线前的航向对准时间(毫秒) */

/* ==================== 距离辅助判断 ==================== */
#define H2024_STRAIGHT_MIN_MM              800u /* 直线段到达路径点的最小行驶距离(毫米) */
#define H2024_ARC_MIN_MM                   600u /* 弧线段最小行驶距离(毫米) */

#include "framework/ec_mode_manager.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief H2024竞赛任务ID枚举
 * 对应2024年竞速赛的H1~H4四个难度递进的场地任务
 */
typedef enum
{
    H2024_TASK_1 = 0,  /* H1：最简单的直线任务 */
    H2024_TASK_2,       /* H2：含一个弧线的基本任务 */
    H2024_TASK_3,       /* H3：含两个弧线的进阶任务 */
    H2024_TASK_4,       /* H4：需要完成4圈的高阶任务(最复杂) */
    H2024_TASK_COUNT    /* 任务总数，用于数组大小计算 */
} h2024_task_id_t;

/**
 * @brief H2024任务状态机枚举
 *
 * 描述车辆在完成一个竞赛任务过程中所处的阶段。
 * 典型流程：LEAVE_A -> FIRST_STRAIGHT -> FIRST_ARC ->
 * SECOND_STRAIGHT -> SECOND_ARC -> (可选) LAP_ALIGN -> DONE
 */
typedef enum
{
    H2024_STATE_STOPPED = 0,         /* 停止状态：任务未激活或已停止 */
    H2024_STATE_INIT_TURN,            /* 初始转向：H3/H4正放A点后自动转向对角线 */
    H2024_STATE_LEAVE_A,             /* 离开A点：从起点驶出，进入第一直线段 */
    H2024_STATE_FIRST_STRAIGHT,      /* 第一直线段：沿直线从A到B(或A到C) */
    H2024_STATE_PRE_FIRST_ARC,       /* 弧前对准：到达第一个弧线入口，先对齐航向 */
    H2024_STATE_FIRST_ARC,           /* 第一弧线段：在B点循迹过弧 */
    H2024_STATE_PRE_SECOND_STRAIGHT, /* 弧后对准：第一弧线出口，对齐航向后进入第二直线 */
    H2024_STATE_SECOND_STRAIGHT,     /* 第二直线段：从弧线出口到下一个入弯点 */
    H2024_STATE_PRE_SECOND_ARC,      /* 弧前对准：到达第二个弧线入口，先对齐航向 */
    H2024_STATE_SECOND_ARC,          /* 第二弧线段：循迹过第二个弧 */
    H2024_STATE_LAP_ALIGN,           /* 圈间对准：多圈任务中完成一圈后的方向校准 */
    H2024_STATE_DONE,                /* 完成状态：任务成功执行完毕 */
    H2024_STATE_TIMEOUT,             /* 超时状态：任务超时未完成 */
    H2024_STATE_FAULT                /* 故障状态：执行过程中出现硬件或软件错误 */
} h2024_task_state_t;

/**
 * @brief 车辆运行时状态结构体
 *
 * 由vehicle_port.read_state回调函数填充，描述车辆的当前运动状态。
 * 这些信息被任务状态机用来判断何时切换到下一个阶段。
 */
typedef struct
{
    float heading_deg;          /* 当前航向角(度)，相对于初始方向 */
    uint32_t line_enter_count;  /* 进入黑线的累计次数(用于判断交叉点和路径点) */
    uint32_t line_exit_count;   /* 离开黑线的累计次数(用于判断路径切换) */
    bool on_line;               /* 当前是否稳定在黑线上 */
    float travel_mm;            /* 平均行驶距离(毫米)，(左+右)/2 */
} h2024_vehicle_state_t;

/**
 * @brief 注册H2024竞赛任务到模式管理器
 * @param manager 模式管理器实例指针
 * @return true 注册成功；false 注册失败(参数无效或达到模式上限)
 * @note 会自动创建H2024_TASK_COUNT个任务模式并添加到管理器中。
 *       底层驱动通过 line_car.h 直接调用，无需接口注入。
 */
bool h2024_tasks_register(ec_mode_manager_t *manager);

/**
 * @brief 检查是否有H2024任务正在运行
 * @param manager 模式管理器实例指针
 * @return true 有任务运行中；false 无任务运行
 */
bool h2024_tasks_is_active(const ec_mode_manager_t *manager);

/**
 * @brief 获取当前运行H2024任务的状态
 * @param manager 模式管理器实例指针
 * @return 当前任务状态；无任务运行时返回H2024_STATE_STOPPED
 */
h2024_task_state_t h2024_tasks_active_state(
    const ec_mode_manager_t *manager);

/**
 * @brief 获取当前运行H2024任务的状态描述字符串
 * @param manager 模式管理器实例指针
 * @return 状态描述字符串(如"H STRAIGHT1", "H ARC1"等)
 * @note 返回值指向静态字符串，无需释放
 */
const char *h2024_tasks_active_status(const ec_mode_manager_t *manager);

#endif
