/**
 * @file    ec_scheduler.h
 * @brief   协作式任务调度器模块
 * @details 本模块实现了一个基于优先级的协作式任务调度器，支持周期任务的注册、
 *          执行和运行时监控（包括超时检测、错过截止期统计等）。调度器在主循环中
 *          被调用，所有任务按注册顺序依次执行，每个任务执行完毕后才会执行下一个，
 *          因此称为"协作式"(cooperative)调度。
 *
 *          【设计思路】
 *          1. 协作式 vs 抢占式：本调度器不使用硬件定时器中断来切换任务，而是
 *             在 while(1) 主循环中轮询所有任务。优点是实现简单、无竞态条件、
 *             无需复杂同步机制；缺点是某个任务执行时间过长会阻塞其他任务。
 *          2. 最大任务数通过 EC_SCHEDULER_MAX_TASKS 宏限制，防止运行时动态
 *             分配内存带来的不可预测性，适合嵌入式实时系统的确定性要求。
 *          3. 每个任务都记录运行统计信息（运行次数、超时次数、最大执行时间等），
 *             便于调试和性能分析。
 *
 * @note    本调度器不提供任务优先级抢占，任务执行顺序由注册顺序决定。
 *          任务函数必须是非阻塞的，并且执行时间应远小于其周期。
 */
#ifndef EC_SCHEDULER_H
#define EC_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#define EC_SCHEDULER_MAX_TASKS 12u

/**
 * @brief 任务函数指针类型
 * @param now_ms 当前系统时间（毫秒），由调度器传入
 * @param context 用户自定义上下文指针，在注册任务时设定
 */
typedef void (*ec_task_fn)(uint32_t now_ms, void *context);

/**
 * @brief 任务控制块结构体
 * @details 每个任务对应一个 ec_task_t 实例，记录了任务的全部属性、调度信息和运行统计。
 *          该结构体是调度器的核心数据结构，用于跟踪和管理每个任务的执行状态。
 *
 *          【运行时统计字段说明】
 *          - run_count：累计执行次数，可用于计算任务平均执行频率
 *          - missed_deadlines：截止期错过次数统计
 *          - last_start_lateness_ms / max_start_lateness_ms：任务实际启动相对于
 *            计划启动的延迟，反映系统的调度压力
 *          - last_runtime_ms / max_runtime_ms：任务单次执行耗时，用于发现性能瓶颈
 *          - overrun_count：执行超时次数（执行时间 >= 周期），标识需要优化的任务
 */
typedef struct
{
    const char *name;           /**< 任务名称，用于调试和日志输出 */
    ec_task_fn run;             /**< 任务执行函数指针 */
    void *context;              /**< 用户传递给任务函数的上下文数据指针 */
    uint32_t period_ms;         /**< 任务执行周期（毫秒），决定任务的执行频率 */
    uint32_t next_run_ms;       /**< 下一次计划执行的时间戳（毫秒），调度器根据此值判断是否执行任务 */
    uint32_t run_count;         /**< 累计执行次数，从注册时开始计数 */
    uint32_t missed_deadlines;  /**< 错过截止期的累计次数。当任务延迟超过一个周期时，认为错过了一个截止期 */
    uint32_t last_start_lateness_ms;    /**< 最近一次启动的延迟时间（毫秒），即实际开始时间减去计划时间 */
    uint32_t max_start_lateness_ms;     /**< 历史最大启动延迟时间（毫秒），用于评估最坏情况下的调度延迟 */
    uint32_t last_runtime_ms;           /**< 最近一次执行耗时（毫秒） */
    uint32_t max_runtime_ms;            /**< 历史最大执行耗时（毫秒），用于评估任务的最坏情况执行时间（WCET） */
    uint32_t overrun_count;     /**< 执行超时次数。当一次执行耗时 >= 任务周期时计数，标识任务可能过重 */
    bool enabled;               /**< 任务使能标志。true=允许执行，false=跳过该任务 */
} ec_task_t;

/**
 * @brief 调度器结构体
 * @details 调度器是一个简单的容器，包含一个固定大小的任务数组和当前已注册的任务数量。
 *          采用静态数组而非动态链表的方式，避免了内存碎片化问题，符合嵌入式系统的可靠性要求。
 *
 *          【设计选择】
 *          使用静态数组而不是链表的理由：
 *          1. 确定性内存使用 - 编译时即可确定最大内存占用
 *          2. 无内存碎片 - 不需要动态分配/释放
 *          3. 访问速度快 - 数组的连续内存布局有利于缓存利用
 *          4. 实现简单 - 不需要链表操作带来的额外代码复杂度
 */
typedef struct
{
    ec_task_t tasks[EC_SCHEDULER_MAX_TASKS];    /**< 固定大小的任务数组 */
    uint8_t count;                              /**< 当前已注册的任务数量 */
} ec_scheduler_t;

/**
 * @brief 初始化调度器
 * @param scheduler 指向调度器实例的指针
 * @note 初始化会将所有任务槽位清零并置为禁用状态
 */
void ec_scheduler_init(ec_scheduler_t *scheduler);

/**
 * @brief 向调度器注册一个新任务
 * @param scheduler 指向调度器实例的指针
 * @param name 任务名称字符串指针（必须保持有效，不会被拷贝）
 * @param run 任务执行函数指针
 * @param context 传递给任务函数的上下文数据
 * @param period_ms 任务执行周期（毫秒），必须大于0
 * @param start_ms 首次执行的起始时间戳（毫秒）
 * @return true 注册成功，false 注册失败（参数无效或任务已满）
 */
bool ec_scheduler_add(ec_scheduler_t *scheduler, const char *name,
    ec_task_fn run, void *context, uint32_t period_ms, uint32_t start_ms);

/**
 * @brief 运行调度器，检查并执行到期的任务
 * @param scheduler 指向调度器实例的指针
 * @param now_ms 当前系统时间戳（毫秒）
 * @note 每次调用时，当前时间从 ec_time_ms() 刷新，确保任务间时间基准一致。
 *       该函数应在主循环中周期性调用，或者在空闲时调用以处理待执行任务。
 *       所有任务以"协作式"方式运行——一个任务执行完才会执行下一个。
 */
void ec_scheduler_run(ec_scheduler_t *scheduler, uint32_t now_ms);

#endif
