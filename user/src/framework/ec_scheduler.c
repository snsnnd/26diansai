/**
 * @file    ec_scheduler.c
 * @brief   协作式任务调度器 - 实现文件
 * @details 本文件实现了 ec_scheduler 模块的所有功能，包括调度器初始化、
 *          任务注册和任务调度执行。调度器采用"时间触发、协作式、周期轮询"
 *          的设计模式，是智能车控制系统的核心调度组件。
 *
 *          【调度算法说明】
 *          调度器通过比较当前时间(now_ms)和每个任务的下一执行时间(next_run_ms)
 *          来判断任务是否该执行。如果当前时间 >= 计划时间，则执行该任务，并
 *          根据延迟情况补偿计算下一次执行时间。
 *
 *          【延迟补偿策略】
 *          当任务延迟执行时，调度器采用"跳过快过期周期"的策略：
 *          - 计算错过了多少个周期 (lateness / period_ms)
 *          - 下次执行时间 = 下次计划时间 + (错过周期数 + 1) * 周期
 *          这样避免了"追赶效应"——不会因为一次延迟而连续密集执行多次。
 *
 *          【运行时统计的意义】
 *          - lateness：衡量系统实时性，反映调度压力
 *          - runtime_ms：衡量任务计算量，帮助发现性能瓶颈
 *          - overrun_count：执行超时计数，用于识别需要优化或降频的任务
 */
#include "framework/ec_scheduler.h"
#include "framework/ec_time.h"

#include <stddef.h>
#include <stdint.h>

/**
 * @brief 初始化调度器
 * @details 清空任务数组，将所有任务槽位的运行函数指针置空并使能标志置为 false。
 *          此函数应在系统启动时、注册任何任务之前被调用。
 *
 * @param scheduler 指向待初始化的调度器实例的指针
 * @note 如果 scheduler 为 NULL，函数直接返回，不做任何操作——这是一种防御性编程风格
 * @see ec_scheduler_add
 */
void ec_scheduler_init(ec_scheduler_t *scheduler)
{
    uint8_t i;

    if (scheduler == NULL)
    {
        return;
    }

    scheduler->count = 0u;
    /* 遍历所有任务槽位，全部初始化为空/禁用状态 */
    for (i = 0u; i < EC_SCHEDULER_MAX_TASKS; i++)
    {
        scheduler->tasks[i].run = NULL;
        scheduler->tasks[i].enabled = false;
    }
}

/**
 * @brief 向调度器注册一个新任务
 * @details 在调度器的任务数组中添加一个周期性任务。任务注册后即被启用，
 *          调度器会在每次 ec_scheduler_run 被调用时检查并执行该任务。
 *
 *          【参数校验逻辑】
 *          1. 检查所有必要指针参数不为 NULL
 *          2. 检查周期必须在 (0, INT32_MAX] 范围内
 *             - 周期为0会导致除零（后续有除法运算）
 *             - 周期超过 INT32_MAX 会导致有符号比较时溢出
 *          3. 检查已注册任务数未达到上限
 *
 * @param scheduler 调度器实例指针
 * @param name 任务名称（字符串指针会被直接保存而非拷贝，调用者需保证其持久有效）
 * @param run 任务执行函数指针
 * @param context 用户自定义上下文（可以是结构体指针，用于传递状态数据）
 * @param period_ms 执行周期（毫秒），例如 period_ms=10 表示每10ms执行一次
 * @param start_ms 首次执行的起始时间戳，通常设为当前时间(立即执行)或当前时间+偏移量(延时启动)
 * @return true 注册成功；false 注册失败（参数无效或任务数组已满）
 */
bool ec_scheduler_add(ec_scheduler_t *scheduler, const char *name,
    ec_task_fn run, void *context, uint32_t period_ms, uint32_t start_ms)
{
    ec_task_t *task;

    /* 严格的参数合法性检查：防止空指针、无效周期和任务数量溢出 */
    if (scheduler == NULL || name == NULL || run == NULL || period_ms == 0u ||
        period_ms > (uint32_t)INT32_MAX ||
        scheduler->count >= EC_SCHEDULER_MAX_TASKS)
    {
        return false;
    }

    /* 获取下一个空闲的任务槽位，并将任务计数器加1 */
    task = &scheduler->tasks[scheduler->count++];
    task->name = name;
    task->run = run;
    task->context = context;
    task->period_ms = period_ms;
    task->next_run_ms = start_ms;       /* 设置首次执行时间 */
    /* 清零所有运行时统计计数器 */
    task->run_count = 0u;
    task->missed_deadlines = 0u;
    task->last_start_lateness_ms = 0u;
    task->max_start_lateness_ms = 0u;
    task->last_runtime_ms = 0u;
    task->max_runtime_ms = 0u;
    task->overrun_count = 0u;
    task->enabled = true;               /* 注册后默认启用 */
    return true;
}

/**
 * @brief 运行调度器，遍历所有任务并执行到期的任务
 * @details 这是调度器的主函数，每次调用时遍历所有已注册的任务，
 *          检查是否有任务到期（当前时间 >= 下次执行时间），对到期的任务：
 *          1. 统计延迟信息并更新最大延迟记录
 *          2. 计算合理的下次执行时间（跳过已经错过的周期）
 *          3. 执行任务函数并计时
 *          4. 更新运行时统计信息
 *
 *          【重要设计细节】
 *          - 每次任务循环内部重新获取 ec_time_ms()，确保即使前一个任务执行
 *            消耗了时间，后续任务也能基于最新的时间做判断
 *          - 延迟计算使用有符号整数 (int32_t)，确保能处理时间回绕(wraparound)场景
 *          - 任务执行时间超过周期时会记录超时，这有助于发现需要优化的任务
 *
 * @param scheduler 调度器实例指针
 * @param now_ms 当前系统时间（毫秒），实际会被 ec_time_ms() 的返回值覆盖
 * @note 此函数应在超级循环(super loop)中被频繁调用，通常放置在 while(1) 中
 */
void ec_scheduler_run(ec_scheduler_t *scheduler, uint32_t now_ms)
{
    uint8_t i;

    if (scheduler == NULL)
    {
        return;
    }

    /* 遍历所有已注册的任务 */
    for (i = 0u; i < scheduler->count; i++)
    {
        ec_task_t *task = &scheduler->tasks[i];
        int32_t lateness;       /* 任务启动延迟，有符号数以支持时间回绕检测 */
        uint32_t start_ms;      /* 任务开始执行的时间戳，用于计算执行耗时 */
        uint32_t runtime_ms;    /* 任务实际执行耗时 */

        /* 每次重新获取当前时间，使后续任务能感知前一个任务消耗的时间 */
        now_ms = ec_time_ms();

        /* 检查任务是否被禁用 */
        if (!task->enabled)
        {
            continue;
        }

        /**
         * 计算任务启动延迟。
         * 使用有符号减法：(int32_t)(now_ms - next_run_ms)
         * - 正数：任务已到期且延迟了 lateness 毫秒
         * - 零：任务刚好在计划时间点执行
         * - 负数：任务还未到期，跳过执行
         * 使用有符号类型是为了处理 unsigned 减法不会产生负数的问题。
         */
        lateness = (int32_t)(now_ms - task->next_run_ms);
        if (lateness < 0)
        {
            continue;   /* 任务尚未到期，继续检查下一个任务 */
        }

        /**
         * 检测是否错过了任务截止期。
         * 如果延迟时间 >= 一个周期，说明任务至少被跳过一次执行。
         * 统计错过的周期数（按完整周期计算），用于评估调度器负载。
         */
        if ((uint32_t)lateness >= task->period_ms)
        {
            task->missed_deadlines += (uint32_t)lateness / task->period_ms;
        }

        /* 更新启动延迟统计 */
        task->last_start_lateness_ms = (uint32_t)lateness;
        if (task->last_start_lateness_ms > task->max_start_lateness_ms)
        {
            task->max_start_lateness_ms = task->last_start_lateness_ms;
        }

        /**
         * 计算下一次执行时间——这是调度算法的核心：
         * 将 lateness 除周期后加1，再乘以周期，实现"跳过多余周期"的补偿策略。
         * 例如：周期=10ms，延迟=25ms，错过 25/10=2 个周期，
         * 则下次执行时间 = next_run_ms + (2+1)*10 = next_run_ms + 30
         * 这样任务会跳过中间错过的2个执行点，直接在下个周期点执行。
         */
        task->next_run_ms += ((uint32_t)lateness / task->period_ms + 1u) * task->period_ms;
        task->run_count++;                              /* 累加运行次数 */
        start_ms = now_ms;                              /* 记录执行开始时间 */
        task->run(start_ms, task->context);              /* 实际执行任务函数 */
        runtime_ms = ec_time_ms() - start_ms;            /* 计算执行耗时 */
        task->last_runtime_ms = runtime_ms;
        if (runtime_ms > task->max_runtime_ms)
        {
            task->max_runtime_ms = runtime_ms;           /* 更新最大执行耗时 */
        }
        /**
         * 如果任务执行时间 >= 任务周期，说明任务计算量过大，占用了整个周期
         * 甚至更多，这会导致其他任务无法及时执行。记录超时次数用于调试分析。
         */
        if (runtime_ms >= task->period_ms)
        {
            task->overrun_count++;
        }
    }
}
