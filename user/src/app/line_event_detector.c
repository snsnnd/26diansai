#include "app/line_event_detector.h"

#include <stddef.h>

/**
 * @brief 初始化巡线事件检测器
 *
 * 将检测器置为未初始化状态。第一个有效样本会直接初始化稳定状态，
 * 这样可以避免在传感器启动阶段产生虚假的事件计数。
 *
 * @param detector 检测器实例指针(不能为NULL)
 * @param debounce_samples 去抖所需连续样本数(至少为1，传入0会自动修正为1)
 */
void line_event_detector_init(line_event_detector_t *detector,
    uint8_t debounce_samples)
{
    if (detector == NULL)
    {
        return;
    }

    detector->initialized = false;                  /* 尚未初始化 */
    detector->stable_on_line = false;               /* 默认离线，首次更新时会修正 */
    detector->debounce_samples = (debounce_samples == 0u) ? 1u : debounce_samples;
    detector->candidate_samples = 0u;                /* 候选计数清零 */
    detector->enter_count = 0u;                      /* 上线事件计数清零 */
    detector->exit_count = 0u;                       /* 离线事件计数清零 */
    detector->last_transition_ms = 0u;               /* 上次切换时间清零 */
}

/**
 * @brief 更新巡线事件检测器
 *
 * 实现基于连续确认的去抖算法(debounce algorithm)：
 *
 * 1. 首次调用时直接使用原始值初始化稳定状态。
 * 2. 如果原始信号与稳定状态一致，说明无变化，清零候选计数。
 * 3. 如果原始信号与稳定状态不一致，累加候选计数。
 * 4. 当候选计数达到去抖阈值时，确认状态切换：
 *    - 更新稳定状态为新值
 *    - 累加对应的事件计数(enter_count或exit_count)
 *    - 记录切换时间
 *
 * 这种去抖方式比固定延时去抖更合适，因为传感器的采样率是固定的，
 * 通过样本计数而非时间延时可以更好地适应不同的采样频率。
 *
 * @param detector 检测器实例指针
 * @param raw_on_line 传感器原始信号(true=检测到黑线, false=未检测到)
 * @param now_ms 当前系统时间戳(毫秒)
 */
void line_event_detector_update(line_event_detector_t *detector,
    bool raw_on_line, uint32_t now_ms)
{
    if (detector == NULL)
    {
        return;
    }

    /* ========== 首次更新：用原始值初始化稳定状态 ========== */
    if (!detector->initialized)
    {
        detector->initialized = true;                     /* 标记已初始化 */
        detector->stable_on_line = raw_on_line;           /* 用当前值作为稳定状态 */
        detector->candidate_samples = 0u;                  /* 候选计数清零 */
        detector->last_transition_ms = now_ms;             /* 记录初始时间 */
        return;
    }

    /* ========== 信号未变化：稳定状态不变 ========== */
    if (raw_on_line == detector->stable_on_line)
    {
        detector->candidate_samples = 0u;    /* 信号一致，清零候选计数(有效防止噪声累积) */
        return;
    }

    /* ========== 信号有变化：累加候选计数 ========== */
    if (detector->candidate_samples < detector->debounce_samples)
    {
        detector->candidate_samples++;       /* 递增候选计数 */
    }
    if (detector->candidate_samples < detector->debounce_samples)
    {
        return;                              /* 尚未达到确认阈值，不切换状态 */
    }

    /* ========== 确认状态切换：达到去抖阈值 ========== */
    detector->stable_on_line = raw_on_line;               /* 更新稳定状态 */
    detector->candidate_samples = 0u;                      /* 重置候选计数 */
    detector->last_transition_ms = now_ms;                 /* 记录切换时间 */

    /* 根据切换方向累加事件计数 */
    if (raw_on_line)
    {
        detector->enter_count++;    /* 进入黑线(上线)事件 */
    }
    else
    {
        detector->exit_count++;     /* 离开黑线(离线)事件 */
    }
}
