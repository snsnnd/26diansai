#ifndef LINE_EVENT_DETECTOR_H
#define LINE_EVENT_DETECTOR_H

/**
 * @file line_event_detector.h
 * @brief 巡线事件检测器模块
 *
 * 基于去抖算法的巡线状态检测器。原始的循迹传感器信号可能存在
 * 噪声和抖动，本模块通过连续采样确认的方式提供稳定的"上线/离线"
 * 状态判断，并累加进出线事件的计数。
 */

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 巡线事件检测器状态结构体
 *
 * 通过去抖算法(debounce)处理原始传感器的二值信号，
 * 输出稳定的线上/离线状态，并统计进出线事件次数。
 */
typedef struct
{
    bool initialized;               /* 初始化标志：true表示已接收过样本 */
    bool stable_on_line;            /* 去抖后的稳定状态：true=在线，false=离线 */
    uint8_t debounce_samples;       /* 去抖所需样本数：连续N个样本一致才确认状态切换 */
    uint8_t candidate_samples;      /* 当前候选计数值：检测到状态切换的连续样本数 */
    uint32_t enter_count;           /* 进入黑线(上线)事件的累计次数 */
    uint32_t exit_count;            /* 离开黑线(离线)事件的累计次数 */
    uint32_t last_transition_ms;    /* 最后一次状态切换的时间戳(毫秒) */
} line_event_detector_t;

/**
 * @brief 初始化巡线事件检测器
 * @param detector 检测器实例指针
 * @param debounce_samples 去抖所需样本数(至少为1)
 */
void line_event_detector_init(line_event_detector_t *detector,
    uint8_t debounce_samples);

/**
 * @brief 更新巡线事件检测器
 *
 * 输入原始的线上/离线信号(raw_on_line)，通过去抖算法判断是否
 * 发生真实的状态切换。当确认切换时，更新稳定状态并累加事件计数。
 *
 * @param detector 检测器实例指针
 * @param raw_on_line 传感器原始信号(可能带噪声)
 * @param now_ms 当前时间戳(毫秒)，用于记录切换时间
 */
void line_event_detector_update(line_event_detector_t *detector,
    bool raw_on_line, uint32_t now_ms);

#endif
