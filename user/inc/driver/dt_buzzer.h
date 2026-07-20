#ifndef _DT_BUZZER_H_
#define _DT_BUZZER_H_

#include "zf_common_headfile.h"

/**
 * @brief 蜂鸣器单步动作定义
 *        用于描述蜂鸣器序列中的一个节拍（响/停 + 持续时间）
 */
typedef struct {
    bool on;                 /**< true=蜂鸣器开启发声, false=蜂鸣器关闭静音 */
    uint32_t duration_ms;    /**< 本步持续的时间长度，单位：毫秒 */
} dt_buzzer_step_t;

/**
 * @brief 蜂鸣器驱动配置结构体
 *        存储蜂鸣器的硬件引脚信息、播放序列状态以及服务控制标志
 */
typedef struct {
    gpio_pin_enum pin;                   /**< 蜂鸣器连接的GPIO引脚编号 */
    const dt_buzzer_step_t *sequence;    /**< 指向播放序列（节拍数组）的指针 */
    uint32_t deadline_ms;                /**< 当前节拍结束的时间戳（绝对时间，单位ms） */
    uint8_t sequence_length;             /**< 序列中总节拍数 */
    uint8_t sequence_index;              /**< 当前正在执行的节拍索引 */
    bool service_active;                 /**< 服务是否激活（true=正在播放序列中的节拍） */
} dt_buzzer_config_t;

/**
 * @brief 初始化蜂鸣器GPIO引脚及内部状态
 * @param cfg 蜂鸣器配置结构体指针，包含引脚编号等信息
 */
void dt_buzzer_init(dt_buzzer_config_t *cfg);

/**
 * @brief 立即打开蜂鸣器（取消任何正在播放的序列）
 * @param cfg 蜂鸣器配置结构体指针
 */
void dt_buzzer_on(dt_buzzer_config_t *cfg);

/**
 * @brief 立即关闭蜂鸣器（取消任何正在播放的序列）
 * @param cfg 蜂鸣器配置结构体指针
 */
void dt_buzzer_off(dt_buzzer_config_t *cfg);

/**
 * @brief 同步方式使蜂鸣器鸣叫指定时长（阻塞调用，会等待duration_ms时间）
 * @param cfg 蜂鸣器配置结构体指针
 * @param duration_ms 鸣叫持续时间（毫秒）
 */
void dt_buzzer_beep(dt_buzzer_config_t *cfg, uint32_t duration_ms);

/**
 * @brief 异步方式启动蜂鸣器鸣叫（非阻塞，通过service轮询判断何时关闭）
 * @param cfg 蜂鸣器配置结构体指针
 * @param duration_ms 鸣叫持续时间（毫秒）
 * @param now_ms 当前系统时间戳（毫秒），需要与后续传入service的时间源一致
 */
void dt_buzzer_beep_async(dt_buzzer_config_t *cfg, uint32_t duration_ms,
    uint32_t now_ms);

/**
 * @brief 异步播放一段蜂鸣序列（多个节拍，如"滴-滴-滴"）
 * @param cfg 蜂鸣器配置结构体指针
 * @param sequence 指向节拍序列数组的指针
 * @param length 序列长度（节拍数量）
 * @param now_ms 当前系统时间戳（毫秒）
 */
void dt_buzzer_play_sequence(dt_buzzer_config_t *cfg,
    const dt_buzzer_step_t *sequence, uint8_t length, uint32_t now_ms);

/**
 * @brief 蜂鸣器服务函数（需在主循环中周期性调用，用于节拍切换和超时处理）
 * @param cfg 蜂鸣器配置结构体指针
 * @param now_ms 当前系统时间戳（毫秒）
 */
void dt_buzzer_service(dt_buzzer_config_t *cfg, uint32_t now_ms);

/**
 * @brief 兼容任务调度器的服务回调封装（通过context参数传入cfg指针）
 * @param now_ms 当前系统时间戳（毫秒）
 * @param context 指向蜂鸣器配置结构体的void指针（需在调用方转换为dt_buzzer_config_t*）
 */
void dt_buzzer_service_task(uint32_t now_ms, void *context);

#endif
