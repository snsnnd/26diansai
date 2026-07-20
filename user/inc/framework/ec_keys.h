/**
 * @file    ec_keys.h
 * @brief   按键输入管理与消抖模块
 * @details 本模块提供了3个独立按键（KEY1/KEY2/KEY3）的完整输入管理方案，
 *          支持硬件外部中断触发、软件消抖、启动锁定及紧急停止功能。
 *
 *          【按键系统的分层设计】
 *          1. 硬件层：3个按键连接到 GPIO 引脚，配置为上拉输入
 *          2. 中断层：EXTI 外部中断检测下降沿（按键按下）
 *          3. 消抖层：软件计时消抖，避免机械弹跳产生的误触发
 *          4. 队列层：环形缓冲区存储按键事件，解耦中断产生和业务消费
 *          5. 安全层：启动锁定防止上电误触，紧急停止用于安全处理
 *
 *          【按键事件流】
 *          按键按下 → GPIO下降沿中断 → 消抖判断 → 入队 → 业务层ec_keys_pop读取
 *
 *          【紧急停止机制】
 *          KEY1 按键被设计为紧急停止键(emergency key)。当 KEY1 被按下时：
 *          1. 调用注册的紧急停止钩子函数(emergency_hook)
 *          2. 设置紧急停止标志(emergency_pending)
 *          3. 同时作为普通按键事件入队
 *          业务层可以通过 ec_keys_emergency_pending() 查询并清除急停状态。
 */
#ifndef EC_KEYS_H
#define EC_KEYS_H

#include <stdbool.h>
#include <stdint.h>

#include "zf_common_headfile.h"

/**
 * @brief 紧急停止钩子函数类型
 * @details 当 KEY1 被按下时，如果注册了此钩子，会在中断上下文中被调用。
 *          钩子通常用于执行立即停止电机、设置安全状态等紧急操作。
 *
 * @param context 用户自定义上下文指针
 * @note 此函数在中断上下文中执行，必须遵守 ISR 约束：
 *       - 非阻塞
 *       - 执行时间短
 *       - 不可调用 printf/delay 等非可重入函数
 *       - 不可调用可能触发调度的函数
 */
typedef void (*ec_keys_emergency_hook_t)(void *context);

/**
 * @brief 按键模块配置结构体
 * @details 初始化按键模块时需要提供此结构体，配置引脚映射、
 *          消抖参数和安全参数。这种通过结构体集中配置的设计
 *          使得按键模块可以灵活适应不同硬件布局。
 *
 * @note 所有字段必须在调用 ec_keys_init 前正确填写
 */
typedef struct
{
    gpio_pin_enum key1_pin;        /**< KEY1 的 GPIO 引脚（紧急停止键） */
    gpio_pin_enum key2_pin;        /**< KEY2 的 GPIO 引脚 */
    gpio_pin_enum key3_pin;        /**< KEY3 的 GPIO 引脚 */
    uint32_t debounce_ms;          /**< 按键消抖时间（毫秒）。两次按键事件之间的最小间隔，
                                        用于滤除机械弹跳产生的虚假信号。通常设置为 20-50ms */
    uint32_t startup_lock_ms;      /**< 启动锁定时间（毫秒）。系统启动后在此时间内屏蔽所有按键事件，
                                        防止上电/复位过程中因电平不稳定导致的误触发。
                                        建议设置为 500-2000ms */
    ec_keys_emergency_hook_t emergency_hook;  /**< 紧急停止钩子函数指针。KEY1按下时在ISR中调用。
                                                   传NULL表示不注册钩子 */
    void *emergency_context;       /**< 传递给紧急停止钩子的上下文指针 */
} ec_keys_config_t;

/**
 * @brief 初始化按键模块
 * @param config 按键配置结构体指针。传入 NULL 将跳过初始化
 * @note 此函数会：
 *       1. 配置三个按键的 GPIO 为上拉输入模式
 *       2. 配置外部中断(EXTI)为下降沿触发
 *       3. 清零按键事件队列
 *       4. 记录启动时间用于启动锁定
 */
void ec_keys_init(const ec_keys_config_t *config);

/**
 * @brief 从按键事件队列中弹出一个按键事件
 * @details 非阻塞式按键事件读取。如果队列中有按键事件，将最早的事件
 *          复制到 key 指向的变量中并从队列移除。
 *
 *          返回值约定：
 *          - key = 1: KEY1 被按下
 *          - key = 2: KEY2 被按下
 *          - key = 3: KEY3 被按下
 *
 * @param key 输出参数，用于接收按键编号（1/2/3）
 * @return true=成功弹出按键事件，false=队列为空或参数无效
 */
bool ec_keys_pop(uint8_t *key);

/**
 * @brief 查询并清除紧急停止状态
 * @details 检查是否有紧急停止事件发生（KEY1被按下）。
 *          此函数会自动清除紧急停止标志，因此多次调用只有第一次返回 true。
 *          这是一种"边沿触发"而非"电平触发"的行为模式。
 *
 * @return true=有紧急停止事件待处理，false=无紧急停止事件
 * @note 此函数是中断安全的，内部会禁用中断来原子地读取和清除标志
 */
bool ec_keys_emergency_pending(void);

#endif
