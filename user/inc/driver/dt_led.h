/**
 * @file dt_led.h
 * @brief 单色LED指示灯驱动头文件
 *        支持任意GPIO引脚控制、可配置高/低电平有效、提供开关和翻转功能。
 */

#ifndef _DT_LED_H_
#define _DT_LED_H_

#include "zf_common_headfile.h"

/**
 * @brief LED控制结构体
 *        抽象一个LED灯，包含引脚、有效电平及当前状态
 */
typedef struct
{
    gpio_pin_enum pin;           /**< LED连接的GPIO引脚 */
    gpio_level_enum on_level;    /**< 点亮时的GPIO电平（GPIO_HIGH或GPIO_LOW） */
    gpio_level_enum off_level;   /**< 熄灭时的GPIO电平 */
    bool is_on;                  /**< 当前是否点亮（true=亮） */
} dt_led_t;

/**
 * @brief 初始化LED引脚
 * @param led LED控制结构体指针
 */
void dt_led_init(dt_led_t *led);

/**
 * @brief 设置LED开关状态
 * @param led LED控制结构体指针
 * @param on true=点亮，false=熄灭
 */
void dt_led_set(dt_led_t *led, bool on);

/**
 * @brief 点亮LED
 * @param led LED控制结构体指针
 */
void dt_led_on(dt_led_t *led);

/**
 * @brief 熄灭LED
 * @param led LED控制结构体指针
 */
void dt_led_off(dt_led_t *led);

/**
 * @brief 翻转LED状态（亮->灭 或 灭->亮）
 * @param led LED控制结构体指针
 */
void dt_led_toggle(dt_led_t *led);

/**
 * @brief 查询LED当前是否点亮
 * @param led LED控制结构体指针
 * @return true=亮，false=灭
 */
bool dt_led_is_on(const dt_led_t *led);

/**
 * @brief 获取LED引脚当前实际的GPIO输出电平
 * @param led LED控制结构体指针
 * @return GPIO_HIGH 或 GPIO_LOW
 */
gpio_level_enum dt_led_get_output_level(const dt_led_t *led);

#endif
