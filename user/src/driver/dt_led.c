/**
 * @file dt_led.c
 * @brief 单色LED指示灯驱动实现
 *        提供通用的GPIO LED控制，支持高低电平有效配置。
 */

#include "driver/dt_led.h"

/**
 * @brief 初始化LED
 *        配置GPIO为推挽输出，初始状态为熄灭
 * @param led LED控制结构体指针
 */
void dt_led_init(dt_led_t *led)
{
    if (led == NULL)
    {
        return;
    }

    gpio_init(led->pin, GPO, led->off_level, GPO_PUSH_PULL); /* 推挽输出，初始熄灭 */
    led->is_on = false;  /* 记录状态为灭 */
}

/**
 * @brief 设置LED开关状态
 *        根据on参数选择输出on_level或off_level电平
 * @param led LED控制结构体指针
 * @param on true=点亮，false=熄灭
 */
void dt_led_set(dt_led_t *led, bool on)
{
    if (led == NULL)
    {
        return;
    }

    gpio_set_level(led->pin, on ? led->on_level : led->off_level);
    led->is_on = on;
}

/**
 * @brief 点亮LED
 * @param led LED控制结构体指针
 */
void dt_led_on(dt_led_t *led)
{
    dt_led_set(led, true);
}

/**
 * @brief 熄灭LED
 * @param led LED控制结构体指针
 */
void dt_led_off(dt_led_t *led)
{
    dt_led_set(led, false);
}

/**
 * @brief 翻转LED状态
 * @param led LED控制结构体指针
 */
void dt_led_toggle(dt_led_t *led)
{
    if (led != NULL)
    {
        dt_led_set(led, !led->is_on);  /* 取反当前状态 */
    }
}

/**
 * @brief 检查LED是否点亮
 * @param led LED控制结构体指针
 * @return true=亮，false=灭或指针为NULL
 */
bool dt_led_is_on(const dt_led_t *led)
{
    return led != NULL && led->is_on;
}

/**
 * @brief 读取LED引脚当前的GPIO电平
 * @param led LED控制结构体指针
 * @return GPIO_HIGH 或 GPIO_LOW（指针为NULL时返回GPIO_LOW）
 */
gpio_level_enum dt_led_get_output_level(const dt_led_t *led)
{
    if (led == NULL)
    {
        return GPIO_LOW;
    }

    return gpio_get_level(led->pin) == GPIO_HIGH ? GPIO_HIGH : GPIO_LOW;
}
