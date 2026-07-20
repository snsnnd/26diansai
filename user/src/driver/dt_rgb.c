/**
 * @file dt_rgb.c
 * @brief RGB三色LED驱动实现
 *        通过三个独立的GPIO引脚分别控制红、绿、蓝通道，
 *        支持组合颜色显示。可配置为共阴或共阳极接法。
 */

#include "driver/dt_rgb.h"
#include "pin_mapping.h"

/**
 * @brief 设置单个颜色通道的输出电平
 * @param pin 通道GPIO引脚
 * @param on_level 点亮电平
 * @param off_level 熄灭电平
 * @param on true=点亮该通道，false=熄灭该通道
 */
static void dt_rgb_set_channel(gpio_pin_enum pin, gpio_level_enum on_level,
    gpio_level_enum off_level, bool on)
{
    gpio_set_level(pin, on ? on_level : off_level);
}

/**
 * @brief 初始化RGB LED
 *        三个通道引脚均配置为推挽输出，初始状态为熄灭
 * @param rgb RGB LED结构体指针
 */
void dt_rgb_init(dt_rgb_t *rgb)
{
    if (rgb == NULL)
    {
        return;
    }

    /* 配置 RGB 三个通道的 GPIO 引脚 */
    rgb->red_pin   = PIN_RGB_R;          // PB0 — 红色通道
    rgb->green_pin = PIN_RGB_G;          // PB1 — 绿色通道
    rgb->blue_pin  = PIN_RGB_B;          // PA29 — 蓝色通道

    /*
    *   共阳接法（公共端接 VCC），所以：
    *   低电平 (GPIO_LOW)  → 点亮（VCC → LED → GPIO 有电流）
    *   高电平 (GPIO_HIGH) → 熄灭（VCC 和 GPIO 之间无压差）
    */
    rgb->on_level  = GPIO_LOW;   // 共阳：低电平点亮
    rgb->off_level = GPIO_HIGH;  // 共阳：高电平熄灭
    
    /* 三个通道均初始化为熄灭状态 */
    gpio_init(rgb->red_pin, GPO, rgb->off_level, GPO_PUSH_PULL);
    gpio_init(rgb->green_pin, GPO, rgb->off_level, GPO_PUSH_PULL);
    gpio_init(rgb->blue_pin, GPO, rgb->off_level, GPO_PUSH_PULL);
    rgb->color = DT_RGB_COLOR_OFF; /* 初始颜色：灭 */

}

/**
 * @brief 设置RGB LED颜色
 *        根据颜色枚举值的位掩码分别控制三个通道的亮灭。
 *        例如：DT_RGB_COLOR_YELLOW=0x03 表示R和G同时亮。
 * @param rgb RGB LED结构体指针
 * @param color 颜色枚举值（仅低3位有效）
 */
void dt_rgb_set_color(dt_rgb_t *rgb, dt_rgb_color_t color)
{
    uint8_t channels;

    if (rgb == NULL)
    {
        return;
    }

    /* 提取有效的颜色位（低3位，屏蔽高位干扰） */
    channels = (uint8_t)color & (uint8_t)DT_RGB_COLOR_WHITE;

    /* 分别控制三个通道 */
    dt_rgb_set_channel(rgb->red_pin, rgb->on_level, rgb->off_level,
        (channels & (uint8_t)DT_RGB_COLOR_RED) != 0u);
    dt_rgb_set_channel(rgb->green_pin, rgb->on_level, rgb->off_level,
        (channels & (uint8_t)DT_RGB_COLOR_GREEN) != 0u);
    dt_rgb_set_channel(rgb->blue_pin, rgb->on_level, rgb->off_level,
        (channels & (uint8_t)DT_RGB_COLOR_BLUE) != 0u);

    rgb->color = (dt_rgb_color_t)channels; /* 保存当前颜色 */
}

/**
 * @brief 关闭RGB LED（全灭）
 * @param rgb RGB LED结构体指针
 */
void dt_rgb_off(dt_rgb_t *rgb)
{
    dt_rgb_set_color(rgb, DT_RGB_COLOR_OFF);
}

/**
 * @brief 获取当前设置的颜色值
 * @param rgb RGB LED结构体指针
 * @return 颜色枚举值（指针为NULL时返回DT_RGB_COLOR_OFF）
 */
dt_rgb_color_t dt_rgb_get_color(const dt_rgb_t *rgb)
{
    return rgb == NULL ? DT_RGB_COLOR_OFF : rgb->color;
}

/**
 * @brief 读取三个通道的实际GPIO输出电平，解析为颜色位掩码
 *        可用于硬件诊断，确认输出状态是否正确
 * @param rgb RGB LED结构体指针
 * @return 颜色位掩码（bit0=R，bit1=G，bit2=B）
 */
uint8_t dt_rgb_get_output_levels(const dt_rgb_t *rgb)
{
    uint8_t levels = 0u;

    if (rgb == NULL)
    {
        return levels;
    }

    /* 检查每个通道引脚是否为高电平 */
    if (gpio_get_level(rgb->red_pin) == GPIO_HIGH)
    {
        levels |= (uint8_t)DT_RGB_COLOR_RED;
    }
    if (gpio_get_level(rgb->green_pin) == GPIO_HIGH)
    {
        levels |= (uint8_t)DT_RGB_COLOR_GREEN;
    }
    if (gpio_get_level(rgb->blue_pin) == GPIO_HIGH)
    {
        levels |= (uint8_t)DT_RGB_COLOR_BLUE;
    }
    return levels;
}

void dt_rgb_test(void){
    dt_rgb_t tmp;

    dt_rgb_init(&tmp);

    /* 三色循环：红 → 绿 → 蓝 → 灭，每个状态持续 500ms */
    while (1)
    {
        dt_rgb_set_color(&tmp, DT_RGB_COLOR_RED);
        system_delay_ms(500);

        dt_rgb_set_color(&tmp, DT_RGB_COLOR_GREEN);
        system_delay_ms(500);

        dt_rgb_set_color(&tmp, DT_RGB_COLOR_BLUE);
        system_delay_ms(500);

        dt_rgb_set_color(&tmp, DT_RGB_COLOR_OFF);
        system_delay_ms(2000);
    }
    
    // 实际现象是蓝绿可以用，红不可以
}
