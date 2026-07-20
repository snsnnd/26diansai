/**
 * @file dt_rgb.h
 * @brief RGB三色LED驱动头文件
 *        支持红、绿、蓝单色以及黄、紫、青、白复合颜色，
 *        可配置高/低电平有效，兼容共阳和共阴两种接法。
 */

#ifndef _DT_RGB_H_
#define _DT_RGB_H_

#include "zf_common_headfile.h"

/**
 * @brief RGB颜色枚举（按位定义）
 *        bit0=红, bit1=绿, bit2=蓝
 *        复合颜色通过按位或组合产生
 */
typedef enum
{
    DT_RGB_COLOR_OFF = 0x00u,                          /**< 全灭 */
    DT_RGB_COLOR_RED = 0x01u,                          /**< 红色（仅R通道亮） */
    DT_RGB_COLOR_GREEN = 0x02u,                        /**< 绿色（仅G通道亮） */
    DT_RGB_COLOR_BLUE = 0x04u,                         /**< 蓝色（仅B通道亮） */
    DT_RGB_COLOR_YELLOW = DT_RGB_COLOR_RED | DT_RGB_COLOR_GREEN,  /**< 黄色（R+G） */
    DT_RGB_COLOR_MAGENTA = DT_RGB_COLOR_RED | DT_RGB_COLOR_BLUE,  /**< 紫色（R+B） */
    DT_RGB_COLOR_CYAN = DT_RGB_COLOR_GREEN | DT_RGB_COLOR_BLUE,   /**< 青色（G+B） */
    DT_RGB_COLOR_WHITE = DT_RGB_COLOR_RED | DT_RGB_COLOR_GREEN |
        DT_RGB_COLOR_BLUE                                         /**< 白色（R+G+B） */
} dt_rgb_color_t;

/**
 * @brief RGB LED配置结构体
 *        定义三个颜色通道的GPIO引脚、有效电平及当前颜色
 */
typedef struct
{
    gpio_pin_enum red_pin;     /**< 红色通道GPIO引脚 */
    gpio_pin_enum green_pin;   /**< 绿色通道GPIO引脚 */
    gpio_pin_enum blue_pin;    /**< 蓝色通道GPIO引脚 */
    gpio_level_enum on_level;  /**< 点亮时的电平（共阴=GPIO_HIGH，共阳=GPIO_LOW） */
    gpio_level_enum off_level; /**< 熄灭时的电平 */
    dt_rgb_color_t color;      /**< 当前显示的颜色 */
} dt_rgb_t;

/**
 * @brief 初始化RGB LED的三个GPIO引脚
 * @param rgb RGB LED结构体指针
 */
void dt_rgb_init(dt_rgb_t *rgb);

/**
 * @brief 设置RGB LED显示颜色
 * @param rgb RGB LED结构体指针
 * @param color 目标颜色（使用dt_rgb_color_t枚举）
 */
void dt_rgb_set_color(dt_rgb_t *rgb, dt_rgb_color_t color);

/**
 * @brief 关闭RGB LED（所有通道熄灭）
 * @param rgb RGB LED结构体指针
 */
void dt_rgb_off(dt_rgb_t *rgb);

/**
 * @brief 获取当前设置的颜色
 * @param rgb RGB LED结构体指针
 * @return 当前颜色枚举值
 */
dt_rgb_color_t dt_rgb_get_color(const dt_rgb_t *rgb);

/**
 * @brief 读取当前各引脚的GPIO实际电平，解析为颜色值
 *        用于诊断输出状态（与dt_rgb_get_color可能不同，因为引脚可能被外部改变）
 * @param rgb RGB LED结构体指针
 * @return 根据GPIO电平解析出的颜色位掩码
 */
uint8_t dt_rgb_get_output_levels(const dt_rgb_t *rgb);


/**
 * @brief rgb测试函数    
 */
void dt_rgb_test(void);

#endif

