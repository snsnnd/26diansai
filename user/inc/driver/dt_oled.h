/**
 * @file dt_oled.h
 * @brief 0.96寸 OLED 显示屏驱动头文件（SSD1306控制器）
 *        128x64 分辨率，IIC接口，支持页寻址模式。
 *        提供帧缓冲机制（双缓冲），减少IIC通信次数提高性能。
 */

#ifndef _DT_OLED_H_
#define _DT_OLED_H_

#include "zf_common_headfile.h"

#define DT_OLED_DEFAULT_ADDR   0x3C   /**< SSD1306 OLED I2C默认地址 */
#define DT_OLED_WIDTH          128    /**< 屏幕宽度（像素） */
#define DT_OLED_HEIGHT         64     /**< 屏幕高度（像素） */
#define DT_OLED_PAGE_COUNT     8      /**< 页数（64行 / 8行每页 = 8页） */

/**
 * @brief OLED配置结构体
 *        包含IIC接口、帧缓冲区和脏页标记
 */
typedef struct {
    soft_iic_info_struct iic;                                     /**< IIC接口结构体 */
    uint8_t framebuffer[DT_OLED_PAGE_COUNT][DT_OLED_WIDTH];     /**< 帧缓冲：8页 x 128字节 */
    uint8_t dirty_pages;                                         /**< 脏页标记位图（bit0~7对应page0~7） */
} dt_oled_config_t;

/**
 * @brief 初始化OLED显示屏
 *        发送初始化命令序列、清空帧缓冲并标记所有页为脏
 * @param cfg OLED配置结构体指针
 */
void dt_oled_init(dt_oled_config_t *cfg);

/**
 * @brief 清屏（填充0x00，所有像素熄灭）
 * @param cfg OLED配置结构体指针
 */
void dt_oled_clear(dt_oled_config_t *cfg);

/**
 * @brief 全屏填充指定数据
 * @param cfg OLED配置结构体指针
 * @param data 填充数据（每个bit对应一个像素，1=亮）
 */
void dt_oled_fill(dt_oled_config_t *cfg, uint8_t data);

/**
 * @brief 设置OLED页地址和列地址（写入位置）
 * @param cfg OLED配置结构体指针
 * @param x 列地址（0~127）
 * @param y 页地址（0~7）
 */
void dt_oled_set_pos(dt_oled_config_t *cfg, uint8_t x, uint8_t y);

/**
 * @brief 在指定位置显示一个字符（6x8点阵）
 * @param cfg OLED配置结构体指针
 * @param x 起始列（0~127）
 * @param y 页地址（0~7）
 * @param ch 要显示的字符
 */
void dt_oled_show_char(dt_oled_config_t *cfg, uint8_t x, uint8_t y, char ch);

/**
 * @brief 在指定位置显示字符串
 *        自动换行处理（超出宽度换到下一页）
 * @param cfg OLED配置结构体指针
 * @param x 起始列
 * @param y 起始页
 * @param str 以'\0'结尾的字符串
 */
void dt_oled_show_string(dt_oled_config_t *cfg, uint8_t x, uint8_t y, const char *str);

/**
 * @brief 显示整数
 * @param cfg OLED配置结构体指针
 * @param x 起始列
 * @param y 起始页
 * @param num 整数（支持负数）
 * @param len 数字位数
 */
void dt_oled_show_num(dt_oled_config_t *cfg, uint8_t x, uint8_t y, int32_t num, uint8_t len);

/**
 * @brief 显示十六进制数
 * @param cfg OLED配置结构体指针
 * @param x 起始列
 * @param y 起始页
 * @param num 32位无符号整数
 * @param len 显示的十六进制位数
 */
void dt_oled_show_hex(dt_oled_config_t *cfg, uint8_t x, uint8_t y, uint32_t num, uint8_t len);

/**
 * @brief 显示浮点数
 * @param cfg OLED配置结构体指针
 * @param x 起始列
 * @param y 起始页
 * @param num 浮点数
 * @param int_len 整数部分位数
 * @param dec_len 小数部分位数
 */
void dt_oled_show_float(dt_oled_config_t *cfg, uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len);

/**
 * @brief 将指定页标记为脏（需要刷新到屏幕）
 * @param cfg OLED配置结构体指针
 * @param page 页索引（0~7）
 */
void dt_oled_mark_page_dirty(dt_oled_config_t *cfg, uint8_t page);

/**
 * @brief 将指定行（页的别名）标记为脏
 * @param cfg OLED配置结构体指针
 * @param line 行/页索引
 */
void dt_oled_mark_line_dirty(dt_oled_config_t *cfg, uint8_t line);

/**
 * @brief 刷新指定页到OLED（通过IIC发送数据）
 * @param cfg OLED配置结构体指针
 * @param page 页索引
 */
void dt_oled_refresh_page(dt_oled_config_t *cfg, uint8_t page);

/**
 * @brief 刷新指定行（页的别名）到OLED
 * @param cfg OLED配置结构体指针
 * @param line 行/页索引
 */
void dt_oled_refresh_line(dt_oled_config_t *cfg, uint8_t line);

/**
 * @brief 只刷新第一个脏页（用于分散刷新，减少单次IIC通信耗时）
 * @param cfg OLED配置结构体指针
 */
void dt_oled_refresh_one_dirty(dt_oled_config_t *cfg);

/**
 * @brief 刷新所有脏页
 * @param cfg OLED配置结构体指针
 */
void dt_oled_refresh_dirty(dt_oled_config_t *cfg);

/**
 * @brief 任务调度器回调封装，用于定时刷新脏页
 * @param now_ms 当前时间戳（未使用）
 * @param context 指向dt_oled_config_t的指针
 */
void dt_oled_refresh_task(uint32_t now_ms, void *context);

#endif
