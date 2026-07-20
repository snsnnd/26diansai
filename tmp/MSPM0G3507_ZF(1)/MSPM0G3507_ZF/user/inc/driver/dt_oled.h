#ifndef _DT_OLED_H_
#define _DT_OLED_H_

#include "zf_common_headfile.h"

#define DT_OLED_DEFAULT_ADDR   0x3C
#define DT_OLED_WIDTH          128
#define DT_OLED_HEIGHT         64
#define DT_OLED_PAGE_COUNT     8

typedef struct {
    soft_iic_info_struct  iic;
} dt_oled_config_t;

void dt_oled_init(dt_oled_config_t *cfg);
void dt_oled_clear(dt_oled_config_t *cfg);
void dt_oled_fill(dt_oled_config_t *cfg, uint8_t data);
void dt_oled_set_pos(dt_oled_config_t *cfg, uint8_t x, uint8_t y);
void dt_oled_show_char(dt_oled_config_t *cfg, uint8_t x, uint8_t y, char ch);
void dt_oled_show_string(dt_oled_config_t *cfg, uint8_t x, uint8_t y, const char *str);
void dt_oled_show_num(dt_oled_config_t *cfg, uint8_t x, uint8_t y, int32_t num, uint8_t len);
void dt_oled_show_hex(dt_oled_config_t *cfg, uint8_t x, uint8_t y, uint32_t num, uint8_t len);
void dt_oled_show_float(dt_oled_config_t *cfg, uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len);

#endif
