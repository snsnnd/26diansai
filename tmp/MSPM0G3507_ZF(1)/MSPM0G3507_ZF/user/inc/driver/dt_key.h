#ifndef _DT_KEY_H_
#define _DT_KEY_H_

#include "zf_common_headfile.h"

typedef struct {
    gpio_pin_enum  pin;
    uint8_t        active_low;
} dt_key_config_t;

void    dt_key_init(dt_key_config_t *cfg);
uint8_t dt_key_is_pressed(dt_key_config_t *cfg);

#endif
