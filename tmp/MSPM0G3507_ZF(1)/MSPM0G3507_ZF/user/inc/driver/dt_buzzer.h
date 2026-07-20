#ifndef _DT_BUZZER_H_
#define _DT_BUZZER_H_

#include "zf_common_headfile.h"

typedef struct {
    gpio_pin_enum  pin;
} dt_buzzer_config_t;

void dt_buzzer_init(dt_buzzer_config_t *cfg);
void dt_buzzer_on(dt_buzzer_config_t *cfg);
void dt_buzzer_off(dt_buzzer_config_t *cfg);
void dt_buzzer_beep(dt_buzzer_config_t *cfg, uint32_t duration_ms);

#endif
