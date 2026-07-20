#include "driver/dt_buzzer.h"

void dt_buzzer_init(dt_buzzer_config_t *cfg)
{
    gpio_init(cfg->pin, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void dt_buzzer_on(dt_buzzer_config_t *cfg)
{
    gpio_high(cfg->pin);
}

void dt_buzzer_off(dt_buzzer_config_t *cfg)
{
    gpio_low(cfg->pin);
}

void dt_buzzer_beep(dt_buzzer_config_t *cfg, uint32_t duration_ms)
{
    dt_buzzer_on(cfg);
    system_delay_ms(duration_ms);
    dt_buzzer_off(cfg);
}
