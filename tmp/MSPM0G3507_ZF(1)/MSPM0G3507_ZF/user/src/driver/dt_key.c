#include "driver/dt_key.h"

void dt_key_init(dt_key_config_t *cfg)
{
    if (cfg->active_low)
    {
        gpio_init(cfg->pin, GPI, GPIO_HIGH, GPI_PULL_UP);
    }
    else
    {
        gpio_init(cfg->pin, GPI, GPIO_LOW, GPI_PULL_DOWN);
    }
}

uint8_t dt_key_is_pressed(dt_key_config_t *cfg)
{
    uint8_t level = gpio_get_level(cfg->pin);
    if (cfg->active_low)
    {
        return (level == 0) ? 1 : 0;
    }
    else
    {
        return (level == 1) ? 1 : 0;
    }
}
