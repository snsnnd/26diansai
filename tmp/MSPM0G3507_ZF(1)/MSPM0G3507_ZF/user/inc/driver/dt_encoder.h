#ifndef _DT_ENCODER_H_
#define _DT_ENCODER_H_

#include "zf_common_headfile.h"

typedef struct {
    gpio_pin_enum    a_pin;
    uint16_t         counts_per_rev;
    volatile uint32_t edge_count;
    uint32_t         last_edge_count;
    float            rpm_lpf_alpha;
    float            rpm;
} dt_encoder_t;

void    dt_encoder_init(dt_encoder_t *enc);
uint32_t dt_encoder_get_edges(dt_encoder_t *enc);
float   dt_encoder_compute_rpm(dt_encoder_t *enc, uint32_t dt_ms);

#endif
