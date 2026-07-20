#include "driver/dt_encoder.h"

static void dt_encoder_isr(uint32_t event, void *ptr)
{
    dt_encoder_t *enc = (dt_encoder_t *)ptr;
    (void)event;

    enc->edge_count++;
}

void dt_encoder_init(dt_encoder_t *enc)
{
    if (enc->rpm_lpf_alpha <= 0.0f || enc->rpm_lpf_alpha > 1.0f)
    {
        enc->rpm_lpf_alpha = 0.35f;
    }

    enc->edge_count = 0;
    enc->last_edge_count = 0;
    enc->rpm        = 0.0f;

    gpio_init(enc->a_pin, GPI, GPIO_LOW, GPI_PULL_UP);

    exti_init(enc->a_pin, EXTI_TRIGGER_BOTH, dt_encoder_isr, enc);
}

uint32_t dt_encoder_get_edges(dt_encoder_t *enc)
{
    uint32_t edges;

    __disable_irq();
    edges = enc->edge_count;
    __enable_irq();

    return edges;
}

float dt_encoder_compute_rpm(dt_encoder_t *enc, uint32_t dt_ms)
{
    uint32_t edge_delta;
    float rpm_raw;

    __disable_irq();
    edge_delta = enc->edge_count - enc->last_edge_count;
    enc->last_edge_count = enc->edge_count;
    __enable_irq();

    if (edge_delta != 0 && dt_ms > 0 && enc->counts_per_rev > 0)
    {
        rpm_raw = (float)edge_delta * 60000.0f / (float)enc->counts_per_rev / (float)dt_ms;
    }
    else
    {
        rpm_raw = 0.0f;
    }

    enc->rpm = enc->rpm * (1.0f - enc->rpm_lpf_alpha) + rpm_raw * enc->rpm_lpf_alpha;
    return enc->rpm;
}
