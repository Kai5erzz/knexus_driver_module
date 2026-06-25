/* Moving average filter implementation */
#include "knx_filter.h"

void knx_mavg_init(knx_mavg_t *f, uint32_t window)
{
    if (f == NULL || window == 0) return;
    if (window > KNX_FILTER_MAX_WINDOW) {
        window = KNX_FILTER_MAX_WINDOW;
    }
    f->window = window;
    f->idx    = 0;
    f->count  = 0;
    f->sum    = 0.0f;
    for (uint32_t i = 0; i < KNX_FILTER_MAX_WINDOW; i++) {
        f->buf[i] = 0.0f;
    }
}

float knx_mavg_update(knx_mavg_t *f, float sample)
{
    if (f == NULL) return 0.0f;

    /* If buffer is full, subtract the oldest sample */
    if (f->count >= f->window) {
        f->sum -= f->buf[f->idx];
    }

    f->buf[f->idx] = sample;
    f->sum += sample;
    f->idx = (f->idx + 1) % f->window;

    if (f->count < f->window) {
        f->count++;
    }

    return f->sum / (float)f->count;
}

float knx_mavg_get(const knx_mavg_t *f)
{
    if (f == NULL || f->count == 0) return 0.0f;
    return f->sum / (float)f->count;
}

void knx_mavg_reset(knx_mavg_t *f)
{
    if (f == NULL) return;
    f->idx   = 0;
    f->count = 0;
    f->sum   = 0.0f;
    for (uint32_t i = 0; i < KNX_FILTER_MAX_WINDOW; i++) {
        f->buf[i] = 0.0f;
    }
}
