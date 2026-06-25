#ifndef KNX_FILTER_H
#define KNX_FILTER_H

#include <stdint.h>
#include <stddef.h>

/* Simple moving average filter (float) */
#define KNX_FILTER_MAX_WINDOW  32

typedef struct {
    float    buf[KNX_FILTER_MAX_WINDOW];
    uint32_t idx;
    uint32_t count;      /* number of samples added so far (up to window) */
    uint32_t window;     /* window size (1..KNX_FILTER_MAX_WINDOW) */
    float    sum;        /* running sum for O(1) update */
} knx_mavg_t;

/* Initialize the filter with a given window size */
void  knx_mavg_init(knx_mavg_t *f, uint32_t window);

/* Push a new sample, return the current average */
float knx_mavg_update(knx_mavg_t *f, float sample);

/* Get current average without adding a new sample */
float knx_mavg_get(const knx_mavg_t *f);

/* Reset the filter */
void  knx_mavg_reset(knx_mavg_t *f);

#endif /* KNX_FILTER_H */
