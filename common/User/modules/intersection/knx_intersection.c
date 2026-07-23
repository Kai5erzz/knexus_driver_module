#include "knx_intersection.h"
#include "knx_target_config.h"
#include "knx_time.h"
#include <string.h>

#if defined(KNX_PLATFORM_STM32)
#include "track_tinycnn.h"
#define KNX_INTERSECTION_USE_NN 1
#else
#define KNX_INTERSECTION_USE_NN 0
#define TRACK_NN_ROWS 64
#define TRACK_NN_COLS KNX_GRAYSCALE_CH_NUM
#endif

static uint8_t s_window[TRACK_NN_ROWS][TRACK_NN_COLS];
static uint8_t s_row;
static knx_intersection_result_t s_result;

void knx_intersection_init(void)
{
    memset(s_window, 0, sizeof(s_window));
    memset(&s_result, 0, sizeof(s_result));
    s_row = 0U;
}

static uint8_t normalized_to_u8(float value)
{
    if (value <= 0.0f) return 0U;
    if (value >= 1.0f) return 255U;
    return (uint8_t)(value * 255.0f + 0.5f);
}

static uint8_t bit_count(uint8_t value)
{
    uint8_t count = 0U;
    while (value != 0U) {
        count += value & 1U;
        value >>= 1U;
    }
    return count;
}

knx_status_t knx_intersection_update(const knx_track_state_t *track)
{
    if (track == NULL) return KNX_INVALID_ARG;
    s_result.fresh = false;
    if (track->line_lost) return KNX_NOT_READY;

    for (uint8_t col = 0U; col < TRACK_NN_COLS; ++col) {
        s_window[s_row][col] = normalized_to_u8(track->sensor.normalized[col]);
    }
    s_row++;
    if (s_row < TRACK_NN_ROWS) return KNX_OK;
    s_row = 0U;

#if KNX_INTERSECTION_USE_NN
    track_tinycnn_result_t prediction = {0};
    track_tinycnn_predict_u8(s_window, &prediction);
    s_result.type = (prediction.pred_class == 0U) ?
                    KNX_INTERSECTION_NORMAL : KNX_INTERSECTION_COMPLEX;
    s_result.confidence = prediction.confidence;
#else
    /* MSPM0 fallback: preserve API parity without the STM32 model footprint.
     * Wide activation is treated as a candidate complex intersection. */
    uint32_t wide_rows = 0U;
    for (uint8_t row = 0U; row < TRACK_NN_ROWS; ++row) {
        uint8_t bits = 0U;
        for (uint8_t col = 0U; col < TRACK_NN_COLS; ++col) {
            if (s_window[row][col] >= 128U) bits |= (uint8_t)(1U << col);
        }
        if (bit_count(bits) >= 5U) wide_rows++;
    }
    s_result.type = (wide_rows >= 6U) ? KNX_INTERSECTION_COMPLEX : KNX_INTERSECTION_NORMAL;
    s_result.confidence = (float)((wide_rows >= 6U) ? wide_rows : (TRACK_NN_ROWS - wide_rows)) /
                          (float)TRACK_NN_ROWS;
#endif
    s_result.inference_count++;
    s_result.timestamp_ms = knx_millis();
    s_result.fresh = true;
    return KNX_OK;
}

void knx_intersection_snapshot(knx_intersection_result_t *out)
{
    if (out != NULL) *out = s_result;
}

void knx_intersection_clear_fresh(void)
{
    s_result.fresh = false;
}
