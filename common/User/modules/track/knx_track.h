#ifndef KNX_TRACK_H
#define KNX_TRACK_H

#include "knx_grayscale.h"

typedef struct {
    knx_grayscale_data_t sensor;
    float line_strength;
    bool line_lost;
    uint32_t update_count;
    uint32_t timestamp_ms;
    knx_status_t last_status;
} knx_track_state_t;

void knx_track_init(void);
knx_status_t knx_track_update(void);
knx_status_t knx_track_capture_black(void);
knx_status_t knx_track_capture_white(void);
void knx_track_set_calibration(const uint16_t black[KNX_GRAYSCALE_CH_NUM],
                               const uint16_t white[KNX_GRAYSCALE_CH_NUM]);
void knx_track_snapshot(knx_track_state_t *out);

#endif
