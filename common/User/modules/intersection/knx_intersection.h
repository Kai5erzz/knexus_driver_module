#ifndef KNX_INTERSECTION_H
#define KNX_INTERSECTION_H

#include "knx_track.h"

typedef enum {
    KNX_INTERSECTION_UNKNOWN = 0,
    KNX_INTERSECTION_NORMAL = 1,
    KNX_INTERSECTION_COMPLEX = 2,
} knx_intersection_class_t;

typedef struct {
    knx_intersection_class_t type;
    float confidence;
    bool fresh;
    uint32_t inference_count;
    uint32_t timestamp_ms;
} knx_intersection_result_t;

void knx_intersection_init(void);
knx_status_t knx_intersection_update(const knx_track_state_t *track);
void knx_intersection_snapshot(knx_intersection_result_t *out);
void knx_intersection_clear_fresh(void);

#endif
