#ifndef KNX_HEALTH_H
#define KNX_HEALTH_H

#include "knx_types.h"
#include "octolinker.h"
#include <stdint.h>

typedef enum {
    KNX_HEALTH_SOURCE_SYS = 0,
    KNX_HEALTH_SOURCE_SAFETY,
    KNX_HEALTH_SOURCE_VISION,
    KNX_HEALTH_SOURCE_DM_IMU_L1,
    KNX_HEALTH_SOURCE_GIMBAL,
    KNX_HEALTH_SOURCE_JC,
    KNX_HEALTH_SOURCE_HOST_COMM,
    KNX_HEALTH_SOURCE_PARAM_HOST,
    KNX_HEALTH_SOURCE_COUNT,
} knx_health_source_t;

typedef enum {
    KNX_HEALTH_STATE_UNKNOWN = 0,
    KNX_HEALTH_STATE_OK,
    KNX_HEALTH_STATE_WARN,
    KNX_HEALTH_STATE_FAULT,
    KNX_HEALTH_STATE_STALE,
    KNX_HEALTH_STATE_DISABLED,
} knx_health_state_t;

typedef struct {
    knx_health_state_t state;
    int32_t status;
    uint32_t flags;
    uint32_t error_count;
    uint32_t update_count;
    uint32_t last_update_ms;
} knx_health_record_t;

void knx_health_init(void);
knx_status_t knx_health_report(knx_health_source_t source,
                               knx_health_state_t state,
                               int32_t status,
                               uint32_t flags,
                               uint32_t error_count);
knx_status_t knx_health_heartbeat(knx_health_source_t source);
knx_status_t knx_health_get(knx_health_source_t source,
                            knx_health_record_t *record);
uint32_t knx_health_age_ms(knx_health_source_t source);
knx_health_state_t knx_health_overall_state(void);
void knx_health_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id);

#endif /* KNX_HEALTH_H */
