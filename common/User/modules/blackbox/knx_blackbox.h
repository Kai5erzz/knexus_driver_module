#ifndef KNX_BLACKBOX_H
#define KNX_BLACKBOX_H

#include "knx_types.h"
#include "octolinker.h"
#include <stdint.h>

#define KNX_BLACKBOX_CAPACITY 32U

typedef enum {
    KNX_BLACKBOX_CODE_BOOT = 1,
    KNX_BLACKBOX_CODE_HEALTH_CHANGE = 2,
    KNX_BLACKBOX_CODE_CAN_UNMATCHED = 3,
    KNX_BLACKBOX_CODE_SAFETY_CLEAR = 4,
} knx_blackbox_code_t;

typedef struct {
    uint32_t timestamp_ms;
    uint16_t code;
    uint16_t source;
    int32_t status;
    uint32_t arg0;
    uint32_t arg1;
} knx_blackbox_event_t;

void knx_blackbox_init(void);
void knx_blackbox_log(uint16_t code,
                      uint16_t source,
                      int32_t status,
                      uint32_t arg0,
                      uint32_t arg1);
uint32_t knx_blackbox_count(void);
uint32_t knx_blackbox_dropped_count(void);
knx_status_t knx_blackbox_latest(knx_blackbox_event_t *event);
knx_status_t knx_blackbox_get_recent(uint8_t recent_index,
                                     knx_blackbox_event_t *event);
void knx_blackbox_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id);

#endif /* KNX_BLACKBOX_H */
