#include "knx_blackbox.h"
#include "knx_time.h"
#include <string.h>

static knx_blackbox_event_t s_events[KNX_BLACKBOX_CAPACITY];
static uint32_t s_write_index;
static uint32_t s_count;
static uint32_t s_dropped_count;

void knx_blackbox_init(void)
{
    memset(s_events, 0, sizeof(s_events));
    s_write_index = 0U;
    s_count = 0U;
    s_dropped_count = 0U;
    knx_blackbox_log(KNX_BLACKBOX_CODE_BOOT, 0U, KNX_OK, 0U, 0U);
}

void knx_blackbox_log(uint16_t code,
                      uint16_t source,
                      int32_t status,
                      uint32_t arg0,
                      uint32_t arg1)
{
    knx_blackbox_event_t *event = &s_events[s_write_index % KNX_BLACKBOX_CAPACITY];
    event->timestamp_ms = knx_millis();
    event->code = code;
    event->source = source;
    event->status = status;
    event->arg0 = arg0;
    event->arg1 = arg1;

    s_write_index++;
    if (s_count < KNX_BLACKBOX_CAPACITY) {
        s_count++;
    } else {
        s_dropped_count++;
    }
}

uint32_t knx_blackbox_count(void)
{
    return s_count;
}

uint32_t knx_blackbox_dropped_count(void)
{
    return s_dropped_count;
}

knx_status_t knx_blackbox_latest(knx_blackbox_event_t *event)
{
    return knx_blackbox_get_recent(0U, event);
}

knx_status_t knx_blackbox_get_recent(uint8_t recent_index,
                                     knx_blackbox_event_t *event)
{
    if (event == NULL || recent_index >= s_count || recent_index >= KNX_BLACKBOX_CAPACITY) {
        return KNX_INVALID_ARG;
    }

    uint32_t index = (s_write_index + KNX_BLACKBOX_CAPACITY - 1U - recent_index)
                   % KNX_BLACKBOX_CAPACITY;
    *event = s_events[index];
    return KNX_OK;
}

void knx_blackbox_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id)
{
    if (octo == NULL) {
        return;
    }

    knx_blackbox_event_t event;
    (void)Octolinker_SendU32(octo, base_id + 0U, s_count);
    (void)Octolinker_SendU32(octo, base_id + 1U, s_dropped_count);
    (void)Octolinker_SendU32(octo, base_id + 2U, s_write_index);

    if (knx_blackbox_latest(&event) == KNX_OK) {
        (void)Octolinker_SendU32(octo, base_id + 3U, event.timestamp_ms);
        (void)Octolinker_SendU32(octo, base_id + 4U, event.code);
        (void)Octolinker_SendU32(octo, base_id + 5U, event.source);
        (void)Octolinker_SendI32(octo, base_id + 6U, event.status);
        (void)Octolinker_SendU32(octo, base_id + 7U, event.arg0);
        (void)Octolinker_SendU32(octo, base_id + 8U, event.arg1);
    }

    for (uint8_t i = 0U; i < 4U; i++) {
        if (knx_blackbox_get_recent(i, &event) == KNX_OK) {
            (void)Octolinker_SendU32(octo, (uint16_t)(base_id + 20U + i), event.code);
            (void)Octolinker_SendU32(octo, (uint16_t)(base_id + 30U + i), event.source);
            (void)Octolinker_SendI32(octo, (uint16_t)(base_id + 40U + i), event.status);
        }
    }
}
