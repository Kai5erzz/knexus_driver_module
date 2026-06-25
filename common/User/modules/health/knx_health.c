#include "knx_health.h"
#include "knx_blackbox.h"
#include "knx_time.h"
#include <string.h>

static knx_health_record_t s_records[KNX_HEALTH_SOURCE_COUNT];

static uint8_t is_valid_source(knx_health_source_t source)
{
    return ((uint32_t)source < (uint32_t)KNX_HEALTH_SOURCE_COUNT) ? 1U : 0U;
}

static uint8_t health_rank(knx_health_state_t state)
{
    switch (state) {
    case KNX_HEALTH_STATE_FAULT: return 5U;
    case KNX_HEALTH_STATE_STALE: return 4U;
    case KNX_HEALTH_STATE_WARN: return 3U;
    case KNX_HEALTH_STATE_UNKNOWN: return 2U;
    case KNX_HEALTH_STATE_DISABLED: return 1U;
    case KNX_HEALTH_STATE_OK:
    default:
        return 0U;
    }
}

void knx_health_init(void)
{
    memset(s_records, 0, sizeof(s_records));
    for (uint8_t i = 0U; i < (uint8_t)KNX_HEALTH_SOURCE_COUNT; i++) {
        s_records[i].state = KNX_HEALTH_STATE_UNKNOWN;
        s_records[i].status = KNX_NOT_READY;
    }
}

knx_status_t knx_health_report(knx_health_source_t source,
                               knx_health_state_t state,
                               int32_t status,
                               uint32_t flags,
                               uint32_t error_count)
{
    if (is_valid_source(source) == 0U) {
        return KNX_INVALID_ARG;
    }

    knx_health_record_t *record = &s_records[(uint32_t)source];
    uint8_t changed = (record->state != state ||
                       record->status != status ||
                       record->flags != flags) ? 1U : 0U;

    record->state = state;
    record->status = status;
    record->flags = flags;
    record->error_count = error_count;
    record->update_count++;
    record->last_update_ms = knx_millis();

    if (changed != 0U) {
        knx_blackbox_log(KNX_BLACKBOX_CODE_HEALTH_CHANGE,
                         (uint16_t)source,
                         status,
                         ((uint32_t)state << 24) | (flags & 0x00FFFFFFUL),
                         error_count);
    }
    return KNX_OK;
}

knx_status_t knx_health_heartbeat(knx_health_source_t source)
{
    if (is_valid_source(source) == 0U) {
        return KNX_INVALID_ARG;
    }

    knx_health_record_t *record = &s_records[(uint32_t)source];
    return knx_health_report(source,
                             record->state,
                             record->status,
                             record->flags,
                             record->error_count);
}

knx_status_t knx_health_get(knx_health_source_t source,
                            knx_health_record_t *record)
{
    if (is_valid_source(source) == 0U || record == NULL) {
        return KNX_INVALID_ARG;
    }

    *record = s_records[(uint32_t)source];
    return KNX_OK;
}

uint32_t knx_health_age_ms(knx_health_source_t source)
{
    if (is_valid_source(source) == 0U) {
        return 0xFFFFFFFFU;
    }

    const knx_health_record_t *record = &s_records[(uint32_t)source];
    if (record->update_count == 0U) {
        return 0xFFFFFFFFU;
    }
    return knx_millis() - record->last_update_ms;
}

knx_health_state_t knx_health_overall_state(void)
{
    knx_health_state_t overall = KNX_HEALTH_STATE_OK;
    uint8_t overall_rank = 0U;

    for (uint8_t i = 0U; i < (uint8_t)KNX_HEALTH_SOURCE_COUNT; i++) {
        uint8_t rank = health_rank(s_records[i].state);
        if (rank > overall_rank) {
            overall_rank = rank;
            overall = s_records[i].state;
        }
    }
    return overall;
}

void knx_health_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id)
{
    if (octo == NULL) {
        return;
    }

    (void)Octolinker_SendU8(octo, base_id + 0U, (uint8_t)knx_health_overall_state());
    (void)Octolinker_SendU8(octo, base_id + 1U, (uint8_t)KNX_HEALTH_SOURCE_COUNT);

    for (uint8_t i = 0U; i < (uint8_t)KNX_HEALTH_SOURCE_COUNT; i++) {
        const knx_health_record_t *record = &s_records[i];
        (void)Octolinker_SendU8(octo, (uint16_t)(base_id + 10U + i), (uint8_t)record->state);
        (void)Octolinker_SendU32(octo, (uint16_t)(base_id + 30U + i), knx_health_age_ms((knx_health_source_t)i));
        (void)Octolinker_SendI32(octo, (uint16_t)(base_id + 50U + i), record->status);
        (void)Octolinker_SendU32(octo, (uint16_t)(base_id + 70U + i), record->flags);
        (void)Octolinker_SendU32(octo, (uint16_t)(base_id + 90U + i), record->error_count);
        (void)Octolinker_SendU32(octo, (uint16_t)(base_id + 110U + i), record->update_count);
    }
}
