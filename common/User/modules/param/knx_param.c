#include "knx_param.h"
#include <stddef.h>
#include <string.h>
#include <math.h>

#define KNX_PARAM_MAX_TABLES  12U

typedef struct {
    const knx_param_entry_t *entries;
    uint16_t count;
} knx_param_table_t;

static knx_param_table_t s_tables[KNX_PARAM_MAX_TABLES];
static uint16_t s_table_count;

static knx_param_value_t clamp_value(const knx_param_entry_t *entry,
                                     knx_param_value_t value)
{
    switch (entry->type) {
    case KNX_PARAM_TYPE_F32:
        if (isnan(value.f32) || isinf(value.f32)) {
            value.f32 = entry->default_value.f32;
        }
        if (value.f32 < entry->min_value.f32) {
            value.f32 = entry->min_value.f32;
        }
        if (value.f32 > entry->max_value.f32) {
            value.f32 = entry->max_value.f32;
        }
        break;

    case KNX_PARAM_TYPE_U32:
        if (value.u32 < entry->min_value.u32) {
            value.u32 = entry->min_value.u32;
        }
        if (value.u32 > entry->max_value.u32) {
            value.u32 = entry->max_value.u32;
        }
        break;

    case KNX_PARAM_TYPE_I32:
        if (value.i32 < entry->min_value.i32) {
            value.i32 = entry->min_value.i32;
        }
        if (value.i32 > entry->max_value.i32) {
            value.i32 = entry->max_value.i32;
        }
        break;

    case KNX_PARAM_TYPE_U8:
        if (value.u8 < entry->min_value.u8) {
            value.u8 = entry->min_value.u8;
        }
        if (value.u8 > entry->max_value.u8) {
            value.u8 = entry->max_value.u8;
        }
        break;

    default:
        break;
    }

    return value;
}

static knx_status_t read_value(const knx_param_entry_t *entry,
                               knx_param_value_t *value)
{
    if (entry == NULL || value == NULL || entry->storage == NULL) {
        return KNX_INVALID_ARG;
    }

    switch (entry->type) {
    case KNX_PARAM_TYPE_F32:
        value->f32 = *(const float *)entry->storage;
        return KNX_OK;

    case KNX_PARAM_TYPE_U32:
        value->u32 = *(const uint32_t *)entry->storage;
        return KNX_OK;

    case KNX_PARAM_TYPE_I32:
        value->i32 = *(const int32_t *)entry->storage;
        return KNX_OK;

    case KNX_PARAM_TYPE_U8:
        value->u8 = *(const uint8_t *)entry->storage;
        return KNX_OK;

    default:
        return KNX_INVALID_ARG;
    }
}

static knx_status_t write_value(const knx_param_entry_t *entry,
                                knx_param_value_t value)
{
    if (entry == NULL || entry->storage == NULL) {
        return KNX_INVALID_ARG;
    }
    if ((entry->flags & KNX_PARAM_FLAG_READONLY) != 0U) {
        return KNX_NOT_READY;
    }

    if (entry->type == KNX_PARAM_TYPE_F32) {
        if (isnan(value.f32) || isinf(value.f32)) {
            return KNX_INVALID_ARG;
        }
    }

    value = clamp_value(entry, value);

    switch (entry->type) {
    case KNX_PARAM_TYPE_F32:
        *(float *)entry->storage = value.f32;
        return KNX_OK;

    case KNX_PARAM_TYPE_U32:
        *(uint32_t *)entry->storage = value.u32;
        return KNX_OK;

    case KNX_PARAM_TYPE_I32:
        *(int32_t *)entry->storage = value.i32;
        return KNX_OK;

    case KNX_PARAM_TYPE_U8:
        *(uint8_t *)entry->storage = value.u8;
        return KNX_OK;

    default:
        return KNX_INVALID_ARG;
    }
}

void knx_param_init(void)
{
    s_table_count = 0U;
    for (uint16_t i = 0U; i < KNX_PARAM_MAX_TABLES; i++) {
        s_tables[i].entries = NULL;
        s_tables[i].count = 0U;
    }
}

knx_status_t knx_param_register_table(const knx_param_entry_t *entries,
                                      uint16_t count)
{
    if (entries == NULL || count == 0U) {
        return KNX_INVALID_ARG;
    }

    for (uint16_t i = 0U; i < s_table_count; i++) {
        if (s_tables[i].entries == entries) {
            return KNX_OK;
        }
    }

    for (uint16_t i = 0U; i < count; i++) {
        if (entries[i].storage == NULL || entries[i].name == NULL) {
            return KNX_INVALID_ARG;
        }
        if (knx_param_find_by_id(entries[i].id) != NULL) {
            return KNX_BUSY;
        }
        for (uint16_t j = (uint16_t)(i + 1U); j < count; j++) {
            if (entries[i].id == entries[j].id) {
                return KNX_BUSY;
            }
        }
    }

    if (s_table_count >= KNX_PARAM_MAX_TABLES) {
        return KNX_BUSY;
    }

    s_tables[s_table_count].entries = entries;
    s_tables[s_table_count].count = count;
    s_table_count++;
    return KNX_OK;
}

uint16_t knx_param_count(void)
{
    uint16_t total = 0U;
    for (uint16_t i = 0U; i < s_table_count; i++) {
        total = (uint16_t)(total + s_tables[i].count);
    }
    return total;
}

const knx_param_entry_t *knx_param_entry_at(uint16_t index)
{
    for (uint16_t i = 0U; i < s_table_count; i++) {
        if (index < s_tables[i].count) {
            return &s_tables[i].entries[index];
        }
        index = (uint16_t)(index - s_tables[i].count);
    }
    return NULL;
}

const knx_param_entry_t *knx_param_find_by_id(uint16_t id)
{
    for (uint16_t i = 0U; i < s_table_count; i++) {
        for (uint16_t j = 0U; j < s_tables[i].count; j++) {
            if (s_tables[i].entries[j].id == id) {
                return &s_tables[i].entries[j];
            }
        }
    }
    return NULL;
}

const knx_param_entry_t *knx_param_find_by_name(const char *name)
{
    if (name == NULL) {
        return NULL;
    }

    for (uint16_t i = 0U; i < s_table_count; i++) {
        for (uint16_t j = 0U; j < s_tables[i].count; j++) {
            const char *entry_name = s_tables[i].entries[j].name;
            if (entry_name != NULL && strcmp(entry_name, name) == 0) {
                return &s_tables[i].entries[j];
            }
        }
    }
    return NULL;
}

knx_status_t knx_param_get(uint16_t id, knx_param_value_t *value)
{
    return read_value(knx_param_find_by_id(id), value);
}

knx_status_t knx_param_set(uint16_t id, knx_param_value_t value)
{
    return write_value(knx_param_find_by_id(id), value);
}

knx_status_t knx_param_reset(uint16_t id)
{
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    if (entry == NULL) {
        return KNX_INVALID_ARG;
    }

    return write_value(entry, entry->default_value);
}

knx_status_t knx_param_reset_all(void)
{
    knx_status_t status = KNX_OK;

    for (uint16_t i = 0U; i < knx_param_count(); i++) {
        const knx_param_entry_t *entry = knx_param_entry_at(i);
        if (entry != NULL && (entry->flags & KNX_PARAM_FLAG_READONLY) == 0U) {
            knx_status_t item_status = write_value(entry, entry->default_value);
            if (item_status != KNX_OK) {
                status = item_status;
            }
        }
    }

    return status;
}

knx_status_t knx_param_get_f32(uint16_t id, float *value)
{
    knx_param_value_t raw;
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    if (entry == NULL || entry->type != KNX_PARAM_TYPE_F32 || value == NULL) {
        return KNX_INVALID_ARG;
    }

    knx_status_t status = read_value(entry, &raw);
    if (status == KNX_OK) {
        *value = raw.f32;
    }
    return status;
}

knx_status_t knx_param_set_f32(uint16_t id, float value)
{
    if (isnan(value) || isinf(value)) {
        return KNX_INVALID_ARG;
    }
    knx_param_value_t raw = { .f32 = value };
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    if (entry == NULL || entry->type != KNX_PARAM_TYPE_F32) {
        return KNX_INVALID_ARG;
    }
    return write_value(entry, raw);
}

knx_status_t knx_param_get_u32(uint16_t id, uint32_t *value)
{
    knx_param_value_t raw;
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    if (entry == NULL || entry->type != KNX_PARAM_TYPE_U32 || value == NULL) {
        return KNX_INVALID_ARG;
    }

    knx_status_t status = read_value(entry, &raw);
    if (status == KNX_OK) {
        *value = raw.u32;
    }
    return status;
}

knx_status_t knx_param_set_u32(uint16_t id, uint32_t value)
{
    knx_param_value_t raw = { .u32 = value };
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    if (entry == NULL || entry->type != KNX_PARAM_TYPE_U32) {
        return KNX_INVALID_ARG;
    }
    return write_value(entry, raw);
}

knx_status_t knx_param_get_i32(uint16_t id, int32_t *value)
{
    knx_param_value_t raw;
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    if (entry == NULL || entry->type != KNX_PARAM_TYPE_I32 || value == NULL) {
        return KNX_INVALID_ARG;
    }

    knx_status_t status = read_value(entry, &raw);
    if (status == KNX_OK) {
        *value = raw.i32;
    }
    return status;
}

knx_status_t knx_param_set_i32(uint16_t id, int32_t value)
{
    knx_param_value_t raw = { .i32 = value };
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    if (entry == NULL || entry->type != KNX_PARAM_TYPE_I32) {
        return KNX_INVALID_ARG;
    }
    return write_value(entry, raw);
}

knx_status_t knx_param_get_u8(uint16_t id, uint8_t *value)
{
    knx_param_value_t raw;
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    if (entry == NULL || entry->type != KNX_PARAM_TYPE_U8 || value == NULL) {
        return KNX_INVALID_ARG;
    }

    knx_status_t status = read_value(entry, &raw);
    if (status == KNX_OK) {
        *value = raw.u8;
    }
    return status;
}

knx_status_t knx_param_set_u8(uint16_t id, uint8_t value)
{
    knx_param_value_t raw = { .u8 = value };
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    if (entry == NULL || entry->type != KNX_PARAM_TYPE_U8) {
        return KNX_INVALID_ARG;
    }
    return write_value(entry, raw);
}
