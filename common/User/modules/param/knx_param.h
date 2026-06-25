#ifndef KNX_PARAM_H
#define KNX_PARAM_H

#include "knx_types.h"
#include <stdint.h>

#define KNX_PARAM_FLAG_READONLY  (1U << 0)

typedef enum {
    KNX_PARAM_TYPE_F32 = 0,
    KNX_PARAM_TYPE_U32,
    KNX_PARAM_TYPE_I32,
    KNX_PARAM_TYPE_U8,
} knx_param_type_t;

typedef union {
    float f32;
    uint32_t u32;
    int32_t i32;
    uint8_t u8;
} knx_param_value_t;

typedef struct {
    uint16_t id;
    const char *name;
    knx_param_type_t type;
    void *storage;
    knx_param_value_t default_value;
    knx_param_value_t min_value;
    knx_param_value_t max_value;
    uint8_t flags;
} knx_param_entry_t;

#define KNX_PARAM_F32(id_, name_, storage_, def_, min_, max_, flags_) \
    { (id_), (name_), KNX_PARAM_TYPE_F32, (storage_), \
      { .f32 = (def_) }, { .f32 = (min_) }, { .f32 = (max_) }, (flags_) }

#define KNX_PARAM_U32(id_, name_, storage_, def_, min_, max_, flags_) \
    { (id_), (name_), KNX_PARAM_TYPE_U32, (storage_), \
      { .u32 = (def_) }, { .u32 = (min_) }, { .u32 = (max_) }, (flags_) }

#define KNX_PARAM_I32(id_, name_, storage_, def_, min_, max_, flags_) \
    { (id_), (name_), KNX_PARAM_TYPE_I32, (storage_), \
      { .i32 = (def_) }, { .i32 = (min_) }, { .i32 = (max_) }, (flags_) }

#define KNX_PARAM_U8(id_, name_, storage_, def_, min_, max_, flags_) \
    { (id_), (name_), KNX_PARAM_TYPE_U8, (storage_), \
      { .u8 = (def_) }, { .u8 = (min_) }, { .u8 = (max_) }, (flags_) }

void knx_param_init(void);
knx_status_t knx_param_register_table(const knx_param_entry_t *entries,
                                      uint16_t count);
uint16_t knx_param_count(void);
const knx_param_entry_t *knx_param_entry_at(uint16_t index);
const knx_param_entry_t *knx_param_find_by_id(uint16_t id);
const knx_param_entry_t *knx_param_find_by_name(const char *name);

knx_status_t knx_param_get(uint16_t id, knx_param_value_t *value);
knx_status_t knx_param_set(uint16_t id, knx_param_value_t value);
knx_status_t knx_param_reset(uint16_t id);
knx_status_t knx_param_reset_all(void);

knx_status_t knx_param_get_f32(uint16_t id, float *value);
knx_status_t knx_param_set_f32(uint16_t id, float value);
knx_status_t knx_param_get_u32(uint16_t id, uint32_t *value);
knx_status_t knx_param_set_u32(uint16_t id, uint32_t value);
knx_status_t knx_param_get_i32(uint16_t id, int32_t *value);
knx_status_t knx_param_set_i32(uint16_t id, int32_t value);
knx_status_t knx_param_get_u8(uint16_t id, uint8_t *value);
knx_status_t knx_param_set_u8(uint16_t id, uint8_t value);

#endif /* KNX_PARAM_H */
