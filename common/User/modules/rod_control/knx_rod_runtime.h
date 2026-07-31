#ifndef KNX_ROD_RUNTIME_H
#define KNX_ROD_RUNTIME_H

#include "knx_types.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool platform_supported;
    bool tilt_valid;
    bool motor_feedback_valid;
    bool control_ready;
    bool motor_enabled;
    bool upper_limit_active;
    bool lower_limit_active;
    int8_t blocked_direction;
    float target_deg;
    float roll_deg;
    float roll_rate_dps;
    uint32_t stop_count;
    knx_status_t dji_init_status;
    knx_status_t dm_init_status;
    knx_status_t dm_sub_active_status;
    knx_status_t dm_sub_reply_status;
} knx_rod_runtime_state_t;

knx_status_t knx_rod_runtime_init(void);
void knx_rod_runtime_update(float target_deg, float feedforward_rpm,
                            bool requested_enable,
                            bool soft_limit_enable, float dt_s);
void knx_rod_runtime_stop(void);
void knx_rod_runtime_snapshot(knx_rod_runtime_state_t *out);

#endif
