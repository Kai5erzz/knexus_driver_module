#ifndef KNX_CONTROL_H
#define KNX_CONTROL_H

#include <stdint.h>

typedef struct {
    float integral;
    float last_error;
    uint8_t initialized;
} knx_ctrl_pidf_t;

float knx_ctrl_limit_abs_f32(float value, float limit);
uint32_t knx_ctrl_period_ms_from_f32(float value, uint32_t fallback);
void knx_ctrl_pidf_reset(knx_ctrl_pidf_t *pid);
float knx_ctrl_pidf_update(knx_ctrl_pidf_t *pid,
                           float error,
                           float kp,
                           float ki,
                           float kd,
                           float dt,
                           float integral_limit);

#endif /* KNX_CONTROL_H */
