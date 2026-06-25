#include "knx_control.h"
#include "knx_math.h"
#include <stddef.h>

float knx_ctrl_limit_abs_f32(float value, float limit)
{
    float abs_limit = knx_abs_f(limit);
    return knx_clamp_f(value, -abs_limit, abs_limit);
}

uint32_t knx_ctrl_period_ms_from_f32(float value, uint32_t fallback)
{
    if (value < 1.0f) {
        return fallback;
    }
    return (uint32_t)value;
}

void knx_ctrl_pidf_reset(knx_ctrl_pidf_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->initialized = 0U;
}

float knx_ctrl_pidf_update(knx_ctrl_pidf_t *pid,
                           float error,
                           float kp,
                           float ki,
                           float kd,
                           float dt,
                           float integral_limit)
{
    if (pid == NULL) {
        return 0.0f;
    }
    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    if (pid->initialized == 0U) {
        pid->last_error = error;
        pid->initialized = 1U;
    }

    pid->integral += error * dt;
    pid->integral = knx_ctrl_limit_abs_f32(pid->integral, integral_limit);

    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    return (kp * error) + (ki * pid->integral) + (kd * derivative);
}
