#include "knx_rod_angle_ctrl.h"
#include "knexus_config.h"
#include <math.h>

static knx_rod_angle_ctrl_state_t s_state;

float knexus_rod_target_deg = KNEXUS_ROD_TARGET_DEG_DEFAULT;
float knexus_rod_angle_kp = KNEXUS_ROD_ANGLE_KP_DEFAULT;
float knexus_rod_angle_ki = KNEXUS_ROD_ANGLE_KI_DEFAULT;
float knexus_rod_angle_kd = KNEXUS_ROD_ANGLE_KD_DEFAULT;
float knexus_rod_integral_max_rpm =
    KNEXUS_ROD_INTEGRAL_MAX_RPM_DEFAULT;
float knexus_rod_integral_zone_deg =
    KNEXUS_ROD_INTEGRAL_ZONE_DEG_DEFAULT;
float knexus_rod_integral_rate_zone_dps =
    KNEXUS_ROD_INTEGRAL_RATE_ZONE_DPS_DEFAULT;
float knexus_rod_max_rpm = KNEXUS_ROD_MAX_RPM_DEFAULT;
float knexus_rod_rpm_slew_rpmps = KNEXUS_ROD_RPM_SLEW_RPMPS_DEFAULT;
float knexus_rod_deadband_deg = KNEXUS_ROD_DEADBAND_DEG_DEFAULT;
float knexus_rod_rate_deadband_dps =
    KNEXUS_ROD_RATE_DEADBAND_DPS_DEFAULT;

static float limit_abs(float value, float limit)
{
    limit = fabsf(limit);
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void knx_rod_angle_ctrl_reset(void)
{
    s_state = (knx_rod_angle_ctrl_state_t){0};
}

float knx_rod_angle_ctrl_update(float target_deg, float roll_deg,
                                float roll_rate_dps, float kp, float ki,
                                float kd, float integral_max_rpm,
                                float integral_zone_deg,
                                float integral_rate_zone_dps,
                                float max_rpm, float slew_rpmps,
                                float deadband_deg,
                                float rate_deadband_dps,
                                float feedforward_rpm,
                                float motor_sign, float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.01f;

    s_state.target_deg = target_deg;
    s_state.roll_deg = roll_deg;
    s_state.roll_rate_dps = roll_rate_dps;
    s_state.error_deg = target_deg - roll_deg;
    s_state.p_rpm = kp * s_state.error_deg;
    s_state.d_rpm = -kd * roll_rate_dps;
    s_state.feedforward_rpm = feedforward_rpm;

    /*
     * The screw has a large static-friction region: a small P command can
     * leave the motor at zero speed with a repeatable 1~2 degree error.
     * Integrate only near the target and while the rod is slow.  Clearing the
     * term outside that window prevents a large disturbance or a mechanical
     * limit from storing energy for the next reversal.
     */
    bool integral_window =
        fabsf(s_state.error_deg) > fabsf(deadband_deg) &&
        fabsf(s_state.error_deg) <= fabsf(integral_zone_deg) &&
        fabsf(roll_rate_dps) <= fabsf(integral_rate_zone_dps);
    if (integral_window) {
        if (s_state.i_rpm * s_state.error_deg < 0.0f) {
            s_state.i_rpm = 0.0f;
        }
        s_state.i_rpm += ki * s_state.error_deg * dt_s;
        s_state.i_rpm = limit_abs(s_state.i_rpm, integral_max_rpm);
    } else {
        s_state.i_rpm = 0.0f;
    }

    /* 实测 +rpm 会使 roll 增大，所以 target-roll 的误差可直接产生正确方向。 */
    s_state.raw_rpm = motor_sign *
                      (s_state.p_rpm + s_state.i_rpm + s_state.d_rpm +
                       s_state.feedforward_rpm);
    if (fabsf(s_state.error_deg) <= fabsf(deadband_deg) &&
        fabsf(roll_rate_dps) <= fabsf(rate_deadband_dps) &&
        fabsf(s_state.feedforward_rpm) <= 1.0f) {
        s_state.raw_rpm = 0.0f;
        s_state.i_rpm = 0.0f;
    }
    s_state.raw_rpm = limit_abs(s_state.raw_rpm, max_rpm);

    float max_step = fabsf(slew_rpmps) * dt_s;
    float step = s_state.raw_rpm - s_state.command_rpm;
    if (step > max_step) step = max_step;
    if (step < -max_step) step = -max_step;
    s_state.command_rpm += step;
    return s_state.command_rpm;
}

void knx_rod_angle_ctrl_snapshot(knx_rod_angle_ctrl_state_t *out)
{
    if (out != NULL) *out = s_state;
}
