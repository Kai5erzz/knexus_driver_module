#ifndef KNX_ROD_ANGLE_CTRL_H
#define KNX_ROD_ANGLE_CTRL_H

#include <stdbool.h>

typedef struct {
    float target_deg;
    float roll_deg;
    float roll_rate_dps;
    float error_deg;
    float p_rpm;
    float i_rpm;
    float d_rpm;
    float feedforward_rpm;
    float raw_rpm;
    float command_rpm;
} knx_rod_angle_ctrl_state_t;

/* Runtime copies of the user-facing defaults.  Both the static rod mode and
 * the line-follow/ball mode use these variables, so OctoLink/GDB tuning has
 * exactly the same meaning in either mode. */
extern float knexus_rod_target_deg;
extern float knexus_rod_angle_kp;
extern float knexus_rod_angle_ki;
extern float knexus_rod_angle_kd;
extern float knexus_rod_integral_max_rpm;
extern float knexus_rod_integral_zone_deg;
extern float knexus_rod_integral_rate_zone_dps;
extern float knexus_rod_max_rpm;
extern float knexus_rod_rpm_slew_rpmps;
extern float knexus_rod_deadband_deg;
extern float knexus_rod_rate_deadband_dps;

void knx_rod_angle_ctrl_reset(void);
float knx_rod_angle_ctrl_update(float target_deg, float roll_deg,
                                float roll_rate_dps, float kp, float ki,
                                float kd, float integral_max_rpm,
                                float integral_zone_deg,
                                float integral_rate_zone_dps,
                                float max_rpm, float slew_rpmps,
                                float deadband_deg,
                                float rate_deadband_dps,
                                float feedforward_rpm,
                                float motor_sign, float dt_s);
void knx_rod_angle_ctrl_snapshot(knx_rod_angle_ctrl_state_t *out);

#endif
