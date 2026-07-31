#ifndef KNX_BALL_MOTION_COMP_H
#define KNX_BALL_MOTION_COMP_H

#include <stdbool.h>
#include <stdint.h>

#define KNX_BALL_ACCEL_AXIS_X       0
#define KNX_BALL_ACCEL_AXIS_Y       1
#define KNX_BALL_ACCEL_AXIS_Z       2
#define KNX_BALL_ACCEL_AXIS_UNKNOWN 3

typedef struct {
    float raw_accel_mps2[3];
    float bias_mps2[3];
    float gravity_compensated_accel_mps2[3];
    float selected_specific_force_mps2;
    float filtered_specific_force_mps2;
    float chassis_accel_mps2;
    float chassis_encoder_accel_mps2;
    float chassis_command_accel_mps2;
    float feedforward_roll_deg;
    float target_roll_deg;
    float target_roll_rate_dps;
    float predicted_ball_accel_mps2;
    float predicted_ball_velocity_mps;
    float predicted_ball_position_m;
    uint32_t bias_sample_count;
    uint32_t mapping_sample_count;
    bool bias_ready;
    bool mapping_ready;
    bool chassis_imu_ready;
    bool chassis_imu_mapping_ready;
    bool chassis_stationary_locked;
    bool compensation_active;
    bool target_limited;
} knx_ball_motion_comp_state_t;

extern int32_t knexus_ball_accel_axis;
extern float knexus_ball_accel_axis_sign;
extern float knexus_ball_roll_gravity_sign;
extern float knexus_ball_compensation_enable;
extern float knexus_ball_accel_compensation_gain;
extern float knexus_ball_drag_s_inv;
extern float knexus_ball_accel_lpf_hz;

void knx_ball_motion_comp_init(void);
void knx_ball_motion_comp_update(const float chassis_accel_mps2[3],
                                 float chassis_roll_deg,
                                 float chassis_pitch_deg,
                                 bool chassis_imu_valid,
                                 float rod_roll_deg,
                                 float chassis_encoder_accel_mps2,
                                 bool chassis_encoder_valid,
                                 float chassis_command_accel_mps2,
                                 bool stationary_for_bias,
                                 bool chassis_stationary,
                                 bool compensation_requested, float dt_s);
void knx_ball_motion_comp_snapshot(knx_ball_motion_comp_state_t *out);

#endif
