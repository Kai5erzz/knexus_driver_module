#ifndef KNX_PARAMS_H
#define KNX_PARAMS_H

#include "knx_types.h"

typedef struct {
    float wheel_radius_m;
    float wheel_base_m;
    float track_width_m;
    float body_mass_kg;
    float body_inertia_kgm2;
    float body_pitch_inertia_kgm2;
    float com_height_m;
    float motor_torque_constant_nm_per_a;
    float motor_gear_ratio;
    float motor_torque_efficiency;
} knx_mech_params_t;

typedef struct {
    float max_tilt_deg;
    float max_current_a;
    float max_speed_mps;
    uint32_t imu_timeout_ms;
    float warmup_temp_tolerance_c;
    uint32_t warmup_stable_ms;
} knx_safety_params_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float max_out;
    float integral_limit;
    float deadband;
} knx_pid_params_t;

typedef struct {
    bool drive_en;
    float left_cmd_sign;
    float right_cmd_sign;
    float left_feedback_sign;
    float right_feedback_sign;
    float angular_feedback_sign;
    float max_linear_speed_mps;
    float max_angular_speed_radps;
    float max_linear_accel_mps2;
    float max_angular_accel_radps2;
    uint32_t command_timeout_ms;
    knx_pid_params_t position_pid;
    knx_pid_params_t angle_pid;
    knx_pid_params_t linear_speed_pid;
    knx_pid_params_t angular_speed_pid;
} knx_drive_params_t;

typedef struct {
    knx_mech_params_t    mech;
    knx_safety_params_t  safety;
    knx_drive_params_t   drive;
} knx_params_t;

extern knx_params_t g_knx_params;

#endif /* KNX_PARAMS_H */
