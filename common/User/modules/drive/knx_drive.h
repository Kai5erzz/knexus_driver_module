#ifndef KNX_DRIVE_H
#define KNX_DRIVE_H

#include "knx_types.h"

typedef enum {
    KNX_DRIVE_MODE_IDLE = 0,
    KNX_DRIVE_MODE_VELOCITY,
    KNX_DRIVE_MODE_POSITION,
    KNX_DRIVE_MODE_ANGLE,
    KNX_DRIVE_MODE_POSITION_ANGLE,
} knx_drive_mode_t;

typedef struct {
    knx_drive_mode_t mode;
    float target_linear_mps;
    float target_angular_radps;
    float target_position_m;
    float target_heading_rad;
    float profiled_linear_mps;
    float profiled_angular_radps;
    float measured_linear_mps;
    float measured_angular_radps;
    float position_loop_out_mps;
    float angle_loop_out_radps;
    float linear_speed_loop_out_mps;
    float angular_speed_loop_out_radps;
    float position_m;
    float heading_rad;
    float left_target_mps;
    float right_target_mps;
    uint32_t last_command_ms;
    bool command_active;
} knx_drive_state_t;

knx_status_t knx_drive_init(void);
knx_status_t knx_drive_update(float dt_s);
knx_status_t knx_drive_set_command(float linear_mps, float angular_radps);
knx_status_t knx_drive_set_velocity(float linear_mps, float angular_radps);
knx_status_t knx_drive_set_position(float position_m);
knx_status_t knx_drive_set_angle(float heading_rad);
knx_status_t knx_drive_set_position_angle(float position_m, float heading_rad);
knx_status_t knx_drive_stop(void);
void knx_drive_reset_odometry(void);
void knx_drive_set_heading(float heading_rad);
void knx_drive_snapshot(knx_drive_state_t *out);

#endif /* KNX_DRIVE_H */
