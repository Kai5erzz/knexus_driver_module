#ifndef KNX_CHASSIS_H
#define KNX_CHASSIS_H

#include "knx_drive.h"
#include "knx_motor.h"
#include "knx_types.h"

typedef struct {
    bool enabled;
    uint32_t fast_updates;
    uint32_t control_updates;
    knx_status_t last_status;
    knx_drive_state_t drive;
    knx_motor_state_t left_motor;
    knx_motor_state_t right_motor;
} knx_chassis_state_t;

knx_status_t knx_chassis_init(void);
knx_status_t knx_chassis_enable(void);
knx_status_t knx_chassis_disable(void);
knx_status_t knx_chassis_set_velocity(float linear_mps, float angular_radps);
knx_status_t knx_chassis_set_wheel_speed(float left_mps, float right_mps);
knx_status_t knx_chassis_set_position(float position_m);
knx_status_t knx_chassis_set_angle(float heading_rad);
knx_status_t knx_chassis_set_position_angle(float position_m, float heading_rad);
knx_status_t knx_chassis_update_fast(void);
knx_status_t knx_chassis_update_control(float dt_s);
knx_status_t knx_chassis_stop(void);
void knx_chassis_reset_odometry(void);
void knx_chassis_snapshot(knx_chassis_state_t *out);

#endif
