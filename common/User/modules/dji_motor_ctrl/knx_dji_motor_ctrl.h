#ifndef KNX_DJI_MOTOR_CTRL_H
#define KNX_DJI_MOTOR_CTRL_H

#include "knx_types.h"

typedef struct {
    bool initialized;
    bool enabled;
    float target_rpm;
    float target_used_rpm;
    float target_slew_rpmps;
    float speed_rpm;
    float current;
    float angle_deg;
    uint8_t temperature_c;
    int16_t pid_output;
    uint32_t tx_ok;
    uint32_t tx_error;
    uint32_t tx_busy;
    uint32_t rx_count;
    uint32_t feedback_age_ms;
    knx_status_t init_status;
    knx_status_t last_tx_status;
} knx_dji_motor_state_t;

extern volatile int32_t dji_init_status;
extern volatile uint32_t dji_tx_count;
extern volatile uint32_t dji_tx_error_count;
extern volatile uint32_t dji_tx_busy_count;
extern volatile uint32_t dji_rx_count;
extern volatile int32_t dji_pid_output;

knx_status_t knx_dji_motor_ctrl_init(void);
void knx_dji_motor_ctrl_update(float dt_s);
void knx_dji_motor_ctrl_set_target(float target_rpm);
knx_status_t knx_dji_motor_ctrl_set_speed_pid(float kp, float ki, float kd,
                                              float integral_limit,
                                              float max_current_cmd);
knx_status_t knx_dji_motor_ctrl_set_current_feedforward(
    float current_cmd, float min_target_rpm);
knx_status_t knx_dji_motor_ctrl_set_target_slew(float slew_rpmps);
void knx_dji_motor_ctrl_stop(void);
void knx_dji_motor_ctrl_enable(void);
void knx_dji_motor_ctrl_snapshot(knx_dji_motor_state_t *out);

#endif
