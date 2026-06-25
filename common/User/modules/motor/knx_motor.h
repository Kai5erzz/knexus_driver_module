#ifndef KNX_MOTOR_H
#define KNX_MOTOR_H

#include "knx_types.h"

/* Motor module — wraps drv8701e and encoder into a unified motor abstraction */

typedef struct {
    float target_speed;     /* m/s */
    float target_current;   /* A */
    float target_torque;    /* N*m at gearbox output */
    float target_duty;      /* normalized PWM duty, -1.0 ~ 1.0 */
    float current_speed;    /* m/s */
    float current_filtered; /* A */
    float torque_estimated; /* N*m at gearbox output */
    float duty;             /* -1.0 ~ 1.0 */
} knx_motor_state_t;

knx_status_t knx_motor_init(void);
knx_status_t knx_motor_update(void);       /* called periodically (1ms current loop) */
knx_status_t knx_motor_set_target(float left_mps, float right_mps);
knx_status_t knx_motor_set_current_target(float left_a, float right_a);
knx_status_t knx_motor_set_torque_target(float left_nm, float right_nm);
knx_status_t knx_motor_set_duty_target(float left_norm, float right_norm);
knx_status_t knx_motor_stop(void);

float knx_motor_current_to_torque(float current_a);
float knx_motor_torque_to_current(float torque_nm);

/* Accessors for telemetry / debug */
const knx_motor_state_t *knx_motor_get_left(void);
const knx_motor_state_t *knx_motor_get_right(void);
void knx_motor_snapshot(knx_motor_state_t *left, knx_motor_state_t *right);

#endif /* KNX_MOTOR_H */
