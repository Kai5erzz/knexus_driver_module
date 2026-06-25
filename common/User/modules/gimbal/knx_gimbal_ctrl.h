#ifndef KNX_GIMBAL_CTRL_H
#define KNX_GIMBAL_CTRL_H

#include "knx_types.h"
#include "octolinker.h"
#include <stdint.h>

typedef enum {
    KNX_GIMBAL_CTRL_BOOT = 0,
    KNX_GIMBAL_CTRL_ENTER_SERVO,
    KNX_GIMBAL_CTRL_SPEED_MODE,
    KNX_GIMBAL_CTRL_CONTROL,
} knx_gimbal_ctrl_state_t;

typedef enum {
    KNX_GIMBAL_PARAM_ENABLE = 1000,
    KNX_GIMBAL_PARAM_BOOT_DELAY_MS,
    KNX_GIMBAL_PARAM_MODE_GAP_MS,
    KNX_GIMBAL_PARAM_CONTROL_PERIOD_MS,
    KNX_GIMBAL_PARAM_VISION_TIMEOUT_MS,
    KNX_GIMBAL_PARAM_MIN_CONF,
    KNX_GIMBAL_PARAM_YAW_KP,
    KNX_GIMBAL_PARAM_YAW_KI,
    KNX_GIMBAL_PARAM_YAW_KD,
    KNX_GIMBAL_PARAM_PITCH_KP,
    KNX_GIMBAL_PARAM_PITCH_KI,
    KNX_GIMBAL_PARAM_PITCH_KD,
    KNX_GIMBAL_PARAM_HOLD_YAW_KP,
    KNX_GIMBAL_PARAM_HOLD_YAW_KI,
    KNX_GIMBAL_PARAM_HOLD_YAW_KD,
    KNX_GIMBAL_PARAM_HOLD_PITCH_KP,
    KNX_GIMBAL_PARAM_HOLD_PITCH_KI,
    KNX_GIMBAL_PARAM_HOLD_PITCH_KD,
    KNX_GIMBAL_PARAM_OUTPUT_LIMIT_RPM,
    KNX_GIMBAL_PARAM_HOLD_OUTPUT_LIMIT_RPM,
    KNX_GIMBAL_PARAM_INTEGRAL_LIMIT,
    KNX_GIMBAL_PARAM_YAW_TARGET_DEG,
    KNX_GIMBAL_PARAM_PITCH_TARGET_DEG,
    KNX_GIMBAL_PARAM_YAW_LIMIT_DEG,
    KNX_GIMBAL_PARAM_PITCH_LIMIT_DEG,
    KNX_GIMBAL_PARAM_STOP_PERIOD_MS,
    KNX_GIMBAL_PARAM_YAW_SPEED_TO_ANGLE_SIGN,
    KNX_GIMBAL_PARAM_PITCH_SPEED_TO_ANGLE_SIGN,
} knx_gimbal_param_id_t;

knx_status_t knx_gimbal_ctrl_register_params(void);
knx_status_t knx_gimbal_ctrl_init(void);
knx_status_t knx_gimbal_ctrl_update(float dt);
void knx_gimbal_ctrl_restart(void);
void knx_gimbal_ctrl_stop(void);
void knx_gimbal_ctrl_set_enable(uint8_t enable);
void knx_gimbal_ctrl_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id);

extern float knx_gimbal_ctrl_enable;
extern float knx_gimbal_ctrl_boot_delay_ms;
extern float knx_gimbal_ctrl_mode_gap_ms;
extern float knx_gimbal_ctrl_control_period_ms;
extern float knx_gimbal_ctrl_vision_timeout_ms;
extern float knx_gimbal_ctrl_min_conf;

extern float knx_gimbal_ctrl_yaw_kp;
extern float knx_gimbal_ctrl_yaw_ki;
extern float knx_gimbal_ctrl_yaw_kd;
extern float knx_gimbal_ctrl_pitch_kp;
extern float knx_gimbal_ctrl_pitch_ki;
extern float knx_gimbal_ctrl_pitch_kd;

extern float knx_gimbal_ctrl_hold_yaw_kp;
extern float knx_gimbal_ctrl_hold_yaw_ki;
extern float knx_gimbal_ctrl_hold_yaw_kd;
extern float knx_gimbal_ctrl_hold_pitch_kp;
extern float knx_gimbal_ctrl_hold_pitch_ki;
extern float knx_gimbal_ctrl_hold_pitch_kd;

extern float knx_gimbal_ctrl_output_limit_rpm;
extern float knx_gimbal_ctrl_hold_output_limit_rpm;
extern float knx_gimbal_ctrl_integral_limit;
extern float knx_gimbal_ctrl_yaw_target_deg;
extern float knx_gimbal_ctrl_pitch_target_deg;
extern float knx_gimbal_ctrl_yaw_limit_deg;
extern float knx_gimbal_ctrl_pitch_limit_deg;
extern float knx_gimbal_ctrl_stop_period_ms;
extern float knx_gimbal_ctrl_yaw_speed_to_angle_sign;
extern float knx_gimbal_ctrl_pitch_speed_to_angle_sign;

extern float knx_gimbal_ctrl_state;
extern float knx_gimbal_ctrl_imu_yaw_deg;
extern float knx_gimbal_ctrl_imu_pitch_deg;
extern float knx_gimbal_ctrl_yaw_hold_error_deg;
extern float knx_gimbal_ctrl_pitch_hold_error_deg;
extern float knx_gimbal_ctrl_yaw_error_px;
extern float knx_gimbal_ctrl_pitch_error_px;
extern float knx_gimbal_ctrl_yaw_cmd_rpm;
extern float knx_gimbal_ctrl_pitch_cmd_rpm;
extern float knx_gimbal_ctrl_vision_valid;
extern float knx_gimbal_ctrl_tracking_active;
extern float knx_gimbal_ctrl_yaw_limit_active;
extern float knx_gimbal_ctrl_pitch_limit_active;
extern uint32_t knx_gimbal_ctrl_clock_ms;
extern uint32_t knx_gimbal_ctrl_update_count;
extern int32_t knx_gimbal_ctrl_last_status;

#endif /* KNX_GIMBAL_CTRL_H */
