#include "knexus_mode_backend.h"
#include "knexus_mode_line_follow_core.h"
#include "knexus_h_hooks.h"
#include "knexus_config.h"
#include "knx26_app.h"
#include "knx_ball_motion_comp.h"
#include "knx_dji_motor_ctrl.h"
#include "knx_beep.h"
#include "knx_chassis.h"
#include "knx_key.h"
#include "knx_rod_angle_ctrl.h"
#include "knx_rod_runtime.h"
#include "knx_safety.h"
#include <math.h>

#if defined(KNX_PLATFORM_STM32)
#include "dm_imu_l1.h"
#endif

#if defined(KNEXUS_MODE_LINE_FOLLOW_BALL_CENTER)

/*
 * 循迹归中 V0：管道沿车体前后中轴线。
 * 当前没有小球位置观测，因此只做底盘纵向加速度前馈；预测位置只输出调试，
 * 不参与控制。底板BMI088提供车体加速度，DM-IMU-L1只提供管道杆角。
 */

static float s_last_chassis_speed_mps;
static float s_encoder_accel_mps2;
static uint8_t s_dm_ready;
static uint8_t s_safety_fault;
static uint8_t s_manual_armed;
static uint8_t s_manual_prelevel_done;

void knexus_h_ball_user_init(void)
{
    s_last_chassis_speed_mps = 0.0f;
    s_encoder_accel_mps2 = 0.0f;
    s_dm_ready = 0U;
    s_safety_fault = 0U;
    s_manual_armed = 0U;
    s_manual_prelevel_done = 0U;
    knexus_rod_angle_kp = KNEXUS_BALL_ROD_ANGLE_KP;
    knexus_rod_angle_ki = KNEXUS_BALL_ROD_ANGLE_KI;
    knexus_rod_angle_kd = KNEXUS_BALL_ROD_ANGLE_KD;
    knexus_rod_max_rpm = KNEXUS_BALL_ROD_MAX_RPM;
    knexus_rod_rpm_slew_rpmps = KNEXUS_BALL_ROD_RPM_SLEW_RPMPS;
    knx_ball_motion_comp_init();
    (void)knx_rod_runtime_init();
    (void)knx_dji_motor_ctrl_set_target_slew(
        KNEXUS_BALL_DJI_TARGET_SLEW_RPMPS);
}

void knexus_h_ball_user_update(
    const knexus_h_ball_request_t *request,
    const struct knx26_context *raw_context,
    float dt_s,
    knexus_h_ball_status_t *status)
{
    const knx26_context_t *context = (const knx26_context_t *)raw_context;
    if (request == NULL || context == NULL || status == NULL) return;
    if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.01f;

    float measured_speed = context->chassis.drive.measured_linear_mps;
    float encoder_accel_raw =
        (measured_speed - s_last_chassis_speed_mps) / dt_s;
    float encoder_tau_s = 1.0f /
        (6.28318530718f * KNEXUS_BALL_ACCEL_LPF_HZ);
    float encoder_alpha = dt_s / (encoder_tau_s + dt_s);
    s_encoder_accel_mps2 +=
        encoder_alpha * (encoder_accel_raw - s_encoder_accel_mps2);
    s_last_chassis_speed_mps = measured_speed;

    float chassis_accel[3] = {
        context->imu.accel[0],
        context->imu.accel[1],
        context->imu.accel[2]
    };
    uint32_t chassis_imu_age_ms =
        context->now_ms - context->imu.last_update_ms;
    bool chassis_imu_valid = context->imu.accel_ok &&
        context->imu.ekf_ready && chassis_imu_age_ms <= 50U;
    float roll_deg = 0.0f;
    float roll_rate_dps = 0.0f;
#if defined(KNX_PLATFORM_STM32)
    dm_imu_l1_data_t imu;
    DM_IMU_L1_Snapshot(&imu);
    roll_deg = imu.roll;
    roll_rate_dps = KNEXUS_DM_IMU_ROLL_GYRO_SIGN *
                    imu.gyro[0] * 57.2957795f;
    uint32_t age_ms = context->now_ms - imu.timestamp;
    s_dm_ready =
        DM_IMU_L1_IsDataReady() != 0U &&
        DM_IMU_L1_IsTiltReady() != 0U &&
        age_ms <= KNEXUS_SCREW_TILT_MAX_AGE_MS;
#else
    s_dm_ready = 0U;
#endif

    bool stationary_for_bias =
        chassis_imu_valid && !context->chassis.enabled &&
        s_manual_armed == 0U &&
        fabsf(measured_speed) < 0.02f &&
        fabsf(roll_rate_dps) < 0.5f;
    bool chassis_stationary =
        fabsf(measured_speed) < KNEXUS_BALL_STATIONARY_SPEED_MPS &&
        fabsf(s_encoder_accel_mps2) <
            KNEXUS_BALL_STATIONARY_ENCODER_ACCEL_MPS2;
    float chassis_command_accel_mps2 =
        knexus_line_follow_core_get_accel_command_mps2();
#if KNEXUS_BALL_MANUAL_COMP_TEST_ENABLE
    if (s_manual_armed == 0U) {
        s_manual_prelevel_done = 0U;
    } else if (fabsf(roll_deg) <= 0.5f &&
               fabsf(roll_rate_dps) <= 1.0f) {
        s_manual_prelevel_done = 1U;
    }
    bool motion_requested =
        s_manual_armed != 0U && s_manual_prelevel_done != 0U;
#else
    bool motion_requested =
        request->mode != KNEXUS_H_BALL_DISABLED &&
        context->chassis.enabled;
#endif

    knx_ball_motion_comp_update(
        chassis_accel, context->imu.roll, context->imu.pitch,
        chassis_imu_valid, roll_deg, s_encoder_accel_mps2,
        context->chassis.enabled, chassis_command_accel_mps2,
        stationary_for_bias, chassis_stationary, motion_requested, dt_s);

    knx_ball_motion_comp_state_t comp;
    knx_ball_motion_comp_snapshot(&comp);
    s_safety_fault =
        knx_safety_get_level() == KNX_SAFETY_LEVEL_FAULT ? 1U : 0U;
    knx26_line_state_t line_state = knexus_line_follow_core_get_state();
#if KNEXUS_BALL_MANUAL_COMP_TEST_ENABLE
    bool prelevel_requested =
        s_manual_armed != 0U && comp.bias_ready && comp.mapping_ready &&
        knexus_ball_compensation_enable > 0.5f;
#else
    bool prelevel_requested =
        comp.bias_ready && comp.mapping_ready &&
        knexus_ball_compensation_enable > 0.5f &&
        (line_state == KNX26_LINE_READY ||
         line_state == KNX26_LINE_STOPPED ||
         line_state == KNX26_LINE_COMPLETE);
#endif
    /* Power-on and sensor calibration remain passive.  READY is reached only
     * after the operator has completed KEY0 calibration, so pre-leveling is
     * explicit and finishes before KEY1 may start the chassis. */
    bool rod_enable = s_dm_ready != 0U && s_safety_fault == 0U &&
                      (comp.compensation_active || prelevel_requested);
    float rod_feedforward_rpm = comp.target_roll_rate_dps *
        KNEXUS_BALL_ROD_TARGET_RATE_FF_RPM_PER_DPS;
    knx_rod_runtime_update(
        comp.target_roll_deg, rod_feedforward_rpm, rod_enable,
        KNEXUS_BALL_ROD_SOFT_LIMIT_ENABLE != 0U, dt_s);

    knx_rod_runtime_state_t rod;
    knx_rod_runtime_snapshot(&rod);
    status->implemented = true;
    status->ready = rod.control_ready && comp.bias_ready &&
                    comp.mapping_ready &&
                    knexus_ball_compensation_enable > 0.5f &&
                    fabsf(rod.roll_deg - comp.target_roll_deg) <= 0.5f &&
                    fabsf(rod.roll_rate_dps) <= 1.0f;
    status->fault = s_safety_fault != 0U ||
                    (rod.platform_supported &&
                     (!rod.tilt_valid || !rod.motor_feedback_valid));
    status->target_position_cm = request->target_position_cm;
    status->measured_position_cm =
        comp.predicted_ball_position_m * 100.0f;
    status->control_output = comp.target_roll_deg;
}

static void ball_center_debug(Octolinker_Instance_t *octo)
{
    if (octo == NULL) return;
    const uint16_t id = KNEXUS_BALL_OCTO_BASE_ID;
    knx_ball_motion_comp_state_t comp;
    knx_rod_runtime_state_t rod;
    knx_dji_motor_state_t dji;
    knx_ball_motion_comp_snapshot(&comp);
    knx_rod_runtime_snapshot(&rod);
    knx_dji_motor_ctrl_snapshot(&dji);

    (void)Octolinker_SendF32(octo, id + 0U, comp.raw_accel_mps2[0]);
    (void)Octolinker_SendF32(octo, id + 1U, comp.raw_accel_mps2[1]);
    (void)Octolinker_SendF32(octo, id + 2U, comp.raw_accel_mps2[2]);
    (void)Octolinker_SendF32(octo, id + 3U, comp.bias_mps2[0]);
    (void)Octolinker_SendF32(octo, id + 4U, comp.bias_mps2[1]);
    (void)Octolinker_SendF32(octo, id + 5U, comp.bias_mps2[2]);
    (void)Octolinker_SendF32(octo, id + 6U, s_last_chassis_speed_mps);
    (void)Octolinker_SendF32(octo, id + 7U, s_encoder_accel_mps2);
    (void)Octolinker_SendF32(octo, id + 8U,
                             comp.selected_specific_force_mps2);
    (void)Octolinker_SendF32(octo, id + 9U,
                             comp.filtered_specific_force_mps2);
    (void)Octolinker_SendF32(octo, id + 10U, comp.chassis_accel_mps2);
    (void)Octolinker_SendF32(octo, id + 11U, comp.feedforward_roll_deg);
    (void)Octolinker_SendF32(octo, id + 12U, comp.target_roll_deg);
    (void)Octolinker_SendF32(octo, id + 13U, rod.roll_deg);
    (void)Octolinker_SendF32(octo, id + 14U, rod.roll_rate_dps);
    (void)Octolinker_SendF32(octo, id + 15U, dji.target_used_rpm);
    (void)Octolinker_SendF32(octo, id + 16U, dji.speed_rpm);
    (void)Octolinker_SendF32(octo, id + 17U, dji.current);
    (void)Octolinker_SendU8(octo, id + 18U,
                            (uint8_t)rod.control_ready);
    (void)Octolinker_SendU8(octo, id + 19U,
                            (uint8_t)comp.compensation_active);
    (void)Octolinker_SendI32(octo, id + 20U, knexus_ball_accel_axis);
    (void)Octolinker_SendF32(octo, id + 21U,
                             knexus_ball_accel_axis_sign);
    (void)Octolinker_SendF32(octo, id + 22U,
                             knexus_ball_roll_gravity_sign);
    (void)Octolinker_SendU32(octo, id + 23U,
                             comp.bias_sample_count);
    (void)Octolinker_SendU8(octo, id + 24U,
                            (uint8_t)comp.bias_ready);
    (void)Octolinker_SendF32(octo, id + 25U,
                             comp.predicted_ball_accel_mps2);
    (void)Octolinker_SendF32(octo, id + 26U,
                             comp.predicted_ball_velocity_mps);
    (void)Octolinker_SendF32(octo, id + 27U,
                             comp.predicted_ball_position_m);
    (void)Octolinker_SendF32(octo, id + 28U,
                             knexus_ball_drag_s_inv);
    (void)Octolinker_SendU8(octo, id + 29U,
                            (uint8_t)comp.target_limited);
    (void)Octolinker_SendU8(octo, id + 30U,
                            (uint8_t)rod.motor_enabled);
    (void)Octolinker_SendU8(octo, id + 31U, s_dm_ready);
    (void)Octolinker_SendU8(octo, id + 32U, s_safety_fault);
    (void)Octolinker_SendF32(octo, id + 33U,
                             knexus_ball_compensation_enable);
    (void)Octolinker_SendU8(octo, id + 34U,
                            (uint8_t)comp.mapping_ready);
    (void)Octolinker_SendU8(octo, id + 35U, s_manual_armed);
    (void)Octolinker_SendU8(octo, id + 36U, s_manual_prelevel_done);
    (void)Octolinker_SendF32(octo, id + 37U,
                             comp.target_roll_rate_dps);
    (void)Octolinker_SendF32(octo, id + 38U,
                             comp.target_roll_rate_dps *
                             KNEXUS_BALL_ROD_TARGET_RATE_FF_RPM_PER_DPS);
    (void)Octolinker_SendF32(octo, id + 39U,
                             knexus_rod_angle_kp);
    (void)Octolinker_SendF32(octo, id + 40U,
                             knexus_rod_max_rpm);
    (void)Octolinker_SendF32(octo, id + 41U,
                             dji.target_slew_rpmps);
    (void)Octolinker_SendU8(octo, id + 42U,
                            (uint8_t)comp.chassis_imu_ready);
    (void)Octolinker_SendU8(octo, id + 43U,
                            (uint8_t)comp.chassis_stationary_locked);
    (void)Octolinker_SendU8(octo, id + 44U,
                            (uint8_t)comp.chassis_imu_mapping_ready);
    (void)Octolinker_SendU32(octo, id + 45U,
                             comp.mapping_sample_count);
    (void)Octolinker_SendF32(octo, id + 46U,
                             comp.gravity_compensated_accel_mps2[0]);
    (void)Octolinker_SendF32(octo, id + 47U,
                             comp.gravity_compensated_accel_mps2[1]);
    (void)Octolinker_SendF32(octo, id + 48U,
                             comp.gravity_compensated_accel_mps2[2]);
    (void)Octolinker_SendF32(octo, id + 49U,
                             knexus_rod_angle_ki);
    (void)Octolinker_SendF32(octo, id + 50U,
                             knexus_rod_angle_kd);
    (void)Octolinker_SendF32(octo, id + 51U,
                             knexus_ball_accel_compensation_gain);
    (void)Octolinker_SendF32(octo, id + 52U,
                             comp.chassis_command_accel_mps2);
}

void knexus_mode_line_follow_ball_center_init(void)
{
    knexus_line_follow_core_init();
}

void knexus_mode_line_follow_ball_center_update(
    const struct knx26_context *context)
{
#if KNEXUS_BALL_MANUAL_COMP_TEST_ENABLE
    (void)context;
    (void)knx_chassis_disable();
    if (knx_key_just_pressed(KNX_KEY_1)) {
        s_manual_armed = 0U;
        s_manual_prelevel_done = 0U;
        knx_beep_beep(100U);
        return;
    }
    if (knx_key_just_pressed(KNX_KEY_0)) {
        knx_ball_motion_comp_state_t comp;
        knx_ball_motion_comp_snapshot(&comp);
        if (comp.bias_ready && comp.mapping_ready &&
            knexus_ball_compensation_enable > 0.5f) {
            s_manual_armed = 1U;
            s_manual_prelevel_done = 0U;
            knx_beep_beep(100U);
        } else {
            s_manual_armed = 0U;
            knx_beep_beep(400U);
        }
    }
#else
    knexus_line_follow_core_update(context);
#endif
}

void knexus_mode_line_follow_ball_center_on_intersection(
    const knx_intersection_result_t *result)
{
    knexus_line_follow_core_on_intersection(result);
}

void knexus_mode_line_follow_ball_center_on_peer_message(
    const knx_comm_message_t *message)
{
    knexus_line_follow_core_on_peer_message(message);
}

void knexus_mode_line_follow_ball_center_debug_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    knexus_line_follow_core_debug_octo(octo, base_id);
}

void knexus_mode_line_follow_ball_center_debug_control_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    knexus_line_follow_core_debug_control_octo(octo, base_id);
    ball_center_debug(octo);
}

#endif
