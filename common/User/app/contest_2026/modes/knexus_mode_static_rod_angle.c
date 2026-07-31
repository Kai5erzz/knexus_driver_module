#include "knexus_mode_backend.h"
#include "knexus_config.h"
#include "knx26_app.h"
#include "knx_chassis.h"
#include "knx_dji_motor_ctrl.h"
#include "knx_rod_angle_ctrl.h"
#include "knx_rod_runtime.h"

#if defined(KNX_PLATFORM_STM32)
#include "dm_imu_l1.h"
#endif

/* Static bench mode: the chassis is disabled and the shared rod runtime holds
 * the configured angle target.  The same hardware path is reused by the
 * line-follow/ball-centering mode. */

void knexus_mode_static_rod_angle_init(void)
{
    (void)knx_chassis_disable();
    (void)knx_rod_runtime_init();
}

void knexus_mode_static_rod_angle_update(
    const struct knx26_context *raw_context)
{
    const knx26_context_t *context = (const knx26_context_t *)raw_context;
    if (context == NULL) return;
    if (context->chassis.enabled) (void)knx_chassis_disable();
    knx_rod_runtime_update(
        knexus_rod_target_deg, 0.0f, true,
        KNEXUS_ROD_SOFT_LIMIT_ENABLE != 0U,
        (float)KNEXUS_APP_PERIOD_MS * 0.001f);
}

void knexus_mode_static_rod_angle_on_intersection(
    const knx_intersection_result_t *result)
{
    (void)result;
}

void knexus_mode_static_rod_angle_on_peer_message(
    const knx_comm_message_t *message)
{
    (void)message;
}

void knexus_mode_static_rod_angle_debug_control_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    (void)base_id;
    if (octo == NULL) return;
    const uint16_t id = KNEXUS_ROD_OCTO_BASE_ID;
    knx_rod_runtime_state_t rod;
    knx_rod_angle_ctrl_state_t ctrl;
    knx_dji_motor_state_t dji;
    knx_rod_runtime_snapshot(&rod);
    knx_rod_angle_ctrl_snapshot(&ctrl);
    knx_dji_motor_ctrl_snapshot(&dji);

    (void)Octolinker_SendF32(octo, id + 0U, rod.target_deg);
    (void)Octolinker_SendF32(octo, id + 1U, rod.roll_deg);
    (void)Octolinker_SendF32(octo, id + 2U, ctrl.error_deg);
    (void)Octolinker_SendF32(octo, id + 3U, rod.roll_rate_dps);
    (void)Octolinker_SendF32(octo, id + 4U, ctrl.p_rpm);
    (void)Octolinker_SendF32(octo, id + 5U, ctrl.d_rpm);
    (void)Octolinker_SendF32(octo, id + 6U, ctrl.raw_rpm);
    (void)Octolinker_SendF32(octo, id + 7U, ctrl.command_rpm);
    (void)Octolinker_SendF32(octo, id + 8U, dji.target_rpm);
    (void)Octolinker_SendF32(octo, id + 9U, dji.target_used_rpm);
    (void)Octolinker_SendF32(octo, id + 10U, dji.speed_rpm);
    (void)Octolinker_SendF32(octo, id + 11U, dji.current);
    (void)Octolinker_SendI32(octo, id + 12U, dji.pid_output);
    (void)Octolinker_SendU8(octo, id + 13U, (uint8_t)rod.control_ready);
    (void)Octolinker_SendU8(octo, id + 14U, (uint8_t)rod.motor_enabled);
    (void)Octolinker_SendU8(octo, id + 15U, (uint8_t)rod.upper_limit_active);
    (void)Octolinker_SendU8(octo, id + 16U, (uint8_t)rod.lower_limit_active);
    (void)Octolinker_SendI32(octo, id + 17U, rod.blocked_direction);
    (void)Octolinker_SendF32(octo, id + 18U, knexus_rod_angle_kp);
    (void)Octolinker_SendF32(octo, id + 19U, knexus_rod_angle_kd);
    (void)Octolinker_SendF32(octo, id + 20U, knexus_rod_max_rpm);
    (void)Octolinker_SendF32(octo, id + 21U, knexus_rod_rpm_slew_rpmps);
    (void)Octolinker_SendF32(octo, id + 22U, knexus_rod_deadband_deg);
    (void)Octolinker_SendF32(octo, id + 54U, knexus_rod_angle_ki);
    (void)Octolinker_SendF32(octo, id + 55U, ctrl.i_rpm);
    (void)Octolinker_SendF32(octo, id + 56U, knexus_rod_integral_max_rpm);
    (void)Octolinker_SendF32(octo, id + 57U, knexus_rod_integral_zone_deg);
    (void)Octolinker_SendF32(octo, id + 58U,
                             knexus_rod_integral_rate_zone_dps);
    (void)Octolinker_SendU8(octo, id + 59U,
                            KNEXUS_ROD_SOFT_LIMIT_ENABLE);
    (void)Octolinker_SendF32(octo, id + 60U,
                             dji.target_used_rpm - dji.speed_rpm);
    (void)Octolinker_SendF32(octo, id + 61U, KNEXUS_ROD_DJI_SPEED_KP);
    (void)Octolinker_SendF32(octo, id + 62U, KNEXUS_ROD_DJI_SPEED_KI);
    (void)Octolinker_SendF32(octo, id + 63U,
                             KNEXUS_ROD_DJI_SPEED_I_LIMIT);
    (void)Octolinker_SendF32(octo, id + 64U,
                             KNEXUS_ROD_DJI_MAX_CURRENT_CMD);
    (void)Octolinker_SendF32(octo, id + 65U,
                             KNEXUS_ROD_DJI_CURRENT_FF_CMD);
    (void)Octolinker_SendF32(octo, id + 66U,
                             KNEXUS_ROD_DJI_CURRENT_FF_MIN_RPM);
}

void knexus_mode_static_rod_angle_debug_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    (void)base_id;
    if (octo == NULL) return;
    const uint16_t id = KNEXUS_ROD_OCTO_BASE_ID;
    knx_rod_runtime_state_t rod;
    knx_dji_motor_state_t dji;
    knx_rod_runtime_snapshot(&rod);
    knx_dji_motor_ctrl_snapshot(&dji);

    (void)Octolinker_SendU32(octo, id + 23U, dji.feedback_age_ms);
    (void)Octolinker_SendF32(octo, id + 24U, dji.angle_deg);
    (void)Octolinker_SendU8(octo, id + 25U,
                            (uint8_t)rod.platform_supported);
    (void)Octolinker_SendI32(octo, id + 26U, rod.dji_init_status);
    (void)Octolinker_SendI32(octo, id + 27U, rod.dm_init_status);
    (void)Octolinker_SendI32(octo, id + 28U,
                             rod.dm_sub_active_status);
    (void)Octolinker_SendI32(octo, id + 29U,
                             rod.dm_sub_reply_status);
    (void)Octolinker_SendU8(octo, id + 30U,
                            (uint8_t)rod.motor_feedback_valid);
    (void)Octolinker_SendU32(octo, id + 31U, rod.stop_count);
#if defined(KNX_PLATFORM_STM32)
    DM_IMU_L1_DebugOcto(octo, (uint16_t)(id + 32U));
#endif
}
