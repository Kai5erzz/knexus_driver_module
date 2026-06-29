#include "knx_safety.h"
#include "knx_blackbox.h"
#include "knx_control.h"
#include "knx_debug_config.h"
#include "knx_health.h"
#include "knx_imu.h"
#include "knx_math.h"
#include "knx_motor.h"
#include "knx_params.h"
#include "knx_port.h"
#include "knx_project_config.h"
#include "knx_sys.h"
#include "knx_time.h"
#include "FreeRTOS.h"
#include "task.h"

#if (KNX_MODULE_GIMBAL_EN)
#include "dm_imu_l1.h"
#include "knx_gimbal_ctrl.h"
#include "knx_vision.h"
#endif

static knx_safety_level_t safety_level = KNX_SAFETY_LEVEL_OK;
static volatile uint32_t fault_flags = KNX_FAULT_NONE;
static volatile uint32_t warning_flags = KNX_WARN_NONE;
static uint32_t safety_init_ms = 0U;

#define KNX_SAFETY_HOST_COMM_TIMEOUT_MS 1000U

float knx_safety_gimbal_enable = 1.0f;
float knx_safety_gimbal_startup_grace_ms = 3000.0f;
float knx_safety_gimbal_imu_timeout_ms = 100.0f;
float knx_safety_gimbal_vision_warn_ms = 500.0f;
float knx_safety_gimbal_yaw_fault_deg = 95.0f;
float knx_safety_gimbal_pitch_fault_deg = 50.0f;
float knx_safety_gimbal_cmd_fault_rpm = 80.0f;

float knx_safety_level_debug = 0.0f;
float knx_safety_faults_debug = 0.0f;
float knx_safety_warnings_debug = 0.0f;
float knx_safety_imu_age_ms_debug = 0.0f;
float knx_safety_vision_age_ms_debug = 0.0f;
uint32_t knx_safety_update_count = 0U;

static uint8_t startup_grace_active(void)
{
    uint32_t grace_ms = knx_ctrl_period_ms_from_f32(knx_safety_gimbal_startup_grace_ms, 3000U);
    return ((knx_millis() - safety_init_ms) < grace_ms) ? 1U : 0U;
}

static uint32_t age_from_timestamp_ms(uint32_t timestamp)
{
    if (timestamp == 0U) {
        return 0xFFFFFFFFU;
    }
    return knx_millis() - timestamp;
}

static void sync_debug_mirrors(uint32_t imu_age_ms, uint32_t vision_age_ms)
{
    knx_safety_level_debug = (float)safety_level;
    knx_safety_faults_debug = (float)fault_flags;
    knx_safety_warnings_debug = (float)warning_flags;
    knx_safety_imu_age_ms_debug = (float)imu_age_ms;
    knx_safety_vision_age_ms_debug = (float)vision_age_ms;
}

static void apply_safety_result(uint32_t new_faults,
                                uint32_t new_warnings,
                                uint32_t imu_age_ms,
                                uint32_t vision_age_ms)
{
    uint32_t local_faults;

    taskENTER_CRITICAL();
    warning_flags = new_warnings;
    fault_flags |= new_faults;
    local_faults = fault_flags;
    taskEXIT_CRITICAL();

    if (local_faults != 0U) {
        safety_level = KNX_SAFETY_LEVEL_FAULT;
        knx_sys_report_fault(local_faults);
    } else if (new_warnings != 0U) {
        safety_level = KNX_SAFETY_LEVEL_WARN;
    } else {
        safety_level = KNX_SAFETY_LEVEL_OK;
    }

    sync_debug_mirrors(imu_age_ms, vision_age_ms);
    (void)knx_health_report(KNX_HEALTH_SOURCE_SAFETY,
                            (safety_level == KNX_SAFETY_LEVEL_FAULT) ? KNX_HEALTH_STATE_FAULT :
                            (safety_level == KNX_SAFETY_LEVEL_WARN) ? KNX_HEALTH_STATE_WARN :
                                                                      KNX_HEALTH_STATE_OK,
                            KNX_OK,
                            knx_safety_get_faults() | (new_warnings << 16),
                            knx_safety_update_count);
}

knx_status_t knx_safety_init(void)
{
    safety_level = KNX_SAFETY_LEVEL_OK;
    fault_flags = KNX_FAULT_NONE;
    warning_flags = KNX_WARN_NONE;
    safety_init_ms = knx_millis();
    knx_safety_update_count = 0U;
    sync_debug_mirrors(0xFFFFFFFFU, 0xFFFFFFFFU);
    (void)knx_health_report(KNX_HEALTH_SOURCE_SAFETY,
                            KNX_HEALTH_STATE_OK,
                            KNX_OK,
                            0U,
                            0U);
    return KNX_OK;
}

#if (KNX_MODULE_GIMBAL_EN)
static knx_status_t knx_safety_update_gimbal(void)
{
    uint32_t faults = KNX_FAULT_NONE;
    uint32_t warnings = KNX_WARN_NONE;
    uint32_t imu_age_ms = age_from_timestamp_ms(dm_imu_l1_data.timestamp);
    uint32_t vision_age_ms = knx_vision_age_ms();
    uint8_t in_startup_grace = startup_grace_active();
    uint32_t imu_timeout_ms =
        knx_ctrl_period_ms_from_f32(knx_safety_gimbal_imu_timeout_ms, 100U);
    uint32_t vision_warn_ms =
        knx_ctrl_period_ms_from_f32(knx_safety_gimbal_vision_warn_ms, 500U);

    if (knx_safety_gimbal_enable <= 0.0f) {
        warnings |= KNX_WARN_DISABLED;
        apply_safety_result(KNX_FAULT_NONE, warnings, imu_age_ms, vision_age_ms);
        return KNX_OK;
    }

    if ((dm_imu_l1_data.rx_count == 0U) ||
        ((dm_imu_l1_data.data_flags & DM_IMU_L1_FLAG_EULER) == 0U) ||
        (imu_age_ms > imu_timeout_ms)) {
        if (in_startup_grace != 0U) {
            warnings |= KNX_WARN_STARTUP;
        } else {
            faults |= KNX_FAULT_IMU_FAIL;
        }
    }

    if (knx_vision_rx_count == 0U || vision_age_ms > vision_warn_ms) {
        warnings |= KNX_WARN_VISION_STALE;
    }

    if (knx_abs_f(dm_imu_l1_data.yaw_total) >
            knx_abs_f(knx_safety_gimbal_yaw_fault_deg) ||
        knx_abs_f(dm_imu_l1_data.pitch) >
            knx_abs_f(knx_safety_gimbal_pitch_fault_deg)) {
        faults |= KNX_FAULT_TILT | KNX_FAULT_GIMBAL_LIMIT;
    }

    if (knx_abs_f(knx_gimbal_ctrl_yaw_cmd_rpm) >
            knx_abs_f(knx_safety_gimbal_cmd_fault_rpm) ||
        knx_abs_f(knx_gimbal_ctrl_pitch_cmd_rpm) >
            knx_abs_f(knx_safety_gimbal_cmd_fault_rpm)) {
        faults |= KNX_FAULT_OVERSPEED | KNX_FAULT_GIMBAL_COMMAND;
    }

    if (in_startup_grace == 0U &&
        knx_gimbal_ctrl_state >= (float)KNX_GIMBAL_CTRL_ENTER_SERVO &&
        knx_gimbal_ctrl_last_status < 0) {
        faults |= KNX_FAULT_WATCHDOG;
    }

    apply_safety_result(faults, warnings, imu_age_ms, vision_age_ms);
    return KNX_OK;
}
#else
static knx_status_t knx_safety_update_drive(void)
{
    uint32_t faults = KNX_FAULT_NONE;
    uint32_t warnings = KNX_WARN_NONE;
    knx_imu_data_t imu;
    knx_motor_state_t left;
    knx_motor_state_t right;
    knx_sys_state_t sys_state = knx_sys_get_state();
    bool strict_check = (sys_state == KNX_SYS_READY || sys_state == KNX_SYS_RUNNING);

    knx_imu_snapshot(&imu);
    knx_motor_snapshot(&left, &right);

    if (strict_check) {
        if (!knx_imu_is_ready() || !knx_imu_is_fresh(g_knx_params.safety.imu_timeout_ms)) {
            faults |= KNX_FAULT_IMU_FAIL;
        }

        if (knx_abs_f(imu.pitch) > g_knx_params.safety.max_tilt_deg ||
            knx_abs_f(imu.roll) > g_knx_params.safety.max_tilt_deg) {
            faults |= KNX_FAULT_TILT;
        }

        if (knx_health_age_ms(KNX_HEALTH_SOURCE_HOST_COMM) > KNX_SAFETY_HOST_COMM_TIMEOUT_MS) {
            faults |= KNX_FAULT_HOST_COMM_LOST;
        }
    }
    if (!strict_check) {
        warnings |= KNX_WARN_STARTUP;
    }

    if (knx_abs_f(left.current_filtered) > g_knx_params.safety.max_current_a ||
        knx_abs_f(right.current_filtered) > g_knx_params.safety.max_current_a) {
        faults |= KNX_FAULT_OVERCURRENT;
    }

    if (knx_abs_f(left.current_speed) > g_knx_params.safety.max_speed_mps ||
        knx_abs_f(right.current_speed) > g_knx_params.safety.max_speed_mps) {
        faults |= KNX_FAULT_OVERSPEED;
    }

    apply_safety_result(faults, warnings, 0U, 0U);
    return KNX_OK;
}
#endif

knx_status_t knx_safety_update(void)
{
    knx_safety_update_count++;
    knx_port_watchdog_refresh();

#if (KNX_ACTIVE_TEST_MODE != KNX_ACTIVE_TEST_MODE_NONE)
    warning_flags = KNX_WARN_STARTUP;
    if (fault_flags != 0U) {
        safety_level = KNX_SAFETY_LEVEL_FAULT;
    } else {
        safety_level = KNX_SAFETY_LEVEL_WARN;
    }
    sync_debug_mirrors(0xFFFFFFFFU, 0xFFFFFFFFU);
    return KNX_OK;
#else
#if (KNX_MODULE_GIMBAL_EN)
    return knx_safety_update_gimbal();
#else
    return knx_safety_update_drive();
#endif
#endif
}

knx_safety_level_t knx_safety_get_level(void)
{
    return safety_level;
}

uint32_t knx_safety_get_faults(void)
{
    uint32_t flags;
    taskENTER_CRITICAL();
    flags = fault_flags | knx_sys_get_fault_latch();
    taskEXIT_CRITICAL();
    return flags;
}

uint32_t knx_safety_get_warnings(void)
{
    return warning_flags;
}

void knx_safety_clear_faults(void)
{
    taskENTER_CRITICAL();
    fault_flags = KNX_FAULT_NONE;
    warning_flags = KNX_WARN_NONE;
    taskEXIT_CRITICAL();
    safety_level = KNX_SAFETY_LEVEL_OK;
    safety_init_ms = knx_millis();
    knx_sys_clear_faults();
    sync_debug_mirrors(0xFFFFFFFFU, 0xFFFFFFFFU);
    knx_blackbox_log(KNX_BLACKBOX_CODE_SAFETY_CLEAR,
                     KNX_HEALTH_SOURCE_SAFETY,
                     KNX_OK,
                     0U,
                     0U);
    (void)knx_health_report(KNX_HEALTH_SOURCE_SAFETY,
                            KNX_HEALTH_STATE_OK,
                            KNX_OK,
                            0U,
                            0U);
}

void knx_safety_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id)
{
    if (octo == NULL) {
        return;
    }

    (void)Octolinker_SendF32(octo, base_id + 0U, knx_safety_level_debug);
    (void)Octolinker_SendU32(octo, base_id + 1U, knx_safety_get_faults());
    (void)Octolinker_SendU32(octo, base_id + 2U, warning_flags);
    (void)Octolinker_SendF32(octo, base_id + 3U, knx_safety_imu_age_ms_debug);
    (void)Octolinker_SendF32(octo, base_id + 4U, knx_safety_vision_age_ms_debug);
    (void)Octolinker_SendU32(octo, base_id + 5U, knx_safety_update_count);
    (void)Octolinker_SendF32(octo, base_id + 6U, knx_safety_gimbal_startup_grace_ms);
    (void)Octolinker_SendF32(octo, base_id + 7U, knx_safety_gimbal_imu_timeout_ms);
    (void)Octolinker_SendF32(octo, base_id + 8U, knx_safety_gimbal_yaw_fault_deg);
    (void)Octolinker_SendF32(octo, base_id + 9U, knx_safety_gimbal_pitch_fault_deg);
}
