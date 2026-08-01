#include "knexus_mode_backend.h"
#include "knexus_mode_line_follow_core.h"
#include "knexus_config.h"
#include "knx26_app.h"
#include "knx_ball_motion_comp.h"
#include "knx_beep.h"
#include "knx_chassis.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_port.h"
#include "knx_time.h"
#include <math.h>

#if defined(KNX_PLATFORM_STM32)
#include "dm_imu_l1.h"
#include "jc_driver.h"
#include "knx_board.h"
#include "knx_can_bus.h"
#endif

#if defined(KNEXUS_MODE_JC4310_LINK_CENTER) || \
    defined(KNEXUS_MODE_SIX_MENU)

/*
 * JC4310 连杆运动补偿：
 *   - 底盘保持失能，上电进入伺服/力矩模式并持续发送0 N*m；
 *   - 底盘BMI088纵向加速度生成目标杆角，DM-IMU-L1提供实际杆角；
 *   - KEY0启动补偿，KEY1立即停止；未启动、数据失效或故障时只发0 N*m；
 *   - 动态复核确认正力矩使roll增大，因此杆角PD输出到电机力矩的极性为+1；
 *   - JC编码器-46°端只禁负力矩，+39°端只禁正力矩，反向退出始终允许。
 */

typedef enum {
    JC_POLARITY_ENTER_SERVO = 0,
    JC_POLARITY_SWITCH_TORQUE = 1,
    JC_POLARITY_READY = 2,
    JC_POLARITY_FAULT = 3,
} jc_polarity_state_t;

typedef enum {
    JC_POLARITY_FAIL_NONE = 0,
    JC_POLARITY_FAIL_PLATFORM = 1,
    JC_POLARITY_FAIL_CAN_SUBSCRIBE = 2,
    JC_POLARITY_FAIL_DRIVER_INIT = 3,
    JC_POLARITY_FAIL_DM_SUBSCRIBE = 4,
    JC_POLARITY_FAIL_DM_INIT = 5,
    JC_POLARITY_FAIL_TX = 6,
    JC_POLARITY_FAIL_DRIVER_ERROR = 7,
} jc_polarity_fail_t;

typedef enum {
    JC_REVERSE_IDLE = 0,
    JC_REVERSE_CONFIRM = 1,
    JC_REVERSE_KICK = 2,
    JC_REVERSE_SETTLE = 3,
} jc_reverse_phase_t;

float knexus_jc4310_angle_kp = KNEXUS_JC4310_ANGLE_KP_DEFAULT;
float knexus_jc4310_angle_ki = KNEXUS_JC4310_ANGLE_KI_DEFAULT;
float knexus_jc4310_angle_kd = KNEXUS_JC4310_ANGLE_KD_DEFAULT;
float knexus_jc4310_angle_i_limit_nm =
    KNEXUS_JC4310_ANGLE_I_LIMIT_NM_DEFAULT;
float knexus_jc4310_max_torque_nm =
    KNEXUS_JC4310_MAX_TORQUE_NM_DEFAULT;
float knexus_jc4310_torque_slew_nmps =
    KNEXUS_JC4310_TORQUE_SLEW_NMPS_DEFAULT;

static jc_polarity_state_t s_state;
static jc_polarity_fail_t s_fail_code;
static uint32_t s_state_tick;
static uint8_t s_state_command_sent;
static uint8_t s_ready;
static uint8_t s_platform_supported;
static knx_status_t s_last_status;
static knx_status_t s_subscribe_status;
static knx_status_t s_driver_init_status;
static knx_status_t s_dm_sub_active_status;
static knx_status_t s_dm_sub_reply_status;
static knx_status_t s_dm_init_status;

static int8_t s_command_direction;
static int8_t s_last_command_direction;
static float s_command_torque_nm;
static uint32_t s_zero_command_count;
static uint32_t s_nonzero_command_count;
static uint32_t s_feedback_poll_fail_count;
static uint8_t s_feedback_poll_index;
static uint8_t s_slow_feedback_poll_index;

static float s_voltage_v;
static float s_bus_current_a;
static float s_speed_rpm;
static float s_position_deg;
static uint32_t s_position_tick_ms;
static uint32_t s_position_age_ms;
static uint8_t s_position_valid;
static float s_position_velocity_dps;
static float s_drv_temp_c;
static float s_mot_temp_c;
static uint32_t s_error_code;

static float s_initial_position_deg;
static float s_position_delta_deg;
static float s_initial_roll_deg;
static float s_roll_deg;
static float s_roll_sensor_deg;
static float s_roll_delta_deg;
static float s_roll_rate_dps;
static uint32_t s_tilt_age_ms;
static uint8_t s_tilt_valid;
static uint32_t s_tilt_future_timestamp_clamp_count;
static uint8_t s_upper_limit_active;
static uint8_t s_lower_limit_active;
static uint8_t s_limit_blocked;
static float s_limit_torque_scale;
static float s_limit_hold_torque_nm;
static uint8_t s_compensation_active;
static uint8_t s_motion_comp_requested;
static uint8_t s_external_control_active;
static uint8_t s_external_motion_comp_enabled;
static float s_external_target_roll_deg;
static uint8_t s_chassis_imu_valid;
static float s_last_chassis_speed_mps;
static float s_encoder_accel_mps2;
static float s_target_roll_deg;
static float s_target_roll_rate_dps;
static float s_jc_target_roll_deg;
static float s_angle_error_deg;
static float s_rate_target_dps;
static float s_rate_error_dps;
static float s_rate_torque_limit_nm;
static float s_p_torque_nm;
static float s_i_torque_nm;
static float s_d_torque_nm;
static float s_raw_torque_nm;
static float s_predicted_error_deg;
static float s_d_opposition_ratio;
static uint8_t s_reversal_lead_active;
static float s_effective_kp_gain;
static float s_effective_kd_gain;
static float s_motion_target_filtered_deg;
static uint32_t s_reversal_candidate_since_ms;
static uint32_t s_reversal_latched_ms;
static uint32_t s_reversal_enter_count;
static jc_reverse_phase_t s_reverse_phase;
static int8_t s_reverse_direction;
static int8_t s_reverse_candidate_direction;
static uint32_t s_reverse_phase_tick_ms;
static uint32_t s_reverse_last_kick_ms;
static uint32_t s_reverse_kick_count;
static float s_reverse_applied_torque_nm;
static uint8_t s_limit_post_settle_active;
static uint8_t s_control_was_active;
static uint8_t s_startup_active;
static uint32_t s_startup_tick_ms;
static float s_startup_reference_roll_deg;
static float s_static_torque_nm;
static float s_limit_predicted_position_deg;
static float s_limit_target_velocity_dps;
static uint8_t s_roll_guard_active;
static float s_roll_guard_torque_nm;
/* +1表示从-46°端向正方向退出，-1表示从+39°端向负方向退出。 */
static int8_t s_limit_recovery_direction;
static uint32_t s_limit_recovery_count;

static float jc_clampf(float value, float low, float high);
static void jc_reverse_reset(void);

#if defined(KNX_PLATFORM_STM32)
static void jc_polarity_rx_handler(uint32_t std_id, const uint8_t *data,
                                   uint8_t len, void *user)
{
    (void)user;
    JC_RxHandler(std_id, data, len);
}

static void jc_polarity_dm_rx_handler(uint32_t std_id,
                                      const uint8_t *data,
                                      uint8_t len, void *user)
{
    (void)user;
    DM_IMU_L1_UpdateData(std_id, data, len);
}

static void jc_polarity_dm_init(void)
{
    knx_can_t *can = knx_board_get_can_bus(
        KNEXUS_SCREW_TEST_DM_CAN_BUS_INDEX);
    if (can == NULL) return;

    DM_IMU_L1_AttachCAN(can, DM_IMU_L1_DEFAULT_CAN_ID);
    s_dm_sub_active_status = knx_can_bus_subscribe(
        KNX_CAN_BUS_1, DM_IMU_L1_ACTIVE_REPORT_CAN_ID,
        DM_IMU_L1_ACTIVE_REPORT_CAN_ID,
        jc_polarity_dm_rx_handler, NULL);
    s_dm_sub_reply_status = knx_can_bus_subscribe(
        KNX_CAN_BUS_1, DM_IMU_L1_DEFAULT_CAN_ID,
        DM_IMU_L1_DEFAULT_CAN_ID,
        jc_polarity_dm_rx_handler, NULL);
    if (s_dm_sub_active_status != KNX_OK ||
        s_dm_sub_reply_status != KNX_OK) {
        return;
    }

    knx_port_watchdog_refresh();
    s_dm_init_status = DM_IMU_L1_InitExternalRx();
    knx_port_watchdog_refresh();
    if (s_dm_init_status == KNX_OK &&
        KNEXUS_SCREW_TEST_DM_INTERVAL_MS != 1U) {
        s_dm_init_status = DM_IMU_L1_SetActiveIntervalMs(
            KNEXUS_SCREW_TEST_DM_INTERVAL_MS);
    }
}

static bool jc_polarity_read_float(float (*reader)(uint8_t), float *out)
{
    JC_Stats before;
    JC_Stats after;
    JC_GetStats(&before);
    float value = reader(KNEXUS_JC4310_LINK_MOTOR_ID);
    JC_GetStats(&after);
    s_last_status = after.last_status;
    if (after.last_status != KNX_OK || after.rx_count <= before.rx_count) {
        ++s_feedback_poll_fail_count;
        return false;
    }
    *out = value;
    return true;
}

static void jc_polarity_read_error(void)
{
    JC_Stats before;
    JC_Stats after;
    JC_GetStats(&before);
    uint32_t value = JC_ReadError(KNEXUS_JC4310_LINK_MOTOR_ID);
    JC_GetStats(&after);
    s_last_status = after.last_status;
    if (after.last_status != KNX_OK || after.rx_count <= before.rx_count) {
        ++s_feedback_poll_fail_count;
        return;
    }
    s_error_code = value;
    if (value != JC_ERR_NONE) {
        s_fail_code = JC_POLARITY_FAIL_DRIVER_ERROR;
    }
}

static void jc_polarity_read_position(void)
{
    float position_deg;
    if (!jc_polarity_read_float(JC_ReadPosition, &position_deg)) return;

    uint32_t now_ms = knx_millis();
    if (s_position_valid != 0U) {
        uint32_t elapsed_ms = now_ms - s_position_tick_ms;
        if (elapsed_ms > 0U && elapsed_ms <= 100U) {
            float raw_velocity_dps =
                (position_deg - s_position_deg) * 1000.0f /
                (float)elapsed_ms;
            raw_velocity_dps = jc_clampf(
                raw_velocity_dps, -5000.0f, 5000.0f);
            float alpha = jc_clampf(
                KNEXUS_JC4310_POSITION_VELOCITY_LPF_ALPHA,
                0.0f, 1.0f);
            s_position_velocity_dps += alpha *
                (raw_velocity_dps - s_position_velocity_dps);
        }
    } else {
        s_position_velocity_dps = 0.0f;
    }
    s_position_deg = position_deg;
    s_position_tick_ms = now_ms;
    s_position_valid = 1U;
}

static void jc_polarity_poll_feedback(void)
{
    /* Position is the mechanical-limit source, so reserve 9/10 polls for
     * it.  The remaining telemetry is intentionally slow; safety must not
    * wait behind voltage/temperature reads. */
    if (s_feedback_poll_index != 9U) {
        jc_polarity_read_position();
        s_feedback_poll_index++;
        return;
    }

    switch (s_slow_feedback_poll_index) {
    case 0U:
        (void)jc_polarity_read_float(JC_ReadSpeed, &s_speed_rpm);
        break;
    case 1U:
        (void)jc_polarity_read_float(JC_ReadBusCurrent,
                                     &s_bus_current_a);
        break;
    case 2U:
        (void)jc_polarity_read_float(JC_ReadVoltage, &s_voltage_v);
        break;
    case 3U:
        (void)jc_polarity_read_float(JC_ReadDrvTemp, &s_drv_temp_c);
        break;
    case 4U:
        (void)jc_polarity_read_float(JC_ReadMotTemp, &s_mot_temp_c);
        break;
    default:
        jc_polarity_read_error();
        break;
    }
    s_slow_feedback_poll_index =
        (uint8_t)((s_slow_feedback_poll_index + 1U) % 6U);
    s_feedback_poll_index = 0U;
}

static bool jc_polarity_send_torque(float torque_nm)
{
    s_command_torque_nm = torque_nm;
    s_last_status = JC_SetTorque(
        KNEXUS_JC4310_LINK_MOTOR_ID, torque_nm);
    if (s_last_status != KNX_OK) {
        s_fail_code = JC_POLARITY_FAIL_TX;
        return false;
    }
    if (torque_nm == 0.0f) ++s_zero_command_count;
    else ++s_nonzero_command_count;
    return true;
}

void knexus_jc4310_external_target_set(float target_roll_deg, bool enabled)
{
    knexus_jc4310_external_target_set_ex(
        target_roll_deg, enabled, false);
}

void knexus_jc4310_external_target_set_ex(
    float target_roll_deg, bool enabled,
    bool add_motion_compensation)
{
    uint8_t was_external_active = s_external_control_active;
    s_external_target_roll_deg = jc_clampf(
        target_roll_deg, KNEXUS_JC4310_TARGET_ROLL_MIN_DEG,
        KNEXUS_JC4310_TARGET_ROLL_MAX_DEG);
    s_external_control_active = enabled ? 1U : 0U;
    s_external_motion_comp_enabled =
        enabled && add_motion_compensation ? 1U : 0U;
    s_motion_comp_requested = s_external_motion_comp_enabled;
    if (!enabled) {
        s_compensation_active = 0U;
        s_i_torque_nm = 0.0f;
        s_jc_target_roll_deg = 0.0f;
        s_motion_target_filtered_deg = 0.0f;
        s_limit_recovery_direction = 0;
        s_control_was_active = 0U;
        s_startup_active = 0U;
        s_static_torque_nm = 0.0f;
        jc_reverse_reset();
    } else if (was_external_active == 0U) {
        /* The update loop performs the actual bumpless handover using the
         * newest DM-IMU sample. */
        s_control_was_active = 0U;
    }
}

void knexus_jc4310_motion_comp_request(bool enabled)
{
    s_motion_comp_requested = enabled ? 1U : 0U;
    if (!enabled) {
        s_compensation_active = 0U;
        s_i_torque_nm = 0.0f;
        s_jc_target_roll_deg = 0.0f;
        s_motion_target_filtered_deg = 0.0f;
        s_reversal_candidate_since_ms = 0U;
        s_reversal_latched_ms = 0U;
        s_reversal_lead_active = 0U;
        s_limit_recovery_direction = 0;
        s_control_was_active = 0U;
        s_startup_active = 0U;
        s_static_torque_nm = 0.0f;
        jc_reverse_reset();
    }
}

void knexus_jc4310_force_zero(void)
{
    s_compensation_active = 0U;
    s_motion_comp_requested = 0U;
    s_external_control_active = 0U;
    s_external_motion_comp_enabled = 0U;
    s_external_target_roll_deg = 0.0f;
    s_jc_target_roll_deg = 0.0f;
    s_i_torque_nm = 0.0f;
    s_command_direction = 0;
    s_motion_target_filtered_deg = 0.0f;
    s_reversal_candidate_since_ms = 0U;
    s_reversal_latched_ms = 0U;
    s_reversal_lead_active = 0U;
    s_limit_recovery_direction = 0;
    s_control_was_active = 0U;
    s_startup_active = 0U;
    s_static_torque_nm = 0.0f;
    jc_reverse_reset();
    (void)jc_polarity_send_torque(0.0f);
}

uint8_t knexus_jc4310_is_ready(void)
{
    return s_ready;
}

uint8_t knexus_jc4310_motion_comp_is_ready(void)
{
    knx_ball_motion_comp_state_t comp;
    knx_ball_motion_comp_snapshot(&comp);
    return (s_ready != 0U && s_tilt_valid != 0U &&
            s_chassis_imu_valid != 0U && comp.bias_ready &&
            comp.mapping_ready &&
            knexus_ball_compensation_enable > 0.5f) ? 1U : 0U;
}

uint8_t knexus_jc4310_is_settled(void)
{
    return (s_ready != 0U && s_tilt_valid != 0U &&
            s_position_valid != 0U &&
            s_limit_recovery_direction == 0 &&
            s_startup_active == 0U &&
            fabsf(s_angle_error_deg) <= 0.8f &&
            fabsf(s_roll_rate_dps) <= 3.0f &&
            s_position_deg > KNEXUS_JC4310_MOTOR_MIN_POSITION_DEG &&
            s_position_deg < KNEXUS_JC4310_MOTOR_MAX_POSITION_DEG) ? 1U : 0U;
}
#endif

static void jc_polarity_enter(jc_polarity_state_t state)
{
    s_state = state;
    s_state_tick = knx_millis();
    s_state_command_sent = 0U;
}

static float jc_clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float jc_slew(float current, float target, float rate_per_s,
                     float dt_s)
{
    float max_step = fabsf(rate_per_s) * dt_s;
    return current + jc_clampf(target - current, -max_step, max_step);
}

static int8_t jc_torque_direction(float torque_nm, float threshold_nm)
{
    if (torque_nm > fabsf(threshold_nm)) return 1;
    if (torque_nm < -fabsf(threshold_nm)) return -1;
    return 0;
}

static void jc_reverse_reset(void)
{
    s_reverse_phase = JC_REVERSE_IDLE;
    s_reverse_direction = 0;
    s_reverse_candidate_direction = 0;
    s_reverse_phase_tick_ms = 0U;
    s_reverse_applied_torque_nm = 0.0f;
    s_limit_post_settle_active = 0U;
}

static void jc_polarity_reset_data(void)
{
    s_fail_code = JC_POLARITY_FAIL_NONE;
    s_ready = 0U;
    s_command_direction = 0;
    s_last_command_direction = 0;
    s_command_torque_nm = 0.0f;
    s_zero_command_count = 0U;
    s_nonzero_command_count = 0U;
    s_feedback_poll_fail_count = 0U;
    s_feedback_poll_index = 0U;
    s_slow_feedback_poll_index = 0U;
    s_voltage_v = -1.0f;
    s_bus_current_a = 0.0f;
    s_speed_rpm = 0.0f;
    s_position_deg = 0.0f;
    s_position_tick_ms = 0U;
    s_position_age_ms = 0xFFFFFFFFU;
    s_position_valid = 0U;
    s_position_velocity_dps = 0.0f;
    s_drv_temp_c = -1.0f;
    s_mot_temp_c = -1.0f;
    s_error_code = 0U;
    s_initial_position_deg = 0.0f;
    s_position_delta_deg = 0.0f;
    s_initial_roll_deg = 0.0f;
    s_roll_deg = 0.0f;
    s_roll_sensor_deg = 0.0f;
    s_roll_delta_deg = 0.0f;
    s_roll_rate_dps = 0.0f;
    s_tilt_age_ms = 0xFFFFFFFFU;
    s_tilt_valid = 0U;
    s_tilt_future_timestamp_clamp_count = 0U;
    s_upper_limit_active = 0U;
    s_lower_limit_active = 0U;
    s_limit_blocked = 0U;
    s_limit_torque_scale = 1.0f;
    s_limit_hold_torque_nm = 0.0f;
    s_compensation_active = 0U;
    s_motion_comp_requested = 0U;
    s_external_control_active = 0U;
    s_external_motion_comp_enabled = 0U;
    s_external_target_roll_deg = 0.0f;
    s_chassis_imu_valid = 0U;
    s_last_chassis_speed_mps = 0.0f;
    s_encoder_accel_mps2 = 0.0f;
    s_target_roll_deg = 0.0f;
    s_target_roll_rate_dps = 0.0f;
    s_jc_target_roll_deg = 0.0f;
    s_angle_error_deg = 0.0f;
    s_rate_target_dps = 0.0f;
    s_rate_error_dps = 0.0f;
    s_rate_torque_limit_nm = 0.0f;
    s_p_torque_nm = 0.0f;
    s_i_torque_nm = 0.0f;
    s_d_torque_nm = 0.0f;
    s_raw_torque_nm = 0.0f;
    s_predicted_error_deg = 0.0f;
    s_d_opposition_ratio = 0.0f;
    s_reversal_lead_active = 0U;
    s_effective_kp_gain = 1.0f;
    s_effective_kd_gain = 1.0f;
    s_motion_target_filtered_deg = 0.0f;
    s_reversal_candidate_since_ms = 0U;
    s_reversal_latched_ms = 0U;
    s_reversal_enter_count = 0U;
    jc_reverse_reset();
    s_reverse_last_kick_ms = 0U;
    s_reverse_kick_count = 0U;
    s_control_was_active = 0U;
    s_startup_active = 0U;
    s_startup_tick_ms = 0U;
    s_startup_reference_roll_deg = 0.0f;
    s_static_torque_nm = 0.0f;
    s_limit_predicted_position_deg = 0.0f;
    s_limit_target_velocity_dps = 0.0f;
    s_roll_guard_active = 0U;
    s_roll_guard_torque_nm = 0.0f;
    s_limit_recovery_direction = 0;
    s_limit_recovery_count = 0U;
}

void knexus_mode_jc4310_link_center_init(void)
{
    s_platform_supported = 0U;
    s_last_status = KNX_NOT_READY;
    s_subscribe_status = KNX_NOT_READY;
    s_driver_init_status = KNX_NOT_READY;
    s_dm_sub_active_status = KNX_NOT_READY;
    s_dm_sub_reply_status = KNX_NOT_READY;
    s_dm_init_status = KNX_NOT_READY;
    jc_polarity_reset_data();
    knx_ball_motion_comp_init();
    knexus_ball_accel_lpf_hz = KNEXUS_JC4310_ACCEL_LPF_HZ;
    jc_polarity_enter(JC_POLARITY_ENTER_SERVO);
    (void)knx_chassis_disable();

#if defined(KNX_PLATFORM_STM32)
    knx_can_t *can = knx_board_get_can_bus(
        KNEXUS_JC4310_LINK_CAN_BUS_INDEX);
    if (can != NULL) {
        s_platform_supported = 1U;
        JC_AttachCAN(can);
        JC_SetTimeoutMs(KNEXUS_JC4310_LINK_TIMEOUT_MS);
        s_subscribe_status = knx_can_bus_subscribe(
            KNX_CAN_BUS_2,
            (uint16_t)(JC_RX_ID_BASE + KNEXUS_JC4310_LINK_MOTOR_ID),
            (uint16_t)(JC_RX_ID_BASE + KNEXUS_JC4310_LINK_MOTOR_ID),
            jc_polarity_rx_handler, NULL);
        s_driver_init_status = JC_InitExternalRx();
    }
    jc_polarity_dm_init();

    if (!s_platform_supported) {
        s_fail_code = JC_POLARITY_FAIL_PLATFORM;
        jc_polarity_enter(JC_POLARITY_FAULT);
    } else if (s_subscribe_status != KNX_OK) {
        s_fail_code = JC_POLARITY_FAIL_CAN_SUBSCRIBE;
        jc_polarity_enter(JC_POLARITY_FAULT);
    } else if (s_driver_init_status != KNX_OK) {
        s_fail_code = JC_POLARITY_FAIL_DRIVER_INIT;
        jc_polarity_enter(JC_POLARITY_FAULT);
    } else if (s_dm_sub_active_status != KNX_OK ||
               s_dm_sub_reply_status != KNX_OK) {
        s_fail_code = JC_POLARITY_FAIL_DM_SUBSCRIBE;
        jc_polarity_enter(JC_POLARITY_FAULT);
    } else if (s_dm_init_status != KNX_OK) {
        s_fail_code = JC_POLARITY_FAIL_DM_INIT;
        jc_polarity_enter(JC_POLARITY_FAULT);
    } else {
        /* 在进入伺服模式前也先写一次0力矩，保证寄存器中不存在旧目标。 */
        (void)jc_polarity_send_torque(0.0f);
    }
#else
    s_fail_code = JC_POLARITY_FAIL_PLATFORM;
    jc_polarity_enter(JC_POLARITY_FAULT);
#endif

    knx_led_set(KNX_LED_1, false);
    knx_led_set(KNX_LED_2, false);
}

void knexus_mode_jc4310_link_center_update(
    const struct knx26_context *raw_context)
{
    const knx26_context_t *context = (const knx26_context_t *)raw_context;
    if (context == NULL) return;
#if !defined(KNEXUS_MODE_SIX_MENU)
    (void)knx_chassis_disable();
#endif

#if !defined(KNX_PLATFORM_STM32)
    return;
#else
    dm_imu_l1_data_t imu;
    DM_IMU_L1_Snapshot(&imu);
    /* context快照早于本函数读取DM-IMU：若CAN回调恰好在两者之间更新，
     * imu.timestamp会比context->now_ms新1~2ms。旧无符号减法会下溢成
     * 0xFFFFFFFE并周期性撤销控制。使用当前系统时钟并把微小未来时间夹到0。 */
    uint32_t tilt_now_ms = knx_millis();
    int32_t tilt_age_signed_ms =
        (int32_t)(tilt_now_ms - imu.timestamp);
    if (tilt_age_signed_ms < 0) {
        s_tilt_age_ms = 0U;
        s_tilt_future_timestamp_clamp_count++;
    } else {
        s_tilt_age_ms = (uint32_t)tilt_age_signed_ms;
    }
    s_roll_sensor_deg = imu.roll;
    s_roll_deg = s_roll_sensor_deg -
        KNEXUS_JC4310_ROD_ZERO_ROLL_DEG;
    s_roll_rate_dps = KNEXUS_DM_IMU_ROLL_GYRO_SIGN *
                      imu.gyro[0] * 57.2957795f;
    s_tilt_valid =
        (DM_IMU_L1_IsDataReady() != 0U &&
         DM_IMU_L1_IsTiltReady() != 0U &&
         s_tilt_age_ms <= KNEXUS_SCREW_TILT_MAX_AGE_MS) ? 1U : 0U;
    s_position_age_ms = s_position_valid != 0U
        ? context->now_ms - s_position_tick_ms : 0xFFFFFFFFU;
    if (s_position_age_ms > KNEXUS_JC4310_POSITION_MAX_AGE_MS) {
        s_position_valid = 0U;
    }
    s_upper_limit_active =
        (s_position_valid &&
         s_position_deg <= KNEXUS_JC4310_MOTOR_MIN_POSITION_DEG) ? 1U : 0U;
    s_lower_limit_active =
        (s_position_valid &&
         s_position_deg >= KNEXUS_JC4310_MOTOR_MAX_POSITION_DEG) ? 1U : 0U;

    const float dt_s = (float)KNEXUS_APP_PERIOD_MS * 0.001f;
    float measured_speed = context->chassis.drive.measured_linear_mps;
    float encoder_accel_raw =
        (measured_speed - s_last_chassis_speed_mps) / dt_s;
    float encoder_tau_s =
        1.0f / (6.28318530718f * KNEXUS_BALL_ACCEL_LPF_HZ);
    float encoder_alpha = dt_s / (encoder_tau_s + dt_s);
    s_encoder_accel_mps2 +=
        encoder_alpha * (encoder_accel_raw - s_encoder_accel_mps2);
    s_last_chassis_speed_mps = measured_speed;

    uint32_t chassis_imu_age_ms =
        context->now_ms - context->imu.last_update_ms;
    s_chassis_imu_valid =
        (context->imu.accel_ok && context->imu.ekf_ready &&
         chassis_imu_age_ms <= 50U) ? 1U : 0U;
    bool stationary_for_bias =
        s_compensation_active == 0U && s_external_control_active == 0U &&
        s_chassis_imu_valid != 0U &&
        fabsf(measured_speed) < 0.02f &&
        fabsf(s_roll_rate_dps) < 0.5f;
    bool chassis_stationary =
        fabsf(measured_speed) < KNEXUS_BALL_STATIONARY_SPEED_MPS &&
        fabsf(s_encoder_accel_mps2) <
            KNEXUS_BALL_STATIONARY_ENCODER_ACCEL_MPS2;
    float chassis_accel[3] = {
        context->imu.accel[0], context->imu.accel[1],
        context->imu.accel[2]
    };
    float chassis_command_accel_mps2 = context->chassis.enabled
        ? knexus_line_follow_core_get_accel_command_mps2() : 0.0f;
    knx_ball_motion_comp_update(
        chassis_accel, context->imu.roll, context->imu.pitch,
        s_chassis_imu_valid != 0U, s_roll_deg,
        s_encoder_accel_mps2, context->chassis.enabled,
        chassis_command_accel_mps2,
        stationary_for_bias, chassis_stationary,
        s_compensation_active != 0U, dt_s);
    knx_ball_motion_comp_state_t comp;
    knx_ball_motion_comp_snapshot(&comp);

#if !defined(KNEXUS_MODE_SIX_MENU)
    if (knx_key_just_pressed(KNX_KEY_1)) {
        s_compensation_active = 0U;
        s_i_torque_nm = 0.0f;
        knx_beep_beep(100U);
    }
    if (knx_key_just_pressed(KNX_KEY_0)) {
        if (s_ready != 0U && s_tilt_valid != 0U &&
            comp.bias_ready && comp.mapping_ready &&
            knexus_ball_compensation_enable > 0.5f) {
            s_compensation_active = 1U;
            s_i_torque_nm = 0.0f;
            knx_beep_beep(100U);
        } else {
            s_compensation_active = 0U;
            knx_beep_beep(400U);
        }
    }
#else
    if (s_motion_comp_requested != 0U && s_ready != 0U &&
        s_tilt_valid != 0U && comp.bias_ready && comp.mapping_ready &&
        knexus_ball_compensation_enable > 0.5f) {
        s_compensation_active = 1U;
    } else {
        s_compensation_active = 0U;
    }
#endif

    bool control_active =
        s_compensation_active != 0U || s_external_control_active != 0U;
    bool control_ready = control_active && s_tilt_valid != 0U &&
                         s_position_valid != 0U;
    bool control_just_started =
        control_ready && s_control_was_active == 0U;
    if (control_just_started) {
        /* Bumpless transfer: take over at the measured rod angle first,
         * then blend toward the requested angle. */
        s_startup_active = 1U;
        s_startup_tick_ms = context->now_ms;
        s_startup_reference_roll_deg = jc_clampf(
            s_roll_deg, KNEXUS_JC4310_TARGET_ROLL_MIN_DEG,
            KNEXUS_JC4310_TARGET_ROLL_MAX_DEG);
        s_jc_target_roll_deg = s_startup_reference_roll_deg;
        s_i_torque_nm = 0.0f;
        s_reversal_candidate_since_ms = 0U;
        s_reversal_latched_ms = 0U;
        s_reversal_lead_active = 0U;
        jc_reverse_reset();
    } else if (!control_ready) {
        s_startup_active = 0U;
        s_static_torque_nm = 0.0f;
    }
    s_control_was_active = control_ready ? 1U : 0U;

    bool limit_recovery_just_started = false;
    bool limit_recovery_just_completed = false;
    if (!control_active || s_position_valid == 0U) {
        s_limit_recovery_direction = 0;
    } else if (s_upper_limit_active != 0U) {
        if (s_limit_recovery_direction != 1) {
            s_limit_recovery_direction = 1;
            s_limit_recovery_count++;
            limit_recovery_just_started = true;
            s_i_torque_nm = 0.0f;
            s_startup_active = 1U;
            s_startup_tick_ms = context->now_ms;
            s_startup_reference_roll_deg = jc_clampf(
                s_roll_deg, KNEXUS_JC4310_TARGET_ROLL_MIN_DEG,
                KNEXUS_JC4310_TARGET_ROLL_MAX_DEG);
            s_jc_target_roll_deg = s_startup_reference_roll_deg;
        }
    } else if (s_lower_limit_active != 0U) {
        if (s_limit_recovery_direction != -1) {
            s_limit_recovery_direction = -1;
            s_limit_recovery_count++;
            limit_recovery_just_started = true;
            s_i_torque_nm = 0.0f;
            s_startup_active = 1U;
            s_startup_tick_ms = context->now_ms;
            s_startup_reference_roll_deg = jc_clampf(
                s_roll_deg, KNEXUS_JC4310_TARGET_ROLL_MIN_DEG,
                KNEXUS_JC4310_TARGET_ROLL_MAX_DEG);
            s_jc_target_roll_deg = s_startup_reference_roll_deg;
        }
    } else if (s_limit_recovery_direction > 0) {
        float target_position = KNEXUS_JC4310_MOTOR_MIN_POSITION_DEG +
                                KNEXUS_JC4310_LIMIT_RETURN_MARGIN_DEG;
        /* Recovery is one-way: once the encoder has crossed the safe return
         * line it must not wait to land inside a narrow +/- tolerance band.
         * A startup CAN sample can briefly report an old out-of-range angle;
         * the following valid sample may already be well inside the range.
         * The old fabsf() test then latched recovery forever because the
         * target had been overshot.  Keep recovery only while the mechanism
         * is still outside the return line, or is moving rapidly back toward
         * the same hard limit. */
        bool crossed_return_line = s_position_deg >= target_position;
        bool moving_outward_fast = s_position_velocity_dps <
            -KNEXUS_JC4310_LIMIT_RECOVERY_EXIT_VEL_DPS;
        bool deep_inside_safe_range = s_position_deg >=
            target_position + KNEXUS_JC4310_LIMIT_BRAKE_ZONE_DEG;
        if (crossed_return_line &&
            (!moving_outward_fast || deep_inside_safe_range)) {
            s_limit_recovery_direction = 0;
            limit_recovery_just_completed = true;
        }
    } else if (s_limit_recovery_direction < 0) {
        float target_position = KNEXUS_JC4310_MOTOR_MAX_POSITION_DEG -
                                KNEXUS_JC4310_LIMIT_RETURN_MARGIN_DEG;
        bool crossed_return_line = s_position_deg <= target_position;
        bool moving_outward_fast = s_position_velocity_dps >
            KNEXUS_JC4310_LIMIT_RECOVERY_EXIT_VEL_DPS;
        bool deep_inside_safe_range = s_position_deg <=
            target_position - KNEXUS_JC4310_LIMIT_BRAKE_ZONE_DEG;
        if (crossed_return_line &&
            (!moving_outward_fast || deep_inside_safe_range)) {
            s_limit_recovery_direction = 0;
            limit_recovery_just_completed = true;
        }
    }
    if (limit_recovery_just_completed) {
        /* Restart the bumpless handover after the safety controller has
         * settled at its return point. */
        s_startup_active = 1U;
        s_startup_tick_ms = context->now_ms;
        s_startup_reference_roll_deg = jc_clampf(
            s_roll_deg, KNEXUS_JC4310_TARGET_ROLL_MIN_DEG,
            KNEXUS_JC4310_TARGET_ROLL_MAX_DEG);
        s_jc_target_roll_deg = s_startup_reference_roll_deg;
        s_i_torque_nm = 0.0f;
        jc_reverse_reset();
    }

    uint32_t elapsed = context->now_ms - s_state_tick;
    if (s_state == JC_POLARITY_ENTER_SERVO) {
        (void)jc_polarity_send_torque(0.0f);
        if (s_state_command_sent == 0U) {
            s_last_status = JC_EnterServo(KNEXUS_JC4310_LINK_MOTOR_ID);
            if (s_last_status != KNX_OK) {
                s_fail_code = JC_POLARITY_FAIL_TX;
                jc_polarity_enter(JC_POLARITY_FAULT);
            } else {
                s_state_command_sent = 1U;
                s_state_tick = context->now_ms;
            }
        } else if (elapsed >= KNEXUS_JC4310_LINK_MODE_GAP_MS) {
            jc_polarity_enter(JC_POLARITY_SWITCH_TORQUE);
        }
    } else if (s_state == JC_POLARITY_SWITCH_TORQUE) {
        (void)jc_polarity_send_torque(0.0f);
        if (s_state_command_sent == 0U) {
            s_last_status = JC_SwitchMode(
                KNEXUS_JC4310_LINK_MOTOR_ID, JC_MODE_TORQUE);
            if (s_last_status != KNX_OK) {
                s_fail_code = JC_POLARITY_FAIL_TX;
                jc_polarity_enter(JC_POLARITY_FAULT);
            } else {
                s_state_command_sent = 1U;
                s_state_tick = context->now_ms;
            }
        } else if (elapsed >= KNEXUS_JC4310_LINK_MODE_GAP_MS) {
            s_ready = 1U;
            s_initial_position_deg = s_position_deg;
            s_initial_roll_deg = s_roll_deg;
            jc_polarity_enter(JC_POLARITY_READY);
            knx_beep_beep(100U);
        }
    } else if (s_state == JC_POLARITY_READY) {
        float requested_torque_nm = 0.0f;
        s_target_roll_deg = 0.0f;
        s_target_roll_rate_dps = 0.0f;
        s_angle_error_deg = 0.0f;
        s_p_torque_nm = 0.0f;
        s_d_torque_nm = 0.0f;
        s_raw_torque_nm = 0.0f;
        s_predicted_error_deg = 0.0f;
        s_d_opposition_ratio = 0.0f;
        s_effective_kp_gain = 1.0f;
        s_effective_kd_gain = 1.0f;
        if (control_active && s_tilt_valid != 0U &&
            s_position_valid != 0U) {
            /* The shared motion-compensation module also serves the old
             * screw mechanism and therefore clips its public target to the
             * old -10/+5 degree travel.  JC4310 has a much wider measured
             * range, so use the unclipped feed-forward angle and apply the
             * JC-specific target guard and slew here. */
            float external_motion_roll_deg =
                s_external_motion_comp_enabled != 0U
                    ? comp.feedforward_roll_deg : 0.0f;
            float desired_roll_raw_deg = jc_clampf(
                s_external_control_active != 0U
                    ? s_external_target_roll_deg +
                        external_motion_roll_deg
                    : comp.feedforward_roll_deg,
                KNEXUS_JC4310_TARGET_ROLL_MIN_DEG,
                KNEXUS_JC4310_TARGET_ROLL_MAX_DEG);
            float desired_roll_deg = desired_roll_raw_deg;
            if (s_compensation_active != 0U) {
                if (control_just_started) {
                    s_motion_target_filtered_deg = desired_roll_raw_deg;
                } else {
                    float target_tau_s = 1.0f /
                        (6.28318530718f * KNEXUS_JC4310_TARGET_LPF_HZ);
                    float target_alpha = dt_s / (target_tau_s + dt_s);
                    s_motion_target_filtered_deg += target_alpha *
                        (desired_roll_raw_deg -
                         s_motion_target_filtered_deg);
                }
                desired_roll_deg = s_motion_target_filtered_deg;
            } else {
                s_motion_target_filtered_deg = 0.0f;
            }

            float target_slew_dps =
                KNEXUS_JC4310_TARGET_ROLL_SLEW_DPS;
            if (s_limit_recovery_direction != 0) {
                /* Safety recovery owns the actuator.  Keep the attitude
                 * loop bumpless until encoder position and speed settle. */
                s_startup_active = 1U;
                s_startup_tick_ms = context->now_ms;
                s_startup_reference_roll_deg = jc_clampf(
                    s_roll_deg, KNEXUS_JC4310_TARGET_ROLL_MIN_DEG,
                    KNEXUS_JC4310_TARGET_ROLL_MAX_DEG);
                s_jc_target_roll_deg = s_startup_reference_roll_deg;
                desired_roll_deg = s_startup_reference_roll_deg;
                target_slew_dps =
                    KNEXUS_JC4310_STARTUP_TARGET_SLEW_DPS;
            } else if (s_startup_active != 0U) {
                uint32_t startup_elapsed_ms =
                    context->now_ms - s_startup_tick_ms;
                float blend = jc_clampf(
                    (float)startup_elapsed_ms /
                        (float)KNEXUS_JC4310_STARTUP_BLEND_MS,
                    0.0f, 1.0f);
                /* Smoothstep gives zero target-rate discontinuity at both
                 * ends of the handover. */
                blend = blend * blend * (3.0f - 2.0f * blend);
                desired_roll_deg = s_startup_reference_roll_deg + blend *
                    (desired_roll_deg - s_startup_reference_roll_deg);
                target_slew_dps =
                    KNEXUS_JC4310_STARTUP_TARGET_SLEW_DPS;
                if (startup_elapsed_ms >=
                    KNEXUS_JC4310_STARTUP_BLEND_MS) {
                    s_startup_active = 0U;
                }
            }
            float target_step_deg = jc_clampf(
                desired_roll_deg - s_jc_target_roll_deg,
                -target_slew_dps * dt_s,
                target_slew_dps * dt_s);
            s_jc_target_roll_deg += target_step_deg;
            s_target_roll_deg = s_jc_target_roll_deg;
            s_target_roll_rate_dps = target_step_deg / dt_s;
            s_angle_error_deg = s_target_roll_deg - s_roll_deg;
            if (fabsf(s_angle_error_deg) <=
                    KNEXUS_JC4310_ANGLE_DEADBAND_DEG &&
                fabsf(s_target_roll_rate_dps - s_roll_rate_dps) <=
                    KNEXUS_JC4310_RATE_DEADBAND_DPS) {
                s_angle_error_deg = 0.0f;
            }
#if KNEXUS_JC4310_CASCADE_RATE_ENABLE
            /*
             * Explicit angle-rate cascade:
             *   angle error -> requested roll rate -> rate PI -> torque.
             *
             * The old parallel PD was mathematically similar only while it
             * remained linear.  Its D-opposition limiter and open-loop
             * reversal kick removed braking exactly when the rod was fast,
             * allowing a +/-2.45 deg command to become +11.74/-6.69 deg.
             * This controller always treats measured angular velocity as the
             * quantity to close first and gives braking more authority than
             * acceleration.
             */
            float error_abs_deg = fabsf(s_angle_error_deg);
            float near_rate_blend = jc_clampf(
                error_abs_deg /
                    KNEXUS_JC4310_ANGLE_RATE_NEAR_ZONE_DEG,
                0.0f, 1.0f);
            float angle_to_rate_gain =
                KNEXUS_JC4310_ANGLE_TO_RATE_KP_DPS_PER_DEG *
                (KNEXUS_JC4310_ANGLE_RATE_NEAR_GAIN +
                 (1.0f - KNEXUS_JC4310_ANGLE_RATE_NEAR_GAIN) *
                     near_rate_blend);
            float target_rate_ff_dps = jc_clampf(
                KNEXUS_JC4310_RATE_TARGET_FF_GAIN *
                    s_target_roll_rate_dps,
                -KNEXUS_JC4310_RATE_TARGET_FF_LIMIT_DPS,
                KNEXUS_JC4310_RATE_TARGET_FF_LIMIT_DPS);
            s_rate_target_dps = jc_clampf(
                angle_to_rate_gain * s_angle_error_deg +
                    target_rate_ff_dps,
                -KNEXUS_JC4310_ANGLE_RATE_MAX_DPS,
                KNEXUS_JC4310_ANGLE_RATE_MAX_DPS);
            s_rate_error_dps = s_rate_target_dps - s_roll_rate_dps;
            s_p_torque_nm = KNEXUS_JC4310_RATE_KP_NM_PER_DPS *
                s_rate_error_dps;
            s_d_torque_nm = 0.0f;
            s_effective_kp_gain = angle_to_rate_gain /
                KNEXUS_JC4310_ANGLE_TO_RATE_KP_DPS_PER_DEG;
            s_effective_kd_gain = 1.0f;
            s_d_opposition_ratio = 0.0f;
            s_predicted_error_deg = s_angle_error_deg +
                KNEXUS_JC4310_REVERSAL_LOOKAHEAD_S *
                    (s_target_roll_rate_dps - s_roll_rate_dps);
            s_reversal_candidate_since_ms = 0U;
            s_reversal_latched_ms = 0U;
            s_reversal_lead_active = 0U;

            /* A small smooth static term gets the linkage moving; the rate
             * integrator then learns the torque needed to hold the angle. */
            s_static_torque_nm = 0.0f;
            if (s_startup_active == 0U &&
                s_limit_recovery_direction == 0 &&
                fabsf(s_roll_rate_dps) <
                    KNEXUS_JC4310_STATIC_MAX_ROLL_RATE_DPS &&
                error_abs_deg > KNEXUS_JC4310_STATIC_ENTER_ERROR_DEG) {
                float static_span =
                    KNEXUS_JC4310_STATIC_FULL_ERROR_DEG -
                    KNEXUS_JC4310_STATIC_ENTER_ERROR_DEG;
                float static_blend = static_span > 0.01f
                    ? jc_clampf((error_abs_deg -
                            KNEXUS_JC4310_STATIC_ENTER_ERROR_DEG) /
                            static_span,
                        0.0f, 1.0f)
                    : 1.0f;
                s_static_torque_nm =
                    (s_angle_error_deg > 0.0f ? 1.0f : -1.0f) *
                    KNEXUS_JC4310_STATIC_TORQUE_NM * static_blend;
            }

            float torque_without_i_nm =
                s_p_torque_nm + s_static_torque_nm;
            bool braking = fabsf(s_roll_rate_dps) >=
                    KNEXUS_JC4310_RATE_BRAKE_MIN_SPEED_DPS &&
                torque_without_i_nm * s_roll_rate_dps < 0.0f;
            s_rate_torque_limit_nm = braking
                ? KNEXUS_JC4310_RATE_BRAKE_MAX_TORQUE_NM
                : KNEXUS_JC4310_RATE_DRIVE_MAX_TORQUE_NM;

            if (s_upper_limit_active != 0U ||
                s_lower_limit_active != 0U ||
                s_limit_recovery_direction != 0 ||
                s_startup_active != 0U) {
                s_i_torque_nm = 0.0f;
            } else {
                float i_delta_nm =
                    KNEXUS_JC4310_RATE_KI_NM_PER_DEG *
                    s_rate_error_dps * dt_s;
                float unsaturated_nm = torque_without_i_nm +
                    s_i_torque_nm;
                bool inside_rate_i_window = fabsf(s_rate_error_dps) <=
                    KNEXUS_JC4310_RATE_INTEGRAL_MAX_ERROR_DPS;
                bool unwinding = unsaturated_nm * i_delta_nm < 0.0f;
                if (inside_rate_i_window || unwinding) {
                    s_i_torque_nm = jc_clampf(
                        s_i_torque_nm + i_delta_nm,
                        -KNEXUS_JC4310_RATE_I_LIMIT_NM,
                        KNEXUS_JC4310_RATE_I_LIMIT_NM);
                }
            }
            s_raw_torque_nm = KNEXUS_JC4310_ROLL_TORQUE_SIGN *
                (torque_without_i_nm + s_i_torque_nm);
            requested_torque_nm = jc_clampf(
                s_raw_torque_nm,
                -s_rate_torque_limit_nm,
                s_rate_torque_limit_nm);
#else
            float direction_kp_gain = s_angle_error_deg > 0.0f
                ? KNEXUS_JC4310_DOWN_KP_GAIN : 1.0f;
            float error_abs_deg = fabsf(s_angle_error_deg);
            float far_span_deg = KNEXUS_JC4310_FAR_KP_FULL_DEG -
                                 KNEXUS_JC4310_FAR_KP_START_DEG;
            float far_blend = far_span_deg > 0.01f
                ? jc_clampf((error_abs_deg -
                        KNEXUS_JC4310_FAR_KP_START_DEG) / far_span_deg,
                    0.0f, 1.0f)
                : 1.0f;
            s_effective_kp_gain = 1.0f + far_blend *
                (KNEXUS_JC4310_FAR_KP_GAIN - 1.0f);
            /* Static target + large error means an external disturbance,
             * not a fast feed-forward manoeuvre.  Raise stiffness only in
             * this case so hand/ball disturbances are rejected without
             * amplifying the rapidly changing compensation target. */
            if (error_abs_deg >=
                    KNEXUS_JC4310_DISTURBANCE_ERROR_DEG &&
                fabsf(s_target_roll_rate_dps) <=
                    KNEXUS_JC4310_DISTURBANCE_TARGET_RATE_DPS &&
                s_effective_kp_gain <
                    KNEXUS_JC4310_DISTURBANCE_KP_GAIN) {
                s_effective_kp_gain =
                    KNEXUS_JC4310_DISTURBANCE_KP_GAIN;
            }
            s_p_torque_nm = knexus_jc4310_angle_kp *
                direction_kp_gain * s_effective_kp_gain *
                s_angle_error_deg;
            bool integral_window =
                fabsf(s_angle_error_deg) <=
                    KNEXUS_JC4310_INTEGRAL_MAX_ERROR_DEG &&
                fabsf(s_target_roll_rate_dps) <=
                    KNEXUS_JC4310_INTEGRAL_MAX_TARGET_RATE_DPS &&
                fabsf(s_roll_rate_dps) <=
                    KNEXUS_JC4310_INTEGRAL_MAX_ROLL_RATE_DPS;
            if (s_upper_limit_active == 0U &&
                s_lower_limit_active == 0U &&
                s_limit_recovery_direction == 0 &&
                s_startup_active == 0U && integral_window) {
                s_i_torque_nm += knexus_jc4310_angle_ki *
                    s_angle_error_deg * dt_s;
                s_i_torque_nm = jc_clampf(
                    s_i_torque_nm,
                    -fabsf(knexus_jc4310_angle_i_limit_nm),
                    fabsf(knexus_jc4310_angle_i_limit_nm));
            } else {
                s_i_torque_nm = 0.0f;
            }
            float near_blend = KNEXUS_JC4310_NEAR_KD_ZONE_DEG > 0.01f
                ? 1.0f - jc_clampf(error_abs_deg /
                        KNEXUS_JC4310_NEAR_KD_ZONE_DEG, 0.0f, 1.0f)
                : 0.0f;
            s_effective_kd_gain = 1.0f + near_blend *
                (KNEXUS_JC4310_NEAR_KD_GAIN - 1.0f);
            /* Derivative on measurement avoids a full torque impulse whenever
             * the small ±2° target reverses.  A limited 10% target-rate term
             * keeps useful dynamic following without recreating setpoint kick. */
            float target_rate_ff_dps = jc_clampf(
                KNEXUS_JC4310_TARGET_RATE_FF_GAIN *
                    s_target_roll_rate_dps,
                -KNEXUS_JC4310_TARGET_RATE_FF_LIMIT_DPS,
                KNEXUS_JC4310_TARGET_RATE_FF_LIMIT_DPS);
            s_d_torque_nm = knexus_jc4310_angle_kd *
                s_effective_kd_gain *
                (target_rate_ff_dps - s_roll_rate_dps);
            /*
             * 目标快速反向时，当前角度误差会在几十毫秒后换号。旧保护始终把
             * 反向D限制为P的75%，导致大误差时也只剩约0.05 N*m，杆子无法
             * 提前回拉。现在预测60 ms后的符号，并要求条件连续成立30 ms；
             * 进入制动后至少保持80 ms，避免预测边界逐帧抖动。普通跟踪仍
             * 保留原保护。
             */
            float angle_error_rate_dps =
                s_target_roll_rate_dps - s_roll_rate_dps;
            s_predicted_error_deg = s_angle_error_deg +
                KNEXUS_JC4310_REVERSAL_LOOKAHEAD_S *
                    angle_error_rate_dps;
            bool reversal_condition =
                s_compensation_active != 0U &&
                s_startup_active == 0U &&
                s_limit_recovery_direction == 0 &&
                ((fabsf(s_target_roll_rate_dps) >=
                      KNEXUS_JC4310_REVERSAL_MIN_TARGET_RATE_DPS ||
                  fabsf(s_roll_rate_dps) >=
                      KNEXUS_JC4310_REVERSAL_MIN_ROLL_RATE_DPS) &&
                 s_angle_error_deg * s_predicted_error_deg <= 0.0f);
            if (reversal_condition) {
                if (s_reversal_candidate_since_ms == 0U) {
                    s_reversal_candidate_since_ms = context->now_ms;
                }
                if (s_reversal_lead_active == 0U &&
                    (context->now_ms - s_reversal_candidate_since_ms) >=
                        KNEXUS_JC4310_REVERSAL_CONFIRM_MS) {
                    s_reversal_lead_active = 1U;
                    s_reversal_latched_ms = context->now_ms;
                    s_reversal_enter_count++;
                }
            } else {
                s_reversal_candidate_since_ms = 0U;
                if (s_reversal_lead_active != 0U &&
                    (context->now_ms - s_reversal_latched_ms) >=
                        KNEXUS_JC4310_REVERSAL_HOLD_MS) {
                    s_reversal_lead_active = 0U;
                }
            }
            if (fabsf(s_angle_error_deg) >=
                    KNEXUS_JC4310_D_GUARD_ERROR_DEG &&
                s_p_torque_nm * s_d_torque_nm < 0.0f) {
                s_d_opposition_ratio = s_reversal_lead_active != 0U
                    ? KNEXUS_JC4310_REVERSAL_D_OPPOSE_P_MAX_RATIO
                    : KNEXUS_JC4310_D_OPPOSE_P_MAX_RATIO;
                float d_opposition_limit = fabsf(s_p_torque_nm) *
                    s_d_opposition_ratio;
                s_d_torque_nm = jc_clampf(
                    s_d_torque_nm,
                    -d_opposition_limit, d_opposition_limit);
            }
            /* 动态标定结果：正力矩使roll增大，控制极性为+1。 */
            s_static_torque_nm = 0.0f;
            if (s_startup_active == 0U &&
                s_limit_recovery_direction == 0 &&
                fabsf(s_target_roll_rate_dps) <=
                    KNEXUS_JC4310_STATIC_MAX_TARGET_RATE_DPS &&
                fabsf(s_roll_rate_dps) <
                    KNEXUS_JC4310_STATIC_MAX_ROLL_RATE_DPS) {
                float static_span =
                    KNEXUS_JC4310_STATIC_FULL_ERROR_DEG -
                    KNEXUS_JC4310_STATIC_ENTER_ERROR_DEG;
                float static_blend = static_span > 0.01f
                    ? jc_clampf((error_abs_deg -
                            KNEXUS_JC4310_STATIC_ENTER_ERROR_DEG) /
                            static_span,
                        0.0f, 1.0f)
                    : 0.0f;
                float rate_blend = 1.0f - jc_clampf(
                    fabsf(s_roll_rate_dps) /
                        KNEXUS_JC4310_STATIC_MAX_ROLL_RATE_DPS,
                    0.0f, 1.0f);
                if (s_angle_error_deg > 0.0f) {
                    s_static_torque_nm =
                        KNEXUS_JC4310_STATIC_TORQUE_NM *
                        static_blend * rate_blend;
                } else if (s_angle_error_deg < 0.0f) {
                    s_static_torque_nm =
                        -KNEXUS_JC4310_STATIC_TORQUE_NM *
                        static_blend * rate_blend;
                }
            }
            s_raw_torque_nm = KNEXUS_JC4310_ROLL_TORQUE_SIGN *
                (s_p_torque_nm + s_i_torque_nm + s_d_torque_nm +
                 s_static_torque_nm);
            requested_torque_nm = jc_clampf(
                s_raw_torque_nm, -fabsf(knexus_jc4310_max_torque_nm),
                fabsf(knexus_jc4310_max_torque_nm));
#endif
        } else {
            s_jc_target_roll_deg = 0.0f;
            s_rate_target_dps = 0.0f;
            s_rate_error_dps = 0.0f;
            s_rate_torque_limit_nm = 0.0f;
            s_i_torque_nm = 0.0f;
            s_motion_target_filtered_deg = 0.0f;
            s_reversal_candidate_since_ms = 0U;
            s_reversal_latched_ms = 0U;
            s_reversal_lead_active = 0U;
            s_limit_recovery_direction = 0;
            s_static_torque_nm = 0.0f;
            jc_reverse_reset();
        }

        /*
         * 换向分成两段：预测误差已经越过零点时，不等待慢速PID合力自然换号，
         * 先给新方向一个短脉冲克服惯性；随后限制回拉力矩与斜率，让高速回拉
         * 在接近目标前自然变软。触发条件带确认时间，单帧噪声不会踢电机。
         */
        int8_t command_direction = jc_torque_direction(
            s_command_torque_nm,
            KNEXUS_JC4310_REVERSE_TRIGGER_COMMAND_NM);
        int8_t requested_direction = jc_torque_direction(
            requested_torque_nm,
            KNEXUS_JC4310_REVERSE_TRIGGER_REQUEST_NM);
        int8_t predicted_direction = jc_torque_direction(
            KNEXUS_JC4310_ROLL_TORQUE_SIGN * s_predicted_error_deg,
            KNEXUS_JC4310_ANGLE_DEADBAND_DEG);
        int8_t reverse_trigger_direction = 0;
        if (s_startup_active == 0U &&
            s_limit_recovery_direction == 0 &&
            s_reversal_lead_active != 0U &&
                   predicted_direction != 0 && command_direction != 0 &&
                   predicted_direction != command_direction) {
            reverse_trigger_direction = predicted_direction;
        } else if (s_startup_active == 0U &&
                   s_limit_recovery_direction == 0 &&
                   requested_direction != 0 && command_direction != 0 &&
                   requested_direction != command_direction) {
            reverse_trigger_direction = requested_direction;
        }

        if (KNEXUS_JC4310_REVERSE_KICK_ENABLE == 0U ||
            s_limit_recovery_direction != 0 ||
            s_startup_active != 0U || limit_recovery_just_started) {
            /* The cascaded rate loop performs closed-loop reversal itself.
             * Limit recovery is also velocity controlled; neither case may
             * be bypassed by an open-loop torque kick. */
            jc_reverse_reset();
        } else if (s_reverse_phase == JC_REVERSE_IDLE) {
            bool reverse_rearmed = s_reverse_last_kick_ms == 0U ||
                (context->now_ms - s_reverse_last_kick_ms) >=
                    KNEXUS_JC4310_REVERSE_REARM_MS;
            bool predicted_reverse_ready =
                reverse_rearmed &&
                s_reversal_lead_active != 0U &&
                predicted_direction == reverse_trigger_direction &&
                reverse_trigger_direction != 0;
            if (predicted_reverse_ready) {
                /* 预测反向自身已经连续确认30ms，不再额外等待。 */
                s_reverse_phase = JC_REVERSE_KICK;
                s_reverse_direction = reverse_trigger_direction;
                s_reverse_phase_tick_ms = context->now_ms;
                s_reverse_last_kick_ms = context->now_ms;
                s_limit_post_settle_active = 0U;
                s_reverse_kick_count++;
            } else if (reverse_rearmed &&
                       reverse_trigger_direction != 0) {
                s_reverse_phase = JC_REVERSE_CONFIRM;
                s_reverse_candidate_direction = reverse_trigger_direction;
                s_reverse_phase_tick_ms = context->now_ms;
            }
        } else if (s_reverse_phase == JC_REVERSE_CONFIRM) {
            bool reverse_candidate_valid =
                (s_reversal_lead_active != 0U &&
                 predicted_direction == s_reverse_candidate_direction) ||
                requested_direction == s_reverse_candidate_direction;
            if (!reverse_candidate_valid) {
                jc_reverse_reset();
            } else if (context->now_ms - s_reverse_phase_tick_ms >=
                       KNEXUS_JC4310_REVERSE_TRIGGER_CONFIRM_MS) {
                s_reverse_phase = JC_REVERSE_KICK;
                s_reverse_direction = s_reverse_candidate_direction;
                s_reverse_phase_tick_ms = context->now_ms;
                s_reverse_last_kick_ms = context->now_ms;
                s_limit_post_settle_active = 0U;
                s_reverse_kick_count++;
            }
        } else if (s_reverse_phase == JC_REVERSE_KICK) {
            if (context->now_ms - s_reverse_phase_tick_ms >=
                KNEXUS_JC4310_REVERSE_KICK_MS) {
                s_reverse_phase = JC_REVERSE_SETTLE;
                s_reverse_phase_tick_ms = context->now_ms;
            }
        } else {
            uint32_t settle_ms = s_limit_post_settle_active != 0U
                ? KNEXUS_JC4310_LIMIT_POST_SETTLE_MS
                : KNEXUS_JC4310_REVERSE_SETTLE_MS;
            if (context->now_ms - s_reverse_phase_tick_ms >= settle_ms) {
                jc_reverse_reset();
            }
        }

        float torque_slew_nmps = knexus_jc4310_torque_slew_nmps;
        s_reverse_applied_torque_nm = 0.0f;
        if (s_reverse_phase == JC_REVERSE_KICK &&
            s_reverse_direction != 0) {
            bool near_limit = s_limit_recovery_direction != 0 ||
                s_position_deg <=
                    KNEXUS_JC4310_MOTOR_MIN_POSITION_DEG +
                    KNEXUS_JC4310_LIMIT_BRAKE_ZONE_DEG ||
                s_position_deg >=
                    KNEXUS_JC4310_MOTOR_MAX_POSITION_DEG -
                    KNEXUS_JC4310_LIMIT_BRAKE_ZONE_DEG;
            float kick_torque = near_limit
                ? KNEXUS_JC4310_REVERSE_LIMIT_KICK_TORQUE_NM
                : KNEXUS_JC4310_REVERSE_KICK_TORQUE_NM;
            s_reverse_applied_torque_nm =
                (float)s_reverse_direction * fabsf(kick_torque);
            requested_torque_nm = s_reverse_applied_torque_nm;
            torque_slew_nmps = KNEXUS_JC4310_REVERSE_KICK_SLEW_NMPS;
        } else if (s_reverse_phase == JC_REVERSE_SETTLE) {
            float settle_max_torque = s_limit_post_settle_active != 0U
                ? KNEXUS_JC4310_LIMIT_POST_SETTLE_MAX_TORQUE_NM
                : KNEXUS_JC4310_REVERSE_SETTLE_MAX_TORQUE_NM;
            requested_torque_nm = jc_clampf(
                requested_torque_nm,
                -settle_max_torque, settle_max_torque);
            torque_slew_nmps = s_limit_post_settle_active != 0U
                ? KNEXUS_JC4310_LIMIT_POST_SETTLE_SLEW_NMPS
                : KNEXUS_JC4310_REVERSE_SETTLE_SLEW_NMPS;
        }
        if (s_startup_active != 0U &&
            s_limit_recovery_direction == 0) {
            requested_torque_nm = jc_clampf(
                requested_torque_nm,
                -KNEXUS_JC4310_STARTUP_MAX_TORQUE_NM,
                KNEXUS_JC4310_STARTUP_MAX_TORQUE_NM);
            torque_slew_nmps =
                KNEXUS_JC4310_STARTUP_TORQUE_SLEW_NMPS;
        }
        requested_torque_nm = jc_slew(
            s_command_torque_nm, requested_torque_nm,
            torque_slew_nmps, dt_s);
        s_limit_blocked = 0U;
        s_limit_torque_scale = 1.0f;
        s_limit_hold_torque_nm = 0.0f;
        s_limit_target_velocity_dps = 0.0f;
        s_limit_predicted_position_deg = s_position_deg;
        if (control_active &&
            s_position_valid != 0U) {
            float brake_zone =
                fabsf(KNEXUS_JC4310_LIMIT_BRAKE_ZONE_DEG);
            float lookahead_s = fabsf(
                KNEXUS_JC4310_LIMIT_LOOKAHEAD_S);
            s_limit_predicted_position_deg = s_position_deg +
                s_position_velocity_dps * lookahead_s;
            float upper_guard_position = fminf(
                s_position_deg, s_limit_predicted_position_deg);
            float lower_guard_position = fmaxf(
                s_position_deg, s_limit_predicted_position_deg);
            float upper_distance = upper_guard_position -
                KNEXUS_JC4310_MOTOR_MIN_POSITION_DEG;
            float lower_distance =
                KNEXUS_JC4310_MOTOR_MAX_POSITION_DEG -
                lower_guard_position;

            if (brake_zone > 0.01f && requested_torque_nm < 0.0f) {
                s_limit_torque_scale = jc_clampf(
                    upper_distance / brake_zone, 0.0f, 1.0f);
            } else if (brake_zone > 0.01f &&
                       requested_torque_nm > 0.0f) {
                s_limit_torque_scale = jc_clampf(
                    lower_distance / brake_zone, 0.0f, 1.0f);
            }
            requested_torque_nm *= s_limit_torque_scale;
            if (s_limit_torque_scale < 0.999f) {
                s_limit_blocked = 1U;
            }

            /* Guard the predicted position, not only the last encoder
             * sample.  At the measured 300-700 deg/s, a position-only guard
             * is already one complete travel late. */
            float hard_guard = fabsf(
                KNEXUS_JC4310_LIMIT_HARD_GUARD_MARGIN_DEG);
            if (upper_distance <= hard_guard &&
                requested_torque_nm < 0.0f) {
                requested_torque_nm = 0.0f;
                s_limit_blocked = 1U;
            } else if (lower_distance <= hard_guard &&
                       requested_torque_nm > 0.0f) {
                requested_torque_nm = 0.0f;
                s_limit_blocked = 1U;
            }
            float limit_kd =
                KNEXUS_JC4310_LIMIT_VELOCITY_KD_NM_PER_DPS;
            if (brake_zone > 0.01f &&
                upper_distance < brake_zone &&
                s_position_velocity_dps < 0.0f) {
                float brake_torque =
                    -limit_kd * s_position_velocity_dps;
                requested_torque_nm = fmaxf(
                    requested_torque_nm, brake_torque);
                s_limit_blocked = 1U;
            } else if (brake_zone > 0.01f &&
                       lower_distance < brake_zone &&
                       s_position_velocity_dps > 0.0f) {
                float brake_torque =
                    -limit_kd * s_position_velocity_dps;
                requested_torque_nm = fminf(
                    requested_torque_nm, brake_torque);
                s_limit_blocked = 1U;
            }

            float max_control_torque =
                fabsf(knexus_jc4310_max_torque_nm);
            float min_return_torque = fabsf(
                KNEXUS_JC4310_LIMIT_RETURN_MIN_TORQUE_NM);
            float max_return_torque = fminf(
                max_control_torque,
                fabsf(KNEXUS_JC4310_LIMIT_RETURN_MAX_TORQUE_NM));
            if (max_return_torque < min_return_torque) {
                max_return_torque = min_return_torque;
            }
            float max_brake_torque = fminf(
                max_control_torque,
                fabsf(
                    KNEXUS_JC4310_LIMIT_RECOVERY_MAX_BRAKE_TORQUE_NM));
            float return_margin = fabsf(
                KNEXUS_JC4310_LIMIT_RETURN_MARGIN_DEG);
            if (s_limit_recovery_direction != 0) {
                float target_position =
                    s_limit_recovery_direction > 0
                        ? KNEXUS_JC4310_MOTOR_MIN_POSITION_DEG +
                            return_margin
                        : KNEXUS_JC4310_MOTOR_MAX_POSITION_DEG -
                            return_margin;
                float position_error =
                    target_position - s_position_deg;
                s_limit_target_velocity_dps = jc_clampf(
                    KNEXUS_JC4310_LIMIT_RECOVERY_POS_TO_VEL_GAIN *
                        position_error,
                    -KNEXUS_JC4310_LIMIT_RECOVERY_MAX_VEL_DPS,
                    KNEXUS_JC4310_LIMIT_RECOVERY_MAX_VEL_DPS);
                s_limit_hold_torque_nm =
                    KNEXUS_JC4310_LIMIT_RECOVERY_VEL_KP_NM_PER_DPS *
                    (s_limit_target_velocity_dps -
                     s_position_velocity_dps);
                if (fabsf(position_error) >
                        KNEXUS_JC4310_LIMIT_RECOVERY_POSITION_TOL_DEG &&
                    fabsf(s_limit_hold_torque_nm) < min_return_torque &&
                    fabsf(s_limit_target_velocity_dps) > 1.0f &&
                    s_limit_hold_torque_nm *
                        s_limit_target_velocity_dps > 0.0f) {
                    s_limit_hold_torque_nm =
                        s_limit_target_velocity_dps > 0.0f
                            ? min_return_torque : -min_return_torque;
                }
                if (s_limit_recovery_direction > 0) {
                    float positive_limit =
                        s_position_velocity_dps < 0.0f
                            ? max_brake_torque : max_return_torque;
                    s_limit_hold_torque_nm = jc_clampf(
                        s_limit_hold_torque_nm,
                        -max_brake_torque, positive_limit);
                } else {
                    float negative_limit =
                        s_position_velocity_dps > 0.0f
                            ? -max_brake_torque : -max_return_torque;
                    s_limit_hold_torque_nm = jc_clampf(
                        s_limit_hold_torque_nm,
                        negative_limit, max_brake_torque);
                }

                /* While physically beyond the measured hard stop, never
                 * command farther outward.  As soon as it re-enters the
                 * mechanical range, opposite torque is allowed so the
                 * velocity loop can brake inside the software reserve. */
                if (s_limit_recovery_direction > 0 &&
                    s_position_deg <
                        KNEXUS_JC4310_MECHANICAL_MIN_POSITION_DEG) {
                    s_limit_hold_torque_nm = fmaxf(
                        0.0f, s_limit_hold_torque_nm);
                } else if (s_limit_recovery_direction < 0 &&
                           s_position_deg >
                               KNEXUS_JC4310_MECHANICAL_MAX_POSITION_DEG) {
                    s_limit_hold_torque_nm = fminf(
                        0.0f, s_limit_hold_torque_nm);
                }
                requested_torque_nm = s_limit_hold_torque_nm;
                s_limit_blocked = 1U;
            } else {
                requested_torque_nm = jc_clampf(
                    requested_torque_nm, -max_control_torque,
                    max_control_torque);
            }
        }
        /* Final actual-angle guard.  Target clamping alone cannot constrain
         * inertia: the 2026-07-31T23:44 sample had a ±2° target but measured
         * -7.57/+12.27°.  Predict 40ms ahead and, once the small-angle
         * envelope is threatened, let an independent roll PD own torque. */
        s_roll_guard_active = 0U;
        s_roll_guard_torque_nm = 0.0f;
        if (control_active && s_tilt_valid != 0U) {
            float predicted_roll_deg = s_roll_deg +
                KNEXUS_JC4310_ROLL_GUARD_LOOKAHEAD_S *
                    s_roll_rate_dps;
            int8_t guard_direction = 0;
            if (s_roll_deg > KNEXUS_JC4310_ROLL_GUARD_MAX_DEG ||
                predicted_roll_deg >
                    KNEXUS_JC4310_ROLL_GUARD_MAX_DEG) {
                guard_direction = -1;
            } else if (s_roll_deg <
                           KNEXUS_JC4310_ROLL_GUARD_MIN_DEG ||
                       predicted_roll_deg <
                           KNEXUS_JC4310_ROLL_GUARD_MIN_DEG) {
                guard_direction = 1;
            }
            if (guard_direction != 0) {
                float guard_target_deg =
                    guard_direction < 0
                        ? KNEXUS_JC4310_ROLL_GUARD_RETURN_MAX_DEG
                        : KNEXUS_JC4310_ROLL_GUARD_RETURN_MIN_DEG;
                s_roll_guard_torque_nm =
                    KNEXUS_JC4310_ROLL_GUARD_KP_NM_PER_DEG *
                        (guard_target_deg - s_roll_deg) -
                    KNEXUS_JC4310_ROLL_GUARD_KD_NM_PER_DPS *
                        s_roll_rate_dps;
                s_roll_guard_torque_nm = jc_clampf(
                    s_roll_guard_torque_nm,
                    -fabsf(KNEXUS_JC4310_LIMIT_MAX_TORQUE_NM),
                    fabsf(KNEXUS_JC4310_LIMIT_MAX_TORQUE_NM));
                requested_torque_nm = s_roll_guard_torque_nm;
                s_roll_guard_active = 1U;
                s_limit_blocked = 1U;
                s_i_torque_nm = 0.0f;
                jc_reverse_reset();
            }
        }
        /* The roll guard must never defeat the final mechanical one-way
         * prohibition.  If both protections demand opposite directions,
         * command zero instead of pushing either boundary farther outward. */
        if (s_position_valid != 0U) {
            if (s_position_deg <=
                    KNEXUS_JC4310_MECHANICAL_MIN_POSITION_DEG &&
                requested_torque_nm < 0.0f) {
                requested_torque_nm = 0.0f;
            } else if (s_position_deg >=
                           KNEXUS_JC4310_MECHANICAL_MAX_POSITION_DEG &&
                       requested_torque_nm > 0.0f) {
                requested_torque_nm = 0.0f;
            }
        }
        /* 限位速度阻尼也必须服从电机峰值力矩；旧代码在这里可能叠加到
         * -1.24 N*m，超过JC4310允许的0.50 N*m。 */
        float hardware_max_torque = fminf(
            fabsf(knexus_jc4310_max_torque_nm),
            fabsf(KNEXUS_JC4310_LIMIT_MAX_TORQUE_NM));
        requested_torque_nm = jc_clampf(
            requested_torque_nm,
            -hardware_max_torque, hardware_max_torque);
        if (s_tilt_valid == 0U || s_position_valid == 0U) {
            requested_torque_nm = 0.0f;
        }

        s_command_direction = requested_torque_nm > 0.0001f ? 1 :
                              requested_torque_nm < -0.0001f ? -1 : 0;
        s_last_command_direction = s_command_direction;
        float torque = requested_torque_nm;
        if (!jc_polarity_send_torque(torque)) {
            s_ready = 0U;
            s_compensation_active = 0U;
            s_external_control_active = 0U;
            s_external_motion_comp_enabled = 0U;
            s_limit_recovery_direction = 0;
            jc_reverse_reset();
            jc_polarity_enter(JC_POLARITY_FAULT);
        }
        jc_polarity_poll_feedback();
        if (s_fail_code == JC_POLARITY_FAIL_DRIVER_ERROR) {
            (void)jc_polarity_send_torque(0.0f);
            s_ready = 0U;
            s_compensation_active = 0U;
            s_external_control_active = 0U;
            s_external_motion_comp_enabled = 0U;
            s_command_direction = 0;
            s_limit_recovery_direction = 0;
            jc_reverse_reset();
            jc_polarity_enter(JC_POLARITY_FAULT);
            knx_beep_beep(500U);
        }
    } else {
        /* 故障时也不进入Idle：只要CAN仍可发送，就持续覆盖为0力矩。 */
        (void)jc_polarity_send_torque(0.0f);
        s_ready = 0U;
        s_compensation_active = 0U;
        s_external_control_active = 0U;
        s_external_motion_comp_enabled = 0U;
        s_command_direction = 0;
        s_limit_recovery_direction = 0;
        jc_reverse_reset();
    }

    s_position_delta_deg = s_position_deg - s_initial_position_deg;
    s_roll_delta_deg = s_roll_deg - s_initial_roll_deg;
    knx_led_set(KNX_LED_1, s_command_direction != 0);
    knx_led_set(KNX_LED_2, s_ready != 0U && s_tilt_valid != 0U);
#endif
}

void knexus_mode_jc4310_link_center_on_intersection(
    const knx_intersection_result_t *result)
{
    (void)result;
}

void knexus_mode_jc4310_link_center_on_peer_message(
    const knx_comm_message_t *message)
{
    (void)message;
}

static void jc_polarity_debug(Octolinker_Instance_t *octo)
{
    if (octo == NULL) return;
    const uint16_t id = KNEXUS_JC4310_LINK_OCTO_BASE_ID;
#if defined(KNX_PLATFORM_STM32)
    JC_Stats stats;
    JC_GetStats(&stats);
#else
    const struct {
        uint32_t tx_count, rx_count, timeout_count, error_count;
    } stats = {0};
#endif
    (void)Octolinker_SendU8(octo, id + 0U, (uint8_t)s_state);
    (void)Octolinker_SendU8(octo, id + 1U, s_ready);
    (void)Octolinker_SendU8(octo, id + 2U, s_platform_supported);
    (void)Octolinker_SendU32(octo, id + 3U, (uint32_t)s_fail_code);
    (void)Octolinker_SendU8(octo, id + 4U,
                            s_compensation_active);
    (void)Octolinker_SendI32(octo, id + 5U, (int32_t)s_last_status);
    (void)Octolinker_SendI32(octo, id + 6U,
                             (int32_t)s_subscribe_status);
    (void)Octolinker_SendI32(octo, id + 7U,
                             (int32_t)s_driver_init_status);
    (void)Octolinker_SendU8(octo, id + 8U,
                            KNEXUS_JC4310_LINK_MOTOR_ID);
    (void)Octolinker_SendF32(octo, id + 9U, s_voltage_v);
    (void)Octolinker_SendF32(octo, id + 10U, s_bus_current_a);
    (void)Octolinker_SendF32(octo, id + 11U, s_speed_rpm);
    (void)Octolinker_SendF32(octo, id + 12U, s_position_deg);
    (void)Octolinker_SendF32(octo, id + 13U, s_drv_temp_c);
    (void)Octolinker_SendF32(octo, id + 14U, s_mot_temp_c);
    (void)Octolinker_SendU32(octo, id + 15U, s_error_code);
    (void)Octolinker_SendF32(octo, id + 16U, s_command_torque_nm);
    (void)Octolinker_SendF32(octo, id + 17U,
                             knexus_jc4310_max_torque_nm);
    (void)Octolinker_SendU8(octo, id + 18U,
                            (uint8_t)knx_key_is_pressed(KNX_KEY_0));
    (void)Octolinker_SendU8(octo, id + 19U,
                            (uint8_t)knx_key_is_pressed(KNX_KEY_1));
    (void)Octolinker_SendU32(octo, id + 20U, stats.tx_count);
    (void)Octolinker_SendU32(octo, id + 21U, stats.rx_count);
    (void)Octolinker_SendU32(octo, id + 22U, stats.timeout_count);
    (void)Octolinker_SendU32(octo, id + 23U,
                             s_zero_command_count);
    (void)Octolinker_SendU32(octo, id + 24U,
                             s_nonzero_command_count);
    (void)Octolinker_SendF32(octo, id + 25U,
                             s_target_roll_deg);
    (void)Octolinker_SendF32(octo, id + 26U,
                             s_angle_error_deg);
    (void)Octolinker_SendF32(octo, id + 27U,
                             s_p_torque_nm);
    (void)Octolinker_SendF32(octo, id + 28U, s_roll_deg);
    (void)Octolinker_SendF32(octo, id + 29U, s_d_torque_nm);
    (void)Octolinker_SendF32(octo, id + 30U, s_roll_rate_dps);
    (void)Octolinker_SendU8(octo, id + 31U, s_tilt_valid);
    (void)Octolinker_SendU8(octo, id + 32U,
                            s_upper_limit_active);
    (void)Octolinker_SendU8(octo, id + 33U,
                            s_lower_limit_active);
    (void)Octolinker_SendU8(octo, id + 34U, s_limit_blocked);
    (void)Octolinker_SendI32(octo, id + 35U,
                             (int32_t)s_dm_init_status);
    (void)Octolinker_SendI32(octo, id + 36U,
                             (int32_t)s_dm_sub_active_status);
    (void)Octolinker_SendI32(octo, id + 37U,
                             (int32_t)s_dm_sub_reply_status);
    (void)Octolinker_SendU32(octo, id + 38U, s_tilt_age_ms);
    (void)Octolinker_SendU32(octo, id + 39U,
                             s_feedback_poll_fail_count);
}

static void jc_motion_comp_debug(Octolinker_Instance_t *octo)
{
    if (octo == NULL) return;
    const uint16_t id = KNEXUS_JC4310_COMP_OCTO_BASE_ID;
    knx_ball_motion_comp_state_t comp;
    knx_ball_motion_comp_snapshot(&comp);

    (void)Octolinker_SendF32(octo, id + 0U, comp.raw_accel_mps2[0]);
    (void)Octolinker_SendF32(octo, id + 1U, comp.raw_accel_mps2[1]);
    (void)Octolinker_SendF32(octo, id + 2U, comp.raw_accel_mps2[2]);
    (void)Octolinker_SendF32(octo, id + 3U,
                             comp.selected_specific_force_mps2);
    (void)Octolinker_SendF32(octo, id + 4U,
                             comp.filtered_specific_force_mps2);
    (void)Octolinker_SendF32(octo, id + 5U,
                             comp.chassis_accel_mps2);
    (void)Octolinker_SendF32(octo, id + 6U,
                             comp.feedforward_roll_deg);
    (void)Octolinker_SendF32(octo, id + 7U, s_target_roll_deg);
    (void)Octolinker_SendF32(octo, id + 8U,
                             s_target_roll_rate_dps);
    (void)Octolinker_SendF32(octo, id + 9U, s_roll_deg);
    (void)Octolinker_SendF32(octo, id + 10U, s_roll_rate_dps);
    (void)Octolinker_SendF32(octo, id + 11U, s_angle_error_deg);
    (void)Octolinker_SendF32(octo, id + 12U, s_p_torque_nm);
    (void)Octolinker_SendF32(octo, id + 13U, s_d_torque_nm);
    (void)Octolinker_SendF32(octo, id + 14U, s_raw_torque_nm);
    (void)Octolinker_SendF32(octo, id + 15U, s_command_torque_nm);
    (void)Octolinker_SendU8(octo, id + 16U, s_compensation_active);
    (void)Octolinker_SendU8(octo, id + 17U,
                            (uint8_t)comp.bias_ready);
    (void)Octolinker_SendU8(octo, id + 18U,
                            (uint8_t)comp.mapping_ready);
    (void)Octolinker_SendU8(octo, id + 19U, s_chassis_imu_valid);
    (void)Octolinker_SendU8(octo, id + 20U, s_upper_limit_active);
    (void)Octolinker_SendU8(octo, id + 21U, s_lower_limit_active);
    (void)Octolinker_SendU8(octo, id + 22U, s_limit_blocked);
    (void)Octolinker_SendF32(octo, id + 23U,
                             comp.predicted_ball_accel_mps2);
    (void)Octolinker_SendF32(octo, id + 24U,
                             comp.predicted_ball_velocity_mps);
    (void)Octolinker_SendF32(octo, id + 25U,
                             comp.predicted_ball_position_m);
    (void)Octolinker_SendF32(octo, id + 26U, comp.bias_mps2[0]);
    (void)Octolinker_SendF32(octo, id + 27U, comp.bias_mps2[1]);
    (void)Octolinker_SendF32(octo, id + 28U, comp.bias_mps2[2]);
    (void)Octolinker_SendU32(octo, id + 29U,
                             comp.bias_sample_count);
    (void)Octolinker_SendF32(octo, id + 30U,
                             knexus_jc4310_angle_kp);
    (void)Octolinker_SendF32(octo, id + 31U,
                             knexus_jc4310_angle_kd);
    (void)Octolinker_SendF32(octo, id + 32U,
                             knexus_jc4310_max_torque_nm);
    (void)Octolinker_SendF32(octo, id + 33U,
                             knexus_jc4310_torque_slew_nmps);
    (void)Octolinker_SendU8(octo, id + 34U, s_position_valid);
    (void)Octolinker_SendU32(octo, id + 35U, s_position_age_ms);
    (void)Octolinker_SendF32(octo, id + 36U, s_limit_torque_scale);
    (void)Octolinker_SendF32(
        octo, id + 37U, KNEXUS_DM_IMU_ROLL_ZERO_OFFSET_DEG);
    (void)Octolinker_SendF32(octo, id + 38U,
                             s_position_velocity_dps);
    (void)Octolinker_SendF32(octo, id + 39U,
                             s_limit_hold_torque_nm);
    (void)Octolinker_SendF32(octo, id + 40U,
                             knexus_jc4310_angle_ki);
    (void)Octolinker_SendF32(octo, id + 41U, s_i_torque_nm);
    (void)Octolinker_SendF32(octo, id + 42U,
                             knexus_jc4310_angle_i_limit_nm);
    (void)Octolinker_SendF32(octo, id + 43U,
                             knexus_ball_accel_lpf_hz);
    (void)Octolinker_SendF32(
        octo, id + 44U, KNEXUS_JC4310_TARGET_ROLL_SLEW_DPS);
    (void)Octolinker_SendF32(
        octo, id + 45U, KNEXUS_JC4310_LIMIT_BRAKE_ZONE_DEG);
    (void)Octolinker_SendF32(octo, id + 46U,
                             s_predicted_error_deg);
    (void)Octolinker_SendU8(octo, id + 47U,
                            s_reversal_lead_active);
    (void)Octolinker_SendF32(octo, id + 48U,
                             s_d_opposition_ratio);
    (void)Octolinker_SendF32(octo, id + 49U,
                             s_effective_kp_gain);
    (void)Octolinker_SendF32(octo, id + 50U,
                             s_effective_kd_gain);
    (void)Octolinker_SendF32(octo, id + 51U,
                             s_motion_target_filtered_deg);
    (void)Octolinker_SendU32(octo, id + 52U,
                             s_reversal_enter_count);
    (void)Octolinker_SendU32(octo, id + 53U,
                             s_reversal_candidate_since_ms);
    (void)Octolinker_SendU8(octo, id + 54U,
                            (uint8_t)s_reverse_phase);
    (void)Octolinker_SendI32(octo, id + 55U,
                             (int32_t)s_reverse_direction);
    (void)Octolinker_SendI32(octo, id + 56U,
                             (int32_t)s_limit_recovery_direction);
    (void)Octolinker_SendF32(octo, id + 57U,
                             s_reverse_applied_torque_nm);
    (void)Octolinker_SendU32(octo, id + 58U,
                             s_reverse_kick_count);
    (void)Octolinker_SendU32(octo, id + 59U,
                             s_limit_recovery_count);
    (void)Octolinker_SendU8(octo, id + 60U,
                            s_limit_post_settle_active);
    (void)Octolinker_SendF32(octo, id + 61U,
                             KNEXUS_JC4310_MOTOR_MIN_POSITION_DEG);
    (void)Octolinker_SendF32(octo, id + 62U,
                             KNEXUS_JC4310_MOTOR_MAX_POSITION_DEG);
    (void)Octolinker_SendU8(octo, id + 63U,
                            s_startup_active);
    (void)Octolinker_SendF32(octo, id + 64U,
                             s_static_torque_nm);
    (void)Octolinker_SendF32(octo, id + 65U,
                             s_limit_predicted_position_deg);
    (void)Octolinker_SendF32(octo, id + 66U,
                             s_limit_target_velocity_dps);
    (void)Octolinker_SendF32(octo, id + 67U,
                             KNEXUS_JC4310_MECHANICAL_MIN_POSITION_DEG);
    (void)Octolinker_SendF32(octo, id + 68U,
                             KNEXUS_JC4310_MECHANICAL_MAX_POSITION_DEG);
    (void)Octolinker_SendU8(octo, id + 69U,
                            s_external_motion_comp_enabled);
    (void)Octolinker_SendU32(octo, id + 70U,
                             s_tilt_future_timestamp_clamp_count);
    (void)Octolinker_SendF32(octo, id + 77U,
                             s_rate_target_dps);
    (void)Octolinker_SendF32(octo, id + 78U,
                             s_rate_error_dps);
    (void)Octolinker_SendF32(octo, id + 79U,
                             s_rate_torque_limit_nm);
}

void knexus_mode_jc4310_link_center_debug_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    (void)base_id;
    jc_polarity_debug(octo);
}

void knexus_mode_jc4310_link_center_debug_control_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    (void)base_id;
    jc_motion_comp_debug(octo);
}

void knexus_jc4310_debug_compact_control_octo(
    Octolinker_Instance_t *octo)
{
    if (octo == NULL) return;
    const uint16_t id = KNEXUS_JC4310_COMP_OCTO_BASE_ID;
    knx_ball_motion_comp_state_t comp;
    knx_ball_motion_comp_snapshot(&comp);

    /* Keep existing IDs so old OctoLink workspaces/CSV analysis still work. */
    (void)Octolinker_SendF32(octo, id + 5U,
                             comp.chassis_accel_mps2);
    (void)Octolinker_SendF32(octo, id + 6U,
                             comp.feedforward_roll_deg);
    (void)Octolinker_SendF32(octo, id + 7U, s_target_roll_deg);
    (void)Octolinker_SendF32(octo, id + 9U, s_roll_deg);
    (void)Octolinker_SendF32(octo, id + 10U, s_roll_rate_dps);
    (void)Octolinker_SendF32(octo, id + 11U, s_angle_error_deg);
    (void)Octolinker_SendF32(octo, id + 15U, s_command_torque_nm);
    (void)Octolinker_SendU8(octo, id + 20U, s_upper_limit_active);
    (void)Octolinker_SendU8(octo, id + 21U, s_lower_limit_active);
    (void)Octolinker_SendU8(octo, id + 22U, s_limit_blocked);
    (void)Octolinker_SendU8(octo, id + 54U,
                            (uint8_t)s_reverse_phase);
    (void)Octolinker_SendI32(octo, id + 56U,
                             (int32_t)s_limit_recovery_direction);
    (void)Octolinker_SendF32(octo, id + 71U, s_position_deg);
    (void)Octolinker_SendF32(octo, id + 72U,
                             s_position_velocity_dps);
    (void)Octolinker_SendU8(octo, id + 73U,
                            s_roll_guard_active);
    (void)Octolinker_SendF32(octo, id + 74U,
                             s_roll_guard_torque_nm);
    (void)Octolinker_SendF32(octo, id + 75U,
                             s_roll_sensor_deg);
    (void)Octolinker_SendF32(octo, id + 76U,
                             KNEXUS_JC4310_ROD_ZERO_ROLL_DEG);
    (void)Octolinker_SendF32(octo, id + 77U,
                             s_rate_target_dps);
    (void)Octolinker_SendF32(octo, id + 78U,
                             s_rate_error_dps);
    (void)Octolinker_SendF32(octo, id + 79U,
                             s_rate_torque_limit_nm);
    (void)Octolinker_SendU8(octo, id + 69U,
                            s_external_motion_comp_enabled);
}

#endif
