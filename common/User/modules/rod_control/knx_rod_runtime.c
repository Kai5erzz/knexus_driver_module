#include "knx_rod_runtime.h"
#include "knexus_config.h"
#include "knx_board.h"
#include "knx_can_bus.h"
#include "knx_dji_motor_ctrl.h"
#include "knx_led.h"
#include "knx_port.h"
#include "knx_rod_angle_ctrl.h"
#include "knx_time.h"

#if defined(KNX_PLATFORM_STM32)
#include "dm_imu_l1.h"
#endif

static knx_rod_runtime_state_t s_state;

#if defined(KNX_PLATFORM_STM32)
static void rod_dm_rx_handler(uint32_t std_id, const uint8_t *data,
                              uint8_t len, void *user)
{
    (void)user;
    DM_IMU_L1_UpdateData(std_id, data, len);
}

static void rod_dm_init(void)
{
    knx_can_t *can = knx_board_get_can_bus(
        KNEXUS_SCREW_TEST_DM_CAN_BUS_INDEX);
    if (can == NULL) {
        s_state.dm_sub_active_status = KNX_NOT_READY;
        s_state.dm_sub_reply_status = KNX_NOT_READY;
        s_state.dm_init_status = KNX_NOT_READY;
        return;
    }

    DM_IMU_L1_AttachCAN(can, DM_IMU_L1_DEFAULT_CAN_ID);
    s_state.dm_sub_active_status = knx_can_bus_subscribe(
        KNX_CAN_BUS_1, DM_IMU_L1_ACTIVE_REPORT_CAN_ID,
        DM_IMU_L1_ACTIVE_REPORT_CAN_ID, rod_dm_rx_handler, NULL);
    s_state.dm_sub_reply_status = knx_can_bus_subscribe(
        KNX_CAN_BUS_1, DM_IMU_L1_DEFAULT_CAN_ID,
        DM_IMU_L1_DEFAULT_CAN_ID, rod_dm_rx_handler, NULL);
    if (s_state.dm_sub_active_status != KNX_OK ||
        s_state.dm_sub_reply_status != KNX_OK) {
        s_state.dm_init_status = KNX_ERROR;
        return;
    }

    knx_port_watchdog_refresh();
    s_state.dm_init_status = DM_IMU_L1_InitExternalRx();
    knx_port_watchdog_refresh();
    if (s_state.dm_init_status == KNX_OK &&
        KNEXUS_SCREW_TEST_DM_INTERVAL_MS != 1U) {
        s_state.dm_init_status = DM_IMU_L1_SetActiveIntervalMs(
            KNEXUS_SCREW_TEST_DM_INTERVAL_MS);
    }
}
#endif

void knx_rod_runtime_stop(void)
{
    if (s_state.motor_enabled) {
        knx_dji_motor_ctrl_stop();
        s_state.stop_count++;
    } else {
        knx_dji_motor_ctrl_set_target(0.0f);
    }
    s_state.motor_enabled = false;
    knx_rod_angle_ctrl_reset();
    knx_led_set(KNX_LED_1, false);
}

knx_status_t knx_rod_runtime_init(void)
{
    s_state = (knx_rod_runtime_state_t){0};
    s_state.dji_init_status = KNX_NOT_READY;
    s_state.dm_init_status = KNX_NOT_READY;
    s_state.dm_sub_active_status = KNX_NOT_READY;
    s_state.dm_sub_reply_status = KNX_NOT_READY;
    knx_rod_angle_ctrl_reset();

#if defined(KNX_PLATFORM_STM32)
    s_state.platform_supported = true;
    s_state.dji_init_status = knx_dji_motor_ctrl_init();
    if (s_state.dji_init_status == KNX_OK) {
        s_state.dji_init_status = knx_dji_motor_ctrl_set_speed_pid(
            KNEXUS_ROD_DJI_SPEED_KP, KNEXUS_ROD_DJI_SPEED_KI,
            KNEXUS_ROD_DJI_SPEED_KD, KNEXUS_ROD_DJI_SPEED_I_LIMIT,
            KNEXUS_ROD_DJI_MAX_CURRENT_CMD);
    }
    if (s_state.dji_init_status == KNX_OK) {
        s_state.dji_init_status =
            knx_dji_motor_ctrl_set_current_feedforward(
                KNEXUS_ROD_DJI_CURRENT_FF_CMD,
                KNEXUS_ROD_DJI_CURRENT_FF_MIN_RPM);
    }
    knx_dji_motor_ctrl_stop();
    rod_dm_init();
#endif

    knx_led_set(KNX_LED_1, false);
    knx_led_set(KNX_LED_2, false);
    return (s_state.platform_supported &&
            s_state.dji_init_status == KNX_OK &&
            s_state.dm_init_status == KNX_OK)
               ? KNX_OK
               : KNX_NOT_READY;
}

void knx_rod_runtime_update(float target_deg, float feedforward_rpm,
                            bool requested_enable,
                            bool soft_limit_enable, float dt_s)
{
    knexus_rod_target_deg = target_deg;
    s_state.target_deg = target_deg;

    knx_dji_motor_state_t dji;
    knx_dji_motor_ctrl_snapshot(&dji);
    s_state.motor_feedback_valid =
        dji.rx_count != 0U &&
        dji.feedback_age_ms <= KNEXUS_DJI_FEEDBACK_TIMEOUT_MS;

#if defined(KNX_PLATFORM_STM32)
    dm_imu_l1_data_t imu;
    DM_IMU_L1_Snapshot(&imu);
    uint32_t now = knx_millis();
    uint32_t tilt_age_ms = now - imu.timestamp;
    s_state.roll_deg = imu.roll;
    s_state.roll_rate_dps = KNEXUS_DM_IMU_ROLL_GYRO_SIGN *
                            imu.gyro[0] * 57.2957795f;
    s_state.tilt_valid =
        DM_IMU_L1_IsDataReady() != 0U &&
        DM_IMU_L1_IsTiltReady() != 0U &&
        tilt_age_ms <= KNEXUS_SCREW_TILT_MAX_AGE_MS;
    s_state.upper_limit_active =
        s_state.tilt_valid &&
        s_state.roll_deg <= KNEXUS_SCREW_TILT_UPPER_LIMIT_DEG;
    s_state.lower_limit_active =
        s_state.tilt_valid &&
        s_state.roll_deg >= KNEXUS_SCREW_TILT_LOWER_LIMIT_DEG;
#else
    s_state.tilt_valid = false;
    s_state.upper_limit_active = false;
    s_state.lower_limit_active = false;
#endif

    s_state.control_ready =
        s_state.platform_supported && s_state.dji_init_status == KNX_OK &&
        s_state.dm_init_status == KNX_OK && s_state.tilt_valid &&
        s_state.motor_feedback_valid;
    if (!requested_enable || !s_state.control_ready) {
        s_state.blocked_direction = 0;
        knx_rod_runtime_stop();
        knx_led_set(KNX_LED_2, s_state.tilt_valid);
        return;
    }

    float rpm = knx_rod_angle_ctrl_update(
        target_deg, s_state.roll_deg, s_state.roll_rate_dps,
        knexus_rod_angle_kp, knexus_rod_angle_ki, knexus_rod_angle_kd,
        knexus_rod_integral_max_rpm, knexus_rod_integral_zone_deg,
        knexus_rod_integral_rate_zone_dps, knexus_rod_max_rpm,
        knexus_rod_rpm_slew_rpmps, knexus_rod_deadband_deg,
        knexus_rod_rate_deadband_dps, feedforward_rpm,
        KNEXUS_ROD_MOTOR_SIGN, dt_s);

    s_state.blocked_direction = 0;
    if (soft_limit_enable && rpm < 0.0f &&
        s_state.upper_limit_active) {
        s_state.blocked_direction = KNEXUS_SCREW_MOTOR_UP_DIRECTION;
    } else if (soft_limit_enable && rpm > 0.0f &&
               s_state.lower_limit_active) {
        s_state.blocked_direction = KNEXUS_SCREW_MOTOR_DOWN_DIRECTION;
    }
    if (s_state.blocked_direction != 0) {
        knx_rod_runtime_stop();
    } else {
        if (!s_state.motor_enabled) {
            knx_dji_motor_ctrl_enable();
            s_state.motor_enabled = true;
        }
        knx_dji_motor_ctrl_set_target(rpm);
        knx_led_set(KNX_LED_1, true);
    }
    knx_led_set(KNX_LED_2, true);
}

void knx_rod_runtime_snapshot(knx_rod_runtime_state_t *out)
{
    if (out != NULL) *out = s_state;
}
