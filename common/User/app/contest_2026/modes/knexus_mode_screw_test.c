#include "knexus_mode_backend.h"
#include "knexus_config.h"
#include "knx26_app.h"
#include "knx_board.h"
#include "knx_can_bus.h"
#include "knx_dji_motor_ctrl.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_port.h"
#include "knx_time.h"

#if defined(KNX_PLATFORM_STM32)
#include "dm_imu_l1.h"
#endif

/*
 * 丝杆测试模式（STM32专用硬件组合）
 *   KEY0按住：M3508 +1000 rpm，丝杆下沉，roll增大。
 *   KEY1按住：M3508 -1000 rpm，丝杆上升，roll减小。
 *   roll >= +7 deg：禁止继续下沉，仍允许上升。
 *   roll <= -12 deg：禁止继续上升，仍允许下沉。
 *   两键同时按：停止，避免方向指令冲突。
 *   FDCAN2：C620/M3508；FDCAN1：DM-IMU-L1。
 */

float knexus_screw_test_speed_rpm = KNEXUS_SCREW_TEST_SPEED_RPM;

static int8_t s_direction;
static bool s_motor_enabled;
static uint8_t s_platform_supported;
static knx_status_t s_dji_init_status;
static knx_status_t s_dm_sub_active_status;
static knx_status_t s_dm_sub_reply_status;
static knx_status_t s_dm_init_status;
static float s_tilt_roll_deg;
static uint8_t s_tilt_valid;
static uint8_t s_upper_limit_active;
static uint8_t s_lower_limit_active;
static int8_t s_requested_direction;
static int8_t s_blocked_direction;
static uint32_t s_limit_block_count;

#if defined(KNX_PLATFORM_STM32)
static void screw_dm_rx_handler(uint32_t std_id, const uint8_t *data,
                                uint8_t len, void *user)
{
    (void)user;
    DM_IMU_L1_UpdateData(std_id, data, len);
}

static void screw_dm_init(void)
{
    knx_can_t *can = knx_board_get_can_bus(
        KNEXUS_SCREW_TEST_DM_CAN_BUS_INDEX);
    if (can == NULL) {
        s_dm_sub_active_status = KNX_NOT_READY;
        s_dm_sub_reply_status = KNX_NOT_READY;
        s_dm_init_status = KNX_NOT_READY;
        return;
    }

    DM_IMU_L1_AttachCAN(can, DM_IMU_L1_DEFAULT_CAN_ID);
    s_dm_sub_active_status = knx_can_bus_subscribe(
        KNX_CAN_BUS_1,
        DM_IMU_L1_ACTIVE_REPORT_CAN_ID,
        DM_IMU_L1_ACTIVE_REPORT_CAN_ID,
        screw_dm_rx_handler, NULL);
    s_dm_sub_reply_status = knx_can_bus_subscribe(
        KNX_CAN_BUS_1,
        DM_IMU_L1_DEFAULT_CAN_ID,
        DM_IMU_L1_DEFAULT_CAN_ID,
        screw_dm_rx_handler, NULL);

    if (s_dm_sub_active_status != KNX_OK ||
        s_dm_sub_reply_status != KNX_OK) {
        s_dm_init_status = KNX_ERROR;
        return;
    }

    /* ExternalRx keeps the contest CAN router as the sole RX callback owner.
     * Its required sensor boot wait is close enough to the STM32 watchdog
     * window that we explicitly feed before and after the blocking init. */
    knx_port_watchdog_refresh();
    s_dm_init_status = DM_IMU_L1_InitExternalRx();
    knx_port_watchdog_refresh();
    if (s_dm_init_status == KNX_OK &&
        KNEXUS_SCREW_TEST_DM_INTERVAL_MS != 1U) {
        s_dm_init_status = DM_IMU_L1_SetActiveIntervalMs(
            KNEXUS_SCREW_TEST_DM_INTERVAL_MS);
    }
}
#endif

static void screw_stop(void)
{
    if (s_motor_enabled) {
        knx_dji_motor_ctrl_stop();
    } else {
        knx_dji_motor_ctrl_set_target(0.0f);
    }
    s_motor_enabled = false;
    s_direction = 0;
    knx_led_set(KNX_LED_1, false);
}

void knexus_mode_screw_test_init(void)
{
    s_direction = 0;
    s_motor_enabled = false;
    s_dji_init_status = KNX_NOT_READY;
    s_dm_sub_active_status = KNX_NOT_READY;
    s_dm_sub_reply_status = KNX_NOT_READY;
    s_dm_init_status = KNX_NOT_READY;
    s_tilt_roll_deg = 0.0f;
    s_tilt_valid = 0U;
    s_upper_limit_active = 0U;
    s_lower_limit_active = 0U;
    s_requested_direction = 0;
    s_blocked_direction = 0;
    s_limit_block_count = 0U;

#if defined(KNX_PLATFORM_STM32)
    s_platform_supported = 1U;
    s_dji_init_status = knx_dji_motor_ctrl_init();
    knx_dji_motor_ctrl_stop();
    screw_dm_init();
#else
    /* MSPM0只有一路MCAN，不能同时隔离承载C620与外置IMU。 */
    s_platform_supported = 0U;
#endif

    knx_led_set(KNX_LED_1, false);
    knx_led_set(KNX_LED_2, false);
}

void knexus_mode_screw_test_update(const struct knx26_context *raw_context)
{
    const knx26_context_t *context = (const knx26_context_t *)raw_context;
    if (context == NULL) return;

    bool key0 = knx_key_is_pressed(KNX_KEY_0);
    bool key1 = knx_key_is_pressed(KNX_KEY_1);
    int8_t requested_direction = 0;
    if (key0 && !key1) {
        requested_direction = KNEXUS_SCREW_MOTOR_DOWN_DIRECTION;
    }
    if (key1 && !key0) {
        requested_direction = KNEXUS_SCREW_MOTOR_UP_DIRECTION;
    }
    s_requested_direction = requested_direction;

#if defined(KNX_PLATFORM_STM32)
    dm_imu_l1_data_t imu;
    DM_IMU_L1_Snapshot(&imu);
    uint32_t tilt_age_ms = context->now_ms - imu.timestamp;
    s_tilt_roll_deg = imu.roll;
    s_tilt_valid =
        (DM_IMU_L1_IsDataReady() != 0U &&
         DM_IMU_L1_IsTiltReady() != 0U &&
         tilt_age_ms <= KNEXUS_SCREW_TILT_MAX_AGE_MS)
            ? 1U
            : 0U;
    s_upper_limit_active =
        (s_tilt_valid &&
         s_tilt_roll_deg <= KNEXUS_SCREW_TILT_UPPER_LIMIT_DEG)
            ? 1U
            : 0U;
    s_lower_limit_active =
        (s_tilt_valid &&
         s_tilt_roll_deg >= KNEXUS_SCREW_TILT_LOWER_LIMIT_DEG)
            ? 1U
            : 0U;
#else
    s_tilt_valid = 0U;
    s_upper_limit_active = 0U;
    s_lower_limit_active = 0U;
#endif

    int8_t blocked_direction = 0;
    if ((requested_direction == KNEXUS_SCREW_MOTOR_UP_DIRECTION &&
         s_upper_limit_active) ||
        (requested_direction == KNEXUS_SCREW_MOTOR_DOWN_DIRECTION &&
         s_lower_limit_active)) {
        blocked_direction = requested_direction;
    }
    if (blocked_direction != 0 && s_blocked_direction == 0) {
        s_limit_block_count++;
    }
    s_blocked_direction = blocked_direction;

    if (!s_platform_supported || s_dji_init_status != KNX_OK ||
        !s_tilt_valid || requested_direction == 0 ||
        blocked_direction != 0) {
        screw_stop();
    } else {
        if (!s_motor_enabled) {
            knx_dji_motor_ctrl_enable();
            s_motor_enabled = true;
        }
        s_direction = requested_direction;
        knx_dji_motor_ctrl_set_target(
            (float)s_direction * knexus_screw_test_speed_rpm);
        knx_led_set(KNX_LED_1, true);
    }

#if defined(KNX_PLATFORM_STM32)
    knx_led_set(KNX_LED_2, DM_IMU_L1_IsDataReady() != 0U);
#else
    knx_led_set(KNX_LED_2, false);
#endif
}

void knexus_mode_screw_test_on_intersection(
    const knx_intersection_result_t *result)
{
    (void)result;
}

void knexus_mode_screw_test_on_peer_message(
    const knx_comm_message_t *message)
{
    (void)message;
}

void knexus_mode_screw_test_debug_control_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    (void)base_id;
    if (octo == NULL) return;
    const uint16_t id = KNEXUS_SCREW_TEST_OCTO_BASE_ID;
    knx_dji_motor_state_t dji;
    knx_dji_motor_ctrl_snapshot(&dji);

    (void)Octolinker_SendI32(octo, id + 0U, s_direction);
    (void)Octolinker_SendF32(octo, id + 1U, dji.target_rpm);
    (void)Octolinker_SendF32(octo, id + 2U, dji.target_used_rpm);
    (void)Octolinker_SendF32(octo, id + 3U, dji.speed_rpm);
    (void)Octolinker_SendF32(octo, id + 4U, dji.current);
    (void)Octolinker_SendI32(octo, id + 5U, dji.pid_output);
#if defined(KNX_PLATFORM_STM32)
    dm_imu_l1_data_t imu;
    DM_IMU_L1_Snapshot(&imu);
    (void)Octolinker_SendF32(octo, id + 6U, imu.roll);
    (void)Octolinker_SendF32(octo, id + 7U, imu.pitch);
    (void)Octolinker_SendF32(octo, id + 8U, imu.yaw);
    (void)Octolinker_SendF32(octo, id + 9U, imu.yaw_total);
    (void)Octolinker_SendF32Array(octo, id + 10U, imu.gyro, 3U);
    (void)Octolinker_SendF32(octo, id + 13U, imu.roll_total);
    (void)Octolinker_SendF32(octo, id + 14U, imu.pitch_total);
#endif
    (void)Octolinker_SendI32(octo, id + 15U,
                             (int32_t)s_requested_direction);
    /* Total motor angle makes the screw/rod transmission identifiable from
     * one forward and one reverse pulse, without integrating sampled RPM. */
    (void)Octolinker_SendF32(octo, id + 11U, dji.angle_deg);
    (void)Octolinker_SendU8(octo, id + 12U, dji.temperature_c);
}

void knexus_mode_screw_test_debug_octo(Octolinker_Instance_t *octo,
                                       uint16_t base_id)
{
    (void)base_id;
    if (octo == NULL) return;
    const uint16_t id = KNEXUS_SCREW_TEST_OCTO_BASE_ID;
    knx_dji_motor_state_t dji;
    knx_dji_motor_ctrl_snapshot(&dji);

    (void)Octolinker_SendU8(octo, id + 16U, s_platform_supported);
    (void)Octolinker_SendU8(octo, id + 17U,
                            (uint8_t)knx_key_is_pressed(KNX_KEY_0));
    (void)Octolinker_SendU8(octo, id + 18U,
                            (uint8_t)knx_key_is_pressed(KNX_KEY_1));
    (void)Octolinker_SendU8(octo, id + 19U,
                            (uint8_t)s_motor_enabled);
    (void)Octolinker_SendI32(octo, id + 20U,
                             (int32_t)s_dji_init_status);
    (void)Octolinker_SendU32(octo, id + 21U, dji.rx_count);
    (void)Octolinker_SendU32(octo, id + 22U, dji.tx_ok);
    (void)Octolinker_SendU32(octo, id + 23U, dji.tx_error);
    (void)Octolinker_SendU32(octo, id + 24U, dji.tx_busy);
    (void)Octolinker_SendI32(octo, id + 25U,
                             (int32_t)dji.last_tx_status);
    (void)Octolinker_SendU32(octo, id + 26U, dji.feedback_age_ms);
    (void)Octolinker_SendI32(octo, id + 27U,
                             (int32_t)s_dm_sub_active_status);
    (void)Octolinker_SendI32(octo, id + 28U,
                             (int32_t)s_dm_sub_reply_status);
    (void)Octolinker_SendI32(octo, id + 29U,
                             (int32_t)s_dm_init_status);
#if defined(KNX_PLATFORM_STM32)
    DM_IMU_L1_DebugOcto(octo, (uint16_t)(id + 32U));
#endif
    (void)Octolinker_SendU8(octo, id + 54U, s_tilt_valid);
    (void)Octolinker_SendI32(octo, id + 55U,
                             (int32_t)s_blocked_direction);
    (void)Octolinker_SendU32(octo, id + 56U,
                             s_limit_block_count);
    (void)Octolinker_SendU8(octo, id + 57U,
                            s_upper_limit_active);
    (void)Octolinker_SendU8(octo, id + 58U,
                            s_lower_limit_active);
    (void)Octolinker_SendF32(octo, id + 59U,
                             KNEXUS_SCREW_TILT_UPPER_LIMIT_DEG);
    (void)Octolinker_SendF32(octo, id + 60U,
                             KNEXUS_SCREW_TILT_LOWER_LIMIT_DEG);
}
