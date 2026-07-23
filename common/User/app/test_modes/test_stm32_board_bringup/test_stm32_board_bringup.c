#include "test_stm32_board_bringup.h"

#include "bmi088.h"
#include "drv8701e.h"
#include "fdcan.h"
#include "knx_board.h"
#include "knx_can.h"
#include "knx_imu.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_time.h"
#include "knx_beep.h"
#include "octolinker.h"

#define BOARD_TEST_OCTO_BASE             600U
#define BOARD_TEST_CAN_PERIOD_MS         100U
#define BOARD_TEST_TELEMETRY_PERIOD_MS   100U
#define BOARD_TEST_MOTOR_SEGMENT_MS     2000U

/* These remain debugger-tunable, but the motor test always requires a KEY0
 * long press before it can apply output. Keep the default deliberately low. */
float test_board_bringup_motor_enable = 1.0f;
float test_board_bringup_motor_max_duty = 0.30f;
float test_board_bringup_motor_left_duty = 0.0f;
float test_board_bringup_motor_right_duty = 0.0f;
float test_board_bringup_motor_armed = 0.0f;

uint32_t test_board_bringup_can1_tx_ok = 0U;
uint32_t test_board_bringup_can1_tx_fail = 0U;
uint32_t test_board_bringup_can2_tx_ok = 0U;
uint32_t test_board_bringup_can2_tx_fail = 0U;
int32_t test_board_bringup_can1_last_status = KNX_NOT_READY;
int32_t test_board_bringup_can2_last_status = KNX_NOT_READY;

static Octolinker_Instance_t *s_octo;
static knx_can_t s_fdcan1 = {0};
static uint32_t s_can_tick;
static uint32_t s_telemetry_tick;
static uint32_t s_motor_start_tick;
static uint32_t s_can_sequence;
static uint8_t s_key0_long_seen;

static float board_test_clamp_duty(float duty)
{
    if (duty < 0.0f) {
        return 0.0f;
    }
    if (duty > 0.30f) {
        return 0.30f;
    }
    return duty;
}

static void board_test_stop_motors(void)
{
    test_board_bringup_motor_left_duty = 0.0f;
    test_board_bringup_motor_right_duty = 0.0f;
    DRV8701E_StopAll();
}

static void board_test_update_motor(uint32_t now)
{
    if (knx_key_just_pressed(KNX_KEY_1)) {
        test_board_bringup_motor_armed = 0.0f;
        board_test_stop_motors();
        knx_beep_beep(400U);
    }

    if (!knx_key_is_pressed(KNX_KEY_0)) {
        s_key0_long_seen = 0U;
    } else if (knx_key_is_long_press(KNX_KEY_0) && s_key0_long_seen == 0U) {
        s_key0_long_seen = 1U;
        test_board_bringup_motor_armed =
            (test_board_bringup_motor_armed > 0.0f) ? 0.0f : 1.0f;
        s_motor_start_tick = now;
        board_test_stop_motors();
        knx_beep_beep((test_board_bringup_motor_armed > 0.0f) ? 160U : 320U);
    }

    if (test_board_bringup_motor_enable <= 0.0f ||
        test_board_bringup_motor_armed <= 0.0f) {
        board_test_stop_motors();
        return;
    }

    const float low = board_test_clamp_duty(test_board_bringup_motor_max_duty / 3.0f);
    const float medium = board_test_clamp_duty(test_board_bringup_motor_max_duty * (2.0f / 3.0f));
    const float high = board_test_clamp_duty(test_board_bringup_motor_max_duty);
    uint32_t phase = ((now - s_motor_start_tick) / BOARD_TEST_MOTOR_SEGMENT_MS) % 8U;

    /* Forward-only, left/right separately, then both. This avoids the abrupt
     * direction reversals that are unsuitable for an unverified new board. */
    switch (phase) {
    case 0U: test_board_bringup_motor_left_duty = low;    test_board_bringup_motor_right_duty = 0.0f; break;
    case 1U: test_board_bringup_motor_left_duty = medium; test_board_bringup_motor_right_duty = 0.0f; break;
    case 2U: test_board_bringup_motor_left_duty = high;   test_board_bringup_motor_right_duty = 0.0f; break;
    case 3U: test_board_bringup_motor_left_duty = 0.0f;   test_board_bringup_motor_right_duty = low; break;
    case 4U: test_board_bringup_motor_left_duty = 0.0f;   test_board_bringup_motor_right_duty = medium; break;
    case 5U: test_board_bringup_motor_left_duty = 0.0f;   test_board_bringup_motor_right_duty = high; break;
    case 6U: test_board_bringup_motor_left_duty = medium; test_board_bringup_motor_right_duty = medium; break;
    default: test_board_bringup_motor_left_duty = high;   test_board_bringup_motor_right_duty = high; break;
    }

    DRV8701E_SetDutyNorm(&motor_left, test_board_bringup_motor_left_duty);
    DRV8701E_SetDutyNorm(&motor_right, test_board_bringup_motor_right_duty);
}

static void board_test_send_can(uint32_t now)
{
    if (now - s_can_tick < BOARD_TEST_CAN_PERIOD_MS) {
        return;
    }
    s_can_tick = now;

    uint8_t frame[8] = {
        0x4BU, 0x4EU, 0x58U, 0x42U,
        (uint8_t)(s_can_sequence & 0xFFU),
        (uint8_t)((s_can_sequence >> 8) & 0xFFU),
        (uint8_t)(test_board_bringup_motor_armed > 0.0f),
        0U,
    };
    s_can_sequence++;

    /* FDCAN1: CAN ID 0x701; FDCAN2: CAN ID 0x702. Both are classic CAN,
     * 1 Mbps, eight bytes. The final byte distinguishes the interface. */
    frame[7] = 1U;
    test_board_bringup_can1_last_status =
        knx_can_transmit_std(&s_fdcan1, 0x701U, frame, sizeof(frame), 1U);
    if (test_board_bringup_can1_last_status == KNX_OK) {
        test_board_bringup_can1_tx_ok++;
    } else {
        test_board_bringup_can1_tx_fail++;
    }

    frame[7] = 2U;
    test_board_bringup_can2_last_status =
        knx_can_transmit_std(knx_board_get_jc_can(), 0x702U, frame, sizeof(frame), 1U);
    if (test_board_bringup_can2_last_status == KNX_OK) {
        test_board_bringup_can2_tx_ok++;
    } else {
        test_board_bringup_can2_tx_fail++;
    }
}

static void board_test_send_telemetry(uint32_t now)
{
    if (s_octo == NULL || now - s_telemetry_tick < BOARD_TEST_TELEMETRY_PERIOD_MS) {
        return;
    }
    s_telemetry_tick = now;

    /* Existing BMI088 OctoLink diagnostics: roll/pitch/yaw/temperature/
     * temperature-target/heater-duty at IDs 20..25. */
    BMI088_DebugOcto(s_octo);

    (void)Octolinker_SendU32(s_octo, BOARD_TEST_OCTO_BASE + 0U, now);
    (void)Octolinker_SendF32(s_octo, BOARD_TEST_OCTO_BASE + 1U, test_board_bringup_motor_armed);
    (void)Octolinker_SendF32(s_octo, BOARD_TEST_OCTO_BASE + 2U, test_board_bringup_motor_left_duty);
    (void)Octolinker_SendF32(s_octo, BOARD_TEST_OCTO_BASE + 3U, test_board_bringup_motor_right_duty);
    (void)Octolinker_SendF32(s_octo, BOARD_TEST_OCTO_BASE + 4U, test_board_bringup_motor_max_duty);
    (void)Octolinker_SendU8(s_octo, BOARD_TEST_OCTO_BASE + 5U, knx_key_is_pressed(KNX_KEY_0));
    (void)Octolinker_SendU8(s_octo, BOARD_TEST_OCTO_BASE + 6U, knx_key_is_pressed(KNX_KEY_1));
    (void)Octolinker_SendU8(s_octo, BOARD_TEST_OCTO_BASE + 7U, knx_led_get(KNX_LED_1));
    (void)Octolinker_SendU8(s_octo, BOARD_TEST_OCTO_BASE + 8U, knx_led_get(KNX_LED_2));
    (void)Octolinker_SendI32(s_octo, BOARD_TEST_OCTO_BASE + 9U, test_board_bringup_can1_last_status);
    (void)Octolinker_SendU32(s_octo, BOARD_TEST_OCTO_BASE + 10U, test_board_bringup_can1_tx_ok);
    (void)Octolinker_SendU32(s_octo, BOARD_TEST_OCTO_BASE + 11U, test_board_bringup_can1_tx_fail);
    (void)Octolinker_SendI32(s_octo, BOARD_TEST_OCTO_BASE + 12U, test_board_bringup_can2_last_status);
    (void)Octolinker_SendU32(s_octo, BOARD_TEST_OCTO_BASE + 13U, test_board_bringup_can2_tx_ok);
    (void)Octolinker_SendU32(s_octo, BOARD_TEST_OCTO_BASE + 14U, test_board_bringup_can2_tx_fail);
    (void)Octolinker_SendU8(s_octo, BOARD_TEST_OCTO_BASE + 15U, imu_data.accel_ok);
    (void)Octolinker_SendU8(s_octo, BOARD_TEST_OCTO_BASE + 16U, imu_data.gyro_ok);
    (void)Octolinker_SendU32(s_octo, BOARD_TEST_OCTO_BASE + 17U, imu_data.frame_count);
    (void)Octolinker_SendU32(s_octo, BOARD_TEST_OCTO_BASE + 18U, s_can_sequence);
    (void)Octolinker_SendF32(s_octo, BOARD_TEST_OCTO_BASE + 19U, imu_data.temperature);
}

void test_stm32_board_bringup_init(void *octo)
{
    s_octo = (Octolinker_Instance_t *)octo;
    s_fdcan1.handle = &hfdcan1;
    test_board_bringup_can1_last_status = knx_can_start(&s_fdcan1);
    test_board_bringup_can2_last_status = KNX_OK; /* board init started FDCAN2 */

    s_can_tick = knx_millis();
    s_telemetry_tick = s_can_tick;
    s_motor_start_tick = s_can_tick;
    s_can_sequence = 0U;
    s_key0_long_seen = 0U;
    test_board_bringup_motor_armed = 0.0f;
    board_test_stop_motors();
    knx_led_off(KNX_LED_1);
    knx_led_off(KNX_LED_2);
    knx_beep_beep(80U);
}

void test_stm32_board_bringup_loop(void)
{
    uint32_t now = knx_millis();

    /* BMI088_Read() executes the heater temperature controller after a valid
     * temperature read, so this exercises both the IMU and the heater PWM. */
    (void)knx_imu_update();
    board_test_update_motor(now);
    board_test_send_can(now);

    knx_led_set(KNX_LED_1, test_board_bringup_motor_armed > 0.0f);
    knx_led_set(KNX_LED_2, ((now / 250U) & 1U) != 0U);
    board_test_send_telemetry(now);
}
