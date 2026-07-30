#include "knexus_mode_backend.h"
#include "knexus_mode_common.h"
#include "knexus_config.h"
#include "knx26_app.h"
#include "bmi088.h"
#include "encoder.h"
#include "knx_beep.h"
#include "knx_board.h"
#include "knx_can_bus.h"
#include "knx_chassis.h"
#include "knx_dji_motor_ctrl.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_motor.h"
#include "knx_params.h"
#include "knx_safety.h"
#include "knx_sys.h"
#include "knx_time.h"

/*
 * STM32/MSPM0 共用的全板验收模式。
 * 使用现有板级抽象，不在这里出现任何 MCU 引脚或 HAL/DriverLib 调用。
 */

static bool s_motor_armed;
static bool s_key0_long_seen;
static uint8_t s_motor_phase;
static uint32_t s_motor_start_tick;
static uint32_t s_can_tick;
static uint32_t s_octo_tick;
static uint32_t s_can_sequence;
static uint32_t s_can_tx_ok[KNX_CAN_BUS_COUNT];
static uint32_t s_can_tx_fail[KNX_CAN_BUS_COUNT];
static knx_status_t s_can_last_status[KNX_CAN_BUS_COUNT];
static uint32_t s_peer_message_count;
static uint32_t s_intersection_count;
static float s_target_left_duty;
static float s_target_right_duty;
static float s_dji_target_rpm;
static knx_intersection_result_t s_last_intersection;

static void board_test_stop_motors(void)
{
    s_motor_armed = false;
    s_target_left_duty = 0.0f;
    s_target_right_duty = 0.0f;
    (void)knx_chassis_disable();
    knx_dji_motor_ctrl_stop();
    knx_led_set(KNX_LED_1, false);
}

static void board_test_set_phase_targets(uint8_t phase)
{
    float high = KNEXUS_BOARD_TEST_MOTOR_MAX_DUTY;
    float medium = high * (2.0f / 3.0f);
    float low = high / 3.0f;

    switch (phase) {
    case 0U: s_target_left_duty = low;    s_target_right_duty = 0.0f;   break;
    case 1U: s_target_left_duty = medium; s_target_right_duty = 0.0f;   break;
    case 2U: s_target_left_duty = high;   s_target_right_duty = 0.0f;   break;
    case 3U: s_target_left_duty = 0.0f;   s_target_right_duty = low;    break;
    case 4U: s_target_left_duty = 0.0f;   s_target_right_duty = medium; break;
    case 5U: s_target_left_duty = 0.0f;   s_target_right_duty = high;   break;
    case 6U: s_target_left_duty = medium; s_target_right_duty = medium; break;
    default: s_target_left_duty = high;   s_target_right_duty = high;   break;
    }
}

static void board_test_update_motor(uint32_t now)
{
    if (knx_key_just_pressed(KNX_KEY_1)) {
        board_test_stop_motors();
        knx_beep_beep(400U);
        return;
    }

    if (!knx_key_is_pressed(KNX_KEY_0)) {
        s_key0_long_seen = false;
    } else if (knx_key_is_long_press(KNX_KEY_0) && !s_key0_long_seen) {
        s_key0_long_seen = true;
        if (s_motor_armed) {
            board_test_stop_motors();
            knx_beep_beep(320U);
        } else {
            knexus_mode_apply_motor_config(KNEXUS_BOARD_TEST_MOTOR_MAX_DUTY);
            s_motor_start_tick = now;
            s_motor_phase = 0U;
            s_motor_armed = true;
            (void)knx_chassis_enable();
            (void)knx_sys_set_mode(KNX_RUN_MODE_DUTY);
            (void)knx_motor_set_duty_target(0.0f, 0.0f);
            knx_dji_motor_ctrl_enable();
            knx_led_set(KNX_LED_1, true);
            knx_beep_beep(160U);
        }
    }

    if (!s_motor_armed) return;

    s_motor_phase = (uint8_t)(((now - s_motor_start_tick) /
                               KNEXUS_BOARD_TEST_MOTOR_SEGMENT_MS) % 8U);
    board_test_set_phase_targets(s_motor_phase);

    /*
     * Hardware acceptance deliberately uses open-loop duty instead of the
     * speed PID.  Encoder polarity or a disconnected encoder must never make
     * the wheel that is supposed to be idle run at the duty limit.
     */
    if (knx_sys_set_mode(KNX_RUN_MODE_DUTY) != KNX_OK ||
        knx_motor_set_duty_target(s_target_left_duty,
                                  s_target_right_duty) != KNX_OK) {
        board_test_stop_motors();
        knx_beep_beep(600U);
    }
}

static void board_test_update_dji_motor(uint32_t now)
{
    if (!s_motor_armed) {
        s_dji_target_rpm = 0.0f;
        knx_dji_motor_ctrl_set_target(0.0f);
        return;
    }

    uint32_t phase = ((now - s_motor_start_tick) /
                      KNEXUS_DJI_TEST_REVERSE_PERIOD_MS) & 1U;
    s_dji_target_rpm = (phase == 0U) ? KNEXUS_DJI_TEST_SPEED_RPM
                                     : -KNEXUS_DJI_TEST_SPEED_RPM;
    knx_dji_motor_ctrl_set_target(s_dji_target_rpm);
}

static void board_test_send_can(uint32_t now)
{
    if (now - s_can_tick < KNEXUS_BOARD_TEST_CAN_PERIOD_MS) return;
    s_can_tick = now;

    uint8_t frame[8] = {
        'K', 'N', 'X', 'B',
        (uint8_t)(s_can_sequence & 0xFFU),
        (uint8_t)((s_can_sequence >> 8) & 0xFFU),
        (uint8_t)s_motor_armed,
        0U,
    };
    s_can_sequence++;

    frame[7] = 1U;
    s_can_last_status[KNX_CAN_BUS_1] = knx_can_bus_send(
        KNX_CAN_BUS_1, KNEXUS_BOARD_TEST_CAN1_ID, frame, sizeof(frame));
    if (s_can_last_status[KNX_CAN_BUS_1] == KNX_OK) {
        s_can_tx_ok[KNX_CAN_BUS_1]++;
    } else {
        s_can_tx_fail[KNX_CAN_BUS_1]++;
    }

    frame[7] = 2U;
    s_can_last_status[KNX_CAN_BUS_2] = knx_can_bus_send(
        KNX_CAN_BUS_2, KNEXUS_BOARD_TEST_CAN2_ID, frame, sizeof(frame));
    if (s_can_last_status[KNX_CAN_BUS_2] == KNX_OK) {
        s_can_tx_ok[KNX_CAN_BUS_2]++;
    } else {
        s_can_tx_fail[KNX_CAN_BUS_2]++;
    }
}

static void send_board_test_octo(const knx26_context_t *context)
{
#if KNEXUS_BOARD_TEST_OCTO_ENABLE
    if (context->now_ms - s_octo_tick < KNEXUS_BOARD_TEST_OCTO_PERIOD_MS) return;
    s_octo_tick = context->now_ms;
    Octolinker_Instance_t *octo = knx_board_get_octolinker();
    if (octo == NULL) return;
    knx_dji_motor_state_t dji;
    knx_dji_motor_ctrl_snapshot(&dji);

    uint16_t id = KNEXUS_BOARD_TEST_OCTO_BASE_ID;
    BMI088_DebugOcto(octo);
    (void)Octolinker_SendU32(octo, id + 0U, context->now_ms);
    (void)Octolinker_SendU8(octo, id + 1U, (uint8_t)s_motor_armed);
    (void)Octolinker_SendU8(octo, id + 2U, s_motor_phase);
    (void)Octolinker_SendF32(octo, id + 3U, s_target_left_duty);
    (void)Octolinker_SendF32(octo, id + 4U, s_target_right_duty);
    (void)Octolinker_SendU8(octo, id + 5U,
                            (uint8_t)knx_key_is_pressed(KNX_KEY_0));
    (void)Octolinker_SendU8(octo, id + 6U,
                            (uint8_t)knx_key_is_pressed(KNX_KEY_1));
    (void)Octolinker_SendU8(octo, id + 7U,
                            (uint8_t)knx_led_get(KNX_LED_1));
    (void)Octolinker_SendU8(octo, id + 8U,
                            (uint8_t)knx_led_get(KNX_LED_2));
    (void)Octolinker_SendI32(octo, id + 9U,
                             (int32_t)s_can_last_status[KNX_CAN_BUS_1]);
    (void)Octolinker_SendU32(octo, id + 10U, s_can_tx_ok[KNX_CAN_BUS_1]);
    (void)Octolinker_SendU32(octo, id + 11U, s_can_tx_fail[KNX_CAN_BUS_1]);
    (void)Octolinker_SendI32(octo, id + 12U,
                             (int32_t)s_can_last_status[KNX_CAN_BUS_2]);
    (void)Octolinker_SendU32(octo, id + 13U, s_can_tx_ok[KNX_CAN_BUS_2]);
    (void)Octolinker_SendU32(octo, id + 14U, s_can_tx_fail[KNX_CAN_BUS_2]);
    (void)Octolinker_SendU8(octo, id + 15U, imu_data.accel_ok);
    (void)Octolinker_SendU8(octo, id + 16U, imu_data.gyro_ok);
    (void)Octolinker_SendU32(octo, id + 17U, imu_data.frame_count);
    (void)Octolinker_SendU32(octo, id + 18U, s_can_sequence);
    (void)Octolinker_SendF32(octo, id + 19U, imu_data.temperature);
    (void)Octolinker_SendF32(octo, id + 20U, bmi088_heater_duty);
    (void)Octolinker_SendF32(octo, id + 21U, bmi088_temp_target_c);
    (void)Octolinker_SendF32(octo, id + 22U,
                             context->chassis.left_motor.current_speed);
    (void)Octolinker_SendF32(octo, id + 23U,
                             context->chassis.right_motor.current_speed);
    (void)Octolinker_SendF32(octo, id + 24U,
                             context->chassis.left_motor.duty);
    (void)Octolinker_SendF32(octo, id + 25U,
                             context->chassis.right_motor.duty);
    (void)Octolinker_SendF32(octo, id + 26U,
                             context->chassis.left_motor.current_filtered);
    (void)Octolinker_SendF32(octo, id + 27U,
                             context->chassis.right_motor.current_filtered);
    (void)Octolinker_SendF32(octo, id + 28U, encoder_left.position_m);
    (void)Octolinker_SendF32(octo, id + 29U, encoder_right.position_m);
    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        (void)Octolinker_SendU16(octo, (uint16_t)(id + 30U + i),
                                 context->track.sensor.raw[i]);
        (void)Octolinker_SendF32(octo, (uint16_t)(id + 38U + i),
                                 context->track.sensor.normalized[i]);
    }
    (void)Octolinker_SendF32(octo, id + 46U,
                             context->track.sensor.line_error);
    (void)Octolinker_SendI32(octo, id + 47U,
                             (int32_t)context->track.last_status);
    (void)Octolinker_SendU32(octo, id + 48U, context->track.update_count);
    (void)Octolinker_SendU8(octo, id + 49U,
                            (uint8_t)knx_safety_get_level());
    (void)Octolinker_SendU32(octo, id + 50U, s_peer_message_count);
    (void)Octolinker_SendU32(octo, id + 51U, s_intersection_count);
    (void)Octolinker_SendU8(octo, id + 52U,
                            (uint8_t)s_last_intersection.type);
    (void)Octolinker_SendI32(octo, id + 53U,
                             (int32_t)dji.init_status);
    (void)Octolinker_SendF32(octo, id + 54U, dji.target_rpm);
    (void)Octolinker_SendF32(octo, id + 55U, dji.target_used_rpm);
    (void)Octolinker_SendF32(octo, id + 56U, dji.speed_rpm);
    (void)Octolinker_SendF32(octo, id + 57U, dji.current);
    (void)Octolinker_SendU8(octo, id + 58U, dji.temperature_c);
    (void)Octolinker_SendI32(octo, id + 59U, dji.pid_output);
    (void)Octolinker_SendU32(octo, id + 60U, dji.tx_ok);
    (void)Octolinker_SendU32(octo, id + 61U, dji.tx_error);
    (void)Octolinker_SendU32(octo, id + 62U, dji.rx_count);
    (void)Octolinker_SendI32(octo, id + 63U,
                             (int32_t)dji.last_tx_status);
    (void)Octolinker_SendU32(octo, id + 64U, dji.feedback_age_ms);
#if defined(KNX_PLATFORM_STM32)
    extern volatile uint32_t knx_stm32_reset_cause;
    extern volatile uint32_t knx26_fast_overrun_count;
    extern volatile uint32_t knx26_fast_max_elapsed_ms;
    (void)Octolinker_SendU32(octo, id + 65U, knx_stm32_reset_cause);
    (void)Octolinker_SendU32(octo, id + 67U, knx26_fast_overrun_count);
    (void)Octolinker_SendU32(octo, id + 68U, knx26_fast_max_elapsed_ms);
#else
    (void)Octolinker_SendU32(octo, id + 65U, 0U);
    (void)Octolinker_SendU32(octo, id + 67U, 0U);
    (void)Octolinker_SendU32(octo, id + 68U, 0U);
#endif
    (void)Octolinker_SendU32(octo, id + 66U, dji.tx_busy);
#else
    (void)context;
#endif
}

void knexus_mode_board_test_init(void)
{
    knexus_mode_apply_motor_config(KNEXUS_BOARD_TEST_MOTOR_MAX_DUTY);
    s_motor_armed = false;
    s_key0_long_seen = false;
    s_motor_phase = 0U;
    s_target_left_duty = 0.0f;
    s_target_right_duty = 0.0f;
    s_motor_start_tick = knx_millis();
    s_can_tick = s_motor_start_tick;
    s_octo_tick = s_motor_start_tick;
    s_can_sequence = 0U;
    s_peer_message_count = 0U;
    s_intersection_count = 0U;
    s_dji_target_rpm = 0.0f;
    for (uint8_t i = 0U; i < KNX_CAN_BUS_COUNT; ++i) {
        s_can_tx_ok[i] = 0U;
        s_can_tx_fail[i] = 0U;
        s_can_last_status[i] = KNX_NOT_READY;
    }
    s_last_intersection = (knx_intersection_result_t){0};
    (void)knx_dji_motor_ctrl_init();
    board_test_stop_motors();
    knx_led_set(KNX_LED_2, false);
    knx_beep_beep(80U);
}

void knexus_mode_board_test_update(const struct knx26_context *raw_context)
{
    const knx26_context_t *context = (const knx26_context_t *)raw_context;
    if (context == NULL) return;
    board_test_update_motor(context->now_ms);
    board_test_update_dji_motor(context->now_ms);
    board_test_send_can(context->now_ms);
    knx_led_set(KNX_LED_1, s_motor_armed);
    knx_led_set(KNX_LED_2, ((context->now_ms / 250U) & 1U) != 0U);
    send_board_test_octo(context);
}

void knexus_mode_board_test_on_intersection(
    const knx_intersection_result_t *result)
{
    if (result == NULL) return;
    s_last_intersection = *result;
    s_intersection_count++;
}

void knexus_mode_board_test_on_peer_message(const knx_comm_message_t *message)
{
    if (message != NULL) s_peer_message_count++;
}

void knexus_mode_board_test_debug_octo(Octolinker_Instance_t *octo,
                                       uint16_t base_id)
{
    (void)octo;
    (void)base_id;
}

void knexus_mode_board_test_debug_control_octo(Octolinker_Instance_t *octo,
                                               uint16_t base_id)
{
    (void)octo;
    (void)base_id;
}
