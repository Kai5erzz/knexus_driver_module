#include "knexus_mode_backend.h"
#include "knexus_mode_common.h"
#include "knexus_config.h"
#include "knx26_app.h"
#include "knx_beep.h"
#include "knx_chassis.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_time.h"

typedef enum {
    TUNE_IDLE = 0,
    TUNE_POSITIVE,
    TUNE_PAUSE,
    TUNE_NEGATIVE,
    TUNE_DONE,
} tune_state_t;

float knexus_tune_linear_speed_mps = KNEXUS_TUNE_LINEAR_SPEED_MPS;
float knexus_tune_angular_speed_radps = KNEXUS_TUNE_ANGULAR_SPEED_RADPS;

static tune_state_t s_state;
static uint32_t s_state_tick;
static float s_target_linear;
static float s_target_angular;
static float s_actual_linear;
static float s_actual_angular;

static void tune_enter(tune_state_t state)
{
    s_state = state;
    s_state_tick = knx_millis();
}

static void tune_stop(tune_state_t next)
{
    (void)knx_chassis_disable();
    s_target_linear = 0.0f;
    s_target_angular = 0.0f;
    knx_led_set(KNX_LED_1, false);
    tune_enter(next);
}

static void tune_set_sign(float sign)
{
#if defined(KNEXUS_PID_TUNE_SPEED_STEP)
    s_target_linear = sign * knexus_tune_linear_speed_mps;
    s_target_angular = 0.0f;
#else
    s_target_linear = 0.0f;
    s_target_angular = sign * knexus_tune_angular_speed_radps;
#endif
    (void)knx_chassis_set_velocity(s_target_linear, s_target_angular);
}

void knexus_mode_pid_tune_init(void)
{
    knexus_mode_apply_motor_config(KNEXUS_TUNE_MAX_DUTY_DEFAULT);
    tune_stop(TUNE_IDLE);
    knx_led_set(KNX_LED_2, false);
}

void knexus_mode_pid_tune_update(const struct knx26_context *raw_context)
{
    const knx26_context_t *context = (const knx26_context_t *)raw_context;
    if (context == NULL) return;
    s_actual_linear = context->chassis.drive.measured_linear_mps;
    s_actual_angular = context->chassis.drive.measured_angular_radps;

    if (knx_key_just_pressed(KNX_KEY_1)) {
        tune_stop(TUNE_IDLE);
        knx_led_set(KNX_LED_2, false);
        knx_beep_beep(100U);
        return;
    }
    if (knx_key_just_pressed(KNX_KEY_0)) {
        knexus_mode_apply_motor_config(KNEXUS_TUNE_MAX_DUTY_DEFAULT);
        (void)knx_chassis_enable();
        tune_enter(TUNE_POSITIVE);
        tune_set_sign(1.0f);
        knx_led_set(KNX_LED_1, true);
        knx_led_set(KNX_LED_2, false);
        knx_beep_beep(60U);
        return;
    }

    uint32_t elapsed = context->now_ms - s_state_tick;
    switch (s_state) {
    case TUNE_POSITIVE:
        tune_set_sign(1.0f); /* 持续刷新，避免命令超时。 */
        if (elapsed >= KNEXUS_TUNE_STEP_HOLD_MS) {
            (void)knx_chassis_stop();
            tune_enter(TUNE_PAUSE);
        }
        break;
    case TUNE_PAUSE:
        if (elapsed >= KNEXUS_TUNE_STEP_PAUSE_MS) {
            tune_enter(TUNE_NEGATIVE);
            tune_set_sign(-1.0f);
        }
        break;
    case TUNE_NEGATIVE:
        tune_set_sign(-1.0f);
        if (elapsed >= KNEXUS_TUNE_STEP_HOLD_MS) {
            tune_stop(TUNE_DONE);
            knx_led_set(KNX_LED_2, true);
            knx_beep_beep(200U);
        }
        break;
    default:
        break;
    }
}

void knexus_mode_pid_tune_on_intersection(const knx_intersection_result_t *result)
{
    (void)result;
}

void knexus_mode_pid_tune_on_peer_message(const knx_comm_message_t *message)
{
    (void)message;
}

void knexus_mode_pid_tune_debug_octo(Octolinker_Instance_t *octo,
                                     uint16_t base_id)
{
    if (octo == NULL) return;
    (void)Octolinker_SendU8(octo, base_id, (uint8_t)s_state);
    (void)Octolinker_SendF32(octo, base_id + 1U, s_target_linear);
    (void)Octolinker_SendF32(octo, base_id + 2U, s_actual_linear);
    (void)Octolinker_SendF32(octo, base_id + 3U,
                             s_target_linear - s_actual_linear);
    (void)Octolinker_SendF32(octo, base_id + 4U, s_target_angular);
    (void)Octolinker_SendF32(octo, base_id + 5U, s_actual_angular);
    (void)Octolinker_SendF32(octo, base_id + 6U,
                             s_target_angular - s_actual_angular);
}

void knexus_mode_pid_tune_debug_control_octo(Octolinker_Instance_t *octo,
                                             uint16_t base_id)
{
    (void)octo;
    (void)base_id;
}
