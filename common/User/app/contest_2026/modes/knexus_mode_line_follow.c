#include "knx26_user.h"
#include "knx26_app.h"
#include "knx26_config.h"
#include "knx_beep.h"
#include "knx_chassis.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_params.h"
#include "knx_safety.h"
#include "knx_time.h"
#include "drv8701e.h"
#include "encoder.h"
#include "knexus_h_hooks.h"
#include "OLED.h"
#include <math.h>

/*
 * 已经实车验证过的巡线模式实现。统一入口位于 knexus_mode.c。
 * 文件开头的运行期参数副本由所有模式共享，便于 OctoLink/GDB 在线调节。
 */
#if defined(KNEXUS_MODE_LINE_FOLLOW)
#define knx26_user_init                  knexus_mode_line_follow_init
#define knx26_user_update                knexus_mode_line_follow_update
#define knx26_user_on_intersection       knexus_mode_line_follow_on_intersection
#define knx26_user_on_peer_message       knexus_mode_line_follow_on_peer_message
#define knx26_user_debug_octo            knexus_mode_line_follow_debug_octo
#define knx26_user_debug_control_octo    knexus_mode_line_follow_debug_control_octo
#elif defined(KNEXUS_MODE_LINE_FOLLOW_BALL_CENTER)
#define knx26_user_init                  knexus_line_follow_core_init
#define knx26_user_update                knexus_line_follow_core_update
#define knx26_user_on_intersection       knexus_line_follow_core_on_intersection
#define knx26_user_on_peer_message       knexus_line_follow_core_on_peer_message
#define knx26_user_debug_octo            knexus_line_follow_core_debug_octo
#define knx26_user_debug_control_octo    knexus_line_follow_core_debug_control_octo
#endif

/* 这些变量是运行期副本，可由 OctoLink/GDB 修改。 */
float knx26_test_left_cmd_polarity = KNEXUS_MOTOR_LEFT_CMD_SIGN_DEFAULT;
float knx26_test_right_cmd_polarity = KNEXUS_MOTOR_RIGHT_CMD_SIGN_DEFAULT;
float knx26_test_left_feedback_polarity = KNEXUS_MOTOR_LEFT_FEEDBACK_SIGN_DEFAULT;
float knx26_test_right_feedback_polarity = KNEXUS_MOTOR_RIGHT_FEEDBACK_SIGN_DEFAULT;
#if defined(KNEXUS_MODE_LINE_FOLLOW) || \
    defined(KNEXUS_MODE_LINE_FOLLOW_BALL_CENTER)
float knx26_test_max_duty = KNEXUS_LINE_MAX_DUTY_DEFAULT;
#elif defined(KNEXUS_MODE_INTERSECTION_SAMPLE)
float knx26_test_max_duty = KNEXUS_SAMPLE_MAX_DUTY_DEFAULT;
#elif defined(KNEXUS_MODE_BOARD_TEST)
float knx26_test_max_duty = KNEXUS_BOARD_TEST_MOTOR_MAX_DUTY;
#else
float knx26_test_max_duty = KNEXUS_TUNE_MAX_DUTY_DEFAULT;
#endif

#if KNEXUS_H_TASK_ENABLE
float knx26_line_speed_mps = KNEXUS_H_LINE_SPEED_MPS_DEFAULT;
#else
float knx26_line_speed_mps = KNEXUS_LINE_BASE_SPEED_MPS_DEFAULT;
#endif
float knx26_line_kp = KNEXUS_LINE_KP_DEFAULT;
float knx26_line_ki = KNEXUS_LINE_KI_DEFAULT;
float knx26_line_kd = KNEXUS_LINE_KD_DEFAULT;
float knx26_line_max_angular_radps = KNEXUS_LINE_MAX_ANGULAR_RADPS_DEFAULT;
float knx26_line_min_angular_radps = KNEXUS_LINE_MIN_ANGULAR_RADPS_DEFAULT;
float knx26_line_integral_limit = KNEXUS_LINE_INTEGRAL_LIMIT_DEFAULT;
float knx26_line_min_strength = KNEXUS_LINE_MIN_STRENGTH_DEFAULT;
float knx26_line_recovery_angular_radps = KNEXUS_LINE_RECOVERY_RADPS_DEFAULT;
float knexus_h_stop_after_mark_m = KNEXUS_H_STOP_AFTER_MARK_M_DEFAULT;
float knexus_h_ball_target_cm = KNEXUS_H_BALL_TARGET_CM_DEFAULT;

#if defined(KNEXUS_MODE_LINE_FOLLOW) || \
    defined(KNEXUS_MODE_LINE_FOLLOW_BALL_CENTER)

#define KNX26_CAL_DELAY_MS          KNEXUS_LINE_CAL_DELAY_MS
#define KNX26_CAL_SAMPLES           KNEXUS_LINE_CAL_SAMPLES
#define KNX26_CAL_MIN_SPAN          KNEXUS_LINE_CAL_MIN_SPAN
#define KNX26_TRACK_FRESH_MS        KNEXUS_LINE_TRACK_FRESH_MS

typedef enum {
    KNX26_STOP_NONE = 0,
    KNX26_STOP_CALIBRATION = 1,
    KNX26_STOP_CALIBRATION_FAULT = 2,
    KNX26_STOP_START_DATA_STALE = 3,
    KNX26_STOP_RUNNING_DATA_STALE = 4,
    KNX26_STOP_CHASSIS_COMMAND = 5,
    KNX26_STOP_KEY1 = 6,
    KNX26_STOP_LINE_LOST = 7,
    KNX26_STOP_H_LAP_COMPLETE = 8,
    KNX26_STOP_H_TIMEOUT = 9,
    KNX26_STOP_H_BALL_NOT_READY = 10,
    KNX26_STOP_SAFETY_FAULT = 11,
    KNX26_STOP_CHASSIS_ENABLE = 12,
} knx26_stop_reason_t;

static knx26_line_state_t s_state;
static uint32_t s_state_tick;
static uint32_t s_cal_sample_count;
static uint32_t s_last_cal_track_update;
static uint32_t s_stop_count;
static uint32_t s_line_lost_count;
static uint32_t s_line_lost_since_ms;
static bool s_line_lost_pending;
static uint32_t s_key1_press_count;
static uint32_t s_faults_before_start_clear;
static uint32_t s_start_safety_reject_count;
static knx26_stop_reason_t s_stop_reason;
static uint16_t s_cal_black[KNX_GRAYSCALE_CH_NUM];
static uint16_t s_cal_white[KNX_GRAYSCALE_CH_NUM];
static float s_previous_error;
static float s_filtered_error;
static float s_filtered_derivative;
static float s_error_integral;
static float s_last_valid_error;
static float s_linear_command;
static float s_angular_command;
static float s_requested_speed;
static float s_desired_linear;
static float s_desired_angular;
static float s_line_confidence;
static float s_curve_metric;
static float s_speed_scale;
static float s_steering_command;
static float s_curve_feedforward_activation;
static float s_curve_feedforward;
static float s_curve_direction;
static float s_linear_accel_command;
static float s_measured_linear_accel;
static float s_last_measured_linear;
static uint16_t s_min_cal_span;
static uint32_t s_h_run_start_ms;
static uint32_t s_h_elapsed_ms;
static uint32_t s_h_start_clear_since_ms;
static uint32_t s_h_finish_since_ms;
static uint32_t s_h_finish_detect_count;
static float s_h_start_position_m;
static float s_h_finish_mark_position_m;
static float s_h_lap_distance_m;
static float s_h_finish_mark_travel_m;
static uint8_t s_h_active_sensor_count;
static uint8_t s_h_active_sensor_mask;
static uint8_t s_h_longest_black_run;
static uint8_t s_h_center_active_count;
static bool s_h_start_line_wide;
static bool s_h_start_line_cleared;
static bool s_h_finish_pending;
static bool s_h_finish_armed;
static bool s_h_strong_finish_mark;
static bool s_h_timer_running;
static float s_h_last_ball_target_cm;
static uint32_t s_h_ball_last_update_ms;
static uint32_t s_h_ball_update_count;
static knexus_h_ball_status_t s_h_ball;

static float clamp_abs(float value, float limit)
{
    if (limit < 0.0f) limit = -limit;
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float sign_f(float value)
{
    return (value < 0.0f) ? -1.0f : 1.0f;
}

static float approach_f(float current, float target, float max_step)
{
    if (max_step < 0.0f) max_step = -max_step;
    float delta = target - current;
    if (delta > max_step) delta = max_step;
    if (delta < -max_step) delta = -max_step;
    return current + delta;
}

static float smoothstep01(float value)
{
    if (value <= 0.0f) return 0.0f;
    if (value >= 1.0f) return 1.0f;
    return value * value * (3.0f - 2.0f * value);
}

#if KNEXUS_H_TASK_ENABLE
static float h_curve_distance_window(float distance_m,
                                     float start_m,
                                     float end_m)
{
    float ramp_m = KNEXUS_H_CURVE_RAMP_M;
    if (ramp_m < 0.01f) ramp_m = 0.01f;
    float enter = smoothstep01((distance_m - start_m) / ramp_m);
    float leave = smoothstep01((end_m - distance_m) / ramp_m);
    return (enter < leave) ? enter : leave;
}
#endif

static void apply_polarities(void)
{
    motor_left.dir_sign = (int8_t)sign_f(knx26_test_left_cmd_polarity);
    motor_right.dir_sign = (int8_t)sign_f(knx26_test_right_cmd_polarity);
    encoder_left.dir_sign = (int8_t)sign_f(knx26_test_left_feedback_polarity);
    encoder_right.dir_sign = (int8_t)sign_f(knx26_test_right_feedback_polarity);

    /* The hardware abstraction owns polarity; do not apply it again in drive. */
    g_knx_params.drive.left_cmd_sign = 1.0f;
    g_knx_params.drive.right_cmd_sign = 1.0f;
    g_knx_params.drive.left_feedback_sign = 1.0f;
    g_knx_params.drive.right_feedback_sign = 1.0f;
    g_knx_params.drive.angular_feedback_sign = 1.0f;
}

static void apply_motor_duty_limit(void)
{
    float limit = knx26_test_max_duty;
    if (limit < 0.0f) limit = 0.0f;
    if (limit > 1.0) limit = 1.0;

    uint32_t left_period = 0U;
    uint32_t right_period = 0U;
    if (motor_left.port != NULL) {
        (void)knx_pwm_get_period((knx_pwm_channel_t *)&motor_left.port->pwm,
                                 &left_period);
    }
    if (motor_right.port != NULL) {
        (void)knx_pwm_get_period((knx_pwm_channel_t *)&motor_right.port->pwm,
                                 &right_period);
    }
    if (left_period > 0U) {
        vc_left.out_min = -(float)left_period * limit;
        vc_left.out_max = (float)left_period * limit;
    }
    if (right_period > 0U) {
        vc_right.out_min = -(float)right_period * limit;
        vc_right.out_max = (float)right_period * limit;
    }
}

static void enter_state(knx26_line_state_t state)
{
    s_state = state;
    s_state_tick = knx_millis();
}

static void stop_chassis(knx26_line_state_t state,
                         uint32_t beep_ms,
                         knx26_stop_reason_t reason)
{
    s_stop_reason = reason;
    (void)knx_chassis_disable();
    s_linear_command = 0.0f;
    s_angular_command = 0.0f;
    s_requested_speed = 0.0f;
    s_desired_linear = 0.0f;
    s_desired_angular = 0.0f;
    s_curve_feedforward_activation = 0.0f;
    s_curve_feedforward = 0.0f;
    s_curve_direction = 0.0f;
    s_linear_accel_command = 0.0f;
#if KNEXUS_H_TASK_ENABLE
    if (s_h_timer_running) {
        s_h_elapsed_ms = knx_millis() - s_h_run_start_ms;
    }
    s_h_timer_running = false;
#endif
    knx_led_set(KNX_LED_2, false);
    enter_state(state);
    if (beep_ms > 0U) knx_beep_beep(beep_ms);
}

static bool track_data_is_fresh(const knx26_context_t *context)
{
    uint32_t age_ms = (context->now_ms >= context->track.timestamp_ms)
                          ? (context->now_ms - context->track.timestamp_ms)
                          : 0U;
    return context->track.sensor.is_calibrated != 0U &&
           context->track.update_count != 0U &&
           age_ms <= KNX26_TRACK_FRESH_MS;
}

#if KNEXUS_H_TASK_ENABLE
static uint8_t h_build_active_sensor_mask(const knx26_context_t *context)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        if (context->track.sensor.normalized[i] >=
            KNEXUS_H_START_LINE_CHANNEL_THRESHOLD) {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

static uint8_t h_count_active_sensors(uint8_t mask)
{
    uint8_t count = 0U;
    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        if ((mask & (uint8_t)(1U << i)) != 0U) count++;
    }
    return count;
}

static uint8_t h_longest_active_run(uint8_t mask)
{
    uint8_t longest = 0U;
    uint8_t current = 0U;
    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            current++;
            if (current > longest) longest = current;
        } else {
            current = 0U;
        }
    }
    return longest;
}

static bool h_is_start_line(uint8_t mask,
                            uint8_t active_count,
                            uint8_t longest_run)
{
    const uint8_t center_mask =
        (uint8_t)(mask & KNEXUS_H_START_LINE_CENTER_MASK);
    const uint8_t center_count = h_count_active_sensors(center_mask);

    if (active_count < KNEXUS_H_START_LINE_ACTIVE_MIN ||
        center_count < KNEXUS_H_START_LINE_CENTER_ACTIVE_MIN) {
        return false;
    }

    /* Accept a centered 3/4-channel bar, a slightly shifted continuous bar,
     * and a wider bar with one weak/missing channel. */
    return center_count >= KNEXUS_H_START_LINE_ACTIVE_MIN ||
           longest_run >= KNEXUS_H_START_LINE_CONTIGUOUS_MIN ||
           active_count >= 4U;
}

static float h_travel_from(float position_m, float origin_m)
{
    return fabsf(position_m - origin_m);
}

static void h_complete_lap(const knx26_context_t *context)
{
    s_h_timer_running = false;
    s_h_elapsed_ms = context->now_ms - s_h_run_start_ms;
    s_h_lap_distance_m = h_travel_from(
        context->chassis.drive.position_m, s_h_start_position_m);
    s_stop_count++;
    stop_chassis(KNX26_LINE_COMPLETE, 400U,
                 KNX26_STOP_H_LAP_COMPLETE);
    knexus_h_display_time_ms(s_h_elapsed_ms, false, true);
}

/* 返回true表示本周期已经停车，调用方不得再下发底盘命令。 */
static bool h_update_lap_state(const knx26_context_t *context)
{
    s_h_elapsed_ms = context->now_ms - s_h_run_start_ms;
    s_h_lap_distance_m = h_travel_from(
        context->chassis.drive.position_m, s_h_start_position_m);
    s_h_active_sensor_mask = h_build_active_sensor_mask(context);
    s_h_active_sensor_count =
        h_count_active_sensors(s_h_active_sensor_mask);
    s_h_longest_black_run =
        h_longest_active_run(s_h_active_sensor_mask);
    s_h_center_active_count = h_count_active_sensors(
        (uint8_t)(s_h_active_sensor_mask &
                  KNEXUS_H_START_LINE_CENTER_MASK));
    s_h_start_line_wide = h_is_start_line(s_h_active_sensor_mask,
                                           s_h_active_sensor_count,
                                           s_h_longest_black_run);
    s_h_strong_finish_mark =
        s_h_center_active_count >=
            KNEXUS_H_FINISH_IMMEDIATE_CENTER_MIN ||
        s_h_active_sensor_count >=
            KNEXUS_H_FINISH_IMMEDIATE_ACTIVE_MIN;
    s_h_finish_armed = false;
    knexus_h_display_time_ms(s_h_elapsed_ms, true, false);

    if (s_h_elapsed_ms >= KNEXUS_H_MAX_RUN_MS) {
        s_h_timer_running = false;
        s_stop_count++;
        stop_chassis(KNX26_LINE_STOPPED, 600U,
                     KNX26_STOP_H_TIMEOUT);
        knexus_h_display_time_ms(s_h_elapsed_ms, false, false);
        return true;
    }

    if (s_state == KNX26_LINE_START_CLEAR) {
        if (s_h_start_line_wide) {
            s_h_start_clear_since_ms = 0U;
        } else if (s_h_start_clear_since_ms == 0U) {
            s_h_start_clear_since_ms = context->now_ms;
        } else if ((context->now_ms - s_h_start_clear_since_ms) >=
                   KNEXUS_H_START_CLEAR_CONFIRM_MS) {
            s_h_start_line_cleared = true;
            s_h_finish_pending = false;
            enter_state(KNX26_LINE_RUNNING);
        }
        return false;
    }

    if (s_state == KNX26_LINE_RUNNING) {
        bool armed = s_h_start_line_cleared &&
                     s_h_elapsed_ms >= KNEXUS_H_FINISH_ARM_MIN_TIME_MS &&
                     s_h_lap_distance_m >=
                         KNEXUS_H_FINISH_ARM_MIN_DISTANCE_M;
        s_h_finish_armed = armed;
        if (armed && s_h_start_line_wide) {
            bool confirmed = s_h_strong_finish_mark;

            /* Real-car captures show the centred 0x3C mark for only 10-20 ms
             * at 0.43 m/s.  It is geometrically unambiguous, so accept it in
             * one control cycle.  Shifted/weaker three-channel patterns keep
             * the original continuous-time confirmation against false hits. */
            if (!confirmed) {
                if (!s_h_finish_pending) {
                    s_h_finish_pending = true;
                    s_h_finish_since_ms = context->now_ms;
                } else if ((context->now_ms - s_h_finish_since_ms) >=
                           KNEXUS_H_FINISH_CONFIRM_MS) {
                    confirmed = true;
                }
            }

            if (confirmed) {
                s_h_finish_detect_count++;
                s_h_finish_mark_position_m =
                    context->chassis.drive.position_m;
                s_h_finish_mark_travel_m = 0.0f;
                s_h_finish_pending = false;
                enter_state(KNX26_LINE_FINISH_APPROACH);
                if (knexus_h_stop_after_mark_m <= 0.0f) {
                    h_complete_lap(context);
                    return true;
                }
            }
        } else {
            s_h_finish_pending = false;
            s_h_finish_since_ms = 0U;
        }
        return false;
    }

    if (s_state == KNX26_LINE_FINISH_APPROACH) {
        s_h_finish_mark_travel_m = h_travel_from(
            context->chassis.drive.position_m,
            s_h_finish_mark_position_m);
        if (s_h_finish_mark_travel_m >= knexus_h_stop_after_mark_m) {
            h_complete_lap(context);
            return true;
        }
    }
    return false;
}
#endif

static void start_calibration(void)
{
    stop_chassis(KNX26_LINE_CAL_BLACK_DELAY, 60U,
                 KNX26_STOP_CALIBRATION);
    s_cal_sample_count = 0U;
    s_min_cal_span = 0U;
    knx_led_set(KNX_LED_1, false);
}

static void update_calibration(const knx26_context_t *context)
{
    uint32_t elapsed = context->now_ms - s_state_tick;

    if (s_state == KNX26_LINE_CAL_BLACK_DELAY) {
        knx_led_set(KNX_LED_1, ((elapsed / 100U) & 1U) != 0U);
        if (elapsed >= KNX26_CAL_DELAY_MS) {
            for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
                s_cal_black[i] = 0xFFFFU;
            }
            s_cal_sample_count = 0U;
            s_last_cal_track_update = context->track.update_count;
            enter_state(KNX26_LINE_CAL_BLACK_SAMPLE);
        }
        return;
    }

    if (s_state == KNX26_LINE_CAL_BLACK_SAMPLE) {
        if (context->track.last_status != KNX_OK ||
            context->track.update_count == s_last_cal_track_update) {
            return;
        }
        s_last_cal_track_update = context->track.update_count;
        for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
            if (context->track.sensor.raw[i] < s_cal_black[i]) {
                s_cal_black[i] = context->track.sensor.raw[i];
            }
        }
        if (++s_cal_sample_count >= KNX26_CAL_SAMPLES) {
            knx_beep_beep(150U);
            s_cal_sample_count = 0U;
            enter_state(KNX26_LINE_CAL_WHITE_DELAY);
        }
        return;
    }

    if (s_state == KNX26_LINE_CAL_WHITE_DELAY) {
        knx_led_set(KNX_LED_1, ((elapsed / 250U) & 1U) != 0U);
        if (elapsed >= KNX26_CAL_DELAY_MS) {
            for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
                s_cal_white[i] = 0U;
            }
            s_cal_sample_count = 0U;
            s_last_cal_track_update = context->track.update_count;
            enter_state(KNX26_LINE_CAL_WHITE_SAMPLE);
        }
        return;
    }

    if (s_state == KNX26_LINE_CAL_WHITE_SAMPLE) {
        if (context->track.last_status != KNX_OK ||
            context->track.update_count == s_last_cal_track_update) {
            return;
        }
        s_last_cal_track_update = context->track.update_count;
        for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
            if (context->track.sensor.raw[i] > s_cal_white[i]) {
                s_cal_white[i] = context->track.sensor.raw[i];
            }
        }
        if (++s_cal_sample_count < KNX26_CAL_SAMPLES) return;

        s_min_cal_span = 0xFFFFU;
        for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
            uint16_t span = (s_cal_white[i] > s_cal_black[i])
                                ? (uint16_t)(s_cal_white[i] - s_cal_black[i])
                                : 0U;
            if (span < s_min_cal_span) s_min_cal_span = span;
        }

        if (s_min_cal_span < KNX26_CAL_MIN_SPAN) {
            stop_chassis(KNX26_LINE_CAL_FAULT, 600U,
                         KNX26_STOP_CALIBRATION_FAULT);
            knx_led_set(KNX_LED_1, false);
            return;
        }

        knx_track_set_calibration(s_cal_black, s_cal_white);
        knx_led_set(KNX_LED_1, true);
        enter_state(KNX26_LINE_READY);
        knx_beep_beep(250U);
    }
}

static void start_line_follow(const knx26_context_t *context)
{
    if (!track_data_is_fresh(context)) {
        stop_chassis(KNX26_LINE_STOPPED, 300U,
                     KNX26_STOP_START_DATA_STALE);
        return;
    }

#if KNEXUS_H_TASK_ENABLE
    knexus_h_ball_request_hold(s_h_last_ball_target_cm);
    knexus_h_ball_snapshot(&s_h_ball);
#if KNEXUS_H_REQUIRE_BALL_READY
    if (!s_h_ball.implemented || !s_h_ball.ready || s_h_ball.fault) {
        stop_chassis(KNX26_LINE_STOPPED, 500U,
                     KNX26_STOP_H_BALL_NOT_READY);
        return;
    }
#endif
#endif

    /* KEY1 is an explicit operator start command.  Clear any latched transient
     * boot fault, immediately re-evaluate the live safety inputs, and only then
     * arm both the chassis and timer.  A persistent fault is rejected again. */
    s_faults_before_start_clear = knx_safety_get_faults();
    knx_safety_clear_faults();
    (void)knx_safety_update();
    if (knx_safety_get_level() == KNX_SAFETY_LEVEL_FAULT) {
        s_start_safety_reject_count++;
        stop_chassis(KNX26_LINE_STOPPED, 500U,
                     KNX26_STOP_SAFETY_FAULT);
        return;
    }

    apply_polarities();
    apply_motor_duty_limit();
    s_filtered_error = context->track.sensor.line_error;
    s_previous_error = s_filtered_error;
    s_filtered_derivative = 0.0f;
    s_error_integral = 0.0f;
    s_last_valid_error = s_filtered_error;
    s_line_lost_since_ms = 0U;
    s_line_lost_pending = false;
    s_stop_reason = KNX26_STOP_NONE;
    s_linear_command = 0.0f;
    s_angular_command = 0.0f;
    s_requested_speed = 0.0f;
    s_desired_linear = 0.0f;
    s_desired_angular = 0.0f;
    s_line_confidence = 0.0f;
    s_curve_metric = 0.0f;
    s_speed_scale = 0.0f;
    s_steering_command = 0.0f;
    s_curve_feedforward_activation = 0.0f;
    s_curve_feedforward = 0.0f;
    s_curve_direction = 0.0f;
    s_linear_accel_command = 0.0f;
    s_measured_linear_accel = 0.0f;
    s_last_measured_linear = context->chassis.drive.measured_linear_mps;
    if (knx_chassis_enable() != KNX_OK) {
        stop_chassis(KNX26_LINE_STOPPED, 500U,
                     KNX26_STOP_CHASSIS_ENABLE);
        return;
    }
    knx_led_set(KNX_LED_1, true);
    knx_led_set(KNX_LED_2, true);
#if KNEXUS_H_TASK_ENABLE
    s_h_run_start_ms = context->now_ms;
    s_h_elapsed_ms = 0U;
    s_h_start_position_m = context->chassis.drive.position_m;
    s_h_finish_mark_position_m = s_h_start_position_m;
    s_h_lap_distance_m = 0.0f;
    s_h_finish_mark_travel_m = 0.0f;
    s_h_active_sensor_count = 0U;
    s_h_active_sensor_mask = 0U;
    s_h_longest_black_run = 0U;
    s_h_center_active_count = 0U;
    s_h_start_line_wide = false;
    s_h_start_line_cleared = false;
    s_h_finish_pending = false;
    s_h_finish_armed = false;
    s_h_strong_finish_mark = false;
    s_h_finish_detect_count = 0U;
    s_h_start_clear_since_ms = 0U;
    s_h_finish_since_ms = 0U;
    s_h_timer_running = true;
    enter_state(KNX26_LINE_START_CLEAR);
    knexus_h_display_time_ms(0U, true, false);
#else
    enter_state(KNX26_LINE_RUNNING);
#endif
    knx_beep_beep(60U);
}

static void update_line_follow(const knx26_context_t *context,
                               float requested_speed_mps)
{
    if (knx_safety_get_level() == KNX_SAFETY_LEVEL_FAULT) {
        s_stop_count++;
        stop_chassis(KNX26_LINE_STOPPED, 500U,
                     KNX26_STOP_SAFETY_FAULT);
        return;
    }

    if (!track_data_is_fresh(context)) {
        s_line_lost_count++;
        stop_chassis(KNX26_LINE_STOPPED, 300U,
                     KNX26_STOP_RUNNING_DATA_STALE);
        return;
    }

#if KNEXUS_LINE_LOST_STOP_ENABLE
    /*
     * Stop only after a continuous lost-line window.  Isolated weak samples
     * and short gaps still use the existing last-direction recovery steering,
     * while a genuinely detached car is brought to a safe stop.
     */
    if (context->track.line_lost) {
        if (!s_line_lost_pending) {
            s_line_lost_pending = true;
            s_line_lost_since_ms = context->now_ms;
            s_line_lost_count++;
        }
        if ((context->now_ms - s_line_lost_since_ms) >=
            KNEXUS_LINE_LOST_STOP_CONFIRM_MS) {
            s_stop_count++;
            stop_chassis(KNX26_LINE_LOST, 300U,
                         KNX26_STOP_LINE_LOST);
            return;
        }
    } else {
        s_line_lost_pending = false;
        s_line_lost_since_ms = 0U;
    }
#endif

    const float dt_s = (float)KNX26_APP_PERIOD_MS * 0.001f;
    const float raw_error = clamp_abs(context->track.sensor.line_error,
                                      KNEXUS_LINE_ERROR_ABS_LIMIT);

    /* Blend continuously between normal tracking and last-direction search.
     * A hard strength threshold caused the previous steering command to jump
     * whenever the line response hovered around that threshold. */
    const float weak_floor = KNEXUS_LINE_WEAK_FLOOR;
    float confidence = 1.0f;
    if (knx26_line_min_strength > weak_floor) {
        confidence = (context->track.line_strength - weak_floor) /
                     (knx26_line_min_strength - weak_floor);
    }
    if (confidence < 0.0f) confidence = 0.0f;
    if (confidence > 1.0f) confidence = 1.0f;
    s_line_confidence = confidence;
    if (confidence <= 0.0f) s_line_lost_count++;

    float effective_error = confidence * raw_error +
                            (1.0f - confidence) * s_last_valid_error;

    /* A black line moving between adjacent sensors produces a legitimate
     * step in the weighted centroid.  Filter both the centroid and its
     * derivative so that this step cannot reverse the motors through an
     * impulsive D term. */
    s_filtered_error += KNEXUS_LINE_ERROR_FILTER_ALPHA *
                        (effective_error - s_filtered_error);
    float raw_derivative = (s_filtered_error - s_previous_error) / dt_s;
    s_previous_error = s_filtered_error;
    s_filtered_derivative += KNEXUS_LINE_D_FILTER_ALPHA *
                             (raw_derivative - s_filtered_derivative);

    if (confidence >= KNEXUS_LINE_VALID_CONFIDENCE) {
        s_last_valid_error = s_filtered_error;
    }
    float abs_error = (s_filtered_error < 0.0f)
                          ? -s_filtered_error
                          : s_filtered_error;

    /* Leaky/conditional integral prevents a long turn or line loss from
     * pinning the integral and fighting the next recovery direction. */
    if (confidence < KNEXUS_LINE_VALID_CONFIDENCE) {
        s_error_integral *= KNEXUS_LINE_INTEGRAL_WEAK_DECAY;
    } else if (abs_error < KNEXUS_LINE_STRAIGHT_ZONE_ERROR) {
        /* Do not let a tiny straight-line error build enough integral to
         * overshoot the centre and start a left/right limit cycle. */
        s_error_integral *= KNEXUS_LINE_STRAIGHT_INTEGRAL_DECAY;
    } else {
        if (s_error_integral * s_filtered_error < 0.0f) {
            s_error_integral *= KNEXUS_LINE_INTEGRAL_REVERSE_DECAY;
        } else {
            s_error_integral *= KNEXUS_LINE_INTEGRAL_NORMAL_DECAY;
        }
        s_error_integral += s_filtered_error * dt_s;
    }
    s_error_integral = clamp_abs(s_error_integral,
                                 knx26_line_integral_limit);

    float steering = knx26_line_kp * s_filtered_error +
                     knx26_line_ki * s_error_integral +
                     knx26_line_kd * s_filtered_derivative;
    /* Smooth stiction compensation: unlike a sign/dead-band step, this is
     * continuous through zero and therefore does not kick while recentering. */
    steering += knx26_line_min_angular_radps *
                s_filtered_error /
                (abs_error + KNEXUS_LINE_STICTION_SMOOTHING);

    /* Gain scheduling is confined to the near-centre straight-line zone.
     * Cornering behaviour is bit-for-bit unchanged outside this zone. */
    if (abs_error <= KNEXUS_LINE_STRAIGHT_DEADBAND_ERROR) {
        steering = 0.0f;
    } else if (abs_error < KNEXUS_LINE_STRAIGHT_ZONE_ERROR) {
        float blend = (abs_error - KNEXUS_LINE_STRAIGHT_DEADBAND_ERROR) /
                      (KNEXUS_LINE_STRAIGHT_ZONE_ERROR -
                       KNEXUS_LINE_STRAIGHT_DEADBAND_ERROR);
        /* The old formula started at MIN_GAIN immediately above the
         * dead-band, creating a finite steering step at the boundary.  The
         * extra blend makes the transition continuous while still reaching
         * exactly full cornering authority at STRAIGHT_ZONE_ERROR. */
        float gain = blend *
                     (KNEXUS_LINE_STRAIGHT_MIN_GAIN +
                      (1.0f - KNEXUS_LINE_STRAIGHT_MIN_GAIN) * blend);
        steering *= gain;
    }

    float recovery_steering = sign_f(s_last_valid_error) *
                              knx26_line_recovery_angular_radps;
    steering = confidence * steering +
               (1.0f - confidence) * recovery_steering;

    /* Stable curve metric: derivative look-ahead starts braking before the
     * centroid error grows large.  Fast attack protects corner entry; slow
     * release prevents an abrupt acceleration while leaving the curve. */
    float abs_derivative = (s_filtered_derivative < 0.0f)
                               ? -s_filtered_derivative
                               : s_filtered_derivative;
    float lookahead_error = abs_derivative * KNEXUS_LINE_CURVE_LOOKAHEAD_S;
    if (lookahead_error > KNEXUS_LINE_CURVE_LOOKAHEAD_MAX_ERROR) {
        lookahead_error = KNEXUS_LINE_CURVE_LOOKAHEAD_MAX_ERROR;
    }
    float instant_curve_metric = abs_error + lookahead_error;
    float curve_alpha = (instant_curve_metric > s_curve_metric)
                            ? KNEXUS_LINE_CURVE_ATTACK_ALPHA
                            : KNEXUS_LINE_CURVE_RELEASE_ALPHA;
    s_curve_metric += curve_alpha *
                      (instant_curve_metric - s_curve_metric);

    /*
     * H-track feedforward is scheduled from odometry, not from the same
     * oscillating grayscale signal that the feedback loop is correcting.
     * Smooth 18 cm ramps tolerate start placement and encoder error without
     * producing an angular step at either end of a curve.
     */
#if KNEXUS_H_TASK_ENABLE
    float curve1 = h_curve_distance_window(
        s_h_lap_distance_m,
        KNEXUS_H_CURVE1_START_M, KNEXUS_H_CURVE1_END_M);
    float curve2 = h_curve_distance_window(
        s_h_lap_distance_m,
        KNEXUS_H_CURVE2_START_M, KNEXUS_H_CURVE2_END_M);
    s_curve_feedforward_activation =
        (curve1 > curve2) ? curve1 : curve2;
    s_curve_direction = KNEXUS_H_CURVE_STEERING_SIGN;
#else
    s_curve_feedforward_activation = 0.0f;
    s_curve_direction = 0.0f;
#endif
    s_curve_feedforward = s_curve_direction *
                          KNEXUS_LINE_CURVE_FEEDFORWARD_RADPS *
                          s_curve_feedforward_activation;
    steering += s_curve_feedforward;

#if KNEXUS_H_TASK_ENABLE
    s_speed_scale = 1.0f -
                    (1.0f - KNEXUS_H_CURVE_SPEED_SCALE) *
                    s_curve_feedforward_activation;
#else
    s_speed_scale = 1.0f -
                    KNEXUS_LINE_CURVE_SLOWDOWN_GAIN * s_curve_metric;
#endif
    if (s_speed_scale < KNEXUS_LINE_MIN_SPEED_SCALE) {
        s_speed_scale = KNEXUS_LINE_MIN_SPEED_SCALE;
    }
    if (s_speed_scale > 1.0f) s_speed_scale = 1.0f;
    /* Slow forward motion sharply while searching so the stronger turn can
     * sweep the sensor bar back across the line instead of drawing a wide arc. */
    s_speed_scale *= KNEXUS_LINE_SEARCH_SPEED_BASE +
                     KNEXUS_LINE_SEARCH_SPEED_CONFIDENCE * confidence;

    s_requested_speed = requested_speed_mps;
    s_desired_linear = clamp_abs(requested_speed_mps * s_speed_scale,
                                 g_knx_params.drive.max_linear_speed_mps);
    s_desired_angular = clamp_abs(-steering,
                                  knx26_line_max_angular_radps);
    s_steering_command = steering;

    /* Jerk-limited longitudinal governor.  Angular control deliberately
     * stays on the existing path so the proven cornering authority is kept. */
    float target_accel = (s_desired_linear - s_linear_command) /
                         KNEXUS_LINE_SPEED_RESPONSE_S;
    if (target_accel > KNEXUS_LINE_ACCEL_LIMIT_MPS2) {
        target_accel = KNEXUS_LINE_ACCEL_LIMIT_MPS2;
    }
    if (target_accel < -KNEXUS_LINE_DECEL_LIMIT_MPS2) {
        target_accel = -KNEXUS_LINE_DECEL_LIMIT_MPS2;
    }
    float linear_error_before = s_desired_linear - s_linear_command;
    s_linear_accel_command = approach_f(
        s_linear_accel_command, target_accel,
        KNEXUS_LINE_JERK_LIMIT_MPS3 * dt_s);
    s_linear_command += s_linear_accel_command * dt_s;
    if ((linear_error_before >= 0.0f &&
         s_linear_command > s_desired_linear) ||
        (linear_error_before < 0.0f &&
         s_linear_command < s_desired_linear)) {
        s_linear_command = s_desired_linear;
        s_linear_accel_command = 0.0f;
    }

    float measured_accel =
        (context->chassis.drive.measured_linear_mps -
         s_last_measured_linear) / dt_s;
    s_last_measured_linear = context->chassis.drive.measured_linear_mps;
    s_measured_linear_accel += KNEXUS_LINE_MEASURED_ACCEL_FILTER_ALPHA *
                               (measured_accel - s_measured_linear_accel);

    /* Keep command shaping fast; the PID derivative and stable curve
     * feedforward above now provide damping without another delayed loop. */
    float angular_alpha;
    if (s_desired_angular * s_angular_command < 0.0f) {
        angular_alpha = KNEXUS_LINE_ANGULAR_REVERSE_ALPHA;
    } else if (fabsf(s_desired_angular) > fabsf(s_angular_command)) {
        angular_alpha = KNEXUS_LINE_ANGULAR_ATTACK_ALPHA;
    } else {
        angular_alpha = KNEXUS_LINE_ANGULAR_RELEASE_ALPHA;
    }
    /*
     * The eight-channel centroid changes in discrete sensor-width steps.
     * A single transition can therefore move the desired yaw rate by
     * 0.4--0.8 rad/s even though the car has not changed curvature. Bound
     * only the command slope: full steady-state cornering authority remains.
     */
    float angular_target = s_angular_command +
                           angular_alpha *
                               (s_desired_angular - s_angular_command);
    s_angular_command = approach_f(
        s_angular_command, angular_target,
        KNEXUS_LINE_ANGULAR_SLEW_RADPS2 * dt_s);

    if (knx_chassis_set_velocity(s_linear_command, s_angular_command) != KNX_OK) {
        s_stop_count++;
        stop_chassis(KNX26_LINE_STOPPED, 500U,
                     KNX26_STOP_CHASSIS_COMMAND);
    }
}

void knx26_user_init(void)
{
    apply_polarities();
    apply_motor_duty_limit();

    g_knx_params.drive.max_linear_speed_mps = KNEXUS_LINE_DRIVE_MAX_LINEAR_MPS;
    g_knx_params.drive.max_angular_speed_radps = KNEXUS_LINE_DRIVE_MAX_ANGULAR_RADPS;
    g_knx_params.drive.max_linear_accel_mps2 = KNEXUS_LINE_DRIVE_MAX_LINEAR_ACCEL;
    g_knx_params.drive.max_angular_accel_radps2 = KNEXUS_LINE_DRIVE_MAX_ANGULAR_ACCEL;
    drv8701e_velocity_acc_limit_mps2 = KNEXUS_MOTOR_ACCEL_LIMIT_MPS2_DEFAULT;
    knx_drive_reload_params();

    s_cal_sample_count = 0U;
    s_last_cal_track_update = 0U;
    s_stop_count = 0U;
    s_line_lost_count = 0U;
    s_line_lost_since_ms = 0U;
    s_line_lost_pending = false;
    s_key1_press_count = 0U;
    s_faults_before_start_clear = 0U;
    s_start_safety_reject_count = 0U;
    s_stop_reason = KNX26_STOP_NONE;
    s_previous_error = 0.0f;
    s_filtered_error = 0.0f;
    s_filtered_derivative = 0.0f;
    s_error_integral = 0.0f;
    s_last_valid_error = 0.0f;
    s_linear_command = 0.0f;
    s_angular_command = 0.0f;
    s_requested_speed = 0.0f;
    s_desired_linear = 0.0f;
    s_desired_angular = 0.0f;
    s_line_confidence = 0.0f;
    s_curve_metric = 0.0f;
    s_speed_scale = 0.0f;
    s_steering_command = 0.0f;
    s_curve_feedforward_activation = 0.0f;
    s_curve_feedforward = 0.0f;
    s_curve_direction = 0.0f;
    s_linear_accel_command = 0.0f;
    s_measured_linear_accel = 0.0f;
    s_last_measured_linear = 0.0f;
    s_min_cal_span = 0U;
#if KNEXUS_H_TASK_ENABLE
    s_h_run_start_ms = 0U;
    s_h_elapsed_ms = 0U;
    s_h_start_clear_since_ms = 0U;
    s_h_finish_since_ms = 0U;
    s_h_finish_detect_count = 0U;
    s_h_start_position_m = 0.0f;
    s_h_finish_mark_position_m = 0.0f;
    s_h_lap_distance_m = 0.0f;
    s_h_finish_mark_travel_m = 0.0f;
    s_h_active_sensor_count = 0U;
    s_h_active_sensor_mask = 0U;
    s_h_longest_black_run = 0U;
    s_h_center_active_count = 0U;
    s_h_start_line_wide = false;
    s_h_start_line_cleared = false;
    s_h_finish_pending = false;
    s_h_finish_armed = false;
    s_h_strong_finish_mark = false;
    s_h_timer_running = false;
    s_h_last_ball_target_cm = knexus_h_ball_target_cm;
    s_h_ball_last_update_ms = 0U;
    s_h_ball_update_count = 0U;
    s_h_ball = (knexus_h_ball_status_t){0};
    knexus_h_ball_init();
    knexus_h_ball_request_hold(s_h_last_ball_target_cm);
    knexus_h_display_time_ms(0U, false, false);
#endif
    (void)knx_chassis_disable();
    knx_led_set(KNX_LED_1, false);
    knx_led_set(KNX_LED_2, false);
    enter_state(KNX26_LINE_IDLE);
}

void knx26_user_update(const struct knx26_context *raw_context)
{
    const knx26_context_t *context = (const knx26_context_t *)raw_context;
    if (context == NULL) return;

#if KNEXUS_H_TASK_ENABLE
    if (knexus_h_ball_target_cm != s_h_last_ball_target_cm) {
        s_h_last_ball_target_cm = knexus_h_ball_target_cm;
        knexus_h_ball_request_hold(s_h_last_ball_target_cm);
    }
    knexus_h_ball_snapshot(&s_h_ball);
    knexus_h_display_time_ms(s_h_elapsed_ms,
                             s_h_timer_running,
                             s_state == KNX26_LINE_COMPLETE);
#endif

    if (knx_key_just_pressed(KNX_KEY_0)) {
        start_calibration();
        return;
    }

    if (s_state >= KNX26_LINE_CAL_BLACK_DELAY &&
        s_state <= KNX26_LINE_CAL_WHITE_SAMPLE) {
        update_calibration(context);
        return;
    }

    if (knx_key_just_pressed(KNX_KEY_1)) {
        s_key1_press_count++;
        if (s_state == KNX26_LINE_RUNNING
#if KNEXUS_H_TASK_ENABLE
            || s_state == KNX26_LINE_START_CLEAR ||
               s_state == KNX26_LINE_FINISH_APPROACH
#endif
        ) {
            s_stop_count++;
            stop_chassis(KNX26_LINE_STOPPED, 100U, KNX26_STOP_KEY1);
        } else if (s_state == KNX26_LINE_READY ||
                   s_state == KNX26_LINE_STOPPED ||
                   s_state == KNX26_LINE_LOST
#if KNEXUS_H_TASK_ENABLE
                   || s_state == KNX26_LINE_COMPLETE
#endif
        ) {
            start_line_follow(context);
        } else {
            knx_beep_beep(300U);
        }
        return;
    }

    if (s_state == KNX26_LINE_RUNNING
#if KNEXUS_H_TASK_ENABLE
        || s_state == KNX26_LINE_START_CLEAR ||
           s_state == KNX26_LINE_FINISH_APPROACH
#endif
    ) {
#if KNEXUS_H_TASK_ENABLE
        float requested_speed = (s_state == KNX26_LINE_FINISH_APPROACH)
                                    ? KNEXUS_H_FINISH_APPROACH_SPEED_MPS
                                    : knx26_line_speed_mps;
        update_line_follow(context, requested_speed);
        if (s_state == KNX26_LINE_RUNNING ||
            s_state == KNX26_LINE_START_CLEAR ||
            s_state == KNX26_LINE_FINISH_APPROACH) {
            (void)h_update_lap_state(context);
        }
#else
        update_line_follow(context, knx26_line_speed_mps);
#endif
    }
}

knx26_line_state_t knexus_line_follow_core_get_state(void)
{
    return s_state;
}

float knexus_line_follow_core_get_accel_command_mps2(void)
{
    return s_linear_accel_command;
}

#if KNEXUS_H_TASK_ENABLE
void knx26_h_ball_control_update(void)
{
    uint32_t now = knx_millis();
    if (s_h_ball_last_update_ms != 0U &&
        (now - s_h_ball_last_update_ms) < KNX26_H_BALL_PERIOD_MS) {
        return;
    }
    float dt_s = (s_h_ball_last_update_ms == 0U)
                     ? (float)KNX26_H_BALL_PERIOD_MS * 0.001f
                     : (float)(now - s_h_ball_last_update_ms) * 0.001f;
    s_h_ball_last_update_ms = now;
    knx26_context_t context;
    knx26_context_snapshot(&context);
    if (!context.ready) return;
    knexus_h_ball_update(&context, dt_s);
    s_h_ball_update_count++;
}
#endif

void knx26_user_on_intersection(const knx_intersection_result_t *result)
{
    (void)result;
}

void knx26_user_on_peer_message(const knx_comm_message_t *message)
{
    (void)message;
}

void knx26_user_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id)
{
    if (octo == NULL) return;
    (void)Octolinker_SendU8(octo, base_id + 0U, (uint8_t)s_state);
    (void)Octolinker_SendU8(octo, base_id + 1U,
                            (uint8_t)(s_state >= KNX26_LINE_READY &&
                                      s_state != KNX26_LINE_CAL_FAULT));
    (void)Octolinker_SendU8(octo, base_id + 2U,
                            (uint8_t)(s_state == KNX26_LINE_RUNNING
#if KNEXUS_H_TASK_ENABLE
                                      || s_state == KNX26_LINE_START_CLEAR ||
                                         s_state == KNX26_LINE_FINISH_APPROACH
#endif
                            ));
    (void)Octolinker_SendU32(octo, base_id + 3U, s_cal_sample_count);
    (void)Octolinker_SendU16(octo, base_id + 4U, s_min_cal_span);
    (void)Octolinker_SendF32(octo, base_id + 5U, s_linear_command);
    (void)Octolinker_SendF32(octo, base_id + 6U, s_angular_command);
    (void)Octolinker_SendF32(octo, base_id + 7U, knx26_line_speed_mps);
    (void)Octolinker_SendF32(octo, base_id + 8U, knx26_line_kp);
    (void)Octolinker_SendF32(octo, base_id + 9U, knx26_line_kd);
    (void)Octolinker_SendF32(octo, base_id + 10U, knx26_line_max_angular_radps);
    (void)Octolinker_SendU32(octo, base_id + 11U, s_stop_count);
    (void)Octolinker_SendU32(octo, base_id + 12U, s_line_lost_count);
    (void)Octolinker_SendF32(octo, base_id + 13U, knx26_test_max_duty);
    (void)Octolinker_SendF32(octo, base_id + 14U, knx26_test_left_cmd_polarity);
    (void)Octolinker_SendF32(octo, base_id + 15U, knx26_test_right_cmd_polarity);
    (void)Octolinker_SendU8(octo, base_id + 16U, (uint8_t)s_stop_reason);
    (void)Octolinker_SendU32(octo, base_id + 17U, s_key1_press_count);
    (void)Octolinker_SendU8(octo, 766U,
                            (uint8_t)s_line_lost_pending);
    uint32_t lost_elapsed_ms = s_line_lost_pending
                                   ? (knx_millis() - s_line_lost_since_ms)
                                   : 0U;
    (void)Octolinker_SendU32(octo, 767U, lost_elapsed_ms);
    (void)Octolinker_SendU32(octo, 768U,
                             KNEXUS_LINE_LOST_STOP_CONFIRM_MS);
#if KNEXUS_H_TASK_ENABLE
    (void)Octolinker_SendU32(octo, 810U, s_h_elapsed_ms);
    (void)Octolinker_SendF32(octo, 811U, s_h_lap_distance_m);
    (void)Octolinker_SendU8(octo, 812U, s_h_active_sensor_count);
    (void)Octolinker_SendU8(octo, 813U,
                            (uint8_t)s_h_start_line_wide);
    (void)Octolinker_SendU8(octo, 814U,
                            (uint8_t)s_h_start_line_cleared);
    (void)Octolinker_SendU8(octo, 815U,
                            (uint8_t)s_h_finish_pending);
    (void)Octolinker_SendU32(octo, 816U, s_h_finish_detect_count);
    (void)Octolinker_SendF32(octo, 817U, s_h_finish_mark_travel_m);
    (void)Octolinker_SendF32(octo, 818U, knexus_h_stop_after_mark_m);
    (void)Octolinker_SendF32(octo, 819U, knexus_h_ball_target_cm);
    (void)Octolinker_SendU8(octo, 820U,
                            (uint8_t)s_h_ball.implemented);
    (void)Octolinker_SendU8(octo, 821U, (uint8_t)s_h_ball.ready);
    (void)Octolinker_SendU8(octo, 822U, (uint8_t)s_h_ball.fault);
    (void)Octolinker_SendF32(octo, 823U,
                             s_h_ball.measured_position_cm);
    (void)Octolinker_SendF32(octo, 824U,
                             s_h_ball.control_output);
    (void)Octolinker_SendU32(octo, 825U, s_h_ball_update_count);
    (void)Octolinker_SendU8(octo, 828U, s_h_active_sensor_mask);
    (void)Octolinker_SendU8(octo, 829U, s_h_longest_black_run);
#endif
    /* Motor-loop decomposition for the next closed-loop tuning pass. */
    (void)Octolinker_SendF32(octo, 840U, vc_left.target_used);
    (void)Octolinker_SendF32(octo, 841U, vc_right.target_used);
    (void)Octolinker_SendF32(octo, 842U, vc_left.feedforward);
    (void)Octolinker_SendF32(octo, 843U, vc_right.feedforward);
    (void)Octolinker_SendF32(octo, 844U, vc_left.correction);
    (void)Octolinker_SendF32(octo, 845U, vc_right.correction);
    (void)Octolinker_SendF32(octo, 846U, vc_left.integral);
    (void)Octolinker_SendF32(octo, 847U, vc_right.integral);
    (void)Octolinker_SendF32(octo, 848U, vc_left.output);
    (void)Octolinker_SendF32(octo, 849U, vc_right.output);
    (void)Octolinker_SendF32(octo, 850U,
                             drv8701e_velocity_kv_cnt_per_mps);
    (void)Octolinker_SendF32(octo, 851U,
                             drv8701e_velocity_duty_deadzone_cnt);
#if KNEXUS_H_TASK_ENABLE && KNEXUS_H_OLED_ENABLE
    extern volatile uint32_t knexus_h_oled_displayed_ms;
    extern volatile uint8_t knexus_h_oled_display_state;
    (void)Octolinker_SendU8(octo, 852U, (uint8_t)OLED_IsReady());
    (void)Octolinker_SendU32(octo, 853U, OLED_GetErrorCount());
    (void)Octolinker_SendU32(octo, 854U,
                             knexus_h_oled_displayed_ms);
    (void)Octolinker_SendU8(octo, 855U,
                            knexus_h_oled_display_state);
#endif
    (void)Octolinker_SendU32(octo, 856U,
                             s_faults_before_start_clear);
    (void)Octolinker_SendU32(octo, 857U,
                             s_start_safety_reject_count);
#if KNEXUS_H_TASK_ENABLE
    (void)Octolinker_SendU8(octo, 858U,
                            s_h_center_active_count);
    (void)Octolinker_SendU8(octo, 859U,
                            (uint8_t)s_h_finish_armed);
    (void)Octolinker_SendU8(octo, 860U,
                            (uint8_t)s_h_strong_finish_mark);
#endif
}

void knx26_user_debug_control_octo(Octolinker_Instance_t *octo,
                                   uint16_t base_id)
{
    if (octo == NULL) return;
    (void)Octolinker_SendF32(octo, base_id + 0U, s_filtered_error);
    (void)Octolinker_SendF32(octo, base_id + 1U, s_filtered_derivative);
    (void)Octolinker_SendF32(octo, base_id + 2U, s_linear_command);
    (void)Octolinker_SendF32(octo, base_id + 3U, s_angular_command);
    (void)Octolinker_SendF32(octo, base_id + 12U, s_error_integral);
    (void)Octolinker_SendF32(octo, 830U, s_line_confidence);
    (void)Octolinker_SendF32(octo, 831U, s_curve_metric);
    (void)Octolinker_SendF32(octo, 832U, s_speed_scale);
    (void)Octolinker_SendF32(octo, 833U, s_requested_speed);
    (void)Octolinker_SendF32(octo, 834U, s_desired_linear);
    (void)Octolinker_SendF32(octo, 835U, s_desired_angular);
    (void)Octolinker_SendF32(octo, 836U, s_steering_command);
    (void)Octolinker_SendF32(octo, 837U, s_linear_accel_command);
    (void)Octolinker_SendF32(octo, 838U, s_measured_linear_accel);
    (void)Octolinker_SendF32(octo, 861U,
                             KNEXUS_LINE_ERROR_FILTER_ALPHA);
    (void)Octolinker_SendF32(octo, 862U,
                             KNEXUS_LINE_ANGULAR_ATTACK_ALPHA);
    (void)Octolinker_SendF32(octo, 863U,
                             KNEXUS_LINE_ANGULAR_RELEASE_ALPHA);
    (void)Octolinker_SendF32(octo, 864U,
                             KNEXUS_LINE_ANGULAR_REVERSE_ALPHA);
    (void)Octolinker_SendF32(octo, 869U,
                             s_curve_feedforward_activation);
    (void)Octolinker_SendF32(octo, 870U, s_curve_feedforward);
    (void)Octolinker_SendF32(octo, 871U, s_curve_direction);
    (void)Octolinker_SendF32(octo, 872U,
                             KNEXUS_LINE_CURVE_FEEDFORWARD_RADPS);
}

#endif
