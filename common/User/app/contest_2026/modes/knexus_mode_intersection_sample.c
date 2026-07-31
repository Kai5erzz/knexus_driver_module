#include "knexus_mode_backend.h"
#include "knexus_mode_common.h"
#include "knexus_config.h"
#include "knx26_app.h"
#include "knx_beep.h"
#include "knx_board.h"
#include "knx_chassis.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_params.h"
#include "knx_time.h"
#include "knx_target_config.h"
#include <string.h>

#if defined(KNX_PLATFORM_STM32)
#include "track_tinycnn.h"
_Static_assert(TRACK_NN_ROWS == KNEXUS_SAMPLE_ROWS,
               "TinyCNN rows must equal KNEXUS_SAMPLE_ROWS");
_Static_assert(TRACK_NN_COLS == KNX_GRAYSCALE_CH_NUM,
               "TinyCNN columns must equal grayscale channels");
#endif

typedef enum {
    SAMPLE_IDLE = 0,
    SAMPLE_CAL_BLACK_DELAY,
    SAMPLE_CAL_BLACK,
    SAMPLE_CAL_WHITE_DELAY,
    SAMPLE_CAL_WHITE,
    SAMPLE_READY,
    SAMPLE_RUNNING,
    SAMPLE_DONE,
    SAMPLE_FAULT,
} sample_state_t;

static sample_state_t s_state;
static uint32_t s_state_tick;
static uint32_t s_last_track_update;
static uint16_t s_cal_count;
static uint16_t s_cal_black[KNX_GRAYSCALE_CH_NUM];
static uint16_t s_cal_white[KNX_GRAYSCALE_CH_NUM];
static uint8_t s_matrix[KNEXUS_SAMPLE_ROWS];
static uint8_t s_row;
static float s_start_position_m;
static float s_position_m;
static uint32_t s_settle_tick;
static knx_status_t s_send_status;
static uint8_t s_result_class;
static float s_result_confidence;
static knx_intersection_result_t s_last_intersection;

static void enter_state(sample_state_t state)
{
    s_state = state;
    s_state_tick = knx_millis();
}

static void stop_motion(void)
{
    (void)knx_chassis_disable();
    knx_led_set(KNX_LED_2, false);
}

static void start_calibration(void)
{
    stop_motion();
    s_cal_count = 0U;
    knx_led_set(KNX_LED_1, false);
    enter_state(SAMPLE_CAL_BLACK_DELAY);
    knx_beep_beep(60U);
}

static bool consume_fresh_track(const knx26_context_t *context)
{
    if (context->track.last_status != KNX_OK ||
        context->track.update_count == s_last_track_update) {
        return false;
    }
    s_last_track_update = context->track.update_count;
    return true;
}

static void update_calibration(const knx26_context_t *context)
{
    uint32_t elapsed = context->now_ms - s_state_tick;
    if (s_state == SAMPLE_CAL_BLACK_DELAY) {
        knx_led_set(KNX_LED_1, ((elapsed / 100U) & 1U) != 0U);
        if (elapsed >= KNEXUS_LINE_CAL_DELAY_MS) {
            for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
                s_cal_black[i] = 0xFFFFU;
            }
            s_cal_count = 0U;
            s_last_track_update = context->track.update_count;
            enter_state(SAMPLE_CAL_BLACK);
        }
        return;
    }
    if (s_state == SAMPLE_CAL_BLACK) {
        if (!consume_fresh_track(context)) return;
        for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
            if (context->track.sensor.raw[i] < s_cal_black[i]) {
                s_cal_black[i] = context->track.sensor.raw[i];
            }
        }
        if (++s_cal_count >= KNEXUS_LINE_CAL_SAMPLES) {
            s_cal_count = 0U;
            enter_state(SAMPLE_CAL_WHITE_DELAY);
            knx_beep_beep(150U);
        }
        return;
    }
    if (s_state == SAMPLE_CAL_WHITE_DELAY) {
        knx_led_set(KNX_LED_1, ((elapsed / 250U) & 1U) != 0U);
        if (elapsed >= KNEXUS_LINE_CAL_DELAY_MS) {
            memset(s_cal_white, 0, sizeof(s_cal_white));
            s_cal_count = 0U;
            s_last_track_update = context->track.update_count;
            enter_state(SAMPLE_CAL_WHITE);
        }
        return;
    }
    if (s_state != SAMPLE_CAL_WHITE || !consume_fresh_track(context)) return;

    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        if (context->track.sensor.raw[i] > s_cal_white[i]) {
            s_cal_white[i] = context->track.sensor.raw[i];
        }
    }
    if (++s_cal_count < KNEXUS_LINE_CAL_SAMPLES) return;

    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        uint16_t span = (s_cal_white[i] > s_cal_black[i])
                            ? (uint16_t)(s_cal_white[i] - s_cal_black[i])
                            : 0U;
        if (span < KNEXUS_LINE_CAL_MIN_SPAN) {
            enter_state(SAMPLE_FAULT);
            knx_led_set(KNX_LED_1, false);
            knx_beep_beep(600U);
            return;
        }
    }
    knx_track_set_calibration(s_cal_black, s_cal_white);
    enter_state(SAMPLE_READY);
    knx_led_set(KNX_LED_1, true);
    knx_beep_beep(250U);
}

static void save_row(const knx26_context_t *context)
{
    if (s_row >= KNEXUS_SAMPLE_ROWS) return;
    s_matrix[s_row++] = context->track.sensor.digital_byte;
}

static void fill_rows(void)
{
    uint8_t value = (s_row > 0U) ? s_matrix[s_row - 1U] : 0U;
    while (s_row < KNEXUS_SAMPLE_ROWS) s_matrix[s_row++] = value;
}

static void classify_matrix(void)
{
#if defined(KNX_PLATFORM_STM32)
    static uint8_t input[TRACK_NN_ROWS][TRACK_NN_COLS];
    track_tinycnn_result_t result = {0};
    for (uint8_t row = 0U; row < TRACK_NN_ROWS; ++row) {
        for (uint8_t col = 0U; col < TRACK_NN_COLS; ++col) {
            input[row][col] = ((s_matrix[row] & (uint8_t)(1U << col)) != 0U)
                                  ? 255U : 0U;
        }
    }
    track_tinycnn_predict_u8(input, &result);
    s_result_class = result.pred_class;
    s_result_confidence = result.confidence;
#else
    uint32_t wide_rows = 0U;
    for (uint8_t row = 0U; row < KNEXUS_SAMPLE_ROWS; ++row) {
        uint8_t value = s_matrix[row];
        uint8_t bits = 0U;
        while (value != 0U) {
            bits += value & 1U;
            value >>= 1U;
        }
        if (bits >= 5U) wide_rows++;
    }
    s_result_class = (wide_rows >= 6U) ? 0U : 1U;
    s_result_confidence = (float)((wide_rows >= 6U)
                                      ? wide_rows
                                      : (KNEXUS_SAMPLE_ROWS - wide_rows)) /
                          (float)KNEXUS_SAMPLE_ROWS;
#endif
}

static void send_result(void)
{
    Octolinker_Instance_t *octo = knx_board_get_octolinker();
    if (octo == NULL) return;
    s_send_status = Octolinker_SendLiteBitMatrix(
        octo, (uint8_t)KNEXUS_SAMPLE_OCTO_MATRIX_ID, s_matrix,
        (uint8_t)KNEXUS_SAMPLE_ROWS, KNX_GRAYSCALE_CH_NUM);
    (void)Octolinker_SendLiteU8(
        octo, (uint8_t)KNEXUS_SAMPLE_OCTO_CLASS_ID, s_result_class);
    (void)Octolinker_SendLiteF32(
        octo, (uint8_t)KNEXUS_SAMPLE_OCTO_CONFIDENCE_ID,
        s_result_confidence);
}

static void start_sampling(const knx26_context_t *context)
{
    memset(s_matrix, 0, sizeof(s_matrix));
    s_row = 0U;
    s_settle_tick = 0U;
    s_result_class = 0xFFU;
    s_result_confidence = 0.0f;
    knx_chassis_reset_odometry();
    s_start_position_m = 0.0f;
    knexus_mode_apply_motor_config(KNEXUS_SAMPLE_MAX_DUTY_DEFAULT);
    (void)knx_chassis_enable();
    if (knx_chassis_set_position_angle(KNEXUS_SAMPLE_DISTANCE_M,
                                       context->chassis.drive.heading_rad) != KNX_OK) {
        stop_motion();
        enter_state(SAMPLE_FAULT);
        knx_beep_beep(600U);
        return;
    }
    knx_led_set(KNX_LED_2, true);
    enter_state(SAMPLE_RUNNING);
    knx_beep_beep(60U);
}

void knexus_mode_intersection_sample_init(void)
{
    knexus_mode_apply_motor_config(KNEXUS_SAMPLE_MAX_DUTY_DEFAULT);
    g_knx_params.drive.position_pid.kp = KNEXUS_SAMPLE_POSITION_KP;
    g_knx_params.drive.position_pid.ki = KNEXUS_SAMPLE_POSITION_KI;
    g_knx_params.drive.position_pid.kd = KNEXUS_SAMPLE_POSITION_KD;
    g_knx_params.drive.position_pid.max_out = KNEXUS_SAMPLE_POSITION_MAX_OUT;
    g_knx_params.drive.angle_pid.kp = KNEXUS_SAMPLE_HEADING_KP;
    g_knx_params.drive.angle_pid.ki = KNEXUS_SAMPLE_HEADING_KI;
    g_knx_params.drive.angle_pid.kd = KNEXUS_SAMPLE_HEADING_KD;
    g_knx_params.drive.angle_pid.max_out = KNEXUS_SAMPLE_HEADING_MAX_OUT;
    knx_drive_reload_params();
    memset(&s_last_intersection, 0, sizeof(s_last_intersection));
    memset(s_matrix, 0, sizeof(s_matrix));
    s_send_status = KNX_OK;
    stop_motion();
    knx_led_set(KNX_LED_1, false);
#if KNEXUS_LINE_SENSOR_REQUIRES_CALIBRATION
    enter_state(SAMPLE_IDLE);
#else
    knx_led_set(KNX_LED_1, true);
    enter_state(SAMPLE_READY);
#endif
}

void knexus_mode_intersection_sample_update(
    const struct knx26_context *raw_context)
{
    const knx26_context_t *context = (const knx26_context_t *)raw_context;
    if (context == NULL) return;

    if (knx_key_just_pressed(KNX_KEY_0)) {
        if (s_state == SAMPLE_DONE) {
            send_result();
            knx_beep_beep(100U);
        } else {
#if KNEXUS_LINE_SENSOR_REQUIRES_CALIBRATION
            start_calibration();
#else
            knx_led_set(KNX_LED_1, true);
            enter_state(SAMPLE_READY);
            knx_beep_beep(100U);
#endif
        }
        return;
    }
    if (s_state >= SAMPLE_CAL_BLACK_DELAY && s_state <= SAMPLE_CAL_WHITE) {
        update_calibration(context);
        return;
    }
    if (knx_key_just_pressed(KNX_KEY_1)) {
        if (s_state == SAMPLE_READY || s_state == SAMPLE_DONE) {
            start_sampling(context);
        } else if (s_state == SAMPLE_RUNNING) {
            stop_motion();
            enter_state(SAMPLE_READY);
        } else {
            knx_beep_beep(300U);
        }
        return;
    }
    if (s_state != SAMPLE_RUNNING) return;

    s_position_m = context->chassis.drive.position_m - s_start_position_m;
    while (s_row < KNEXUS_SAMPLE_ROWS &&
           s_position_m >= (float)s_row * KNEXUS_SAMPLE_INTERVAL_M) {
        save_row(context);
    }
    if (context->now_ms - s_state_tick > KNEXUS_SAMPLE_TIMEOUT_MS) {
        fill_rows();
        stop_motion();
        enter_state(SAMPLE_FAULT);
        knx_beep_beep(600U);
        return;
    }
    float error = KNEXUS_SAMPLE_DISTANCE_M - s_position_m;
    if (error < 0.0f) error = -error;
    float speed = context->chassis.drive.measured_linear_mps;
    if (speed < 0.0f) speed = -speed;
    if (error <= KNEXUS_SAMPLE_POSITION_TOLERANCE_M &&
        speed <= KNEXUS_SAMPLE_SPEED_SETTLED_MPS) {
        if (s_settle_tick == 0U) s_settle_tick = context->now_ms;
        if (context->now_ms - s_settle_tick >= KNEXUS_SAMPLE_SETTLE_DELAY_MS) {
            fill_rows();
            classify_matrix();
            stop_motion();
            enter_state(SAMPLE_DONE);
            knx_beep_beep(200U);
        }
    } else {
        s_settle_tick = 0U;
    }
}

void knexus_mode_intersection_sample_on_intersection(
    const knx_intersection_result_t *result)
{
    if (result != NULL) s_last_intersection = *result;
}

void knexus_mode_intersection_sample_on_peer_message(
    const knx_comm_message_t *message)
{
    (void)message;
}

void knexus_mode_intersection_sample_debug_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    if (octo == NULL) return;
    (void)Octolinker_SendU8(octo, base_id, (uint8_t)s_state);
    (void)Octolinker_SendU8(octo, base_id + 1U, s_row);
    (void)Octolinker_SendF32(octo, base_id + 2U, s_position_m);
    (void)Octolinker_SendU8(octo, base_id + 3U, s_result_class);
    (void)Octolinker_SendF32(octo, base_id + 4U, s_result_confidence);
    (void)Octolinker_SendI32(octo, base_id + 5U, (int32_t)s_send_status);
    (void)Octolinker_SendU8(octo, base_id + 6U,
                            (uint8_t)s_last_intersection.type);
    (void)Octolinker_SendU8(
        octo, base_id + 7U,
        (uint8_t)(s_result_class == KNEXUS_SAMPLE_NN_COMPLEX_CLASS &&
                  s_result_confidence >= KNEXUS_SAMPLE_NN_COMPLEX_CONFIDENCE));
}

void knexus_mode_intersection_sample_debug_control_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    (void)octo;
    (void)base_id;
}
