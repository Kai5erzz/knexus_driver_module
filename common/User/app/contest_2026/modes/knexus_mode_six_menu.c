#include "knexus_mode_backend.h"
#include "knexus_mode_line_follow_core.h"
#include "knexus_config.h"
#include "knx26_app.h"
#include "knx_beep.h"
#include "knx_chassis.h"
#include "knx_key.h"
#include "knx_led.h"
#include "knx_pendulum_comm.h"
#include "knx_time.h"
#include "knexus_h_hooks.h"
#include "OLED.h"
#include <math.h>
#include <string.h>

#if defined(KNX_PLATFORM_STM32)
#include "knx_board.h"
#endif

#if defined(KNEXUS_MODE_SIX_MENU)

/*
 * 六模式运行时菜单：
 *   1/6     仅上报模式，不驱动机构；
 *   2       运行现有循迹，JC4310持续0 N*m；
 *   3       底盘静止，钢球-5cm到+5cm并最终保持；
 *   4       A到B循迹，视觉回中叠加底盘运动补偿；
 *   5       整圈循迹，视觉回中叠加底盘运动补偿。
 */
typedef enum {
    MODE3_STAGE_IDLE = 0,
    MODE3_STAGE_TO_NEGATIVE,
    MODE3_STAGE_TO_POSITIVE,
    MODE3_STAGE_HOLD_POSITIVE,
} mode3_stage_t;

typedef enum {
    VISUAL_BREAKAWAY_WAIT = 0,
    VISUAL_BREAKAWAY_PULSE,
    VISUAL_BREAKAWAY_COOLDOWN,
} visual_breakaway_state_t;

static uint8_t s_cursor_mode = 1U;
static uint8_t s_active_mode;
static uint8_t s_mode3_running;
static uint8_t s_mode4_running;
static uint8_t s_mode5_running;
static uint8_t s_ball_control_armed;
static uint8_t s_ball_prelevel_requested;
static uint8_t s_task_completed;
static uint8_t s_task_timeout;
static mode3_stage_t s_mode3_stage;
static uint8_t s_exit_latched;
static uint8_t s_vision_fresh;
static uint32_t s_last_oled_ms;
static uint32_t s_task_start_ms;
static uint32_t s_task_elapsed_ms;
static float s_task_start_position_m;
static float s_task_distance_m;
static float s_task_ball_max_error_cm;
static uint32_t s_task_ball_violation_count;
static uint32_t s_task_last_ball_rx_count;
static uint32_t s_last_visual_rx_count;
static uint32_t s_last_visual_ms;
static float s_ball_target_cm;
static float s_visual_error;
static float s_visual_error_rate;
static float s_visual_error_previous;
static float s_visual_error_integral;
static float s_visual_i_accel_mps2;
static float s_visual_breakaway_accel_mps2;
static float s_visual_target_velocity_mps;
static float s_visual_velocity_error_mps;
static float s_visual_position_to_speed_gain;
static float s_visual_velocity_kp_gain;
static float s_visual_deadband_m;
static float s_desired_ball_accel_mps2;
static float s_gravity_feedforward_roll_deg;
static float s_target_roll_deg;
static float s_mode3_predicted_offset_cm;
static float s_mode3_offset_rate_cmps;
static float s_mode3_last_offset_cm;
static uint32_t s_mode3_last_offset_rx_count;
static uint32_t s_mode3_last_offset_ms;
static uint8_t s_last_uplink_mode;
static uint8_t s_mode4_ball_guard_stop;
static float s_mode4_speed_request_mps;
static visual_breakaway_state_t s_visual_breakaway_state;
static uint32_t s_visual_breakaway_tick_ms;
static int8_t s_visual_breakaway_direction;
static char s_oled_cache[4][17];

static float clampf_local(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float slew_local(float current, float target, float rate_per_s,
                        float dt_s)
{
    float step = fabsf(rate_per_s) * dt_s;
    return current + clampf_local(target - current, -step, step);
}

static bool line_state_is_running(knx26_line_state_t state)
{
    return state == KNX26_LINE_RUNNING ||
           state == KNX26_LINE_START_CLEAR ||
           state == KNX26_LINE_FINISH_APPROACH;
}

static float mode4_speed_request(void)
{
    s_mode4_ball_guard_stop = 0U;
    float speed_mps = KNEXUS_MENU_MODE4_LINE_SPEED_MPS;
    float remaining_m = KNEXUS_MENU_MODE4_AB_DISTANCE_M -
        s_task_distance_m;
    if (remaining_m <= 0.0f) return 0.0f;
    float finish_speed_mps = sqrtf(
        2.0f * KNEXUS_MENU_MODE4_DECEL_LIMIT_MPS2 * remaining_m);
    if (remaining_m <= KNEXUS_MENU_MODE4_FINISH_MARGIN_M &&
        finish_speed_mps < KNEXUS_MENU_MODE4_FINISH_MIN_SPEED_MPS) {
        finish_speed_mps = KNEXUS_MENU_MODE4_FINISH_MIN_SPEED_MPS;
    }
    if (finish_speed_mps < speed_mps) speed_mps = finish_speed_mps;
    return speed_mps;
}

static void update_vision_fresh(
    const knx26_context_t *context,
    const knx_pendulum_comm_state_t *comm)
{
    uint32_t age_ms = comm->rx_count != 0U
        ? context->now_ms - comm->last_rx_ms : 0xFFFFFFFFU;
    bool raw_valid = isfinite(comm->offset) &&
        comm->offset > KNEXUS_VISION_CENTER_INVALID_LOW_CM &&
        comm->offset < KNEXUS_VISION_CENTER_INVALID_HIGH_CM;
    s_vision_fresh = raw_valid &&
        age_ms <= KNEXUS_VISION_CENTER_TIMEOUT_MS ? 1U : 0U;
}

static void visual_controller_reset(
    const knx_pendulum_comm_state_t *comm, float target_cm)
{
    s_ball_target_cm = target_cm;
    float raw_error = KNEXUS_VISION_CENTER_INPUT_SIGN *
        KNEXUS_VISION_CENTER_INPUT_SCALE *
        (comm->offset - s_ball_target_cm);
    s_visual_error = raw_error;
    if (fabsf(s_visual_error) < KNEXUS_VISION_CENTER_DEADBAND) {
        s_visual_error = 0.0f;
    }
    s_visual_error_previous = raw_error;
    s_visual_error_rate = 0.0f;
    s_visual_error_integral = 0.0f;
    s_visual_i_accel_mps2 = 0.0f;
    s_visual_breakaway_accel_mps2 = 0.0f;
    s_visual_target_velocity_mps = 0.0f;
    s_visual_velocity_error_mps = 0.0f;
    s_visual_position_to_speed_gain =
        KNEXUS_VISION_CENTER_POSITION_TO_SPEED_S_INV;
    s_visual_velocity_kp_gain =
        KNEXUS_VISION_CENTER_VELOCITY_KP_S_INV;
    s_visual_deadband_m = KNEXUS_VISION_CENTER_DEADBAND;
    s_visual_breakaway_state = VISUAL_BREAKAWAY_WAIT;
    s_visual_breakaway_tick_ms = 0U;
    s_visual_breakaway_direction = 0;
    s_desired_ball_accel_mps2 = 0.0f;
    s_gravity_feedforward_roll_deg = 0.0f;
    s_target_roll_deg = 0.0f;
    s_last_visual_rx_count = comm->rx_count;
    s_last_visual_ms = comm->last_rx_ms;
}

static void update_auto_prelevel(
    const knx_pendulum_comm_state_t *comm)
{
#if defined(KNX_PLATFORM_STM32)
    if (s_ball_prelevel_requested != 0U &&
        s_ball_control_armed == 0U &&
        s_vision_fresh != 0U &&
        knexus_jc4310_is_ready() != 0U) {
        s_ball_control_armed = 1U;
        visual_controller_reset(comm, 0.0f);
    }
#else
    (void)comm;
#endif
}

static void visual_controller_update(
    const knx26_context_t *context,
    const knx_pendulum_comm_state_t *comm,
    float target_cm, bool enabled,
    bool add_motion_compensation,
    bool allow_breakaway,
    float deadband_m)
{
    const float dt_s = (float)KNEXUS_APP_PERIOD_MS * 0.001f;
    s_ball_target_cm = target_cm;
    s_visual_deadband_m = deadband_m > 0.0f
        ? deadband_m : KNEXUS_VISION_CENTER_DEADBAND;
    if (enabled && s_vision_fresh != 0U) {
        if (comm->rx_count != s_last_visual_rx_count) {
            float raw_error = KNEXUS_VISION_CENTER_INPUT_SIGN *
                KNEXUS_VISION_CENTER_INPUT_SCALE *
                (comm->offset - s_ball_target_cm);
            bool crossed_target =
                raw_error * s_visual_error_previous < 0.0f;
            s_visual_error = raw_error;
            if (fabsf(s_visual_error) < s_visual_deadband_m) {
                s_visual_error = 0.0f;
            }
            float visual_dt_s = s_last_visual_ms != 0U
                ? (float)(comm->last_rx_ms - s_last_visual_ms) * 0.001f
                : dt_s;
            visual_dt_s = clampf_local(visual_dt_s, 0.002f, 0.100f);
            float raw_rate =
                (raw_error - s_visual_error_previous) / visual_dt_s;
            float tau_s = 1.0f /
                (6.28318530718f * KNEXUS_VISION_CENTER_D_LPF_HZ);
            float alpha = visual_dt_s / (tau_s + visual_dt_s);
            s_visual_error_rate +=
                alpha * (raw_rate - s_visual_error_rate);
            if (crossed_target) {
                s_visual_error_integral *= 0.10f;
                s_visual_breakaway_state =
                    VISUAL_BREAKAWAY_COOLDOWN;
                s_visual_breakaway_tick_ms = context->now_ms;
            }
            if (s_visual_error != 0.0f &&
                fabsf(s_visual_error) <=
                    KNEXUS_VISION_CENTER_INTEGRAL_ZONE_M &&
                fabsf(s_visual_error_rate) <=
                    KNEXUS_VISION_CENTER_INTEGRAL_RATE_MAX_MPS) {
                s_visual_error_integral +=
                    s_visual_error * visual_dt_s;
                float integral_limit =
                    KNEXUS_VISION_CENTER_KI_MPS3 > 0.0001f
                        ? KNEXUS_VISION_CENTER_I_ACCEL_LIMIT_MPS2 /
                            KNEXUS_VISION_CENTER_KI_MPS3
                        : 0.0f;
                s_visual_error_integral = clampf_local(
                    s_visual_error_integral,
                    -integral_limit, integral_limit);
            } else {
                s_visual_error_integral *=
                    KNEXUS_VISION_CENTER_INTEGRAL_DECAY;
            }
            s_visual_error_previous = raw_error;
            s_last_visual_rx_count = comm->rx_count;
            s_last_visual_ms = comm->last_rx_ms;
        }
        s_visual_i_accel_mps2 = clampf_local(
            KNEXUS_VISION_CENTER_KI_MPS3 * s_visual_error_integral,
            -KNEXUS_VISION_CENTER_I_ACCEL_LIMIT_MPS2,
            KNEXUS_VISION_CENTER_I_ACCEL_LIMIT_MPS2);
        float error_abs_m = fabsf(s_visual_error);
        int8_t error_direction = s_visual_error > 0.0f
            ? 1 : (s_visual_error < 0.0f ? -1 : 0);
        bool ball_stuck =
            error_abs_m >=
                KNEXUS_VISION_CENTER_BREAKAWAY_ENTER_ERROR_M &&
            fabsf(s_visual_error_rate) <=
                KNEXUS_VISION_CENTER_BREAKAWAY_MAX_RATE_MPS;
        /* 行驶时底盘本身已经提供微振动，不再叠加静摩擦脉冲，避免在
         * +/-1 cm评分边界附近主动制造一次额外的杆角阶跃。 */
        if (add_motion_compensation || !allow_breakaway) {
            ball_stuck = false;
            s_visual_breakaway_state = VISUAL_BREAKAWAY_WAIT;
            s_visual_breakaway_tick_ms = 0U;
            s_visual_breakaway_direction = 0;
        }
        s_visual_breakaway_accel_mps2 = 0.0f;
        if (s_visual_breakaway_state == VISUAL_BREAKAWAY_WAIT) {
            if (!ball_stuck || error_direction == 0) {
                s_visual_breakaway_tick_ms = 0U;
            } else {
                if (s_visual_breakaway_tick_ms == 0U ||
                    s_visual_breakaway_direction != error_direction) {
                    s_visual_breakaway_tick_ms = context->now_ms;
                    s_visual_breakaway_direction = error_direction;
                } else if ((context->now_ms -
                        s_visual_breakaway_tick_ms) >=
                            KNEXUS_VISION_CENTER_BREAKAWAY_DETECT_MS) {
                    s_visual_breakaway_state =
                        VISUAL_BREAKAWAY_PULSE;
                    s_visual_breakaway_tick_ms = context->now_ms;
                }
            }
        }
        if (s_visual_breakaway_state == VISUAL_BREAKAWAY_PULSE) {
            if (!ball_stuck || error_direction !=
                    s_visual_breakaway_direction ||
                (context->now_ms - s_visual_breakaway_tick_ms) >=
                    KNEXUS_VISION_CENTER_BREAKAWAY_PULSE_MS) {
                s_visual_breakaway_state =
                    VISUAL_BREAKAWAY_COOLDOWN;
                s_visual_breakaway_tick_ms = context->now_ms;
            } else {
                s_visual_breakaway_accel_mps2 =
                    (float)s_visual_breakaway_direction *
                    KNEXUS_VISION_CENTER_BREAKAWAY_ACCEL_MPS2;
            }
        } else if (s_visual_breakaway_state ==
                       VISUAL_BREAKAWAY_COOLDOWN &&
                   (context->now_ms - s_visual_breakaway_tick_ms) >=
                       KNEXUS_VISION_CENTER_BREAKAWAY_COOLDOWN_MS) {
            s_visual_breakaway_state = VISUAL_BREAKAWAY_WAIT;
            s_visual_breakaway_tick_ms = 0U;
            s_visual_breakaway_direction = 0;
        }
        bool near_target = error_abs_m <=
            KNEXUS_VISION_CENTER_NEAR_ZONE_M;
        s_visual_position_to_speed_gain = near_target
            ? KNEXUS_VISION_CENTER_NEAR_POSITION_TO_SPEED_S_INV
            : KNEXUS_VISION_CENTER_POSITION_TO_SPEED_S_INV;
        s_visual_velocity_kp_gain = near_target
            ? KNEXUS_VISION_CENTER_NEAR_VELOCITY_KP_S_INV
            : KNEXUS_VISION_CENTER_VELOCITY_KP_S_INV;
        s_visual_target_velocity_mps = clampf_local(
            -s_visual_position_to_speed_gain *
                s_visual_error,
            -KNEXUS_VISION_CENTER_MAX_SPEED_MPS,
            KNEXUS_VISION_CENTER_MAX_SPEED_MPS);
        s_visual_velocity_error_mps = s_visual_error_rate -
            s_visual_target_velocity_mps;
        s_desired_ball_accel_mps2 = clampf_local(
            s_visual_velocity_kp_gain *
                s_visual_velocity_error_mps +
                s_visual_i_accel_mps2 +
                s_visual_breakaway_accel_mps2,
            -KNEXUS_VISION_CENTER_MAX_ACCEL_MPS2,
            KNEXUS_VISION_CENTER_MAX_ACCEL_MPS2);
        float gravity_gain = KNEXUS_VISION_CENTER_ROLLING_GAIN *
            KNEXUS_VISION_CENTER_GRAVITY_MPS2 *
            KNEXUS_VISION_CENTER_GRAVITY_FEEDFORWARD_GAIN;
        float target_roll = asinf(clampf_local(
            s_desired_ball_accel_mps2 / gravity_gain,
            -0.99f, 0.99f)) * 57.2957795f;
        target_roll = clampf_local(
            target_roll,
            -KNEXUS_VISION_CENTER_MAX_ROLL_DEG,
            KNEXUS_VISION_CENTER_MAX_ROLL_DEG);
        s_gravity_feedforward_roll_deg = target_roll;
        s_target_roll_deg = slew_local(
            s_target_roll_deg, target_roll,
            KNEXUS_VISION_CENTER_TARGET_SLEW_DPS, dt_s);
    } else {
        s_visual_error_rate = 0.0f;
        s_visual_error_integral = 0.0f;
        s_visual_i_accel_mps2 = 0.0f;
        s_visual_breakaway_accel_mps2 = 0.0f;
        s_visual_target_velocity_mps = 0.0f;
        s_visual_velocity_error_mps = 0.0f;
        s_visual_position_to_speed_gain =
            KNEXUS_VISION_CENTER_POSITION_TO_SPEED_S_INV;
        s_visual_velocity_kp_gain =
            KNEXUS_VISION_CENTER_VELOCITY_KP_S_INV;
        s_visual_breakaway_state = VISUAL_BREAKAWAY_WAIT;
        s_visual_breakaway_tick_ms = 0U;
        s_visual_breakaway_direction = 0;
        s_desired_ball_accel_mps2 = 0.0f;
        s_gravity_feedforward_roll_deg = 0.0f;
        s_target_roll_deg = slew_local(
            s_target_roll_deg, 0.0f,
            KNEXUS_VISION_CENTER_TARGET_SLEW_DPS, dt_s);
    }

#if defined(KNX_PLATFORM_STM32)
    knexus_jc4310_external_target_set_ex(
        s_target_roll_deg,
        enabled && s_vision_fresh != 0U,
        enabled && add_motion_compensation);
#else
    (void)add_motion_compensation;
#endif
}

static void oled_line(uint8_t line, const char *text)
{
    char next[17];
    uint8_t i = 0U;
    while (i < 16U && text != NULL && text[i] != '\0') {
        next[i] = text[i];
        ++i;
    }
    while (i < 16U) next[i++] = ' ';
    next[16] = '\0';
    for (i = 0U; i < 16U; ++i) {
        if (s_oled_cache[line - 1U][i] != next[i]) {
            OLED_ShowChar(line, (uint8_t)(i + 1U), next[i]);
            s_oled_cache[line - 1U][i] = next[i];
        }
    }
}

static void make_mode_line(char out[17], uint8_t mode, bool selected)
{
    static const char *const names[6] = {
        "M1 RESERVED", "M2 TRACK", "M3 +/-5CM",
        "M4 A-B+BALL", "M5 LAP+BALL", "M6 RESERVED"
    };
    memset(out, ' ', 16U);
    out[16] = '\0';
    out[0] = selected ? '>' : ' ';
    const char *name = names[mode - 1U];
    for (uint8_t i = 0U; i < 14U && name[i] != '\0'; ++i) {
        out[i + 1U] = name[i];
    }
}

static void make_offset_line(char out[17], float offset_cm)
{
    memset(out, ' ', 16U);
    out[16] = '\0';
    memcpy(out, "OFFSET ", 7U);
    if (offset_cm < 0.0f) {
        out[7] = '-';
        offset_cm = -offset_cm;
    } else {
        out[7] = '+';
    }
    if (offset_cm > 99.9f) offset_cm = 99.9f;
    int32_t tenths = (int32_t)(offset_cm * 10.0f + 0.5f);
    out[8] = (char)('0' + (tenths / 100) % 10);
    out[9] = (char)('0' + (tenths / 10) % 10);
    out[10] = '.';
    out[11] = (char)('0' + tenths % 10);
    out[12] = 'C';
    out[13] = 'M';
}

static void make_time_line(char out[17], uint32_t elapsed_ms)
{
    uint32_t seconds = elapsed_ms / 1000U;
    uint32_t millis = elapsed_ms % 1000U;
    if (seconds > 999U) seconds = 999U;
    memset(out, ' ', 16U);
    out[16] = '\0';
    memcpy(out, "TIME 000.000S", 13U);
    out[5] = (char)('0' + (seconds / 100U) % 10U);
    out[6] = (char)('0' + (seconds / 10U) % 10U);
    out[7] = (char)('0' + seconds % 10U);
    out[9] = (char)('0' + (millis / 100U) % 10U);
    out[10] = (char)('0' + (millis / 10U) % 10U);
    out[11] = (char)('0' + millis % 10U);
}

static void send_mode_uplink(uint8_t mode)
{
#if defined(KNX_PLATFORM_STM32)
    /* 当前上位机只实现mode3的数据通道。下位机仍保留3/4/5/6各自任务，
     * 仅把上行选择号兼容映射为3，待上位机补齐后可删除此映射。 */
    uint8_t uplink_mode = mode >= 3U && mode <= 6U ? 3U : mode;
    s_last_uplink_mode = uplink_mode;
    (void)knx_pendulum_comm_send_mode(
        (knx_pendulum_mode_t)uplink_mode);
#else
    (void)mode;
    s_last_uplink_mode = 0U;
#endif
}

static void stop_all_actuators(void)
{
    (void)knx_chassis_disable();
    if (s_active_mode == 2U || s_active_mode == 4U ||
        s_active_mode == 5U) {
        knexus_line_follow_core_force_stop();
    }
    s_mode3_running = 0U;
    s_mode4_running = 0U;
    s_mode5_running = 0U;
    s_ball_control_armed = 0U;
    s_ball_prelevel_requested = 0U;
    s_mode3_stage = MODE3_STAGE_IDLE;
    s_task_completed = 0U;
    s_task_timeout = 0U;
    s_task_start_ms = 0U;
    s_task_elapsed_ms = 0U;
    s_task_start_position_m = 0.0f;
    s_task_distance_m = 0.0f;
    s_task_ball_max_error_cm = 0.0f;
    s_task_ball_violation_count = 0U;
    s_task_last_ball_rx_count = 0U;
    s_visual_error_integral = 0.0f;
    s_visual_i_accel_mps2 = 0.0f;
    s_visual_breakaway_accel_mps2 = 0.0f;
    s_visual_target_velocity_mps = 0.0f;
    s_visual_velocity_error_mps = 0.0f;
    s_visual_position_to_speed_gain =
        KNEXUS_VISION_CENTER_POSITION_TO_SPEED_S_INV;
    s_visual_velocity_kp_gain =
        KNEXUS_VISION_CENTER_VELOCITY_KP_S_INV;
    s_visual_deadband_m = KNEXUS_VISION_CENTER_DEADBAND;
    s_visual_breakaway_state = VISUAL_BREAKAWAY_WAIT;
    s_visual_breakaway_tick_ms = 0U;
    s_visual_breakaway_direction = 0;
    s_desired_ball_accel_mps2 = 0.0f;
    s_gravity_feedforward_roll_deg = 0.0f;
    s_target_roll_deg = 0.0f;
    s_mode3_predicted_offset_cm = 0.0f;
    s_mode3_offset_rate_cmps = 0.0f;
    s_mode3_last_offset_cm = 0.0f;
    s_mode3_last_offset_rx_count = 0U;
    s_mode3_last_offset_ms = 0U;
    s_mode4_ball_guard_stop = 0U;
    s_mode4_speed_request_mps = 0.0f;
#if defined(KNX_PLATFORM_STM32)
    knexus_jc4310_force_zero();
#endif
}

static void enter_selected_mode(void)
{
    stop_all_actuators();
    s_active_mode = s_cursor_mode;
    if (s_active_mode == 3U || s_active_mode == 4U ||
        s_active_mode == 5U) {
        s_ball_prelevel_requested = 1U;
    }
    /* 赛题协议规定：只在确认进入模式时上报一次，不周期重发。 */
    send_mode_uplink(s_active_mode);
    if (s_active_mode == 2U || s_active_mode == 4U ||
        s_active_mode == 5U) {
        knexus_line_follow_core_init();
        if (s_active_mode == 4U) {
            knx26_line_speed_mps = KNEXUS_MENU_MODE4_LINE_SPEED_MPS;
            knexus_line_follow_core_set_longitudinal_limits(
                KNEXUS_MENU_MODE4_ACCEL_LIMIT_MPS2,
                KNEXUS_MENU_MODE4_DECEL_LIMIT_MPS2,
                KNEXUS_MENU_MODE4_JERK_LIMIT_MPS3);
            knexus_line_follow_core_set_constant_speed(true);
        } else {
            knx26_line_speed_mps = KNEXUS_H_LINE_SPEED_MPS_DEFAULT;
            knexus_line_follow_core_set_longitudinal_limits(
                KNEXUS_LINE_ACCEL_LIMIT_MPS2,
                KNEXUS_LINE_DECEL_LIMIT_MPS2,
                KNEXUS_LINE_JERK_LIMIT_MPS3);
            knexus_line_follow_core_set_constant_speed(false);
        }
        knexus_h_display_invalidate();
        if (s_active_mode == 4U) {
            knexus_h_display_set_limit_ms(
                KNEXUS_MENU_MODE4_TIME_LIMIT_MS);
        } else if (s_active_mode == 5U) {
            knexus_h_display_set_limit_ms(
                KNEXUS_MENU_MODE5_TIME_LIMIT_MS);
        } else {
            knexus_h_display_set_limit_ms(
                KNEXUS_H_SCORE_TIME_LIMIT_MS);
        }
    } else {
        memset(s_oled_cache, 0, sizeof(s_oled_cache));
        OLED_Clear();
    }
    knx_beep_beep(100U);
}

static void return_to_menu(void)
{
    stop_all_actuators();
    s_active_mode = 0U;
    s_exit_latched = 1U;
    memset(s_oled_cache, 0, sizeof(s_oled_cache));
    OLED_Clear();
    knx_beep_beep(150U);
}

static void update_menu_oled(uint32_t now_ms,
                             const knx_pendulum_comm_state_t *comm)
{
    if (now_ms - s_last_oled_ms < KNEXUS_MENU_OLED_UPDATE_MS) return;
    s_last_oled_ms = now_ms;

    /* 行驶模式继续使用赛题计时界面，避免菜单缓存覆盖计时显示。 */
    if (s_active_mode == 2U || s_active_mode == 4U ||
        s_active_mode == 5U) return;

    char line[17];
    if (s_active_mode == 0U) {
        uint8_t first = (uint8_t)(((s_cursor_mode - 1U) / 3U) * 3U + 1U);
        for (uint8_t row = 0U; row < 3U; ++row) {
            uint8_t mode = (uint8_t)(first + row);
            make_mode_line(line, mode, mode == s_cursor_mode);
            oled_line((uint8_t)(row + 1U), line);
        }
        oled_line(4U, "K0 NEXT K1 ENTER");
        return;
    }

    make_mode_line(line, s_active_mode, true);
    oled_line(1U, line);
    if (s_active_mode == 2U) {
        oled_line(2U, "K0 START K1 STOP");
        oled_line(3U, "JC4310 0 TORQUE");
    } else if (s_active_mode == 3U) {
        if (!s_mode3_running) {
            if (s_ball_control_armed == 0U ||
                fabsf(comm->offset) > KNEXUS_MENU_BALL_ERROR_LIMIT_CM) {
                oled_line(2U, "AUTO CENTER 0");
            } else {
                oled_line(2U, "READY K0 START");
            }
        } else if (s_mode3_stage == MODE3_STAGE_TO_NEGATIVE) {
            oled_line(2U, "GO -5CM");
        } else if (s_mode3_stage == MODE3_STAGE_TO_POSITIVE) {
            oled_line(2U, "GO +5CM");
        } else {
            oled_line(2U, "HOLD +5CM");
        }
        make_offset_line(line, comm->offset);
        oled_line(3U, line);
    } else {
        oled_line(2U, "UPLINK ONLY");
        oled_line(3U, "NO ACTUATION");
    }
    if (s_active_mode == 3U && s_mode3_running != 0U) {
        make_time_line(line, s_task_elapsed_ms);
        oled_line(4U, line);
    } else {
        oled_line(4U, "HOLD K0+K1 MENU");
    }
}

static void update_mode3(const knx26_context_t *context,
                         const knx_pendulum_comm_state_t *comm)
{
    (void)knx_chassis_disable();
    update_vision_fresh(context, comm);
    update_auto_prelevel(comm);

    if (knx_key_just_pressed(KNX_KEY_1)) {
        s_mode3_running = 0U;
        s_ball_control_armed = 0U;
        s_ball_prelevel_requested = 0U;
        s_mode3_stage = MODE3_STAGE_IDLE;
        s_task_completed = 0U;
        s_task_timeout = 0U;
        knx_beep_beep(100U);
    }
    if (knx_key_just_pressed(KNX_KEY_0)) {
#if defined(KNX_PLATFORM_STM32)
        if (s_vision_fresh != 0U &&
            s_ball_control_armed != 0U &&
            fabsf(comm->offset) <= KNEXUS_MENU_BALL_ERROR_LIMIT_CM &&
            knexus_jc4310_is_ready() != 0U) {
            s_mode3_running = 1U;
            s_mode3_stage = MODE3_STAGE_TO_NEGATIVE;
            s_task_completed = 0U;
            s_task_timeout = 0U;
            s_task_start_ms = context->now_ms;
            s_task_elapsed_ms = 0U;
            s_mode3_predicted_offset_cm = comm->offset;
            s_mode3_offset_rate_cmps = 0.0f;
            s_mode3_last_offset_cm = comm->offset;
            s_mode3_last_offset_rx_count = comm->rx_count;
            s_mode3_last_offset_ms = comm->last_rx_ms;
            visual_controller_reset(
                comm, KNEXUS_MENU_MODE3_NEGATIVE_TARGET_CM);
            knx_beep_beep(100U);
        } else {
            s_mode3_running = 0U;
            s_mode3_stage = MODE3_STAGE_IDLE;
            knx_beep_beep(400U);
        }
#else
        knx_beep_beep(400U);
#endif
    }

    if (s_mode3_running != 0U && s_vision_fresh == 0U) {
        s_mode3_running = 0U;
        s_ball_control_armed = 0U;
        s_ball_prelevel_requested = 0U;
        s_mode3_stage = MODE3_STAGE_IDLE;
        knx_beep_beep(400U);
    }

    if (s_mode3_running != 0U) {
        if (s_task_completed == 0U) {
            s_task_elapsed_ms = context->now_ms - s_task_start_ms;
        }
        if (comm->rx_count != s_mode3_last_offset_rx_count) {
            float offset_dt_s = s_mode3_last_offset_ms != 0U
                ? (float)(comm->last_rx_ms -
                    s_mode3_last_offset_ms) * 0.001f
                : (float)KNEXUS_APP_PERIOD_MS * 0.001f;
            offset_dt_s = clampf_local(offset_dt_s, 0.002f, 0.150f);
            float raw_offset_rate_cmps =
                (comm->offset - s_mode3_last_offset_cm) / offset_dt_s;
            s_mode3_offset_rate_cmps += 0.65f *
                (raw_offset_rate_cmps - s_mode3_offset_rate_cmps);
            s_mode3_last_offset_cm = comm->offset;
            s_mode3_last_offset_rx_count = comm->rx_count;
            s_mode3_last_offset_ms = comm->last_rx_ms;
        }
        s_mode3_predicted_offset_cm = comm->offset +
            s_mode3_offset_rate_cmps *
                KNEXUS_MENU_MODE3_NEGATIVE_BRAKE_LOOKAHEAD_S;
        bool negative_arrival = fabsf(comm->offset -
            KNEXUS_MENU_MODE3_NEGATIVE_TARGET_CM) <=
                KNEXUS_MENU_MODE3_REACH_TOLERANCE_CM;
        bool negative_brake_due =
            comm->offset <= KNEXUS_MENU_MODE3_NEGATIVE_TARGET_CM +
                KNEXUS_MENU_MODE3_NEGATIVE_BRAKE_ARM_DISTANCE_CM &&
            s_mode3_offset_rate_cmps < 0.0f &&
            s_mode3_predicted_offset_cm <=
                KNEXUS_MENU_MODE3_NEGATIVE_TARGET_CM;
        if (s_mode3_stage == MODE3_STAGE_TO_NEGATIVE &&
            (negative_arrival || negative_brake_due)) {
            s_mode3_stage = MODE3_STAGE_TO_POSITIVE;
            visual_controller_reset(
                comm, KNEXUS_MENU_MODE3_POSITIVE_TARGET_CM);
        } else if (s_mode3_stage == MODE3_STAGE_TO_POSITIVE &&
                   fabsf(comm->offset -
                         KNEXUS_MENU_MODE3_POSITIVE_TARGET_CM) <=
                       KNEXUS_MENU_MODE3_REACH_TOLERANCE_CM) {
            s_mode3_stage = MODE3_STAGE_HOLD_POSITIVE;
            s_task_completed = 1U;
            knx_beep_beep(100U);
        }
        if (s_task_completed == 0U &&
            s_task_elapsed_ms > KNEXUS_MENU_MODE3_TIME_LIMIT_MS) {
            s_task_timeout = 1U;
        }
    }

    float target_cm = 0.0f;
    if (s_mode3_stage == MODE3_STAGE_TO_NEGATIVE) {
        target_cm = KNEXUS_MENU_MODE3_NEGATIVE_TARGET_CM;
    } else if (s_mode3_stage == MODE3_STAGE_TO_POSITIVE ||
               s_mode3_stage == MODE3_STAGE_HOLD_POSITIVE) {
        target_cm = KNEXUS_MENU_MODE3_POSITIVE_TARGET_CM;
    }
    visual_controller_update(
        context, comm, target_cm,
        s_ball_control_armed != 0U, false,
        s_mode3_stage != MODE3_STAGE_HOLD_POSITIVE,
        s_mode3_stage == MODE3_STAGE_HOLD_POSITIVE
            ? KNEXUS_MENU_MODE3_HOLD_DEADBAND_CM *
                KNEXUS_VISION_CENTER_INPUT_SCALE
            : KNEXUS_VISION_CENTER_DEADBAND);
}

static void moving_task_record_ball(
    const knx_pendulum_comm_state_t *comm)
{
    if (s_vision_fresh == 0U ||
        comm->rx_count == s_task_last_ball_rx_count) return;
    s_task_last_ball_rx_count = comm->rx_count;
    float error_cm = fabsf(comm->offset);
    if (error_cm > s_task_ball_max_error_cm) {
        s_task_ball_max_error_cm = error_cm;
    }
    if (error_cm > KNEXUS_MENU_BALL_ERROR_LIMIT_CM) {
        s_task_ball_violation_count++;
    }
}

static void moving_task_finish(uint8_t mode, bool completed,
                               bool timeout)
{
    knexus_line_follow_core_force_stop();
    if (mode == 4U) s_mode4_running = 0U;
    if (mode == 5U) s_mode5_running = 0U;
    s_task_completed = completed ? 1U : 0U;
    s_task_timeout = timeout ? 1U : 0U;
    knexus_h_display_time_ms(
        s_task_elapsed_ms, false, completed);
}

static void update_moving_ball_task(
    uint8_t mode, const knx26_context_t *context,
    const knx_pendulum_comm_state_t *comm)
{
    update_vision_fresh(context, comm);
    uint8_t *running = mode == 4U
        ? &s_mode4_running : &s_mode5_running;
    if (*running == 0U && s_vision_fresh == 0U) {
        s_ball_control_armed = 0U;
    }
    update_auto_prelevel(comm);
    bool reject_start = false;

    if (knx_key_just_pressed(KNX_KEY_0)) {
#if defined(KNX_PLATFORM_STM32)
        if (s_vision_fresh != 0U &&
            s_ball_control_armed != 0U &&
            fabsf(comm->offset) <= KNEXUS_MENU_BALL_ERROR_LIMIT_CM &&
            knexus_jc4310_is_ready() != 0U &&
            knexus_jc4310_motion_comp_is_ready() != 0U) {
            *running = 1U;
            s_task_completed = 0U;
            s_task_timeout = 0U;
            s_task_start_ms = 0U;
            s_task_start_position_m = 0.0f;
            s_task_elapsed_ms = 0U;
            s_task_distance_m = 0.0f;
            s_task_ball_max_error_cm = fabsf(comm->offset);
            s_task_ball_violation_count = 0U;
            s_task_last_ball_rx_count = comm->rx_count;
            s_mode4_ball_guard_stop = 0U;
            s_mode4_speed_request_mps = mode == 4U
                ? KNEXUS_MENU_MODE4_LINE_SPEED_MPS
                : KNEXUS_H_LINE_SPEED_MPS_DEFAULT;
        } else {
            reject_start = true;
            *running = 0U;
            knx_beep_beep(400U);
        }
#else
        reject_start = true;
        knx_beep_beep(400U);
#endif
    }

    if (*running != 0U && s_vision_fresh == 0U) {
        moving_task_finish(mode, false, false);
        s_ball_control_armed = 0U;
        s_ball_prelevel_requested = 0U;
        knx_beep_beep(400U);
        reject_start = true;
    }

    if (mode == 4U) {
        s_mode4_speed_request_mps = *running != 0U &&
            s_vision_fresh != 0U
                ? mode4_speed_request()
                : KNEXUS_MENU_MODE4_LINE_SPEED_MPS;
        knx26_line_speed_mps = s_mode4_speed_request_mps;
    } else {
        knx26_line_speed_mps = KNEXUS_H_LINE_SPEED_MPS_DEFAULT;
    }

    if (!reject_start) {
        knexus_line_follow_core_update(
            (const struct knx26_context *)context);
    }

    knx26_line_state_t line_state =
        knexus_line_follow_core_get_state();
    if (*running != 0U && line_state_is_running(line_state)) {
        if (s_task_start_ms == 0U) {
            s_task_start_ms = context->now_ms;
            s_task_start_position_m =
                context->chassis.drive.position_m;
        }
        s_task_elapsed_ms =
            knexus_line_follow_core_get_elapsed_ms();
        float core_distance_m =
            knexus_line_follow_core_get_distance_m();
        float local_distance_m = fabsf(
            context->chassis.drive.position_m -
            s_task_start_position_m);
        s_task_distance_m = core_distance_m > local_distance_m
            ? core_distance_m : local_distance_m;
        moving_task_record_ball(comm);

        uint32_t limit_ms = mode == 4U
            ? KNEXUS_MENU_MODE4_TIME_LIMIT_MS
            : KNEXUS_MENU_MODE5_TIME_LIMIT_MS;
        if (mode == 4U && s_task_distance_m >=
                KNEXUS_MENU_MODE4_AB_DISTANCE_M) {
            moving_task_finish(mode, true, false);
        } else if (s_task_elapsed_ms >= limit_ms) {
            moving_task_finish(mode, false, true);
        }
    } else if (*running != 0U && line_state == KNX26_LINE_COMPLETE) {
        s_task_elapsed_ms =
            knexus_line_follow_core_get_elapsed_ms();
        s_task_distance_m =
            knexus_line_follow_core_get_distance_m();
        *running = 0U;
        s_task_completed = 1U;
        knexus_h_display_time_ms(
            s_task_elapsed_ms, false, true);
    } else if (*running != 0U && !line_state_is_running(line_state)) {
        *running = 0U;
        s_ball_control_armed = 0U;
        s_ball_prelevel_requested = 0U;
    }

    if (knx_key_just_pressed(KNX_KEY_1)) {
        *running = 0U;
        s_ball_control_armed = 0U;
        s_ball_prelevel_requested = 0U;
    }

    visual_controller_update(
        context, comm, 0.0f,
        s_ball_control_armed != 0U,
        *running != 0U,
        *running == 0U,
        KNEXUS_VISION_CENTER_DEADBAND);
}

void knexus_mode_six_menu_init(void)
{
    s_cursor_mode = 1U;
    s_active_mode = 0U;
    s_mode3_running = 0U;
    s_mode4_running = 0U;
    s_mode5_running = 0U;
    s_ball_control_armed = 0U;
    s_ball_prelevel_requested = 0U;
    s_task_completed = 0U;
    s_task_timeout = 0U;
    s_mode3_stage = MODE3_STAGE_IDLE;
    s_exit_latched = 0U;
    s_vision_fresh = 0U;
    s_last_oled_ms = 0U;
    s_task_start_ms = 0U;
    s_task_elapsed_ms = 0U;
    s_task_start_position_m = 0.0f;
    s_task_distance_m = 0.0f;
    s_task_ball_max_error_cm = 0.0f;
    s_task_ball_violation_count = 0U;
    s_task_last_ball_rx_count = 0U;
    s_last_visual_rx_count = 0U;
    s_last_visual_ms = 0U;
    s_ball_target_cm = 0.0f;
    s_visual_error = 0.0f;
    s_visual_error_rate = 0.0f;
    s_visual_error_previous = 0.0f;
    s_visual_error_integral = 0.0f;
    s_visual_i_accel_mps2 = 0.0f;
    s_desired_ball_accel_mps2 = 0.0f;
    s_gravity_feedforward_roll_deg = 0.0f;
    s_target_roll_deg = 0.0f;
    memset(s_oled_cache, 0, sizeof(s_oled_cache));

    knx_pendulum_comm_init();
#if defined(KNX_PLATFORM_STM32)
    knx_host_comm_t *host = knx_board_get_host_comm();
    knx_pendulum_comm_attach_uart(host != NULL ? host->uart : NULL);
#endif
    knexus_line_follow_core_init();
#if defined(KNX_PLATFORM_STM32)
    knexus_mode_jc4310_link_center_init();
#endif
    stop_all_actuators();
    (void)OLED_Init();
    OLED_Clear();
}

void knexus_mode_six_menu_update(const struct knx26_context *raw_context)
{
    const knx26_context_t *context = (const knx26_context_t *)raw_context;
    if (context == NULL) return;
    knx_pendulum_comm_state_t comm;
    knx_pendulum_comm_snapshot(&comm);

    bool both_pressed = knx_key_is_pressed(KNX_KEY_0) &&
                        knx_key_is_pressed(KNX_KEY_1);
    if (!both_pressed) s_exit_latched = 0U;
    if (s_active_mode != 0U && both_pressed) {
        /* Return-menu chord is also an immediate stop chord.  Do not wait
         * one second with either the chassis or link motor still active. */
        stop_all_actuators();
        if (!s_exit_latched &&
            knx_key_get_hold_ms(KNX_KEY_0) >= KNEXUS_MENU_EXIT_HOLD_MS &&
            knx_key_get_hold_ms(KNX_KEY_1) >= KNEXUS_MENU_EXIT_HOLD_MS) {
            return_to_menu();
        }
#if defined(KNX_PLATFORM_STM32)
        knexus_mode_jc4310_link_center_update(raw_context);
#endif
        update_menu_oled(context->now_ms, &comm);
        return;
    }

    if (s_active_mode == 0U) {
        stop_all_actuators();
        if (knx_key_just_pressed(KNX_KEY_0)) {
            s_cursor_mode = (uint8_t)(s_cursor_mode %
                KNEXUS_MENU_MODE_COUNT + 1U);
        } else if (knx_key_just_pressed(KNX_KEY_1)) {
            enter_selected_mode();
            update_menu_oled(context->now_ms, &comm);
            return;
        }
    } else if (s_active_mode == 2U) {
        knexus_line_follow_core_update(raw_context);
#if defined(KNX_PLATFORM_STM32)
        knexus_jc4310_force_zero();
#endif
    } else if (s_active_mode == 3U) {
        update_mode3(context, &comm);
    } else if (s_active_mode == 4U) {
        update_moving_ball_task(4U, context, &comm);
    } else if (s_active_mode == 5U) {
        update_moving_ball_task(5U, context, &comm);
    } else {
        stop_all_actuators();
    }

#if defined(KNX_PLATFORM_STM32)
    knexus_mode_jc4310_link_center_update(raw_context);
#endif
    update_menu_oled(context->now_ms, &comm);
    knx_led_set(KNX_LED_2, s_active_mode != 0U);
}

void knexus_mode_six_menu_on_intersection(
    const knx_intersection_result_t *result)
{
    if (s_active_mode == 2U || s_active_mode == 4U ||
        s_active_mode == 5U) {
        knexus_line_follow_core_on_intersection(result);
    }
}

void knexus_mode_six_menu_on_peer_message(
    const knx_comm_message_t *message)
{
    if (s_active_mode == 2U || s_active_mode == 4U ||
        s_active_mode == 5U) {
        knexus_line_follow_core_on_peer_message(message);
    }
}

void knexus_mode_six_menu_debug_octo(Octolinker_Instance_t *octo,
                                     uint16_t base_id)
{
    (void)base_id;
    if (octo == NULL) return;
    knx_pendulum_comm_state_t comm;
    knx_pendulum_comm_snapshot(&comm);
    const uint16_t id = KNEXUS_MENU_OCTO_BASE_ID;
    (void)Octolinker_SendU8(octo, id + 0U, s_cursor_mode);
    (void)Octolinker_SendU8(octo, id + 1U, s_active_mode);
    (void)Octolinker_SendU8(octo, id + 2U, s_mode3_running);
    (void)Octolinker_SendU8(octo, id + 3U, s_vision_fresh);
    (void)Octolinker_SendF32(octo, id + 4U, comm.offset);
    (void)Octolinker_SendU32(octo, id + 5U, comm.rx_count);
    (void)Octolinker_SendU32(octo, id + 6U, comm.crc_error_count);
    (void)Octolinker_SendU32(octo, id + 7U, comm.tx_count);
    (void)Octolinker_SendI32(octo, id + 8U,
                             (int32_t)comm.last_tx_status);
    (void)Octolinker_SendF32(octo, id + 9U, s_visual_error);
    (void)Octolinker_SendF32(octo, id + 10U, s_visual_error_rate);
    (void)Octolinker_SendF32(octo, id + 11U,
                             s_desired_ball_accel_mps2);
    (void)Octolinker_SendF32(octo, id + 12U, s_target_roll_deg);
    (void)Octolinker_SendU8(octo, id + 13U, s_mode5_running);
    (void)Octolinker_SendF32(octo, id + 14U,
                             s_visual_error_integral);
    (void)Octolinker_SendF32(octo, id + 15U,
                             s_visual_i_accel_mps2);
    (void)Octolinker_SendF32(octo, id + 16U,
                             s_gravity_feedforward_roll_deg);
    (void)Octolinker_SendU8(octo, id + 17U,
                            (uint8_t)s_mode3_stage);
    (void)Octolinker_SendF32(octo, id + 18U,
                             s_ball_target_cm);
    (void)Octolinker_SendU32(octo, id + 19U,
                             s_task_elapsed_ms);
    (void)Octolinker_SendF32(octo, id + 20U,
                             s_task_distance_m);
    (void)Octolinker_SendU8(octo, id + 21U,
                            s_mode4_running);
    (void)Octolinker_SendU8(octo, id + 22U,
                            s_task_completed);
    (void)Octolinker_SendU8(octo, id + 23U,
                            s_task_timeout);
    (void)Octolinker_SendF32(octo, id + 24U,
                             s_task_ball_max_error_cm);
    (void)Octolinker_SendU32(octo, id + 25U,
                             s_task_ball_violation_count);
#if defined(KNX_PLATFORM_STM32)
    (void)Octolinker_SendU8(octo, id + 26U,
                            knexus_jc4310_motion_comp_is_ready());
#else
    (void)Octolinker_SendU8(octo, id + 26U, 0U);
#endif
    (void)Octolinker_SendU8(octo, id + 27U,
                            s_ball_control_armed);
    (void)Octolinker_SendU8(octo, id + 28U,
                            s_ball_prelevel_requested);
#if defined(KNX_PLATFORM_STM32)
    (void)Octolinker_SendU8(octo, id + 29U,
                            knexus_jc4310_is_settled());
#else
    (void)Octolinker_SendU8(octo, id + 29U, 0U);
#endif
    (void)Octolinker_SendF32(octo, id + 30U,
                             s_visual_breakaway_accel_mps2);
    (void)Octolinker_SendF32(octo, id + 31U,
                             s_mode3_predicted_offset_cm);
    (void)Octolinker_SendU8(octo, id + 32U,
                            s_last_uplink_mode);
    (void)Octolinker_SendF32(octo, id + 33U,
                             s_mode3_offset_rate_cmps);
    (void)Octolinker_SendF32(octo, id + 34U,
                             s_visual_target_velocity_mps);
    (void)Octolinker_SendF32(octo, id + 35U,
                             s_visual_velocity_error_mps);
    (void)Octolinker_SendU8(octo, id + 36U,
                            (uint8_t)s_visual_breakaway_state);
    (void)Octolinker_SendU8(octo, id + 37U,
                            s_mode4_ball_guard_stop);
    (void)Octolinker_SendF32(octo, id + 38U,
                             s_mode4_speed_request_mps);
    (void)Octolinker_SendF32(octo, id + 39U,
                             s_visual_position_to_speed_gain);
    (void)Octolinker_SendF32(octo, id + 40U,
                             s_visual_velocity_kp_gain);
    (void)Octolinker_SendF32(octo, id + 41U,
                             s_visual_deadband_m * 100.0f);
    if (s_active_mode == 2U || s_active_mode == 4U ||
        s_active_mode == 5U) {
        knexus_line_follow_core_debug_octo(octo, base_id);
    }
#if defined(KNX_PLATFORM_STM32)
    knexus_mode_jc4310_link_center_debug_octo(octo, base_id);
#endif
}

void knexus_mode_six_menu_debug_control_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    if (s_active_mode == 2U || s_active_mode == 4U ||
        s_active_mode == 5U) {
        knexus_line_follow_core_debug_control_octo(octo, base_id);
    }
#if defined(KNX_PLATFORM_STM32)
    knexus_mode_jc4310_link_center_debug_control_octo(octo, base_id);
#else
    (void)octo;
    (void)base_id;
#endif
}

#endif
