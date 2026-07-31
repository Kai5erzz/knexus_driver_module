#include "knexus_mode_backend.h"
#include "knexus_config.h"
#include "knx26_app.h"
#include "knx_beep.h"
#include "knx_chassis.h"
#include "knx_dji_motor_ctrl.h"
#include "knx_key.h"
#include "knx_rod_angle_ctrl.h"
#include "knx_rod_runtime.h"
#include "knx_time.h"
#include <math.h>

#if defined(KNX_PLATFORM_STM32)
#include "knx_pendulum_comm.h"
#endif

typedef enum {
    OFFSET_STOP_IDLE = 0,
    OFFSET_STOP_ACTIVE = 1,
    OFFSET_STOP_KEY1 = 2,
    OFFSET_STOP_NO_COMM = 3,
    OFFSET_STOP_COMM_STALE = 4,
    OFFSET_STOP_ROD_NOT_READY = 5,
    OFFSET_STOP_ROD_FAULT = 6,
    OFFSET_STOP_UNSUPPORTED = 7,
} offset_stop_reason_t;

typedef enum {
    OFFSET_HOLD_MONITOR = 0,
    OFFSET_HOLD_BREAKAWAY = 1,
    OFFSET_HOLD_COOLDOWN = 2,
} offset_hold_state_t;

float knexus_offset_input_scale = KNEXUS_OFFSET_INPUT_SCALE_DEFAULT;
float knexus_offset_input_sign = KNEXUS_OFFSET_INPUT_SIGN_DEFAULT;
float knexus_offset_kp = KNEXUS_OFFSET_KP_DEFAULT;
float knexus_offset_ki = KNEXUS_OFFSET_KI_DEFAULT;
float knexus_offset_kd = KNEXUS_OFFSET_KD_DEFAULT;
float knexus_offset_d_lpf_hz = KNEXUS_OFFSET_D_LPF_HZ_DEFAULT;
float knexus_offset_d_limit_deg = KNEXUS_OFFSET_D_LIMIT_DEG_DEFAULT;
float knexus_offset_static_tilt_deg =
    KNEXUS_OFFSET_STATIC_TILT_DEG_DEFAULT;
float knexus_offset_static_tilt_enter_error =
    KNEXUS_OFFSET_STATIC_TILT_ENTER_ERROR_DEFAULT;
float knexus_offset_static_tilt_full_error =
    KNEXUS_OFFSET_STATIC_TILT_FULL_ERROR_DEFAULT;
float knexus_offset_static_tilt_max_velocity =
    KNEXUS_OFFSET_STATIC_TILT_MAX_VELOCITY_DEFAULT;
float knexus_offset_near_kp_boost =
    KNEXUS_OFFSET_NEAR_KP_BOOST_DEFAULT;
float knexus_offset_near_kp_zone =
    KNEXUS_OFFSET_NEAR_KP_ZONE_DEFAULT;
float knexus_offset_hold_enter_error =
    KNEXUS_OFFSET_HOLD_ENTER_ERROR_DEFAULT;
float knexus_offset_hold_release_delta =
    KNEXUS_OFFSET_HOLD_RELEASE_DELTA_DEFAULT;
float knexus_offset_hold_ramp_dps =
    KNEXUS_OFFSET_HOLD_RAMP_DPS_DEFAULT;
float knexus_offset_hold_limit_deg =
    KNEXUS_OFFSET_HOLD_LIMIT_DEG_DEFAULT;
float knexus_offset_hold_full_error =
    KNEXUS_OFFSET_HOLD_FULL_ERROR_DEFAULT;
float knexus_offset_integral_limit_deg =
    KNEXUS_OFFSET_INTEGRAL_LIMIT_DEG_DEFAULT;
float knexus_offset_integral_zone = KNEXUS_OFFSET_INTEGRAL_ZONE_DEFAULT;
float knexus_offset_deadband = KNEXUS_OFFSET_DEADBAND_DEFAULT;
float knexus_offset_target_min_deg = KNEXUS_OFFSET_TARGET_ROLL_MIN_DEG;
float knexus_offset_target_max_deg = KNEXUS_OFFSET_TARGET_ROLL_MAX_DEG;
float knexus_offset_target_slew_dps =
    KNEXUS_OFFSET_TARGET_ROLL_SLEW_DPS;

static uint8_t s_active;
static uint8_t s_platform_supported;
static uint8_t s_comm_fresh;
static uint8_t s_last_blocked;
static uint32_t s_key0_count;
static uint32_t s_limit_block_count;
static uint32_t s_rod_fault_since_ms;
static uint32_t s_rod_fault_dwell_ms;
static uint32_t s_vision_invalid_count;
static uint32_t s_vision_invalid_since_ms;
static uint32_t s_vision_invalid_dwell_ms;
static offset_stop_reason_t s_stop_reason;
static uint8_t s_vision_valid;
static float s_raw_offset;
static float s_error;
static float s_p_deg;
static float s_i_deg;
static float s_d_deg;
static float s_derivative_filtered;
static float s_last_error;
static float s_raw_target_deg;
static float s_target_deg;
static float s_last_valid_target_deg;
static uint8_t s_vision_recovery_active;
static uint8_t s_static_tilt_active;
static float s_static_tilt_factor;
static float s_effective_kp;
static uint8_t s_hold_boost_active;
static float s_center_progress_rate;
static offset_hold_state_t s_hold_state;
static float s_hold_anchor_error;
static float s_hold_dwell_s;
static float s_hold_cooldown_s;
static float s_hold_dynamic_limit_deg;
static uint8_t s_pid_started;

static float clampf(float value, float lower, float upper)
{
    if (lower > upper) {
        float tmp = lower;
        lower = upper;
        upper = tmp;
    }
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static float clamp_abs(float value, float limit)
{
    limit = fabsf(limit);
    return clampf(value, -limit, limit);
}

static float slew_target_to(float target_deg, float slew_dps, float dt_s)
{
    float max_step = fabsf(slew_dps) * dt_s;
    float step = clamp_abs(target_deg - s_target_deg, max_step);
    s_target_deg += step;
    s_target_deg = clampf(s_target_deg,
                          knexus_offset_target_min_deg,
                          knexus_offset_target_max_deg);
    return s_target_deg;
}

static void offset_pid_reset(float initial_target_deg)
{
    s_error = 0.0f;
    s_p_deg = 0.0f;
    s_i_deg = 0.0f;
    s_d_deg = 0.0f;
    s_derivative_filtered = 0.0f;
    s_last_error = 0.0f;
    s_raw_target_deg = initial_target_deg;
    s_target_deg = clampf(initial_target_deg,
                            knexus_offset_target_min_deg,
                            knexus_offset_target_max_deg);
    s_static_tilt_active = 0U;
    s_static_tilt_factor = 0.0f;
    s_effective_kp = knexus_offset_kp;
    s_hold_boost_active = 0U;
    s_center_progress_rate = 0.0f;
    s_hold_state = OFFSET_HOLD_MONITOR;
    s_hold_anchor_error = 0.0f;
    s_hold_dwell_s = 0.0f;
    s_hold_cooldown_s = 0.0f;
    s_hold_dynamic_limit_deg = 0.0f;
    s_pid_started = 0U;
}

static float offset_pid_update(float offset, float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.01f;
    s_error = offset * knexus_offset_input_scale *
              knexus_offset_input_sign;
    if (fabsf(s_error) <= fabsf(knexus_offset_deadband)) s_error = 0.0f;

    float derivative = 0.0f;
    if (s_pid_started != 0U) derivative = (s_error - s_last_error) / dt_s;
    else s_pid_started = 1U;
    s_last_error = s_error;

    float cutoff_hz = fabsf(knexus_offset_d_lpf_hz);
    float alpha = 1.0f;
    if (cutoff_hz > 0.01f) {
        float tau_s = 1.0f / (6.28318530718f * cutoff_hz);
        alpha = dt_s / (tau_s + dt_s);
    }
    s_derivative_filtered +=
        alpha * (derivative - s_derivative_filtered);

    float near_zone = fabsf(knexus_offset_near_kp_zone);
    float near_factor = near_zone > 0.01f
        ? clampf(1.0f - fabsf(s_error) / near_zone, 0.0f, 1.0f)
        : 0.0f;
    s_effective_kp = knexus_offset_kp +
                     fabsf(knexus_offset_near_kp_boost) * near_factor;
    s_p_deg = s_effective_kp * s_error;
    float static_enter = fabsf(knexus_offset_static_tilt_enter_error);
    float static_full = fabsf(knexus_offset_static_tilt_full_error);
    if (static_full < static_enter + 0.01f)
        static_full = static_enter + 0.01f;
    float error_factor = clampf(
        (fabsf(s_error) - static_enter) /
            (static_full - static_enter), 0.0f, 1.0f);
    float velocity_limit =
        fabsf(knexus_offset_static_tilt_max_velocity);
    float velocity_factor = velocity_limit > 0.01f
        ? clampf(1.0f - fabsf(s_derivative_filtered) / velocity_limit,
                 0.0f, 1.0f)
        : 0.0f;
    s_static_tilt_factor = error_factor * velocity_factor;
    s_static_tilt_active = s_static_tilt_factor > 0.001f ? 1U : 0U;
    if (s_static_tilt_active != 0U && s_error > 0.0f)
        s_p_deg += fabsf(knexus_offset_static_tilt_deg) *
                   s_static_tilt_factor;
    else if (s_static_tilt_active != 0U && s_error < 0.0f)
        s_p_deg -= fabsf(knexus_offset_static_tilt_deg) *
                   s_static_tilt_factor;
    /* 视觉偏移在画面边界会从约+700跳到-700。速度反馈需要足够强才能
     * 提前刹球，但必须单独限幅，防止识别跳变把杆角瞬间打到机械极限。 */
    s_d_deg = clamp_abs(knexus_offset_kd * s_derivative_filtered,
                         knexus_offset_d_limit_deg);

    if (s_error > 0.0f)
        s_center_progress_rate = -s_derivative_filtered;
    else if (s_error < 0.0f)
        s_center_progress_rate = s_derivative_filtered;
    else
        s_center_progress_rate = 0.0f;
    /* 防卡滞不能按单帧速度直接累加。视觉量化、钢球惯性和杆角执行延迟会让
     * “回中速度不足”在正常运动中频繁成立，旧逻辑因此有一半以上时间都在
     * 追加恢复角。这里改成三阶段状态机：位置在一个观察窗内基本不变才进入
     * BREAKAWAY；一旦检测到真实位移，立即清掉恢复角并进入冷却，交还给PD
     * 提前制动。这样仍能越过不同位置的静摩擦，又不会把能量带过零点。 */
    const float release_delta =
        fabsf(knexus_offset_hold_release_delta);
    const float detect_s =
        (float)KNEXUS_OFFSET_HOLD_DETECT_MS * 0.001f;
    const float cooldown_s =
        (float)KNEXUS_OFFSET_HOLD_COOLDOWN_MS * 0.001f;
    bool hold_eligible =
        s_error != 0.0f &&
        fabsf(s_error) >= fabsf(knexus_offset_hold_enter_error);
    float hold_enter = fabsf(knexus_offset_hold_enter_error);
    float hold_full = fabsf(knexus_offset_hold_full_error);
    if (hold_full < hold_enter + 0.01f)
        hold_full = hold_enter + 0.01f;
    float hold_limit_factor = hold_eligible
        ? clampf((fabsf(s_error) - hold_enter) /
                     (hold_full - hold_enter),
                 0.0f, 1.0f)
        : 0.0f;
    s_hold_dynamic_limit_deg =
        fabsf(knexus_offset_hold_limit_deg) * hold_limit_factor;
    float previous_i = s_i_deg;

    if (!hold_eligible) {
        s_hold_state = OFFSET_HOLD_MONITOR;
        s_hold_anchor_error = s_error;
        s_hold_dwell_s = 0.0f;
        s_hold_cooldown_s = 0.0f;
        s_hold_boost_active = 0U;
        s_i_deg = 0.0f;
        previous_i = 0.0f;
    } else if (s_hold_state == OFFSET_HOLD_MONITOR) {
        s_hold_boost_active = 0U;
        s_i_deg = 0.0f;
        previous_i = 0.0f;
        bool direction_changed =
            s_hold_anchor_error * s_error <= 0.0f;
        bool position_changed =
            fabsf(s_error - s_hold_anchor_error) >= release_delta;
        if (s_hold_dwell_s <= 0.0f || direction_changed ||
            position_changed) {
            s_hold_anchor_error = s_error;
            s_hold_dwell_s = dt_s;
        } else {
            s_hold_dwell_s += dt_s;
        }
        if (s_hold_dwell_s >= detect_s) {
            s_hold_state = OFFSET_HOLD_BREAKAWAY;
            s_hold_anchor_error = s_error;
            s_hold_dwell_s = detect_s;
        }
    } else if (s_hold_state == OFFSET_HOLD_BREAKAWAY) {
        bool direction_changed =
            s_hold_anchor_error * s_error <= 0.0f;
        bool ball_released =
            fabsf(s_error - s_hold_anchor_error) >= release_delta;
        if (direction_changed || ball_released) {
            s_hold_state = OFFSET_HOLD_COOLDOWN;
            s_hold_boost_active = 0U;
            s_hold_cooldown_s = 0.0f;
            s_i_deg = 0.0f;
            previous_i = 0.0f;
        } else {
            s_hold_boost_active = 1U;
            float ramp = fabsf(knexus_offset_hold_ramp_dps) * dt_s;
            s_i_deg += s_error > 0.0f ? ramp : -ramp;
            s_i_deg = clamp_abs(s_i_deg,
                                s_hold_dynamic_limit_deg);
        }
    } else {
        s_hold_boost_active = 0U;
        s_i_deg = 0.0f;
        previous_i = 0.0f;
        s_hold_cooldown_s += dt_s;
        if (s_hold_cooldown_s >= cooldown_s) {
            s_hold_state = OFFSET_HOLD_MONITOR;
            s_hold_anchor_error = s_error;
            s_hold_dwell_s = 0.0f;
            s_hold_cooldown_s = 0.0f;
        }
    }

    float unclamped = s_p_deg + s_i_deg + s_d_deg;
    s_raw_target_deg = clampf(unclamped,
                              knexus_offset_target_min_deg,
                              knexus_offset_target_max_deg);
    if (s_raw_target_deg != unclamped &&
        s_error * (unclamped - s_raw_target_deg) > 0.0f) {
        s_i_deg = previous_i;
        unclamped = s_p_deg + s_i_deg + s_d_deg;
        s_raw_target_deg = clampf(unclamped,
                                  knexus_offset_target_min_deg,
                                  knexus_offset_target_max_deg);
    }

    return slew_target_to(s_raw_target_deg,
                          knexus_offset_target_slew_dps, dt_s);
}

static float offset_invalid_update(uint32_t invalid_dwell_ms, float dt_s)
{
    s_error = 0.0f;
    s_p_deg = 0.0f;
    s_i_deg = 0.0f;
    s_d_deg = 0.0f;
    s_derivative_filtered = 0.0f;
    s_last_error = 0.0f;
    s_raw_target_deg = 0.0f;
    s_pid_started = 0U;
    s_static_tilt_active = 0U;
    s_static_tilt_factor = 0.0f;
    s_hold_boost_active = 0U;
    s_center_progress_rate = 0.0f;
    s_hold_state = OFFSET_HOLD_MONITOR;
    s_hold_anchor_error = 0.0f;
    s_hold_dwell_s = 0.0f;
    s_hold_cooldown_s = 0.0f;
    s_hold_dynamic_limit_deg = 0.0f;
    if (invalid_dwell_ms <= KNEXUS_OFFSET_INVALID_RECOVERY_HOLD_MS) {
        s_vision_recovery_active = 1U;
        s_raw_target_deg = clamp_abs(
            s_last_valid_target_deg,
            KNEXUS_OFFSET_INVALID_RECOVERY_MAX_DEG);
        return slew_target_to(s_raw_target_deg,
                              KNEXUS_OFFSET_TARGET_ROLL_SLEW_DPS, dt_s);
    }
    s_vision_recovery_active = 0U;
    return slew_target_to(0.0f,
                          KNEXUS_OFFSET_INVALID_LEVEL_SLEW_DPS, dt_s);
}

static void offset_stop(offset_stop_reason_t reason, uint16_t beep_ms)
{
    s_active = 0U;
    s_stop_reason = reason;
    offset_pid_reset(0.0f);
    knx_rod_runtime_stop();
    (void)knx_chassis_disable();
    if (beep_ms != 0U) knx_beep_beep(beep_ms);
}

void knexus_mode_screw_offset_center_init(void)
{
    s_active = 0U;
    s_comm_fresh = 0U;
    s_last_blocked = 0U;
    s_key0_count = 0U;
    s_limit_block_count = 0U;
    s_rod_fault_since_ms = 0U;
    s_rod_fault_dwell_ms = 0U;
    s_vision_invalid_count = 0U;
    s_vision_invalid_since_ms = 0U;
    s_vision_invalid_dwell_ms = 0U;
    s_stop_reason = OFFSET_STOP_IDLE;
    s_vision_valid = 0U;
    s_raw_offset = 0.0f;
    s_last_valid_target_deg = 0.0f;
    s_vision_recovery_active = 0U;
    s_static_tilt_active = 0U;
    s_static_tilt_factor = 0.0f;
    s_hold_boost_active = 0U;
    s_center_progress_rate = 0.0f;
    offset_pid_reset(0.0f);
    (void)knx_chassis_disable();

#if defined(KNX_PLATFORM_STM32)
    s_platform_supported = 1U;
    knx_pendulum_comm_init();
    knexus_rod_angle_kp = KNEXUS_OFFSET_ROD_ANGLE_KP;
    knexus_rod_angle_ki = KNEXUS_OFFSET_ROD_ANGLE_KI;
    knexus_rod_angle_kd = KNEXUS_OFFSET_ROD_ANGLE_KD;
    knexus_rod_max_rpm = KNEXUS_OFFSET_ROD_MAX_RPM;
    knexus_rod_rpm_slew_rpmps = KNEXUS_OFFSET_ROD_RPM_SLEW_RPMPS;
    knexus_rod_deadband_deg = KNEXUS_OFFSET_ROD_DEADBAND_DEG;
    knexus_rod_rate_deadband_dps =
        KNEXUS_OFFSET_ROD_RATE_DEADBAND_DPS;
    (void)knx_rod_runtime_init();
    (void)knx_dji_motor_ctrl_set_speed_pid(
        KNEXUS_OFFSET_DJI_SPEED_KP, KNEXUS_OFFSET_DJI_SPEED_KI,
        KNEXUS_OFFSET_DJI_SPEED_KD,
        KNEXUS_OFFSET_DJI_SPEED_I_LIMIT,
        KNEXUS_OFFSET_DJI_MAX_CURRENT_CMD);
    (void)knx_dji_motor_ctrl_set_current_feedforward(
        KNEXUS_OFFSET_DJI_CURRENT_FF_CMD,
        KNEXUS_OFFSET_DJI_CURRENT_FF_MIN_RPM);
    (void)knx_dji_motor_ctrl_set_target_slew(
        KNEXUS_OFFSET_DJI_TARGET_SLEW_RPMPS);
#else
    s_platform_supported = 0U;
    s_stop_reason = OFFSET_STOP_UNSUPPORTED;
#endif
}

void knexus_mode_screw_offset_center_update(
    const struct knx26_context *raw_context)
{
    const knx26_context_t *context = (const knx26_context_t *)raw_context;
    if (context == NULL) return;
    (void)knx_chassis_disable();

#if !defined(KNX_PLATFORM_STM32)
    (void)context;
    return;
#else
    const float dt_s = (float)KNEXUS_APP_PERIOD_MS * 0.001f;
    knx_pendulum_comm_state_t comm;
    knx_pendulum_comm_snapshot(&comm);
    uint32_t age_ms = comm.rx_count == 0U
        ? 0xFFFFFFFFU : context->now_ms - comm.last_rx_ms;
    s_raw_offset = comm.offset;
    s_comm_fresh = comm.rx_count != 0U &&
                   age_ms <= KNEXUS_OFFSET_TIMEOUT_MS;

    knx_rod_runtime_state_t rod;
    knx_rod_runtime_snapshot(&rod);
    if (s_active == 0U) {
        knx_rod_runtime_update(
            0.0f, 0.0f, false,
            KNEXUS_OFFSET_ROD_SOFT_LIMIT_ENABLE != 0U, dt_s);
        knx_rod_runtime_snapshot(&rod);
    }

    if (knx_key_just_pressed(KNX_KEY_1)) {
        offset_stop(OFFSET_STOP_KEY1, 100U);
        return;
    }

    if (knx_key_just_pressed(KNX_KEY_0)) {
        ++s_key0_count;
        if (s_comm_fresh == 0U) {
            offset_stop(OFFSET_STOP_NO_COMM, 400U);
            return;
        }
        if (!rod.control_ready) {
            offset_stop(OFFSET_STOP_ROD_NOT_READY, 400U);
            return;
        }
        s_active = 1U;
        s_stop_reason = OFFSET_STOP_ACTIVE;
        s_rod_fault_since_ms = 0U;
        s_rod_fault_dwell_ms = 0U;
        offset_pid_reset(rod.roll_deg);
        knx_beep_beep(100U);
    }

    if (s_active == 0U) return;
    if (s_comm_fresh == 0U) {
        offset_stop(OFFSET_STOP_COMM_STALE, 400U);
        return;
    }

    uint8_t vision_valid =
        (s_raw_offset > KNEXUS_OFFSET_VISION_INVALID_LOW_RAW &&
         s_raw_offset < KNEXUS_OFFSET_VISION_INVALID_HIGH_RAW) ? 1U : 0U;
    if (vision_valid == 0U) {
        if (s_vision_valid != 0U || s_vision_invalid_since_ms == 0U) {
            ++s_vision_invalid_count;
            s_vision_invalid_since_ms = context->now_ms;
        }
        s_vision_invalid_dwell_ms =
            context->now_ms - s_vision_invalid_since_ms;
    } else {
        s_vision_invalid_since_ms = 0U;
        s_vision_invalid_dwell_ms = 0U;
        if (s_vision_valid == 0U) s_pid_started = 0U;
    }
    s_vision_valid = vision_valid;

    float target_deg;
    if (vision_valid != 0U) {
        target_deg = offset_pid_update(s_raw_offset, dt_s);
        s_last_valid_target_deg = target_deg;
        s_vision_recovery_active = 0U;
    } else {
        target_deg = offset_invalid_update(
            s_vision_invalid_dwell_ms, dt_s);
    }
    knx_rod_runtime_update(
        target_deg, 0.0f, true,
        KNEXUS_OFFSET_ROD_SOFT_LIMIT_ENABLE != 0U, dt_s);
    knx_rod_runtime_snapshot(&rod);
    if (!rod.control_ready) {
        if (s_rod_fault_since_ms == 0U)
            s_rod_fault_since_ms = context->now_ms;
        s_rod_fault_dwell_ms =
            context->now_ms - s_rod_fault_since_ms;
        if (s_rod_fault_dwell_ms >=
            KNEXUS_OFFSET_ROD_FAULT_HOLD_MS) {
            offset_stop(OFFSET_STOP_ROD_FAULT, 500U);
        }
        return;
    }
    s_rod_fault_since_ms = 0U;
    s_rod_fault_dwell_ms = 0U;

    uint8_t blocked = rod.blocked_direction != 0 ? 1U : 0U;
    if (blocked != 0U) {
        s_i_deg = 0.0f;
        if (s_last_blocked == 0U) ++s_limit_block_count;
    }
    s_last_blocked = blocked;
#endif
}

void knexus_mode_screw_offset_center_on_intersection(
    const knx_intersection_result_t *result)
{
    (void)result;
}

void knexus_mode_screw_offset_center_on_peer_message(
    const knx_comm_message_t *message)
{
    (void)message;
}

void knexus_mode_screw_offset_center_debug_control_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    (void)base_id;
    if (octo == NULL) return;
    const uint16_t id = KNEXUS_OFFSET_OCTO_BASE_ID;
    knx_rod_runtime_state_t rod;
    knx_dji_motor_state_t dji;
    knx_rod_runtime_snapshot(&rod);
    knx_dji_motor_ctrl_snapshot(&dji);

    (void)Octolinker_SendU8(octo, id + 0U, s_active);
    (void)Octolinker_SendU8(octo, id + 1U, s_comm_fresh);
    (void)Octolinker_SendF32(octo, id + 2U, s_raw_offset);
    (void)Octolinker_SendF32(octo, id + 3U, s_error);
    (void)Octolinker_SendF32(octo, id + 4U, s_p_deg);
    (void)Octolinker_SendF32(octo, id + 5U, s_i_deg);
    (void)Octolinker_SendF32(octo, id + 6U, s_d_deg);
    (void)Octolinker_SendF32(octo, id + 7U, s_raw_target_deg);
    (void)Octolinker_SendF32(octo, id + 8U, s_target_deg);
    (void)Octolinker_SendF32(octo, id + 9U, rod.roll_deg);
    (void)Octolinker_SendF32(octo, id + 10U, rod.roll_rate_dps);
    (void)Octolinker_SendF32(octo, id + 11U, dji.target_used_rpm);
    (void)Octolinker_SendF32(octo, id + 12U, dji.speed_rpm);
    (void)Octolinker_SendU8(octo, id + 13U,
                            (uint8_t)rod.control_ready);
    (void)Octolinker_SendU8(octo, id + 14U,
                            (uint8_t)rod.motor_enabled);
    (void)Octolinker_SendU8(octo, id + 15U,
                            (uint8_t)rod.upper_limit_active);
    (void)Octolinker_SendU8(octo, id + 16U,
                            (uint8_t)rod.lower_limit_active);
    (void)Octolinker_SendI32(octo, id + 17U,
                             (int32_t)rod.blocked_direction);
    (void)Octolinker_SendI32(octo, id + 18U,
                             (int32_t)s_stop_reason);
}

void knexus_mode_screw_offset_center_debug_octo(
    Octolinker_Instance_t *octo, uint16_t base_id)
{
    (void)base_id;
    if (octo == NULL) return;
    const uint16_t id = KNEXUS_OFFSET_OCTO_BASE_ID;
#if defined(KNX_PLATFORM_STM32)
    knx_pendulum_comm_state_t comm;
    knx_pendulum_comm_snapshot(&comm);
    uint32_t age_ms = comm.rx_count == 0U
        ? 0xFFFFFFFFU : knx_millis() - comm.last_rx_ms;
    (void)Octolinker_SendU32(octo, id + 19U, comm.rx_count);
    (void)Octolinker_SendU32(octo, id + 20U,
                             comm.crc_error_count);
    (void)Octolinker_SendU32(octo, id + 21U,
                             comm.invalid_value_count);
    (void)Octolinker_SendU32(octo, id + 22U,
                             comm.sync_loss_count);
    (void)Octolinker_SendU32(octo, id + 23U, age_ms);
#else
    for (uint16_t n = 19U; n <= 23U; ++n)
        (void)Octolinker_SendU32(octo, id + n, 0U);
#endif
    (void)Octolinker_SendU8(octo, id + 24U,
                            s_platform_supported);
    (void)Octolinker_SendU32(octo, id + 25U, s_key0_count);
    (void)Octolinker_SendU32(octo, id + 26U,
                             s_limit_block_count);
    (void)Octolinker_SendF32(octo, id + 27U, knexus_offset_kp);
    (void)Octolinker_SendF32(octo, id + 28U, knexus_offset_ki);
    (void)Octolinker_SendF32(octo, id + 29U, knexus_offset_kd);
    (void)Octolinker_SendF32(octo, id + 30U,
                             knexus_offset_input_sign);
    (void)Octolinker_SendF32(octo, id + 31U,
                             knexus_offset_input_scale);
    (void)Octolinker_SendF32(octo, id + 32U,
                             knexus_offset_target_min_deg);
    (void)Octolinker_SendF32(octo, id + 33U,
                             knexus_offset_target_max_deg);
    knx_rod_runtime_state_t rod;
    knx_dji_motor_state_t dji;
    knx_rod_runtime_snapshot(&rod);
    knx_dji_motor_ctrl_snapshot(&dji);
    (void)Octolinker_SendU8(octo, id + 34U,
                            (uint8_t)rod.tilt_valid);
    (void)Octolinker_SendU8(octo, id + 35U,
                            (uint8_t)rod.motor_feedback_valid);
    (void)Octolinker_SendU32(octo, id + 36U,
                             dji.feedback_age_ms);
    (void)Octolinker_SendU32(octo, id + 37U,
                             s_rod_fault_dwell_ms);
    (void)Octolinker_SendI32(octo, id + 38U,
                             (int32_t)dji.pid_output);
    (void)Octolinker_SendF32(octo, id + 39U,
                             s_derivative_filtered);
    (void)Octolinker_SendF32(octo, id + 40U,
                             knexus_offset_d_limit_deg);
    (void)Octolinker_SendF32(octo, id + 41U,
                             knexus_offset_static_tilt_deg);
    (void)Octolinker_SendF32(octo, id + 42U,
                             knexus_offset_static_tilt_enter_error);
    (void)Octolinker_SendU8(octo, id + 43U, s_vision_valid);
    (void)Octolinker_SendU32(octo, id + 44U,
                             s_vision_invalid_count);
    (void)Octolinker_SendU32(octo, id + 45U,
                             s_vision_invalid_dwell_ms);
    (void)Octolinker_SendF32(octo, id + 46U,
                             s_last_valid_target_deg);
    (void)Octolinker_SendU8(octo, id + 47U,
                            s_vision_recovery_active);
    (void)Octolinker_SendF32(octo, id + 48U,
                             knexus_offset_static_tilt_max_velocity);
    (void)Octolinker_SendU8(octo, id + 49U,
                            s_static_tilt_active);
    (void)Octolinker_SendF32(octo, id + 50U,
                             knexus_offset_static_tilt_full_error);
    (void)Octolinker_SendF32(octo, id + 51U,
                             s_static_tilt_factor);
    (void)Octolinker_SendF32(octo, id + 52U,
                             s_effective_kp);
    (void)Octolinker_SendU8(octo, id + 53U,
                            s_hold_boost_active);
    (void)Octolinker_SendF32(octo, id + 54U,
                             knexus_offset_hold_limit_deg);
    (void)Octolinker_SendF32(octo, id + 55U,
                             knexus_offset_hold_ramp_dps);
    (void)Octolinker_SendF32(octo, id + 56U,
                             s_center_progress_rate);
    (void)Octolinker_SendF32(octo, id + 57U,
                             knexus_offset_hold_release_delta);
    (void)Octolinker_SendU8(octo, id + 58U,
                            (uint8_t)s_hold_state);
    (void)Octolinker_SendF32(octo, id + 59U,
                             s_hold_dwell_s * 1000.0f);
    (void)Octolinker_SendF32(octo, id + 60U,
                             s_hold_anchor_error);
    (void)Octolinker_SendF32(octo, id + 61U,
                             s_hold_cooldown_s * 1000.0f);
    (void)Octolinker_SendF32(octo, id + 62U,
                             s_hold_dynamic_limit_deg);
    (void)Octolinker_SendF32(octo, id + 63U,
                             knexus_offset_hold_full_error);
}
