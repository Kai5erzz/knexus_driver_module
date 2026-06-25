#include "knx_gimbal_ctrl.h"
#include "dm_imu_l1.h"
#include "jc_driver.h"
#include "knx_control.h"
#include "knx_health.h"
#include "knx_param.h"
#include "knx_time.h"
#include "knx_vision.h"

#define KNX_GIMBAL_CTRL_PITCH_JC2804_ID 1U
#define KNX_GIMBAL_CTRL_YAW_JC4310_ID   3U

float knx_gimbal_ctrl_enable = 1.0f;
float knx_gimbal_ctrl_boot_delay_ms = 500.0f;
float knx_gimbal_ctrl_mode_gap_ms = 300.0f;
float knx_gimbal_ctrl_control_period_ms = 10.0f;
float knx_gimbal_ctrl_vision_timeout_ms = 200.0f;
float knx_gimbal_ctrl_min_conf = 0.15f;

float knx_gimbal_ctrl_yaw_kp = 0.25f;
float knx_gimbal_ctrl_yaw_ki = 0.02f;
float knx_gimbal_ctrl_yaw_kd = 0.01f;
float knx_gimbal_ctrl_pitch_kp = 0.1f;
float knx_gimbal_ctrl_pitch_ki = 0.005f;
float knx_gimbal_ctrl_pitch_kd = 0.01f;



float knx_gimbal_ctrl_hold_yaw_kp = 0.2f;
float knx_gimbal_ctrl_hold_yaw_ki = 0.0f;
float knx_gimbal_ctrl_hold_yaw_kd = 0.0f;
float knx_gimbal_ctrl_hold_pitch_kp = 0.2f;
float knx_gimbal_ctrl_hold_pitch_ki = 0.0f;
float knx_gimbal_ctrl_hold_pitch_kd = 0.0f;

float knx_gimbal_ctrl_output_limit_rpm = 20.0f;
float knx_gimbal_ctrl_hold_output_limit_rpm = 15.0f;
float knx_gimbal_ctrl_integral_limit = 1000.0f;
float knx_gimbal_ctrl_yaw_target_deg = 0.0f;
float knx_gimbal_ctrl_pitch_target_deg = 0.0f;
float knx_gimbal_ctrl_yaw_limit_deg = 0.0f;
float knx_gimbal_ctrl_pitch_limit_deg = 60.0f;
float knx_gimbal_ctrl_stop_period_ms = 20.0f;
float knx_gimbal_ctrl_yaw_speed_to_angle_sign = 1.0f;
float knx_gimbal_ctrl_pitch_speed_to_angle_sign = -1.0f;

float knx_gimbal_ctrl_state = 0.0f;
float knx_gimbal_ctrl_imu_yaw_deg = 0.0f;
float knx_gimbal_ctrl_imu_pitch_deg = 0.0f;
float knx_gimbal_ctrl_yaw_hold_error_deg = 0.0f;
float knx_gimbal_ctrl_pitch_hold_error_deg = 0.0f;
float knx_gimbal_ctrl_yaw_error_px = 0.0f;
float knx_gimbal_ctrl_pitch_error_px = 0.0f;
float knx_gimbal_ctrl_yaw_cmd_rpm = 0.0f;
float knx_gimbal_ctrl_pitch_cmd_rpm = 0.0f;
float knx_gimbal_ctrl_vision_valid = 0.0f;
float knx_gimbal_ctrl_tracking_active = 0.0f;
float knx_gimbal_ctrl_yaw_limit_active = 0.0f;
float knx_gimbal_ctrl_pitch_limit_active = 0.0f;
uint32_t knx_gimbal_ctrl_clock_ms = 0U;
uint32_t knx_gimbal_ctrl_update_count = 0U;
int32_t knx_gimbal_ctrl_last_status = 0;

static knx_gimbal_ctrl_state_t s_state;
static knx_ctrl_pidf_t s_yaw_pid;
static knx_ctrl_pidf_t s_pitch_pid;
static knx_ctrl_pidf_t s_hold_yaw_pid;
static knx_ctrl_pidf_t s_hold_pitch_pid;
static uint32_t s_state_tick;
static uint32_t s_control_tick;
static uint32_t s_stop_tick;
static uint8_t s_command_sent;

static const knx_param_entry_t s_gimbal_params[] = {
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_ENABLE, "gimbal.enable",
                  &knx_gimbal_ctrl_enable, 1.0f, 0.0f, 1.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_BOOT_DELAY_MS, "gimbal.boot_delay_ms",
                  &knx_gimbal_ctrl_boot_delay_ms, 500.0f, 0.0f, 10000.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_MODE_GAP_MS, "gimbal.mode_gap_ms",
                  &knx_gimbal_ctrl_mode_gap_ms, 300.0f, 0.0f, 10000.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_CONTROL_PERIOD_MS, "gimbal.control_period_ms",
                  &knx_gimbal_ctrl_control_period_ms, 10.0f, 1.0f, 1000.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_VISION_TIMEOUT_MS, "gimbal.vision_timeout_ms",
                  &knx_gimbal_ctrl_vision_timeout_ms, 200.0f, 1.0f, 10000.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_MIN_CONF, "gimbal.min_conf",
                  &knx_gimbal_ctrl_min_conf, 0.15f, 0.0f, 1.0f, 0U),

    KNX_PARAM_F32(KNX_GIMBAL_PARAM_YAW_KP, "gimbal.yaw.kp",
                  &knx_gimbal_ctrl_yaw_kp, 0.25f, -100.0f, 100.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_YAW_KI, "gimbal.yaw.ki",
                  &knx_gimbal_ctrl_yaw_ki, 0.02f, -100.0f, 100.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_YAW_KD, "gimbal.yaw.kd",
                  &knx_gimbal_ctrl_yaw_kd, 0.01f, -100.0f, 100.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_PITCH_KP, "gimbal.pitch.kp",
                  &knx_gimbal_ctrl_pitch_kp, 0.1f, -100.0f, 100.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_PITCH_KI, "gimbal.pitch.ki",
                  &knx_gimbal_ctrl_pitch_ki, 0.005f, -100.0f, 100.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_PITCH_KD, "gimbal.pitch.kd",
                  &knx_gimbal_ctrl_pitch_kd, 0.01f, -100.0f, 100.0f, 0U),

    KNX_PARAM_F32(KNX_GIMBAL_PARAM_HOLD_YAW_KP, "gimbal.hold_yaw.kp",
                  &knx_gimbal_ctrl_hold_yaw_kp, 0.2f, -100.0f, 100.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_HOLD_YAW_KI, "gimbal.hold_yaw.ki",
                  &knx_gimbal_ctrl_hold_yaw_ki, 0.0f, -100.0f, 100.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_HOLD_YAW_KD, "gimbal.hold_yaw.kd",
                  &knx_gimbal_ctrl_hold_yaw_kd, 0.0f, -100.0f, 100.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_HOLD_PITCH_KP, "gimbal.hold_pitch.kp",
                  &knx_gimbal_ctrl_hold_pitch_kp, 0.2f, -100.0f, 100.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_HOLD_PITCH_KI, "gimbal.hold_pitch.ki",
                  &knx_gimbal_ctrl_hold_pitch_ki, 0.0f, -100.0f, 100.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_HOLD_PITCH_KD, "gimbal.hold_pitch.kd",
                  &knx_gimbal_ctrl_hold_pitch_kd, 0.0f, -100.0f, 100.0f, 0U),

    KNX_PARAM_F32(KNX_GIMBAL_PARAM_OUTPUT_LIMIT_RPM, "gimbal.output_limit_rpm",
                  &knx_gimbal_ctrl_output_limit_rpm, 20.0f, 0.0f, 500.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_HOLD_OUTPUT_LIMIT_RPM, "gimbal.hold_output_limit_rpm",
                  &knx_gimbal_ctrl_hold_output_limit_rpm, 15.0f, 0.0f, 500.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_INTEGRAL_LIMIT, "gimbal.integral_limit",
                  &knx_gimbal_ctrl_integral_limit, 1000.0f, 0.0f, 100000.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_YAW_TARGET_DEG, "gimbal.yaw_target_deg",
                  &knx_gimbal_ctrl_yaw_target_deg, 0.0f, -360.0f, 360.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_PITCH_TARGET_DEG, "gimbal.pitch_target_deg",
                  &knx_gimbal_ctrl_pitch_target_deg, 0.0f, -180.0f, 180.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_YAW_LIMIT_DEG, "gimbal.yaw_limit_deg",
                  &knx_gimbal_ctrl_yaw_limit_deg, 0.0f, 0.0f, 3600.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_PITCH_LIMIT_DEG, "gimbal.pitch_limit_deg",
                  &knx_gimbal_ctrl_pitch_limit_deg, 60.0f, 0.0f, 90.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_STOP_PERIOD_MS, "gimbal.stop_period_ms",
                  &knx_gimbal_ctrl_stop_period_ms, 20.0f, 1.0f, 1000.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_YAW_SPEED_TO_ANGLE_SIGN, "gimbal.yaw_speed_to_angle_sign",
                  &knx_gimbal_ctrl_yaw_speed_to_angle_sign, 1.0f, -1.0f, 1.0f, 0U),
    KNX_PARAM_F32(KNX_GIMBAL_PARAM_PITCH_SPEED_TO_ANGLE_SIGN, "gimbal.pitch_speed_to_angle_sign",
                  &knx_gimbal_ctrl_pitch_speed_to_angle_sign, -1.0f, -1.0f, 1.0f, 0U),
};

knx_status_t knx_gimbal_ctrl_register_params(void)
{
    return knx_param_register_table(
        s_gimbal_params,
        (uint16_t)(sizeof(s_gimbal_params) / sizeof(s_gimbal_params[0])));
}

static void knx_gimbal_ctrl_reset_all_pids(void)
{
    knx_ctrl_pidf_reset(&s_yaw_pid);
    knx_ctrl_pidf_reset(&s_pitch_pid);
    knx_ctrl_pidf_reset(&s_hold_yaw_pid);
    knx_ctrl_pidf_reset(&s_hold_pitch_pid);
}

static void knx_gimbal_ctrl_enter(knx_gimbal_ctrl_state_t state)
{
    s_state = state;
    s_state_tick = knx_millis();
    s_command_sent = 0U;
    knx_gimbal_ctrl_state = (float)state;
}

static void knx_gimbal_ctrl_capture_pair(knx_status_t yaw_status,
                                         knx_status_t pitch_status)
{
    knx_gimbal_ctrl_last_status =
        (int32_t)((yaw_status != KNX_OK) ? yaw_status : pitch_status);
}

static void knx_gimbal_ctrl_send_zero_speed(void)
{
    knx_gimbal_ctrl_yaw_cmd_rpm = 0.0f;
    knx_gimbal_ctrl_pitch_cmd_rpm = 0.0f;
    knx_gimbal_ctrl_capture_pair(
        JC_SetSpeed(KNX_GIMBAL_CTRL_YAW_JC4310_ID, 0.0f),
        JC_SetSpeed(KNX_GIMBAL_CTRL_PITCH_JC2804_ID, 0.0f));
}

static float knx_gimbal_ctrl_limit_axis_cmd(float cmd,
                                            float angle_deg,
                                            float limit_deg,
                                            float speed_to_angle_sign,
                                            float *active)
{
    float limit = (limit_deg < 0.0f) ? -limit_deg : limit_deg;
    float angle_motion = cmd * speed_to_angle_sign;
    *active = 0.0f;

    if (limit <= 0.0f) {
        return cmd;
    }

    if (angle_deg >= limit && angle_motion > 0.0f) {
        *active = 1.0f;
        return 0.0f;
    }
    if (angle_deg <= -limit && angle_motion < 0.0f) {
        *active = -1.0f;
        return 0.0f;
    }
    return cmd;
}

static void knx_gimbal_ctrl_apply_limited_speed(float yaw_cmd, float pitch_cmd)
{
    knx_gimbal_ctrl_yaw_cmd_rpm =
        knx_gimbal_ctrl_limit_axis_cmd(yaw_cmd,
                                       knx_gimbal_ctrl_imu_yaw_deg,
                                       knx_gimbal_ctrl_yaw_limit_deg,
                                       knx_gimbal_ctrl_yaw_speed_to_angle_sign,
                                       &knx_gimbal_ctrl_yaw_limit_active);
    knx_gimbal_ctrl_pitch_cmd_rpm =
        knx_gimbal_ctrl_limit_axis_cmd(pitch_cmd,
                                       knx_gimbal_ctrl_imu_pitch_deg,
                                       knx_gimbal_ctrl_pitch_limit_deg,
                                       knx_gimbal_ctrl_pitch_speed_to_angle_sign,
                                       &knx_gimbal_ctrl_pitch_limit_active);

    knx_gimbal_ctrl_capture_pair(
        JC_SetSpeed(KNX_GIMBAL_CTRL_YAW_JC4310_ID, knx_gimbal_ctrl_yaw_cmd_rpm),
        JC_SetSpeed(KNX_GIMBAL_CTRL_PITCH_JC2804_ID, knx_gimbal_ctrl_pitch_cmd_rpm));
}

static void knx_gimbal_ctrl_reset_limited_pids(knx_ctrl_pidf_t *yaw_pid,
                                               knx_ctrl_pidf_t *pitch_pid)
{
    if (knx_gimbal_ctrl_yaw_limit_active != 0.0f) {
        knx_ctrl_pidf_reset(yaw_pid);
    }
    if (knx_gimbal_ctrl_pitch_limit_active != 0.0f) {
        knx_ctrl_pidf_reset(pitch_pid);
    }
}

static float knx_gimbal_ctrl_clamp_target_to_limit(float target_deg, float limit_deg)
{
    float limit = (limit_deg < 0.0f) ? -limit_deg : limit_deg;
    if (limit <= 0.0f) {
        return target_deg;
    }
    return knx_ctrl_limit_abs_f32(target_deg, limit);
}

static void knx_gimbal_ctrl_update_imu_mirror(void)
{
    knx_gimbal_ctrl_clock_ms = knx_millis();
    knx_gimbal_ctrl_imu_yaw_deg = dm_imu_l1_data.yaw_total;
    knx_gimbal_ctrl_imu_pitch_deg = dm_imu_l1_data.pitch;
}

static void knx_gimbal_ctrl_report_health(void)
{
    uint32_t flags = (uint32_t)s_state;

    if (knx_gimbal_ctrl_vision_valid > 0.0f) {
        flags |= (1UL << 8);
    }
    if (knx_gimbal_ctrl_tracking_active > 0.0f) {
        flags |= (1UL << 9);
    }
    if (knx_gimbal_ctrl_yaw_limit_active != 0.0f) {
        flags |= (1UL << 10);
    }
    if (knx_gimbal_ctrl_pitch_limit_active != 0.0f) {
        flags |= (1UL << 11);
    }

    knx_health_state_t state = KNX_HEALTH_STATE_OK;
    if (knx_gimbal_ctrl_enable <= 0.0f) {
        state = KNX_HEALTH_STATE_DISABLED;
    } else if (knx_gimbal_ctrl_last_status < 0) {
        state = KNX_HEALTH_STATE_WARN;
    }

    (void)knx_health_report(KNX_HEALTH_SOURCE_GIMBAL,
                            state,
                            knx_gimbal_ctrl_last_status,
                            flags,
                            knx_gimbal_ctrl_update_count);
}

static void knx_gimbal_ctrl_hold_zero_once(float dt)
{
    knx_gimbal_ctrl_tracking_active = 0.0f;
    knx_gimbal_ctrl_yaw_error_px = 0.0f;
    knx_gimbal_ctrl_pitch_error_px = 0.0f;
    knx_gimbal_ctrl_yaw_hold_error_deg =
        knx_gimbal_ctrl_yaw_target_deg - knx_gimbal_ctrl_imu_yaw_deg;
    knx_gimbal_ctrl_pitch_hold_error_deg =
        knx_gimbal_ctrl_clamp_target_to_limit(knx_gimbal_ctrl_pitch_target_deg,
                                              knx_gimbal_ctrl_pitch_limit_deg) -
        knx_gimbal_ctrl_imu_pitch_deg;

    if (knx_gimbal_ctrl_enable <= 0.0f) {
        knx_gimbal_ctrl_send_zero_speed();
        return;
    }

    float yaw_cmd = knx_ctrl_pidf_update(&s_hold_yaw_pid,
                                         knx_gimbal_ctrl_yaw_hold_error_deg,
                                         knx_gimbal_ctrl_hold_yaw_kp,
                                         knx_gimbal_ctrl_hold_yaw_ki,
                                         knx_gimbal_ctrl_hold_yaw_kd,
                                         dt,
                                         knx_gimbal_ctrl_integral_limit);
    float pitch_cmd = knx_ctrl_pidf_update(&s_hold_pitch_pid,
                                           knx_gimbal_ctrl_pitch_hold_error_deg,
                                           knx_gimbal_ctrl_hold_pitch_kp,
                                           knx_gimbal_ctrl_hold_pitch_ki,
                                           knx_gimbal_ctrl_hold_pitch_kd,
                                           dt,
                                           knx_gimbal_ctrl_integral_limit);

    yaw_cmd = knx_ctrl_limit_abs_f32(yaw_cmd, knx_gimbal_ctrl_hold_output_limit_rpm);
    pitch_cmd = knx_ctrl_limit_abs_f32(pitch_cmd, knx_gimbal_ctrl_hold_output_limit_rpm);
    yaw_cmd *= knx_gimbal_ctrl_yaw_speed_to_angle_sign;
    pitch_cmd *= knx_gimbal_ctrl_pitch_speed_to_angle_sign;
    knx_gimbal_ctrl_apply_limited_speed(yaw_cmd, pitch_cmd);
    knx_gimbal_ctrl_reset_limited_pids(&s_hold_yaw_pid, &s_hold_pitch_pid);
}

static void knx_gimbal_ctrl_control_once(float dt)
{
    knx_vision_target_t vision;
    knx_vision_snapshot(&vision);

    knx_gimbal_ctrl_vision_valid =
        (float)knx_vision_is_fresh(knx_ctrl_period_ms_from_f32(knx_gimbal_ctrl_vision_timeout_ms, 200U),
                                   knx_gimbal_ctrl_min_conf);

    if (knx_gimbal_ctrl_vision_valid <= 0.0f) {
        knx_ctrl_pidf_reset(&s_yaw_pid);
        knx_ctrl_pidf_reset(&s_pitch_pid);
        knx_gimbal_ctrl_hold_zero_once(dt);
        return;
    }

    knx_ctrl_pidf_reset(&s_hold_yaw_pid);
    knx_ctrl_pidf_reset(&s_hold_pitch_pid);
    knx_gimbal_ctrl_tracking_active = 1.0f;
    knx_gimbal_ctrl_yaw_hold_error_deg =
        knx_gimbal_ctrl_yaw_target_deg - knx_gimbal_ctrl_imu_yaw_deg;
    knx_gimbal_ctrl_pitch_hold_error_deg =
        knx_gimbal_ctrl_clamp_target_to_limit(knx_gimbal_ctrl_pitch_target_deg,
                                              knx_gimbal_ctrl_pitch_limit_deg) -
        knx_gimbal_ctrl_imu_pitch_deg;

    knx_gimbal_ctrl_yaw_error_px = -vision.err_x;
    knx_gimbal_ctrl_pitch_error_px = vision.err_y;

    if (knx_gimbal_ctrl_enable <= 0.0f) {
        knx_ctrl_pidf_reset(&s_yaw_pid);
        knx_ctrl_pidf_reset(&s_pitch_pid);
        knx_gimbal_ctrl_send_zero_speed();
        return;
    }

    float yaw_cmd = knx_ctrl_pidf_update(&s_yaw_pid,
                                         knx_gimbal_ctrl_yaw_error_px,
                                         knx_gimbal_ctrl_yaw_kp,
                                         knx_gimbal_ctrl_yaw_ki,
                                         knx_gimbal_ctrl_yaw_kd,
                                         dt,
                                         knx_gimbal_ctrl_integral_limit);
    float pitch_cmd = knx_ctrl_pidf_update(&s_pitch_pid,
                                           knx_gimbal_ctrl_pitch_error_px,
                                           knx_gimbal_ctrl_pitch_kp,
                                           knx_gimbal_ctrl_pitch_ki,
                                           knx_gimbal_ctrl_pitch_kd,
                                           dt,
                                           knx_gimbal_ctrl_integral_limit);

    yaw_cmd = knx_ctrl_limit_abs_f32(yaw_cmd, knx_gimbal_ctrl_output_limit_rpm);
    pitch_cmd = knx_ctrl_limit_abs_f32(pitch_cmd, knx_gimbal_ctrl_output_limit_rpm);
    yaw_cmd *= knx_gimbal_ctrl_yaw_speed_to_angle_sign;
    pitch_cmd *= knx_gimbal_ctrl_pitch_speed_to_angle_sign;
    knx_gimbal_ctrl_apply_limited_speed(yaw_cmd, pitch_cmd);
    knx_gimbal_ctrl_reset_limited_pids(&s_yaw_pid, &s_pitch_pid);
}

knx_status_t knx_gimbal_ctrl_init(void)
{
    (void)knx_gimbal_ctrl_register_params();
    s_control_tick = 0U;
    s_stop_tick = 0U;
    knx_gimbal_ctrl_update_count = 0U;
    knx_gimbal_ctrl_last_status = 0;
    knx_gimbal_ctrl_reset_all_pids();
    knx_gimbal_ctrl_enter(KNX_GIMBAL_CTRL_BOOT);
    knx_gimbal_ctrl_report_health();
    return KNX_OK;
}

void knx_gimbal_ctrl_restart(void)
{
    knx_gimbal_ctrl_send_zero_speed();
    knx_gimbal_ctrl_reset_all_pids();
    s_stop_tick = 0U;
    knx_gimbal_ctrl_enter(KNX_GIMBAL_CTRL_BOOT);
}

void knx_gimbal_ctrl_stop(void)
{
    uint32_t now = knx_millis();
    uint32_t stop_period_ms =
        knx_ctrl_period_ms_from_f32(knx_gimbal_ctrl_stop_period_ms, 20U);

    knx_gimbal_ctrl_tracking_active = 0.0f;
    knx_gimbal_ctrl_vision_valid = 0.0f;
    knx_gimbal_ctrl_yaw_error_px = 0.0f;
    knx_gimbal_ctrl_pitch_error_px = 0.0f;
    knx_gimbal_ctrl_yaw_hold_error_deg = 0.0f;
    knx_gimbal_ctrl_pitch_hold_error_deg = 0.0f;
    knx_gimbal_ctrl_reset_all_pids();

    if (s_stop_tick == 0U || now - s_stop_tick >= stop_period_ms) {
        s_stop_tick = now;
        knx_gimbal_ctrl_send_zero_speed();
    }
    knx_gimbal_ctrl_report_health();
}

void knx_gimbal_ctrl_set_enable(uint8_t enable)
{
    knx_gimbal_ctrl_enable = (enable != 0U) ? 1.0f : 0.0f;
}

knx_status_t knx_gimbal_ctrl_update(float dt)
{
    (void)dt;

    uint32_t now = knx_millis();
    uint32_t gap_ms = knx_ctrl_period_ms_from_f32(knx_gimbal_ctrl_mode_gap_ms, 300U);

    knx_gimbal_ctrl_update_count++;
    knx_gimbal_ctrl_update_imu_mirror();

    if (knx_gimbal_ctrl_enable <= 0.0f) {
        knx_gimbal_ctrl_stop();
        return KNX_OK;
    }

    switch (s_state) {
    case KNX_GIMBAL_CTRL_BOOT:
        if (now - s_state_tick >= knx_ctrl_period_ms_from_f32(knx_gimbal_ctrl_boot_delay_ms, 500U)) {
            knx_gimbal_ctrl_enter(KNX_GIMBAL_CTRL_ENTER_SERVO);
        }
        break;

    case KNX_GIMBAL_CTRL_ENTER_SERVO:
        if (s_command_sent == 0U) {
            knx_gimbal_ctrl_capture_pair(
                JC_EnterServo(KNX_GIMBAL_CTRL_YAW_JC4310_ID),
                JC_EnterServo(KNX_GIMBAL_CTRL_PITCH_JC2804_ID));
            s_command_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= gap_ms) {
            knx_gimbal_ctrl_enter(KNX_GIMBAL_CTRL_SPEED_MODE);
        }
        break;

    case KNX_GIMBAL_CTRL_SPEED_MODE:
        if (s_command_sent == 0U) {
            knx_gimbal_ctrl_capture_pair(
                JC_SwitchMode(KNX_GIMBAL_CTRL_YAW_JC4310_ID, JC_MODE_SPEED),
                JC_SwitchMode(KNX_GIMBAL_CTRL_PITCH_JC2804_ID, JC_MODE_SPEED));
            s_command_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= gap_ms) {
            knx_gimbal_ctrl_reset_all_pids();
            s_control_tick = now;
            knx_gimbal_ctrl_enter(KNX_GIMBAL_CTRL_CONTROL);
        }
        break;

    case KNX_GIMBAL_CTRL_CONTROL:
        if (now - s_control_tick >=
            knx_ctrl_period_ms_from_f32(knx_gimbal_ctrl_control_period_ms, 10U)) {
            uint32_t elapsed_ms = now - s_control_tick;
            s_control_tick = now;
            knx_gimbal_ctrl_control_once((float)elapsed_ms * 0.001f);
        }
        break;

    default:
        knx_gimbal_ctrl_enter(KNX_GIMBAL_CTRL_BOOT);
        break;
    }

    knx_gimbal_ctrl_report_health();
    return KNX_OK;
}

void knx_gimbal_ctrl_debug_octo(Octolinker_Instance_t *octo, uint16_t base_id)
{
    if (octo == NULL) {
        return;
    }

    (void)Octolinker_SendU32(octo, base_id + 0U, knx_gimbal_ctrl_clock_ms);
    (void)Octolinker_SendF32(octo, base_id + 1U, knx_gimbal_ctrl_state);
    (void)Octolinker_SendF32(octo, base_id + 2U, knx_vision_err_x);
    (void)Octolinker_SendF32(octo, base_id + 3U, knx_vision_err_y);
    (void)Octolinker_SendF32(octo, base_id + 4U, knx_vision_conf);
    (void)Octolinker_SendF32(octo, base_id + 5U, knx_gimbal_ctrl_vision_valid);
    (void)Octolinker_SendF32(octo, base_id + 6U, knx_gimbal_ctrl_yaw_error_px);
    (void)Octolinker_SendF32(octo, base_id + 7U, knx_gimbal_ctrl_pitch_error_px);
    (void)Octolinker_SendF32(octo, base_id + 8U, knx_gimbal_ctrl_yaw_cmd_rpm);
    (void)Octolinker_SendF32(octo, base_id + 9U, knx_gimbal_ctrl_pitch_cmd_rpm);
    (void)Octolinker_SendU32(octo, base_id + 10U, knx_vision_rx_count);
    (void)Octolinker_SendF32(octo, base_id + 11U, (float)knx_vision_age_ms());
    (void)Octolinker_SendI32(octo, base_id + 12U, knx_gimbal_ctrl_last_status);
    (void)Octolinker_SendU32(octo, base_id + 13U, knx_vision_payload_len_errors);
    (void)Octolinker_SendU32(octo, base_id + 14U, knx_vision_msg_id_errors);
    (void)Octolinker_SendU32(octo, base_id + 15U, knx_gimbal_ctrl_update_count);
    (void)Octolinker_SendF32(octo, base_id + 18U, knx_gimbal_ctrl_imu_yaw_deg);
    (void)Octolinker_SendF32(octo, base_id + 19U, knx_gimbal_ctrl_imu_pitch_deg);
    (void)Octolinker_SendF32(octo, base_id + 20U, knx_gimbal_ctrl_yaw_hold_error_deg);
    (void)Octolinker_SendF32(octo, base_id + 21U, knx_gimbal_ctrl_pitch_hold_error_deg);
    (void)Octolinker_SendF32(octo, base_id + 22U, knx_gimbal_ctrl_tracking_active);
    (void)Octolinker_SendF32(octo, base_id + 23U, knx_gimbal_ctrl_yaw_limit_active);
    (void)Octolinker_SendF32(octo, base_id + 24U, knx_gimbal_ctrl_pitch_limit_active);
    (void)Octolinker_SendF32(octo, base_id + 25U, knx_gimbal_ctrl_yaw_speed_to_angle_sign);
    (void)Octolinker_SendF32(octo, base_id + 26U, knx_gimbal_ctrl_pitch_speed_to_angle_sign);
}
