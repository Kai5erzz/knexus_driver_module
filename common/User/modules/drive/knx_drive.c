#include "knx_drive.h"
#include "knx_control.h"
#include "knx_motor.h"
#include "knx_params.h"
#include "knx_project_config.h"
#include "knx_sys.h"
#include "knx_time.h"
#include "encoder.h"
#include "PID.h"
#include "ramp.h"
#include "FreeRTOS.h"
#include "task.h"

static knx_drive_state_t s_drive_state;
static pid_obj_t *s_position_pid;
static pid_obj_t *s_angle_pid;
static pid_obj_t *s_linear_speed_pid;
static pid_obj_t *s_angular_speed_pid;
static float s_last_left_position_m;
static float s_last_right_position_m;

static float wrap_pi(float angle)
{
    while (angle > 3.14159265f) {
        angle -= 6.28318531f;
    }
    while (angle < -3.14159265f) {
        angle += 6.28318531f;
    }
    return angle;
}

static pid_obj_t *register_pid(const knx_pid_params_t *params)
{
    pid_config_t config = {
        .Kp = params->kp,
        .Ki = params->ki,
        .Kd = params->kd,
        .MaxOut = params->max_out,
        .DeadBand = params->deadband,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = params->integral_limit,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Output_LPF_RC = 0.0f,
        .Derivative_LPF_RC = 0.0f,
    };
    return pid_register(&config);
}

static void clear_controllers(void)
{
    if (s_position_pid != NULL) {
        pid_clear(s_position_pid);
    }
    if (s_angle_pid != NULL) {
        pid_clear(s_angle_pid);
    }
    if (s_linear_speed_pid != NULL) {
        pid_clear(s_linear_speed_pid);
    }
    if (s_angular_speed_pid != NULL) {
        pid_clear(s_angular_speed_pid);
    }
}

static void apply_zero_command(void)
{
    taskENTER_CRITICAL();
    s_drive_state.mode = KNX_DRIVE_MODE_IDLE;
    s_drive_state.target_linear_mps = 0.0f;
    s_drive_state.target_angular_radps = 0.0f;
    s_drive_state.profiled_linear_mps = 0.0f;
    s_drive_state.profiled_angular_radps = 0.0f;
    s_drive_state.left_target_mps = 0.0f;
    s_drive_state.right_target_mps = 0.0f;
    s_drive_state.command_active = false;
    taskEXIT_CRITICAL();

    clear_controllers();
    (void)knx_motor_set_target(0.0f, 0.0f);
}

static void update_odometry(float dt_s)
{
    knx_motor_state_t left;
    knx_motor_state_t right;
    knx_motor_snapshot(&left, &right);

    float left_speed = left.current_speed * g_knx_params.drive.left_feedback_sign;
    float right_speed = right.current_speed * g_knx_params.drive.right_feedback_sign;
    float linear = 0.5f * (left_speed + right_speed);
    float left_position = encoder_left.position_m * g_knx_params.drive.left_feedback_sign;
    float right_position = encoder_right.position_m * g_knx_params.drive.right_feedback_sign;
    float delta_left = left_position - s_last_left_position_m;
    float delta_right = right_position - s_last_right_position_m;
    s_last_left_position_m = left_position;
    s_last_right_position_m = right_position;
    float delta_linear = 0.5f * (delta_left + delta_right);
    float track_width = g_knx_params.mech.track_width_m;
    if (track_width < 0.001f) {
        track_width = 0.001f;
    }
    float angular = ((right_speed - left_speed) / track_width) *
                    g_knx_params.drive.angular_feedback_sign;
    /* 航向积分始终用物理方向, 不受 feedback_sign 影响 */
    float delta_heading = (delta_right - delta_left) / track_width;

    taskENTER_CRITICAL();
    s_drive_state.measured_linear_mps = linear;
    s_drive_state.measured_angular_radps = angular;
    s_drive_state.position_m += delta_linear;
    s_drive_state.heading_rad = wrap_pi(s_drive_state.heading_rad + delta_heading);
    taskEXIT_CRITICAL();
    (void)dt_s;
}

static void set_drive_mode(knx_drive_mode_t mode)
{
    taskENTER_CRITICAL();
    s_drive_state.mode = mode;
    s_drive_state.last_command_ms = knx_millis();
    s_drive_state.command_active = true;
    taskEXIT_CRITICAL();

    (void)knx_sys_set_mode(KNX_RUN_MODE_SPEED);
}

static void update_profile(float *value, float target, float max_step)
{
    if (max_step <= 0.0f) {
        *value = target;
        return;
    }
    slope_following(&target, value, max_step);
}

knx_status_t knx_drive_init(void)
{
    s_drive_state = (knx_drive_state_t){0};
    s_position_pid = register_pid(&g_knx_params.drive.position_pid);
    s_angle_pid = register_pid(&g_knx_params.drive.angle_pid);
    s_linear_speed_pid = register_pid(&g_knx_params.drive.linear_speed_pid);
    s_angular_speed_pid = register_pid(&g_knx_params.drive.angular_speed_pid);
    s_last_left_position_m = encoder_left.position_m * g_knx_params.drive.left_feedback_sign;
    s_last_right_position_m = encoder_right.position_m * g_knx_params.drive.right_feedback_sign;
    return KNX_OK;
}

knx_status_t knx_drive_update(float dt_s)
{
#if KNX_MODULE_DRIVE_EN
    if (dt_s <= 0.0f) {
        dt_s = 0.001f;
    }

    update_odometry(dt_s);

    if (!g_knx_params.drive.drive_en) {
        (void)knx_drive_stop();
        return KNX_OK;
    }

    if (knx_sys_get_mode() != KNX_RUN_MODE_SPEED) {
        return KNX_OK;
    }

    knx_drive_state_t local;
    taskENTER_CRITICAL();
    local = s_drive_state;
    taskEXIT_CRITICAL();

    if (local.mode == KNX_DRIVE_MODE_VELOCITY &&
        (!local.command_active ||
         (knx_millis() - local.last_command_ms) > g_knx_params.drive.command_timeout_ms)) {
        (void)knx_drive_stop();
        return KNX_OK;
    }

    float linear_ref = local.target_linear_mps;
    float angular_ref = local.target_angular_radps;

    if (local.mode == KNX_DRIVE_MODE_POSITION ||
        local.mode == KNX_DRIVE_MODE_POSITION_ANGLE) {
        linear_ref = pid_calculate(s_position_pid,
                                   local.position_m,
                                   local.target_position_m);
    }
    float position_loop_out = linear_ref;

    if (local.mode == KNX_DRIVE_MODE_ANGLE ||
        local.mode == KNX_DRIVE_MODE_POSITION_ANGLE) {
        float angle_error = wrap_pi(local.target_heading_rad - local.heading_rad);
        angular_ref = pid_calculate(s_angle_pid, 0.0f, angle_error);
    }
    float angle_loop_out = angular_ref;

    linear_ref = knx_ctrl_limit_abs_f32(linear_ref, g_knx_params.drive.max_linear_speed_mps);
    angular_ref = knx_ctrl_limit_abs_f32(angular_ref, g_knx_params.drive.max_angular_speed_radps);

    float profiled_linear = local.profiled_linear_mps;
    float profiled_angular = local.profiled_angular_radps;
    update_profile(&profiled_linear,
                   linear_ref,
                   g_knx_params.drive.max_linear_accel_mps2 * dt_s);
    update_profile(&profiled_angular,
                   angular_ref,
                   g_knx_params.drive.max_angular_accel_radps2 * dt_s);

    float linear_cmd = pid_calculate(s_linear_speed_pid,
                                     local.measured_linear_mps,
                                     profiled_linear);
    float angular_cmd = pid_calculate(s_angular_speed_pid,
                                      local.measured_angular_radps,
                                      profiled_angular);

    linear_cmd = knx_ctrl_limit_abs_f32(linear_cmd, g_knx_params.drive.max_linear_speed_mps);
    angular_cmd = knx_ctrl_limit_abs_f32(angular_cmd, g_knx_params.drive.max_angular_speed_radps);

    float half_track = g_knx_params.mech.track_width_m * 0.5f;
    float left_target = (linear_cmd - angular_cmd * half_track) *
                        g_knx_params.drive.left_cmd_sign;
    float right_target = (linear_cmd + angular_cmd * half_track) *
                         g_knx_params.drive.right_cmd_sign;

    taskENTER_CRITICAL();
    s_drive_state.target_linear_mps = linear_ref;
    s_drive_state.target_angular_radps = angular_ref;
    s_drive_state.profiled_linear_mps = profiled_linear;
    s_drive_state.profiled_angular_radps = profiled_angular;
    s_drive_state.position_loop_out_mps = position_loop_out;
    s_drive_state.angle_loop_out_radps = angle_loop_out;
    s_drive_state.linear_speed_loop_out_mps = linear_cmd;
    s_drive_state.angular_speed_loop_out_radps = angular_cmd;
    s_drive_state.left_target_mps = left_target;
    s_drive_state.right_target_mps = right_target;
    taskEXIT_CRITICAL();

    (void)knx_motor_set_target(left_target, right_target);
#else
    (void)dt_s;
#endif
    return KNX_OK;
}

knx_status_t knx_drive_set_command(float linear_mps, float angular_radps)
{
    return knx_drive_set_velocity(linear_mps, angular_radps);
}

knx_status_t knx_drive_set_velocity(float linear_mps, float angular_radps)
{
    if (!g_knx_params.drive.drive_en) {
        return KNX_NOT_READY;
    }

    float linear = knx_ctrl_limit_abs_f32(linear_mps, g_knx_params.drive.max_linear_speed_mps);
    float angular = knx_ctrl_limit_abs_f32(angular_radps, g_knx_params.drive.max_angular_speed_radps);

    taskENTER_CRITICAL();
    s_drive_state.target_linear_mps = linear;
    s_drive_state.target_angular_radps = angular;
    taskEXIT_CRITICAL();

    set_drive_mode(KNX_DRIVE_MODE_VELOCITY);
    return KNX_OK;
}

knx_status_t knx_drive_set_position(float position_m)
{
    if (!g_knx_params.drive.drive_en) {
        return KNX_NOT_READY;
    }

    taskENTER_CRITICAL();
    s_drive_state.target_position_m = position_m;
    s_drive_state.target_angular_radps = 0.0f;
    taskEXIT_CRITICAL();

    clear_controllers();
    set_drive_mode(KNX_DRIVE_MODE_POSITION);
    return KNX_OK;
}

knx_status_t knx_drive_set_angle(float heading_rad)
{
    if (!g_knx_params.drive.drive_en) {
        return KNX_NOT_READY;
    }

    taskENTER_CRITICAL();
    s_drive_state.target_linear_mps = 0.0f;
    s_drive_state.target_heading_rad = wrap_pi(heading_rad);
    taskEXIT_CRITICAL();

    clear_controllers();
    set_drive_mode(KNX_DRIVE_MODE_ANGLE);
    return KNX_OK;
}

knx_status_t knx_drive_set_position_angle(float position_m, float heading_rad)
{
    if (!g_knx_params.drive.drive_en) {
        return KNX_NOT_READY;
    }

    taskENTER_CRITICAL();
    s_drive_state.target_position_m = position_m;
    s_drive_state.target_heading_rad = wrap_pi(heading_rad);
    taskEXIT_CRITICAL();

    clear_controllers();
    set_drive_mode(KNX_DRIVE_MODE_POSITION_ANGLE);
    return KNX_OK;
}

knx_status_t knx_drive_stop(void)
{
    apply_zero_command();
    (void)knx_sys_set_mode(KNX_RUN_MODE_IDLE);
    return KNX_OK;
}

void knx_drive_reset_odometry(void)
{
    taskENTER_CRITICAL();
    s_drive_state.position_m = 0.0f;
    s_drive_state.heading_rad = 0.0f;
    s_drive_state.measured_linear_mps = 0.0f;
    s_drive_state.measured_angular_radps = 0.0f;
    taskEXIT_CRITICAL();
    s_last_left_position_m = encoder_left.position_m * g_knx_params.drive.left_feedback_sign;
    s_last_right_position_m = encoder_right.position_m * g_knx_params.drive.right_feedback_sign;
    clear_controllers();
}

void knx_drive_set_heading(float heading_rad)
{
    taskENTER_CRITICAL();
    s_drive_state.heading_rad = heading_rad;
    taskEXIT_CRITICAL();
}

void knx_drive_snapshot(knx_drive_state_t *out)
{
    if (out == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_drive_state;
    taskEXIT_CRITICAL();
}
