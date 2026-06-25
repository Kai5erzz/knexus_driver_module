#include "test_drive_calib.h"
#include "knx_drive.h"
#include "knx_math.h"
#include "knx_motor.h"
#include "knx_sys.h"
#include "knx_time.h"
#include "drv8701e.h"
#include "encoder.h"
#include "octolinker.h"

typedef enum {
    TEST_DRIVE_CALIB_POLARITY = 0,
    TEST_DRIVE_CALIB_VELOCITY = 1,
    TEST_DRIVE_CALIB_POSITION = 2,
    TEST_DRIVE_CALIB_ANGLE = 3,
    TEST_DRIVE_CALIB_POSITION_ANGLE = 4,
    TEST_DRIVE_CALIB_CURRENT = 5,
    TEST_DRIVE_CALIB_ALL = 6,
} test_drive_calib_mode_t;

typedef struct {
    float left_duty;
    float right_duty;
    float left_speed_mps;
    float right_speed_mps;
    float linear_mps;
    float angular_radps;
    float position_m;
    float heading_rad;
    float left_current_a;
    float right_current_a;
} test_drive_calib_step_t;

float test_drive_calib_enable = 1.0f;
float test_drive_calib_mode = (float)TEST_DRIVE_CALIB_VELOCITY;
float test_drive_calib_duty = 0.12f;
float test_drive_calib_current_a = 0.25f;
float test_drive_calib_linear_mps = 0.12f;
float test_drive_calib_angular_radps = 1.0f;
float test_drive_calib_position_m = 0.20f;
float test_drive_calib_heading_rad = 1.5708f;
float test_drive_calib_step_ms = 1500.0f;
float test_drive_calib_stop_ms = 700.0f;

static Octolinker_Instance_t *s_octo;
static uint32_t s_step_start_ms;
static uint32_t s_ctrl_tick_ms;
static uint32_t s_debug_tick_ms;
static uint32_t s_step_index;
static uint32_t s_auto_group;
static uint32_t s_last_applied_step = 0xFFFFFFFFU;
static float s_debug_left_speed_target;
static float s_debug_right_speed_target;

static uint32_t step_duration_ms(uint32_t step)
{
    float duration = (step & 1U) ? test_drive_calib_stop_ms : test_drive_calib_step_ms;
    if (duration < 50.0f) {
        duration = 50.0f;
    }
    return (uint32_t)duration;
}

static test_drive_calib_step_t make_step(uint32_t step)
{
    float duty = knx_clamp_f(test_drive_calib_duty, 0.0f, 0.4f);
    float current = knx_clamp_f(test_drive_calib_current_a, 0.0f, 1.0f);
    float linear = test_drive_calib_linear_mps;
    float angular = test_drive_calib_angular_radps;
    float position = test_drive_calib_position_m;
    float heading = test_drive_calib_heading_rad;

    switch (step) {
    case 0: return (test_drive_calib_step_t){ .left_duty = duty, .left_speed_mps = linear, .linear_mps = linear, .position_m = position, .heading_rad = heading, .left_current_a = current };
    case 2: return (test_drive_calib_step_t){ .left_duty = -duty, .left_speed_mps = -linear, .linear_mps = -linear, .position_m = -position, .heading_rad = -heading, .left_current_a = -current };
    case 4: return (test_drive_calib_step_t){ .right_duty = duty, .right_speed_mps = linear, .angular_radps = angular, .heading_rad = heading, .right_current_a = current };
    case 6: return (test_drive_calib_step_t){ .right_duty = -duty, .right_speed_mps = -linear, .angular_radps = -angular, .heading_rad = -heading, .right_current_a = -current };
    case 8: return (test_drive_calib_step_t){ .left_duty = duty, .right_duty = duty, .left_speed_mps = linear, .right_speed_mps = linear, .linear_mps = linear, .position_m = position, .left_current_a = current, .right_current_a = current };
    case 10: return (test_drive_calib_step_t){ .left_duty = duty, .right_duty = -duty, .left_speed_mps = linear, .right_speed_mps = -linear, .angular_radps = angular, .heading_rad = heading, .left_current_a = current, .right_current_a = -current };
    default: return (test_drive_calib_step_t){0};
    }
}

static void next_step_if_due(uint32_t now)
{
    if (now - s_step_start_ms < step_duration_ms(s_step_index)) {
        return;
    }

    s_step_index++;
    if (s_step_index > 11U) {
        s_step_index = 0U;
        s_auto_group = 0U;
    }
    s_step_start_ms = now;
}

static void apply_polarity_step(test_drive_calib_step_t step)
{
    (void)knx_sys_set_mode(KNX_RUN_MODE_DUTY);
    DRV8701E_SetDutyNorm(&motor_left, step.left_duty);
    DRV8701E_SetDutyNorm(&motor_right, step.right_duty);
}

static void apply_velocity_step(test_drive_calib_step_t step)
{
    float left_target = 0.0f;
    float right_target = 0.0f;

    if ((s_step_index & 1U) == 0U) {
        left_target = step.left_speed_mps;
        right_target = step.right_speed_mps;
    }

    (void)knx_sys_set_mode(KNX_RUN_MODE_SPEED);
    (void)knx_motor_set_target(left_target, right_target);
    s_debug_left_speed_target = left_target;
    s_debug_right_speed_target = right_target;
    s_last_applied_step = s_step_index;
}

static void apply_closed_loop_step(test_drive_calib_mode_t mode, test_drive_calib_step_t step)
{
    if (mode == TEST_DRIVE_CALIB_CURRENT) {
        (void)knx_sys_set_mode(KNX_RUN_MODE_CURRENT_TEST);
        if ((s_step_index & 1U) == 0U) {
            (void)knx_motor_set_current_target(step.left_current_a, step.right_current_a);
        } else {
            (void)knx_motor_set_current_target(0.0f, 0.0f);
        }
        s_last_applied_step = s_step_index;
        return;
    }

    if (mode == TEST_DRIVE_CALIB_VELOCITY) {
        apply_velocity_step(step);
        s_last_applied_step = s_step_index;
        return;
    }

    if (s_last_applied_step == s_step_index) {
        return;
    }

    if ((s_step_index & 1U) != 0U) {
        (void)knx_drive_stop();
    } else if (mode == TEST_DRIVE_CALIB_POSITION) {
        knx_drive_reset_odometry();
        (void)knx_drive_set_position(step.position_m);
    } else if (mode == TEST_DRIVE_CALIB_ANGLE) {
        knx_drive_reset_odometry();
        (void)knx_drive_set_angle(step.heading_rad);
    } else if (mode == TEST_DRIVE_CALIB_POSITION_ANGLE) {
        knx_drive_reset_odometry();
        (void)knx_drive_set_position_angle(step.position_m, step.heading_rad);
    }

    s_last_applied_step = s_step_index;
}

static void send_debug(test_drive_calib_step_t step)
{
    if (s_octo == NULL) {
        return;
    }

    knx_drive_state_t drive;

    knx_drive_snapshot(&drive);

    Octolinker_SendF32(s_octo, 50, test_drive_calib_mode);
    Octolinker_SendF32(s_octo, 51, (float)s_step_index);
    Octolinker_SendF32(s_octo, 52, step.left_duty);
    Octolinker_SendF32(s_octo, 53, step.right_duty);
    Octolinker_SendF32(s_octo, 54, encoder_left.speed_mps);
    Octolinker_SendF32(s_octo, 55, encoder_right.speed_mps);
    Octolinker_SendF32(s_octo, 56, drv8701e_current.current_left_filtered_a);
    Octolinker_SendF32(s_octo, 57, drv8701e_current.current_right_filtered_a);
    Octolinker_SendF32(s_octo, 58, drive.position_m);
    Octolinker_SendF32(s_octo, 59, drive.heading_rad);
    Octolinker_SendF32(s_octo, 60, drive.measured_linear_mps);
    Octolinker_SendF32(s_octo, 61, drive.measured_angular_radps);
    Octolinker_SendF32(s_octo, 62, drive.position_loop_out_mps);
    Octolinker_SendF32(s_octo, 63, drive.angle_loop_out_radps);
    Octolinker_SendF32(s_octo, 64, drive.linear_speed_loop_out_mps);
    Octolinker_SendF32(s_octo, 65, drive.angular_speed_loop_out_radps);
    Octolinker_SendF32(s_octo, 66, s_debug_left_speed_target);
    Octolinker_SendF32(s_octo, 67, s_debug_right_speed_target);
    Octolinker_SendF32(s_octo, 68, (float)knx_sys_get_mode());
    Octolinker_SendF32(s_octo, 69, step.left_current_a);
    Octolinker_SendF32(s_octo, 70, step.right_current_a);
    Octolinker_SendF32(s_octo, 71, encoder_left.position_m);
    Octolinker_SendF32(s_octo, 72, encoder_right.position_m);
    Octolinker_SendF32(s_octo, 73, (float)s_auto_group);
    Octolinker_SendF32(s_octo, 74, vc_left.output);
    Octolinker_SendF32(s_octo, 75, vc_right.output);
    Octolinker_SendF32(s_octo, 76, encoder_left.speed_mps_raw);
    Octolinker_SendF32(s_octo, 77, encoder_right.speed_mps_raw);
    Octolinker_SendF32(s_octo, 78, vc_left.target_used);
    Octolinker_SendF32(s_octo, 79, vc_right.target_used);
    Octolinker_SendF32(s_octo, 80, vc_left.feedforward);
    Octolinker_SendF32(s_octo, 81, vc_right.feedforward);
    Octolinker_SendF32(s_octo, 82, vc_left.correction);
    Octolinker_SendF32(s_octo, 83, vc_right.correction);
}

void test_drive_calib_init(void *octo)
{
    s_octo = (Octolinker_Instance_t *)octo;
    s_step_start_ms = knx_millis();
    s_ctrl_tick_ms = 0U;
    s_debug_tick_ms = 0U;
    s_step_index = 0U;
    s_auto_group = 0U;
    s_last_applied_step = 0xFFFFFFFFU;
    s_debug_left_speed_target = 0.0f;
    s_debug_right_speed_target = 0.0f;

    knx_drive_reset_odometry();
    DRV8701E_StopAll();
}

void test_drive_calib_loop(void)
{
    uint32_t now = knx_millis();
    next_step_if_due(now);

    test_drive_calib_step_t step = make_step(s_step_index);
    test_drive_calib_mode_t mode = (test_drive_calib_mode_t)((uint32_t)test_drive_calib_mode);
    if (mode != TEST_DRIVE_CALIB_VELOCITY) {
        mode = TEST_DRIVE_CALIB_VELOCITY;
    }

    if (test_drive_calib_enable <= 0.0f) {
        (void)knx_drive_stop();
        DRV8701E_StopAll();
        s_last_applied_step = 0xFFFFFFFFU;
        return;
    }

    if (now - s_ctrl_tick_ms >= 1U) {
        s_ctrl_tick_ms = now;

        if (mode == TEST_DRIVE_CALIB_POLARITY) {
            DRV8701E_Current_Poll(1);
            Encoder_CalcSpeed(&encoder_left);
            Encoder_CalcSpeed(&encoder_right);
            apply_polarity_step(step);
            knx_drive_update(0.001f);
        } else {
            apply_closed_loop_step(mode, step);
            if (mode != TEST_DRIVE_CALIB_VELOCITY) {
                knx_drive_update(0.001f);
            }
            knx_motor_update();
        }
    }

    if (now - s_debug_tick_ms >= 20U) {
        s_debug_tick_ms = now;
        send_debug(step);
    }
}
