#include "test_standard_gimbal.h"
#include "dm_imu_l1.h"
#include "jc_driver.h"
#include "knx_time.h"
#include "octolinker.h"

#define TEST_STANDARD_GIMBAL_TELEM_BASE       250U
#define TEST_STANDARD_GIMBAL_PITCH_JC2804_ID  1U
#define TEST_STANDARD_GIMBAL_YAW_JC4310_ID    3U
#define TEST_STANDARD_GIMBAL_RAD_S_TO_RPM     9.549296586f

typedef enum {
    TEST_STANDARD_GIMBAL_BOOT = 0,
    TEST_STANDARD_GIMBAL_ENTER_SERVO,
    TEST_STANDARD_GIMBAL_SPEED_MODE,
    TEST_STANDARD_GIMBAL_CONTROL,
} test_standard_gimbal_state_t;

typedef struct {
    float integral;
    float last_error;
    uint8_t initialized;
} test_standard_gimbal_pid_state_t;

Octolinker_Instance_t *test_standard_gimbal_octo;

float test_standard_gimbal_enable = 1.0f;
float test_standard_gimbal_boot_delay_ms = 500.0f;
float test_standard_gimbal_mode_gap_ms = 300.0f;
float test_standard_gimbal_control_period_ms = 10.0f;
float test_standard_gimbal_telem_period_ms = 100.0f;
float test_standard_gimbal_restart = 0.0f;

float test_standard_gimbal_pitch_target_deg = 0.0f;
float test_standard_gimbal_yaw_target_deg = 0.0f;

float test_standard_gimbal_pitch_kp = 0.1f;
float test_standard_gimbal_pitch_ki = 0.005f;
float test_standard_gimbal_pitch_kd = 0.01f;
float test_standard_gimbal_yaw_kp = 0.25f;
float test_standard_gimbal_yaw_ki = 0.02f;
float test_standard_gimbal_yaw_kd = 0.01f;

float test_standard_gimbal_output_limit_rad_s = 300.0f;
float test_standard_gimbal_integral_limit = 50.0f;
float test_standard_gimbal_pitch_limit_deg = 60.0f;
float test_standard_gimbal_yaw_speed_to_angle_sign = 1.0f;
float test_standard_gimbal_pitch_speed_to_angle_sign = -1.0f;

float test_standard_gimbal_state = 0.0f;
float test_standard_gimbal_pitch_error_deg = 0.0f;
float test_standard_gimbal_yaw_error_deg = 0.0f;
float test_standard_gimbal_pitch_cmd_rad_s = 0.0f;
float test_standard_gimbal_yaw_cmd_rad_s = 0.0f;
float test_standard_gimbal_pitch_cmd_rpm = 0.0f;
float test_standard_gimbal_yaw_cmd_rpm = 0.0f;
float test_standard_gimbal_imu_roll = 0.0f;
float test_standard_gimbal_imu_pitch = 0.0f;
float test_standard_gimbal_imu_yaw = 0.0f;
float test_standard_gimbal_imu_yaw_total = 0.0f;
float test_standard_gimbal_pitch_limit_active = 0.0f;
uint32_t test_standard_gimbal_clock_ms = 0U;
uint32_t test_standard_gimbal_imu_rx_count = 0U;
int32_t test_standard_gimbal_last_status = 0;

static test_standard_gimbal_state_t s_state;
static test_standard_gimbal_pid_state_t s_pitch_pid;
static test_standard_gimbal_pid_state_t s_yaw_pid;
static uint32_t s_state_tick;
static uint32_t s_control_tick;
static uint32_t s_telem_tick;
static uint8_t s_command_sent;
static float s_last_restart;

static float test_standard_gimbal_clampf(float value, float limit)
{
    if (limit < 0.0f) {
        limit = -limit;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static uint32_t test_standard_gimbal_period(float value, uint32_t fallback)
{
    if (value < 1.0f) {
        return fallback;
    }
    return (uint32_t)value;
}

static void test_standard_gimbal_reset_pid(test_standard_gimbal_pid_state_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->initialized = 0U;
}

static void test_standard_gimbal_enter(test_standard_gimbal_state_t state)
{
    s_state = state;
    s_state_tick = knx_millis();
    s_command_sent = 0U;
    test_standard_gimbal_state = (float)state;
}

static void test_standard_gimbal_capture_pair(knx_status_t pitch_status, knx_status_t yaw_status)
{
    test_standard_gimbal_last_status = (int32_t)((pitch_status != KNX_OK) ? pitch_status : yaw_status);
}

static float test_standard_gimbal_pid_update(test_standard_gimbal_pid_state_t *pid,
                                             float error,
                                             float kp,
                                             float ki,
                                             float kd,
                                             float dt)
{
    if (dt <= 0.0f) {
        dt = 0.001f;
    }

    if (pid->initialized == 0U) {
        pid->last_error = error;
        pid->initialized = 1U;
    }

    pid->integral += error * dt;
    pid->integral = test_standard_gimbal_clampf(pid->integral,
                                                test_standard_gimbal_integral_limit);

    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    return (kp * error) + (ki * pid->integral) + (kd * derivative);
}

static void test_standard_gimbal_update_imu_mirrors(void)
{
    test_standard_gimbal_clock_ms = knx_millis();
    test_standard_gimbal_imu_roll = dm_imu_l1_data.roll;
    test_standard_gimbal_imu_pitch = dm_imu_l1_data.pitch;
    test_standard_gimbal_imu_yaw = dm_imu_l1_data.yaw;
    test_standard_gimbal_imu_yaw_total = dm_imu_l1_data.yaw_total;
    test_standard_gimbal_imu_rx_count = dm_imu_l1_data.rx_count;
}

static void test_standard_gimbal_send_zero_speed(void)
{
    test_standard_gimbal_pitch_cmd_rad_s = 0.0f;
    test_standard_gimbal_yaw_cmd_rad_s = 0.0f;
    test_standard_gimbal_pitch_cmd_rpm = 0.0f;
    test_standard_gimbal_yaw_cmd_rpm = 0.0f;
    test_standard_gimbal_pitch_limit_active = 0.0f;
    test_standard_gimbal_capture_pair(
        JC_SetSpeed(TEST_STANDARD_GIMBAL_PITCH_JC2804_ID, 0.0f),
        JC_SetSpeed(TEST_STANDARD_GIMBAL_YAW_JC4310_ID, 0.0f));
}

static float test_standard_gimbal_limit_pitch_cmd(float cmd_rpm)
{
    float limit = test_standard_gimbal_pitch_limit_deg;
    if (limit < 0.0f) {
        limit = -limit;
    }

    test_standard_gimbal_pitch_limit_active = 0.0f;
    if (limit <= 0.0f) {
        return cmd_rpm;
    }

    float angle_motion = cmd_rpm * test_standard_gimbal_pitch_speed_to_angle_sign;
    if (test_standard_gimbal_imu_pitch >= limit && angle_motion > 0.0f) {
        test_standard_gimbal_pitch_limit_active = 1.0f;
        return 0.0f;
    }
    if (test_standard_gimbal_imu_pitch <= -limit && angle_motion < 0.0f) {
        test_standard_gimbal_pitch_limit_active = -1.0f;
        return 0.0f;
    }
    return cmd_rpm;
}

static void test_standard_gimbal_control_once(float dt)
{
    float pitch_target = test_standard_gimbal_clampf(test_standard_gimbal_pitch_target_deg,
                                                     test_standard_gimbal_pitch_limit_deg);

    test_standard_gimbal_pitch_error_deg =
        pitch_target - test_standard_gimbal_imu_pitch;
    test_standard_gimbal_yaw_error_deg =
        test_standard_gimbal_yaw_target_deg - test_standard_gimbal_imu_yaw;

    if (test_standard_gimbal_enable <= 0.0f) {
        test_standard_gimbal_reset_pid(&s_pitch_pid);
        test_standard_gimbal_reset_pid(&s_yaw_pid);
        test_standard_gimbal_send_zero_speed();
        return;
    }

    float pitch_cmd = test_standard_gimbal_pid_update(&s_pitch_pid,
                                                      test_standard_gimbal_pitch_error_deg,
                                                      test_standard_gimbal_pitch_kp,
                                                      test_standard_gimbal_pitch_ki,
                                                      test_standard_gimbal_pitch_kd,
                                                      dt);
    float yaw_cmd = test_standard_gimbal_pid_update(&s_yaw_pid,
                                                    test_standard_gimbal_yaw_error_deg,
                                                    test_standard_gimbal_yaw_kp,
                                                    test_standard_gimbal_yaw_ki,
                                                    test_standard_gimbal_yaw_kd,
                                                    dt);

    test_standard_gimbal_pitch_cmd_rad_s =
        test_standard_gimbal_clampf(pitch_cmd, test_standard_gimbal_output_limit_rad_s);
    test_standard_gimbal_yaw_cmd_rad_s =
        test_standard_gimbal_clampf(yaw_cmd, test_standard_gimbal_output_limit_rad_s);
    test_standard_gimbal_pitch_cmd_rpm =
        test_standard_gimbal_pitch_cmd_rad_s *
        test_standard_gimbal_pitch_speed_to_angle_sign *
        TEST_STANDARD_GIMBAL_RAD_S_TO_RPM;
    test_standard_gimbal_yaw_cmd_rpm =
        test_standard_gimbal_yaw_cmd_rad_s *
        test_standard_gimbal_yaw_speed_to_angle_sign *
        TEST_STANDARD_GIMBAL_RAD_S_TO_RPM;
    test_standard_gimbal_pitch_cmd_rpm =
        test_standard_gimbal_limit_pitch_cmd(test_standard_gimbal_pitch_cmd_rpm);
    if (test_standard_gimbal_pitch_limit_active != 0.0f) {
        test_standard_gimbal_reset_pid(&s_pitch_pid);
    }

    test_standard_gimbal_capture_pair(
        JC_SetSpeed(TEST_STANDARD_GIMBAL_PITCH_JC2804_ID, test_standard_gimbal_pitch_cmd_rpm),
        JC_SetSpeed(TEST_STANDARD_GIMBAL_YAW_JC4310_ID, test_standard_gimbal_yaw_cmd_rpm));
}

static void test_standard_gimbal_send_telemetry(void)
{
    if (test_standard_gimbal_octo == NULL) {
        return;
    }

    (void)Octolinker_SendU32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 0U,
                             test_standard_gimbal_clock_ms);
    (void)Octolinker_SendF32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 1U,
                             test_standard_gimbal_state);
    (void)Octolinker_SendF32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 2U,
                             test_standard_gimbal_imu_roll);
    (void)Octolinker_SendF32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 3U,
                             test_standard_gimbal_imu_pitch);
    (void)Octolinker_SendF32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 4U,
                             test_standard_gimbal_imu_yaw);
    (void)Octolinker_SendF32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 5U,
                             test_standard_gimbal_pitch_error_deg);
    (void)Octolinker_SendF32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 6U,
                             test_standard_gimbal_yaw_error_deg);
    (void)Octolinker_SendF32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 7U,
                             test_standard_gimbal_pitch_cmd_rad_s);
    (void)Octolinker_SendF32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 8U,
                             test_standard_gimbal_yaw_cmd_rad_s);
    (void)Octolinker_SendF32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 9U,
                             test_standard_gimbal_pitch_cmd_rpm);
    (void)Octolinker_SendF32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 10U,
                             test_standard_gimbal_yaw_cmd_rpm);
    (void)Octolinker_SendU32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 11U,
                             test_standard_gimbal_imu_rx_count);
    (void)Octolinker_SendI32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 12U,
                             test_standard_gimbal_last_status);
    (void)Octolinker_SendF32(test_standard_gimbal_octo,
                             TEST_STANDARD_GIMBAL_TELEM_BASE + 13U,
                             test_standard_gimbal_pitch_limit_active);
}

static void test_standard_gimbal_run_state(void)
{
    uint32_t now = knx_millis();
    uint32_t gap_ms = test_standard_gimbal_period(test_standard_gimbal_mode_gap_ms, 300U);

    switch (s_state) {
    case TEST_STANDARD_GIMBAL_BOOT:
        if (now - s_state_tick >=
            test_standard_gimbal_period(test_standard_gimbal_boot_delay_ms, 500U)) {
            test_standard_gimbal_enter(TEST_STANDARD_GIMBAL_ENTER_SERVO);
        }
        break;

    case TEST_STANDARD_GIMBAL_ENTER_SERVO:
        if (s_command_sent == 0U) {
            test_standard_gimbal_capture_pair(
                JC_EnterServo(TEST_STANDARD_GIMBAL_PITCH_JC2804_ID),
                JC_EnterServo(TEST_STANDARD_GIMBAL_YAW_JC4310_ID));
            s_command_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= gap_ms) {
            test_standard_gimbal_enter(TEST_STANDARD_GIMBAL_SPEED_MODE);
        }
        break;

    case TEST_STANDARD_GIMBAL_SPEED_MODE:
        if (s_command_sent == 0U) {
            test_standard_gimbal_capture_pair(
                JC_SwitchMode(TEST_STANDARD_GIMBAL_PITCH_JC2804_ID, JC_MODE_SPEED),
                JC_SwitchMode(TEST_STANDARD_GIMBAL_YAW_JC4310_ID, JC_MODE_SPEED));
            s_command_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= gap_ms) {
            test_standard_gimbal_reset_pid(&s_pitch_pid);
            test_standard_gimbal_reset_pid(&s_yaw_pid);
            s_control_tick = now;
            test_standard_gimbal_enter(TEST_STANDARD_GIMBAL_CONTROL);
        }
        break;

    case TEST_STANDARD_GIMBAL_CONTROL:
        if (now - s_control_tick >=
            test_standard_gimbal_period(test_standard_gimbal_control_period_ms, 10U)) {
            uint32_t elapsed_ms = now - s_control_tick;
            s_control_tick = now;
            test_standard_gimbal_control_once((float)elapsed_ms * 0.001f);
        }
        break;

    default:
        test_standard_gimbal_enter(TEST_STANDARD_GIMBAL_BOOT);
        break;
    }
}

void test_standard_gimbal_init(void *octo)
{
    test_standard_gimbal_octo = (Octolinker_Instance_t *)octo;
    s_control_tick = 0U;
    s_telem_tick = 0U;
    s_last_restart = test_standard_gimbal_restart;
    test_standard_gimbal_reset_pid(&s_pitch_pid);
    test_standard_gimbal_reset_pid(&s_yaw_pid);
    test_standard_gimbal_enter(TEST_STANDARD_GIMBAL_BOOT);
}

void test_standard_gimbal_loop(void)
{
    uint32_t now = knx_millis();

    if (test_standard_gimbal_restart != s_last_restart) {
        s_last_restart = test_standard_gimbal_restart;
        test_standard_gimbal_send_zero_speed();
        test_standard_gimbal_reset_pid(&s_pitch_pid);
        test_standard_gimbal_reset_pid(&s_yaw_pid);
        test_standard_gimbal_enter(TEST_STANDARD_GIMBAL_BOOT);
    }

    test_standard_gimbal_update_imu_mirrors();
    test_standard_gimbal_run_state();

    if (now - s_telem_tick >=
        test_standard_gimbal_period(test_standard_gimbal_telem_period_ms, 100U)) {
        s_telem_tick = now;
        test_standard_gimbal_send_telemetry();
    }
}
