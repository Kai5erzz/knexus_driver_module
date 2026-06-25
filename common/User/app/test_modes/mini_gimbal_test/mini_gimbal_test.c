#include "mini_gimbal_test.h"
#include "dm_imu_l1.h"
#include "jc_driver.h"
#include "knx_time.h"
#include "octolinker.h"

#define MINI_GIMBAL_TELEM_BASE 210U
#define MINI_GIMBAL_PITCH_MOTOR_ID 1U
#define MINI_GIMBAL_YAW_MOTOR_ID   2U
#define MINI_GIMBAL_RAD_S_TO_RPM   9.549296586f

typedef enum {
    MINI_GIMBAL_BOOT = 0,
    MINI_GIMBAL_ENTER_SERVO,
    MINI_GIMBAL_SPEED_MODE,
    MINI_GIMBAL_CONTROL,
} mini_gimbal_state_t;

typedef struct {
    float integral;
    float last_error;
    uint8_t initialized;
} mini_gimbal_pid_state_t;

Octolinker_Instance_t *mini_gimbal_octo;

float mini_gimbal_enable = 1.0f;
float mini_gimbal_boot_delay_ms = 500.0f;
float mini_gimbal_mode_gap_ms = 300.0f;
float mini_gimbal_control_period_ms = 10.0f;
float mini_gimbal_telem_period_ms = 100.0f;
float mini_gimbal_restart = 0.0f;

float mini_gimbal_pitch_target_deg = 0.0f;
float mini_gimbal_yaw_target_deg = 0.0f;

float mini_gimbal_pitch_kp = 0.1f;
float mini_gimbal_pitch_ki = 0.0f;
float mini_gimbal_pitch_kd = 0.05f;
float mini_gimbal_yaw_kp = 0.3f;
float mini_gimbal_yaw_ki = 0.02f;
float mini_gimbal_yaw_kd = 0.01f;

float mini_gimbal_output_limit_rad_s = 300.0f;
float mini_gimbal_integral_limit = 50.0f;

float mini_gimbal_state = 0.0f;
float mini_gimbal_pitch_error_deg = 0.0f;
float mini_gimbal_yaw_error_deg = 0.0f;
float mini_gimbal_pitch_cmd_rad_s = 0.0f;
float mini_gimbal_yaw_cmd_rad_s = 0.0f;
float mini_gimbal_pitch_cmd_rpm = 0.0f;
float mini_gimbal_yaw_cmd_rpm = 0.0f;
float mini_gimbal_imu_roll = 0.0f;
float mini_gimbal_imu_pitch = 0.0f;
float mini_gimbal_imu_yaw = 0.0f;
float mini_gimbal_imu_yaw_total = 0.0f;
uint32_t mini_gimbal_clock_ms = 0U;
uint32_t mini_gimbal_imu_rx_count = 0U;
int32_t mini_gimbal_last_status = 0;

static mini_gimbal_state_t s_state;
static mini_gimbal_pid_state_t s_pitch_pid;
static mini_gimbal_pid_state_t s_yaw_pid;
static uint32_t s_state_tick;
static uint32_t s_control_tick;
static uint32_t s_telem_tick;
static uint8_t s_command_sent;
static float s_last_restart;

static float mini_gimbal_clampf(float value, float limit)
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

static uint32_t mini_gimbal_period(float value, uint32_t fallback)
{
    if (value < 1.0f) {
        return fallback;
    }
    return (uint32_t)value;
}

static void mini_gimbal_reset_pid(mini_gimbal_pid_state_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->initialized = 0U;
}

static void mini_gimbal_enter(mini_gimbal_state_t state)
{
    s_state = state;
    s_state_tick = knx_millis();
    s_command_sent = 0U;
    mini_gimbal_state = (float)state;
}

static void mini_gimbal_capture_pair(knx_status_t pitch_status, knx_status_t yaw_status)
{
    mini_gimbal_last_status = (int32_t)((pitch_status != KNX_OK) ? pitch_status : yaw_status);
}

static float mini_gimbal_pid_update(mini_gimbal_pid_state_t *pid,
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
    pid->integral = mini_gimbal_clampf(pid->integral, mini_gimbal_integral_limit);

    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    return (kp * error) + (ki * pid->integral) + (kd * derivative);
}

static void mini_gimbal_update_imu_mirrors(void)
{
    mini_gimbal_clock_ms = knx_millis();
    mini_gimbal_imu_roll = dm_imu_l1_data.roll;
    mini_gimbal_imu_pitch = dm_imu_l1_data.pitch;
    mini_gimbal_imu_yaw = dm_imu_l1_data.yaw;
    mini_gimbal_imu_yaw_total = dm_imu_l1_data.yaw_total;
    mini_gimbal_imu_rx_count = dm_imu_l1_data.rx_count;
}

static void mini_gimbal_send_zero_speed(void)
{
    mini_gimbal_pitch_cmd_rad_s = 0.0f;
    mini_gimbal_yaw_cmd_rad_s = 0.0f;
    mini_gimbal_pitch_cmd_rpm = 0.0f;
    mini_gimbal_yaw_cmd_rpm = 0.0f;
    mini_gimbal_capture_pair(JC_SetSpeed(MINI_GIMBAL_PITCH_MOTOR_ID, 0.0f),
                             JC_SetSpeed(MINI_GIMBAL_YAW_MOTOR_ID, 0.0f));
}

static void mini_gimbal_control_once(float dt)
{
    mini_gimbal_pitch_error_deg = mini_gimbal_pitch_target_deg - mini_gimbal_imu_pitch;
    mini_gimbal_yaw_error_deg = mini_gimbal_yaw_target_deg - mini_gimbal_imu_yaw;

    if (mini_gimbal_enable <= 0.0f) {
        mini_gimbal_reset_pid(&s_pitch_pid);
        mini_gimbal_reset_pid(&s_yaw_pid);
        mini_gimbal_send_zero_speed();
        return;
    }

    float pitch_cmd = mini_gimbal_pid_update(&s_pitch_pid,
                                             mini_gimbal_pitch_error_deg,
                                             mini_gimbal_pitch_kp,
                                             mini_gimbal_pitch_ki,
                                             mini_gimbal_pitch_kd,
                                             dt);
    float yaw_cmd = mini_gimbal_pid_update(&s_yaw_pid,
                                           mini_gimbal_yaw_error_deg,
                                           mini_gimbal_yaw_kp,
                                           mini_gimbal_yaw_ki,
                                           mini_gimbal_yaw_kd,
                                           dt);

    mini_gimbal_pitch_cmd_rad_s = mini_gimbal_clampf(pitch_cmd, mini_gimbal_output_limit_rad_s);
    mini_gimbal_yaw_cmd_rad_s = mini_gimbal_clampf(yaw_cmd, mini_gimbal_output_limit_rad_s);
    mini_gimbal_pitch_cmd_rpm = mini_gimbal_pitch_cmd_rad_s * MINI_GIMBAL_RAD_S_TO_RPM;
    mini_gimbal_yaw_cmd_rpm = mini_gimbal_yaw_cmd_rad_s * MINI_GIMBAL_RAD_S_TO_RPM;

    mini_gimbal_capture_pair(JC_SetSpeed(MINI_GIMBAL_PITCH_MOTOR_ID, mini_gimbal_pitch_cmd_rpm),
                             JC_SetSpeed(MINI_GIMBAL_YAW_MOTOR_ID, mini_gimbal_yaw_cmd_rpm));
}

static void mini_gimbal_send_telemetry(void)
{
    if (mini_gimbal_octo == NULL) {
        return;
    }

    (void)Octolinker_SendU32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 0U, mini_gimbal_clock_ms);
    (void)Octolinker_SendF32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 1U, mini_gimbal_state);
    (void)Octolinker_SendF32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 2U, mini_gimbal_imu_roll);
    (void)Octolinker_SendF32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 3U, mini_gimbal_imu_pitch);
    (void)Octolinker_SendF32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 4U, mini_gimbal_imu_yaw);
    (void)Octolinker_SendF32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 5U, mini_gimbal_pitch_error_deg);
    (void)Octolinker_SendF32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 6U, mini_gimbal_yaw_error_deg);
    (void)Octolinker_SendF32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 7U, mini_gimbal_pitch_cmd_rad_s);
    (void)Octolinker_SendF32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 8U, mini_gimbal_yaw_cmd_rad_s);
    (void)Octolinker_SendF32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 9U, mini_gimbal_pitch_cmd_rpm);
    (void)Octolinker_SendF32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 10U, mini_gimbal_yaw_cmd_rpm);
    (void)Octolinker_SendU32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 11U, mini_gimbal_imu_rx_count);
    (void)Octolinker_SendI32(mini_gimbal_octo, MINI_GIMBAL_TELEM_BASE + 12U, mini_gimbal_last_status);
}

static void mini_gimbal_run_state(void)
{
    uint32_t now = knx_millis();
    uint32_t gap_ms = mini_gimbal_period(mini_gimbal_mode_gap_ms, 300U);

    switch (s_state) {
    case MINI_GIMBAL_BOOT:
        if (now - s_state_tick >= mini_gimbal_period(mini_gimbal_boot_delay_ms, 500U)) {
            mini_gimbal_enter(MINI_GIMBAL_ENTER_SERVO);
        }
        break;

    case MINI_GIMBAL_ENTER_SERVO:
        if (s_command_sent == 0U) {
            mini_gimbal_capture_pair(JC_EnterServo(MINI_GIMBAL_PITCH_MOTOR_ID),
                                     JC_EnterServo(MINI_GIMBAL_YAW_MOTOR_ID));
            s_command_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= gap_ms) {
            mini_gimbal_enter(MINI_GIMBAL_SPEED_MODE);
        }
        break;

    case MINI_GIMBAL_SPEED_MODE:
        if (s_command_sent == 0U) {
            mini_gimbal_capture_pair(JC_SwitchMode(MINI_GIMBAL_PITCH_MOTOR_ID, JC_MODE_SPEED),
                                     JC_SwitchMode(MINI_GIMBAL_YAW_MOTOR_ID, JC_MODE_SPEED));
            s_command_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= gap_ms) {
            mini_gimbal_reset_pid(&s_pitch_pid);
            mini_gimbal_reset_pid(&s_yaw_pid);
            s_control_tick = now;
            mini_gimbal_enter(MINI_GIMBAL_CONTROL);
        }
        break;

    case MINI_GIMBAL_CONTROL:
        if (now - s_control_tick >= mini_gimbal_period(mini_gimbal_control_period_ms, 10U)) {
            uint32_t elapsed_ms = now - s_control_tick;
            s_control_tick = now;
            mini_gimbal_control_once((float)elapsed_ms * 0.001f);
        }
        break;

    default:
        mini_gimbal_enter(MINI_GIMBAL_BOOT);
        break;
    }
}

void mini_gimbal_test_init(void *octo)
{
    mini_gimbal_octo = (Octolinker_Instance_t *)octo;
    s_control_tick = 0U;
    s_telem_tick = 0U;
    s_last_restart = mini_gimbal_restart;
    mini_gimbal_reset_pid(&s_pitch_pid);
    mini_gimbal_reset_pid(&s_yaw_pid);
    mini_gimbal_enter(MINI_GIMBAL_BOOT);
}

void mini_gimbal_test_loop(void)
{
    uint32_t now = knx_millis();

    if (mini_gimbal_restart != s_last_restart) {
        s_last_restart = mini_gimbal_restart;
        mini_gimbal_send_zero_speed();
        mini_gimbal_reset_pid(&s_pitch_pid);
        mini_gimbal_reset_pid(&s_yaw_pid);
        mini_gimbal_enter(MINI_GIMBAL_BOOT);
    }

    mini_gimbal_update_imu_mirrors();
    mini_gimbal_run_state();

    if (now - s_telem_tick >= mini_gimbal_period(mini_gimbal_telem_period_ms, 100U)) {
        s_telem_tick = now;
        mini_gimbal_send_telemetry();
    }
}
