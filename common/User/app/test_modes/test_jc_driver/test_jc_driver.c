#include "test_jc_driver.h"
#include "jc_driver.h"
#include "knx_time.h"
#include "octolinker.h"

#define TEST_JC_DRIVER_TELEM_BASE 230U
#define TEST_JC_DRIVER_PITCH_JC2804_ID 1U
#define TEST_JC_DRIVER_JC4310_ID       3U
#define TEST_JC_DRIVER_MODEL_JC2804    2804.0f
#define TEST_JC_DRIVER_MODEL_JC4310    4310.0f
#define TEST_JC_DRIVER_MAX_TORQUE_NM   0.02f
#define TEST_JC_DRIVER_MAX_SPEED_RPM   50.0f

typedef enum {
    TEST_JC_DRIVER_BOOT = 0,
    TEST_JC_DRIVER_ENTER_SERVO,
    TEST_JC_DRIVER_TORQUE_MODE,
    TEST_JC_DRIVER_TORQUE_STEPS,
    TEST_JC_DRIVER_SPEED_MODE,
    TEST_JC_DRIVER_SPEED_STEPS,
    TEST_JC_DRIVER_POSITION_MODE,
    TEST_JC_DRIVER_POSITION_STEPS,
    TEST_JC_DRIVER_STOP,
    TEST_JC_DRIVER_NEXT_MOTOR,
    TEST_JC_DRIVER_DONE,
} test_jc_driver_state_t;

typedef enum {
    TEST_JC_DRIVER_READ_VOLTAGE = 0,
    TEST_JC_DRIVER_READ_BUS_CURRENT,
    TEST_JC_DRIVER_READ_SPEED,
    TEST_JC_DRIVER_READ_POSITION,
    TEST_JC_DRIVER_READ_DRV_TEMP,
    TEST_JC_DRIVER_READ_MOT_TEMP,
    TEST_JC_DRIVER_READ_ERROR,
    TEST_JC_DRIVER_READ_COUNT,
} test_jc_driver_read_step_t;

typedef struct {
    uint8_t id;
    float model;
    float role;
} test_jc_driver_motor_t;

static const test_jc_driver_motor_t s_motors[] = {
    { TEST_JC_DRIVER_PITCH_JC2804_ID, TEST_JC_DRIVER_MODEL_JC2804, 1.0f },
    { TEST_JC_DRIVER_JC4310_ID,       TEST_JC_DRIVER_MODEL_JC4310, 2.0f },
};

static const float s_torque_steps_nm[] = {
    0.00f, 0.005f, 0.010f, 0.015f, 0.020f,
    0.015f, 0.010f, 0.005f, 0.00f,
   -0.005f, -0.010f, -0.015f, -0.020f, 0.00f,
};

static const float s_speed_steps_rpm[] = {
    0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f,
    0.0f, -10.0f, -20.0f, -30.0f, -40.0f, -50.0f, 0.0f,
};

static const float s_position_steps_deg[] = {
    0.0f, 90.0f, 180.0f, 360.0f, 0.0f,
};

Octolinker_Instance_t *test_jc_driver_octo;

float test_jc_driver_auto_enable = 1.0f;
float test_jc_driver_boot_delay_ms = 500.0f;
float test_jc_driver_mode_gap_ms = 300.0f;
float test_jc_driver_step_hold_ms = 1500.0f;
float test_jc_driver_poll_enable = 1.0f;
float test_jc_driver_poll_period_ms = 50.0f;
float test_jc_driver_timeout_ms = 20.0f;
float test_jc_driver_restart = 0.0f;

float test_jc_driver_state = 0.0f;
float test_jc_driver_active_motor_id = 1.0f;
float test_jc_driver_active_model = TEST_JC_DRIVER_MODEL_JC2804;
float test_jc_driver_active_role = 1.0f; /* 1=mini gimbal pitch JC2804, 2=JC4310 ID3 */
float test_jc_driver_step_index = 0.0f;
float test_jc_driver_commanded_torque_nm = 0.0f;
float test_jc_driver_commanded_speed_rpm = 0.0f;
float test_jc_driver_commanded_position_deg = 0.0f;

float test_jc_driver_voltage_v = -1.0f;
float test_jc_driver_bus_current_a = -1.0f;
float test_jc_driver_speed_feedback_rpm = -1.0f;
float test_jc_driver_position_feedback_deg = -1.0f;
float test_jc_driver_drv_temp_c = -1.0f;
float test_jc_driver_mot_temp_c = -1.0f;
uint32_t test_jc_driver_error_code = 0xFFFFFFFFU;
uint32_t test_jc_driver_tx_count = 0U;
uint32_t test_jc_driver_rx_count = 0U;
uint32_t test_jc_driver_timeout_count = 0U;
int32_t test_jc_driver_last_status = 0;

static test_jc_driver_state_t s_state;
static uint32_t s_state_tick;
static uint32_t s_poll_tick;
static uint32_t s_telem_tick;
static uint8_t s_motor_index;
static uint8_t s_step_index;
static uint8_t s_step_sent;
static uint8_t s_read_step;
static float s_last_restart;

static float test_jc_driver_clampf(float value, float limit)
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

static uint32_t test_jc_driver_period(float value, uint32_t fallback)
{
    if (value < 1.0f) {
        return fallback;
    }
    return (uint32_t)value;
}

static const test_jc_driver_motor_t *test_jc_driver_active_motor(void)
{
    if (s_motor_index >= (uint8_t)(sizeof(s_motors) / sizeof(s_motors[0]))) {
        return &s_motors[0];
    }
    return &s_motors[s_motor_index];
}

static void test_jc_driver_refresh_active(void)
{
    const test_jc_driver_motor_t *motor = test_jc_driver_active_motor();
    test_jc_driver_active_motor_id = (float)motor->id;
    test_jc_driver_active_model = motor->model;
    test_jc_driver_active_role = motor->role;
}

static void test_jc_driver_capture_status(knx_status_t status)
{
    test_jc_driver_last_status = (int32_t)status;
}

static void test_jc_driver_capture_stats(void)
{
    JC_Stats stats;
    JC_GetStats(&stats);
    test_jc_driver_tx_count = stats.tx_count;
    test_jc_driver_rx_count = stats.rx_count;
    test_jc_driver_timeout_count = stats.timeout_count;
    test_jc_driver_last_status = (int32_t)stats.last_status;
}

static void test_jc_driver_enter(test_jc_driver_state_t state)
{
    s_state = state;
    s_state_tick = knx_millis();
    s_step_sent = 0U;
    test_jc_driver_state = (float)state;
    test_jc_driver_step_index = (float)s_step_index;
    test_jc_driver_refresh_active();
}

static void test_jc_driver_reset_sequence(void)
{
    s_motor_index = 0U;
    s_step_index = 0U;
    s_step_sent = 0U;
    s_read_step = 0U;
    test_jc_driver_commanded_torque_nm = 0.0f;
    test_jc_driver_commanded_speed_rpm = 0.0f;
    test_jc_driver_commanded_position_deg = 0.0f;
    test_jc_driver_enter(TEST_JC_DRIVER_BOOT);
}

static void test_jc_driver_poll_once(uint8_t id)
{
    switch ((test_jc_driver_read_step_t)s_read_step) {
    case TEST_JC_DRIVER_READ_VOLTAGE:
        test_jc_driver_voltage_v = JC_ReadVoltage(id);
        break;
    case TEST_JC_DRIVER_READ_BUS_CURRENT:
        test_jc_driver_bus_current_a = JC_ReadBusCurrent(id);
        break;
    case TEST_JC_DRIVER_READ_SPEED:
        test_jc_driver_speed_feedback_rpm = JC_ReadSpeed(id);
        break;
    case TEST_JC_DRIVER_READ_POSITION:
        test_jc_driver_position_feedback_deg = JC_ReadPosition(id);
        break;
    case TEST_JC_DRIVER_READ_DRV_TEMP:
        test_jc_driver_drv_temp_c = JC_ReadDrvTemp(id);
        break;
    case TEST_JC_DRIVER_READ_MOT_TEMP:
        test_jc_driver_mot_temp_c = JC_ReadMotTemp(id);
        break;
    case TEST_JC_DRIVER_READ_ERROR:
        test_jc_driver_error_code = JC_ReadError(id);
        break;
    default:
        break;
    }

    test_jc_driver_capture_stats();

    s_read_step++;
    if (s_read_step >= (uint8_t)TEST_JC_DRIVER_READ_COUNT) {
        s_read_step = 0U;
    }
}

static void test_jc_driver_auto_run(void)
{
    uint32_t now = knx_millis();
    uint32_t gap_ms = test_jc_driver_period(test_jc_driver_mode_gap_ms, 300U);
    uint32_t hold_ms = test_jc_driver_period(test_jc_driver_step_hold_ms, 1500U);
    const test_jc_driver_motor_t *motor = test_jc_driver_active_motor();
    uint8_t id = motor->id;

    test_jc_driver_refresh_active();

    switch (s_state) {
    case TEST_JC_DRIVER_BOOT:
        if (now - s_state_tick >= test_jc_driver_period(test_jc_driver_boot_delay_ms, 500U)) {
            test_jc_driver_enter(TEST_JC_DRIVER_ENTER_SERVO);
        }
        break;

    case TEST_JC_DRIVER_ENTER_SERVO:
        if (s_step_sent == 0U) {
            test_jc_driver_capture_status(JC_EnterServo(id));
            s_step_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= gap_ms) {
            test_jc_driver_enter(TEST_JC_DRIVER_TORQUE_MODE);
        }
        break;

    case TEST_JC_DRIVER_TORQUE_MODE:
        if (s_step_sent == 0U) {
            test_jc_driver_capture_status(JC_SwitchMode(id, JC_MODE_TORQUE));
            s_step_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= gap_ms) {
            s_step_index = 0U;
            test_jc_driver_enter(TEST_JC_DRIVER_TORQUE_STEPS);
        }
        break;

    case TEST_JC_DRIVER_TORQUE_STEPS:
        if (s_step_index >= (uint8_t)(sizeof(s_torque_steps_nm) / sizeof(s_torque_steps_nm[0]))) {
            test_jc_driver_enter(TEST_JC_DRIVER_SPEED_MODE);
            break;
        }
        if (s_step_sent == 0U) {
            float torque = test_jc_driver_clampf(s_torque_steps_nm[s_step_index], TEST_JC_DRIVER_MAX_TORQUE_NM);
            test_jc_driver_commanded_torque_nm = torque;
            test_jc_driver_commanded_speed_rpm = 0.0f;
            test_jc_driver_commanded_position_deg = 0.0f;
            test_jc_driver_step_index = (float)s_step_index;
            test_jc_driver_capture_status(JC_SetTorque(id, torque));
            s_step_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= hold_ms) {
            s_step_index++;
            s_step_sent = 0U;
        }
        break;

    case TEST_JC_DRIVER_SPEED_MODE:
        if (s_step_sent == 0U) {
            test_jc_driver_capture_status(JC_SwitchMode(id, JC_MODE_SPEED));
            s_step_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= gap_ms) {
            s_step_index = 0U;
            test_jc_driver_enter(TEST_JC_DRIVER_SPEED_STEPS);
        }
        break;

    case TEST_JC_DRIVER_SPEED_STEPS:
        if (s_step_index >= (uint8_t)(sizeof(s_speed_steps_rpm) / sizeof(s_speed_steps_rpm[0]))) {
            test_jc_driver_enter(TEST_JC_DRIVER_POSITION_MODE);
            break;
        }
        if (s_step_sent == 0U) {
            float speed = test_jc_driver_clampf(s_speed_steps_rpm[s_step_index], TEST_JC_DRIVER_MAX_SPEED_RPM);
            test_jc_driver_commanded_torque_nm = 0.0f;
            test_jc_driver_commanded_speed_rpm = speed;
            test_jc_driver_commanded_position_deg = 0.0f;
            test_jc_driver_step_index = (float)s_step_index;
            test_jc_driver_capture_status(JC_SetSpeed(id, speed));
            s_step_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= hold_ms) {
            s_step_index++;
            s_step_sent = 0U;
        }
        break;

    case TEST_JC_DRIVER_POSITION_MODE:
        if (s_step_sent == 0U) {
            test_jc_driver_capture_status(JC_SwitchMode(id, JC_MODE_POS_DIRECT));
            s_step_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= gap_ms) {
            s_step_index = 0U;
            test_jc_driver_enter(TEST_JC_DRIVER_POSITION_STEPS);
        }
        break;

    case TEST_JC_DRIVER_POSITION_STEPS:
        if (s_step_index >= (uint8_t)(sizeof(s_position_steps_deg) / sizeof(s_position_steps_deg[0]))) {
            test_jc_driver_enter(TEST_JC_DRIVER_STOP);
            break;
        }
        if (s_step_sent == 0U) {
            float position = s_position_steps_deg[s_step_index];
            test_jc_driver_commanded_torque_nm = 0.0f;
            test_jc_driver_commanded_speed_rpm = 0.0f;
            test_jc_driver_commanded_position_deg = position;
            test_jc_driver_step_index = (float)s_step_index;
            test_jc_driver_capture_status(JC_SetAbsPosition(id, position));
            s_step_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= hold_ms) {
            s_step_index++;
            s_step_sent = 0U;
        }
        break;

    case TEST_JC_DRIVER_STOP:
        if (s_step_sent == 0U) {
            (void)JC_SetSpeed(id, 0.0f);
            (void)JC_SetTorque(id, 0.0f);
            test_jc_driver_capture_status(JC_Idle(id));
            test_jc_driver_commanded_torque_nm = 0.0f;
            test_jc_driver_commanded_speed_rpm = 0.0f;
            test_jc_driver_commanded_position_deg = 0.0f;
            s_step_sent = 1U;
            s_state_tick = now;
        } else if (now - s_state_tick >= gap_ms) {
            test_jc_driver_enter(TEST_JC_DRIVER_NEXT_MOTOR);
        }
        break;

    case TEST_JC_DRIVER_NEXT_MOTOR:
        s_motor_index++;
        if (s_motor_index >= (uint8_t)(sizeof(s_motors) / sizeof(s_motors[0]))) {
            test_jc_driver_enter(TEST_JC_DRIVER_DONE);
        } else {
            s_step_index = 0U;
            s_step_sent = 0U;
            s_read_step = 0U;
            test_jc_driver_enter(TEST_JC_DRIVER_BOOT);
        }
        break;

    case TEST_JC_DRIVER_DONE:
    default:
        break;
    }
}

static void test_jc_driver_send_telemetry(void)
{
    if (test_jc_driver_octo == NULL) {
        return;
    }

    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 0U, test_jc_driver_state);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 1U, test_jc_driver_active_motor_id);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 2U, test_jc_driver_active_model);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 3U, test_jc_driver_active_role);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 4U, test_jc_driver_step_index);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 5U, test_jc_driver_commanded_torque_nm);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 6U, test_jc_driver_commanded_speed_rpm);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 7U, test_jc_driver_commanded_position_deg);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 8U, test_jc_driver_voltage_v);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 9U, test_jc_driver_bus_current_a);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 10U, test_jc_driver_speed_feedback_rpm);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 11U, test_jc_driver_position_feedback_deg);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 12U, test_jc_driver_drv_temp_c);
    (void)Octolinker_SendF32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 13U, test_jc_driver_mot_temp_c);
    (void)Octolinker_SendU32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 14U, test_jc_driver_error_code);
    (void)Octolinker_SendU32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 15U, test_jc_driver_tx_count);
    (void)Octolinker_SendU32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 16U, test_jc_driver_rx_count);
    (void)Octolinker_SendU32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 17U, test_jc_driver_timeout_count);
    (void)Octolinker_SendI32(test_jc_driver_octo, TEST_JC_DRIVER_TELEM_BASE + 18U, test_jc_driver_last_status);
}

void test_jc_driver_init(void *octo)
{
    test_jc_driver_octo = (Octolinker_Instance_t *)octo;
    s_poll_tick = 0U;
    s_telem_tick = 0U;
    s_last_restart = test_jc_driver_restart;
    test_jc_driver_reset_sequence();
}

void test_jc_driver_loop(void)
{
    uint32_t now = knx_millis();
    const test_jc_driver_motor_t *motor = test_jc_driver_active_motor();

    JC_SetTimeoutMs(test_jc_driver_period(test_jc_driver_timeout_ms, 20U));

    if (test_jc_driver_restart != s_last_restart) {
        s_last_restart = test_jc_driver_restart;
        test_jc_driver_reset_sequence();
    }

    if (test_jc_driver_auto_enable > 0.0f) {
        test_jc_driver_auto_run();
    }

    if (test_jc_driver_poll_enable > 0.0f &&
        now - s_poll_tick >= test_jc_driver_period(test_jc_driver_poll_period_ms, 50U)) {
        s_poll_tick = now;
        test_jc_driver_poll_once(motor->id);
    }

    if (now - s_telem_tick >= 100U) {
        s_telem_tick = now;
        test_jc_driver_send_telemetry();
    }
}