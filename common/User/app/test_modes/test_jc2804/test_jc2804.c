#include "test_jc2804.h"
#include "jc_driver.h"
#include "knx_time.h"
#include "octolinker.h"

#define TEST_JC2804_TELEM_BASE 150U
#define TEST_JC2804_MOTOR1_ID  1U
#define TEST_JC2804_MOTOR2_ID  2U
#define TEST_JC2804_MAX_TORQUE_NM 0.02f
#define TEST_JC2804_MAX_SPEED_RPM 50.0f

typedef enum {
    TEST_JC_READ_VOLTAGE = 0,
    TEST_JC_READ_BUS_CURRENT,
    TEST_JC_READ_SPEED,
    TEST_JC_READ_POSITION,
    TEST_JC_READ_DRV_TEMP,
    TEST_JC_READ_MOT_TEMP,
    TEST_JC_READ_ERROR,
    TEST_JC_READ_COUNT,
} test_jc_read_step_t;

typedef enum {
    TEST_JC_AUTO_BOOT = 0,
    TEST_JC_AUTO_ENTER_SERVO,
    TEST_JC_AUTO_TORQUE_MODE,
    TEST_JC_AUTO_TORQUE_STEPS,
    TEST_JC_AUTO_SPEED_MODE,
    TEST_JC_AUTO_SPEED_STEPS,
    TEST_JC_AUTO_POSITION_MODE,
    TEST_JC_AUTO_POSITION_STEPS,
    TEST_JC_AUTO_STOP,
    TEST_JC_AUTO_DONE,
} test_jc_auto_state_t;

Octolinker_Instance_t *test_jc2804_octo;

/* Manual/debug control variables. Manual control is used only when
 * test_jc2804_auto_enable <= 0. */
float test_jc2804_motor_id = 1.0f;
float test_jc2804_poll_enable = 1.0f;
float test_jc2804_poll_period_ms = 50.0f;
float test_jc2804_timeout_ms = 20.0f;

/* 0=off, 1=speed, 2=torque, 3=abs position, 4=PV */
float test_jc2804_control_enable = 0.0f;
float test_jc2804_control_mode = 0.0f;
float test_jc2804_speed_rpm = 0.0f;
float test_jc2804_torque_nm = 0.0f;
float test_jc2804_position_deg = 0.0f;
float test_jc2804_pv_velocity_rpm = 0.0f;
float test_jc2804_control_period_ms = 20.0f;

/* Automatic dual-motor boot test. */
float test_jc2804_auto_enable = 1.0f;
float test_jc2804_auto_state = 0.0f;
float test_jc2804_auto_step = 0.0f;
float test_jc2804_boot_delay_ms = 500.0f;
float test_jc2804_mode_gap_ms = 300.0f;
float test_jc2804_step_hold_ms = 1500.0f;
float test_jc2804_active_feedback_id = 1.0f;
float test_jc2804_commanded_torque_nm = 0.0f;
float test_jc2804_commanded_speed_rpm = 0.0f;
float test_jc2804_commanded_position_deg = 0.0f;

/* Change this value from GDB/OctoLink to trigger one-shot commands:
 * 1=idle, 2=enter servo, 3=speed mode, 4=torque mode, 5=position-direct mode,
 * 6=set origin, 7=calibrate, 8=save params, 9=reboot.
 */
float test_jc2804_command = 0.0f;

float test_jc2804_voltage_v = -1.0f;
float test_jc2804_bus_current_a = -1.0f;
float test_jc2804_speed_feedback_rpm = -1.0f;
float test_jc2804_position_feedback_deg = -1.0f;
float test_jc2804_drv_temp_c = -1.0f;
float test_jc2804_mot_temp_c = -1.0f;
uint32_t test_jc2804_error_code = 0xFFFFFFFFU;
int32_t test_jc2804_last_status = 0;

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

static uint32_t s_poll_tick;
static uint32_t s_control_tick;
static uint32_t s_telem_tick;
static uint32_t s_auto_tick;
static uint8_t s_read_step;
static uint8_t s_poll_motor_index;
static uint8_t s_auto_step_index;
static uint8_t s_auto_step_sent;
static test_jc_auto_state_t s_auto_state;
static float s_last_command;

static float test_jc2804_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float test_jc2804_clampf(float value, float limit)
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

static uint8_t test_jc2804_id(void)
{
    int32_t id = (int32_t)test_jc2804_motor_id;
    if (id < (int32_t)JC_MOTOR_ID_MIN) {
        id = (int32_t)JC_MOTOR_ID_MIN;
    } else if (id > (int32_t)JC_MOTOR_ID_MAX) {
        id = (int32_t)JC_MOTOR_ID_MAX;
    }
    return (uint8_t)id;
}

static uint32_t test_jc2804_period(float value, uint32_t fallback)
{
    if (value < 1.0f) {
        return fallback;
    }
    return (uint32_t)value;
}

static void test_jc2804_capture_status(knx_status_t status)
{
    test_jc2804_last_status = (int32_t)status;
}

static void test_jc2804_capture_two(knx_status_t a, knx_status_t b)
{
    test_jc2804_capture_status((a != KNX_OK) ? a : b);
}

static void test_jc2804_auto_enter(test_jc_auto_state_t state)
{
    s_auto_state = state;
    s_auto_tick = knx_millis();
    s_auto_step_index = 0U;
    s_auto_step_sent = 0U;
    test_jc2804_auto_state = (float)state;
    test_jc2804_auto_step = 0.0f;
}

static void test_jc2804_run_command(uint8_t id, uint8_t cmd)
{
    knx_status_t status = KNX_OK;

    switch (cmd) {
    case 1U:
        status = JC_Idle(id);
        break;
    case 2U:
        status = JC_EnterServo(id);
        break;
    case 3U:
        status = JC_SwitchMode(id, JC_MODE_SPEED);
        break;
    case 4U:
        status = JC_SwitchMode(id, JC_MODE_TORQUE);
        break;
    case 5U:
        status = JC_SwitchMode(id, JC_MODE_POS_DIRECT);
        break;
    case 6U:
        status = JC_SetOrigin(id);
        break;
    case 7U:
        status = JC_Calibrate(id);
        break;
    case 8U:
        status = JC_SaveParams(id);
        break;
    case 9U:
        status = JC_Reboot(id);
        break;
    default:
        status = KNX_INVALID_ARG;
        break;
    }

    test_jc2804_capture_status(status);
}

static void test_jc2804_poll_once(uint8_t id)
{
    test_jc2804_active_feedback_id = (float)id;

    switch ((test_jc_read_step_t)s_read_step) {
    case TEST_JC_READ_VOLTAGE:
        test_jc2804_voltage_v = JC_ReadVoltage(id);
        break;
    case TEST_JC_READ_BUS_CURRENT:
        test_jc2804_bus_current_a = JC_ReadBusCurrent(id);
        break;
    case TEST_JC_READ_SPEED:
        test_jc2804_speed_feedback_rpm = JC_ReadSpeed(id);
        break;
    case TEST_JC_READ_POSITION:
        test_jc2804_position_feedback_deg = JC_ReadPosition(id);
        break;
    case TEST_JC_READ_DRV_TEMP:
        test_jc2804_drv_temp_c = JC_ReadDrvTemp(id);
        break;
    case TEST_JC_READ_MOT_TEMP:
        test_jc2804_mot_temp_c = JC_ReadMotTemp(id);
        break;
    case TEST_JC_READ_ERROR:
        test_jc2804_error_code = JC_ReadError(id);
        break;
    default:
        break;
    }

    JC_Stats stats;
    JC_GetStats(&stats);
    test_jc2804_capture_status(stats.last_status);

    s_read_step++;
    if (s_read_step >= (uint8_t)TEST_JC_READ_COUNT) {
        s_read_step = 0U;
        s_poll_motor_index ^= 1U;
    }
}

static void test_jc2804_control_once(uint8_t id)
{
    uint8_t mode = (uint8_t)test_jc2804_control_mode;
    knx_status_t status = KNX_OK;

    if (test_jc2804_control_enable <= 0.0f || mode == 0U) {
        return;
    }

    if (mode == 1U) {
        test_jc2804_speed_rpm = test_jc2804_clampf(test_jc2804_speed_rpm,
                                                   TEST_JC2804_MAX_SPEED_RPM);
        status = JC_SetSpeed(id, test_jc2804_speed_rpm);
    } else if (mode == 2U) {
        test_jc2804_torque_nm = test_jc2804_clampf(test_jc2804_torque_nm,
                                                   TEST_JC2804_MAX_TORQUE_NM);
        status = JC_SetTorque(id, test_jc2804_torque_nm);
    } else if (mode == 3U) {
        status = JC_SetAbsPosition(id, test_jc2804_position_deg);
    } else if (mode == 4U) {
        test_jc2804_pv_velocity_rpm = test_jc2804_clampf(test_jc2804_pv_velocity_rpm,
                                                         TEST_JC2804_MAX_SPEED_RPM);
        status = JC_SendPV(id, test_jc2804_position_deg, test_jc2804_pv_velocity_rpm);
    } else {
        status = KNX_INVALID_ARG;
    }

    test_jc2804_capture_status(status);
}

static void test_jc2804_auto_send_both_mode(JC_ControlMode mode)
{
    knx_status_t s1 = JC_SwitchMode(TEST_JC2804_MOTOR1_ID, mode);
    knx_status_t s2 = JC_SwitchMode(TEST_JC2804_MOTOR2_ID, mode);
    test_jc2804_capture_two(s1, s2);
}

static void test_jc2804_auto_run(void)
{
    uint32_t now = knx_millis();
    uint32_t gap_ms = test_jc2804_period(test_jc2804_mode_gap_ms, 300U);
    uint32_t hold_ms = test_jc2804_period(test_jc2804_step_hold_ms, 1500U);

    switch (s_auto_state) {
    case TEST_JC_AUTO_BOOT:
        if (now - s_auto_tick >= test_jc2804_period(test_jc2804_boot_delay_ms, 500U)) {
            test_jc2804_auto_enter(TEST_JC_AUTO_ENTER_SERVO);
        }
        break;

    case TEST_JC_AUTO_ENTER_SERVO:
        if (s_auto_step_sent == 0U) {
            knx_status_t s1 = JC_EnterServo(TEST_JC2804_MOTOR1_ID);
            knx_status_t s2 = JC_EnterServo(TEST_JC2804_MOTOR2_ID);
            test_jc2804_capture_two(s1, s2);
            s_auto_step_sent = 1U;
            s_auto_tick = now;
        } else if (now - s_auto_tick >= gap_ms) {
            test_jc2804_auto_enter(TEST_JC_AUTO_TORQUE_MODE);
        }
        break;

    case TEST_JC_AUTO_TORQUE_MODE:
        if (s_auto_step_sent == 0U) {
            test_jc2804_auto_send_both_mode(JC_MODE_TORQUE);
            s_auto_step_sent = 1U;
            s_auto_tick = now;
        } else if (now - s_auto_tick >= gap_ms) {
            test_jc2804_auto_enter(TEST_JC_AUTO_TORQUE_STEPS);
        }
        break;

    case TEST_JC_AUTO_TORQUE_STEPS:
        if (s_auto_step_index >= (uint8_t)(sizeof(s_torque_steps_nm) / sizeof(s_torque_steps_nm[0]))) {
            test_jc2804_auto_enter(TEST_JC_AUTO_SPEED_MODE);
            break;
        }
        if (s_auto_step_sent == 0U) {
            float torque = test_jc2804_clampf(s_torque_steps_nm[s_auto_step_index],
                                              TEST_JC2804_MAX_TORQUE_NM);
            test_jc2804_commanded_torque_nm = torque;
            test_jc2804_commanded_speed_rpm = 0.0f;
            test_jc2804_commanded_position_deg = 0.0f;
            test_jc2804_auto_step = (float)s_auto_step_index;
            knx_status_t s1 = JC_SetTorque(TEST_JC2804_MOTOR1_ID, torque);
            knx_status_t s2 = JC_SetTorque(TEST_JC2804_MOTOR2_ID, torque);
            test_jc2804_capture_two(s1, s2);
            s_auto_step_sent = 1U;
            s_auto_tick = now;
        } else if (now - s_auto_tick >= hold_ms) {
            s_auto_step_index++;
            s_auto_step_sent = 0U;
        }
        break;

    case TEST_JC_AUTO_SPEED_MODE:
        if (s_auto_step_sent == 0U) {
            test_jc2804_auto_send_both_mode(JC_MODE_SPEED);
            s_auto_step_sent = 1U;
            s_auto_tick = now;
        } else if (now - s_auto_tick >= gap_ms) {
            test_jc2804_auto_enter(TEST_JC_AUTO_SPEED_STEPS);
        }
        break;

    case TEST_JC_AUTO_SPEED_STEPS:
        if (s_auto_step_index >= (uint8_t)(sizeof(s_speed_steps_rpm) / sizeof(s_speed_steps_rpm[0]))) {
            test_jc2804_auto_enter(TEST_JC_AUTO_POSITION_MODE);
            break;
        }
        if (s_auto_step_sent == 0U) {
            float speed = test_jc2804_clampf(s_speed_steps_rpm[s_auto_step_index],
                                             TEST_JC2804_MAX_SPEED_RPM);
            test_jc2804_commanded_torque_nm = 0.0f;
            test_jc2804_commanded_speed_rpm = speed;
            test_jc2804_commanded_position_deg = 0.0f;
            test_jc2804_auto_step = (float)s_auto_step_index;
            knx_status_t s1 = JC_SetSpeed(TEST_JC2804_MOTOR1_ID, speed);
            knx_status_t s2 = JC_SetSpeed(TEST_JC2804_MOTOR2_ID, speed);
            test_jc2804_capture_two(s1, s2);
            s_auto_step_sent = 1U;
            s_auto_tick = now;
        } else if (now - s_auto_tick >= hold_ms) {
            s_auto_step_index++;
            s_auto_step_sent = 0U;
        }
        break;

    case TEST_JC_AUTO_POSITION_MODE:
        if (s_auto_step_sent == 0U) {
            test_jc2804_auto_send_both_mode(JC_MODE_POS_DIRECT);
            s_auto_step_sent = 1U;
            s_auto_tick = now;
        } else if (now - s_auto_tick >= gap_ms) {
            test_jc2804_auto_enter(TEST_JC_AUTO_POSITION_STEPS);
        }
        break;

    case TEST_JC_AUTO_POSITION_STEPS:
        if (s_auto_step_index >= (uint8_t)(sizeof(s_position_steps_deg) / sizeof(s_position_steps_deg[0]))) {
            test_jc2804_auto_enter(TEST_JC_AUTO_STOP);
            break;
        }
        if (s_auto_step_sent == 0U) {
            float position = s_position_steps_deg[s_auto_step_index];
            test_jc2804_commanded_torque_nm = 0.0f;
            test_jc2804_commanded_speed_rpm = 0.0f;
            test_jc2804_commanded_position_deg = position;
            test_jc2804_auto_step = (float)s_auto_step_index;
            knx_status_t s1 = JC_SetAbsPosition(TEST_JC2804_MOTOR1_ID, position);
            knx_status_t s2 = JC_SetAbsPosition(TEST_JC2804_MOTOR2_ID, position);
            test_jc2804_capture_two(s1, s2);
            s_auto_step_sent = 1U;
            s_auto_tick = now;
        } else if (now - s_auto_tick >= hold_ms) {
            s_auto_step_index++;
            s_auto_step_sent = 0U;
        }
        break;

    case TEST_JC_AUTO_STOP:
        if (s_auto_step_sent == 0U) {
            (void)JC_SetSpeed(TEST_JC2804_MOTOR1_ID, 0.0f);
            (void)JC_SetSpeed(TEST_JC2804_MOTOR2_ID, 0.0f);
            (void)JC_SetTorque(TEST_JC2804_MOTOR1_ID, 0.0f);
            (void)JC_SetTorque(TEST_JC2804_MOTOR2_ID, 0.0f);
            knx_status_t s1 = JC_Idle(TEST_JC2804_MOTOR1_ID);
            knx_status_t s2 = JC_Idle(TEST_JC2804_MOTOR2_ID);
            test_jc2804_capture_two(s1, s2);
            test_jc2804_commanded_torque_nm = 0.0f;
            test_jc2804_commanded_speed_rpm = 0.0f;
            test_jc2804_commanded_position_deg = 0.0f;
            s_auto_step_sent = 1U;
            s_auto_tick = now;
        } else if (now - s_auto_tick >= gap_ms) {
            test_jc2804_auto_enter(TEST_JC_AUTO_DONE);
        }
        break;

    case TEST_JC_AUTO_DONE:
    default:
        break;
    }
}

static void test_jc2804_send_telemetry(void)
{
    if (test_jc2804_octo == NULL) {
        return;
    }

    JC_Stats stats;
    JC_GetStats(&stats);

    (void)Octolinker_SendF32(test_jc2804_octo, TEST_JC2804_TELEM_BASE + 0U, test_jc2804_voltage_v);
    (void)Octolinker_SendF32(test_jc2804_octo, TEST_JC2804_TELEM_BASE + 1U, test_jc2804_bus_current_a);
    (void)Octolinker_SendF32(test_jc2804_octo, TEST_JC2804_TELEM_BASE + 2U, test_jc2804_speed_feedback_rpm);
    (void)Octolinker_SendF32(test_jc2804_octo, TEST_JC2804_TELEM_BASE + 3U, test_jc2804_position_feedback_deg);
    (void)Octolinker_SendF32(test_jc2804_octo, TEST_JC2804_TELEM_BASE + 4U, test_jc2804_drv_temp_c);
    (void)Octolinker_SendF32(test_jc2804_octo, TEST_JC2804_TELEM_BASE + 5U, test_jc2804_mot_temp_c);
    (void)Octolinker_SendU32(test_jc2804_octo, TEST_JC2804_TELEM_BASE + 6U, test_jc2804_error_code);
    (void)Octolinker_SendU32(test_jc2804_octo, TEST_JC2804_TELEM_BASE + 7U, stats.tx_count);
    (void)Octolinker_SendU32(test_jc2804_octo, TEST_JC2804_TELEM_BASE + 8U, stats.rx_count);
    (void)Octolinker_SendU32(test_jc2804_octo, TEST_JC2804_TELEM_BASE + 9U, stats.timeout_count);
    (void)Octolinker_SendI32(test_jc2804_octo, TEST_JC2804_TELEM_BASE + 10U, test_jc2804_last_status);
}

void test_jc2804_init(void *octo)
{
    test_jc2804_octo = (Octolinker_Instance_t *)octo;
    s_poll_tick = 0U;
    s_control_tick = 0U;
    s_telem_tick = 0U;
    s_read_step = 0U;
    s_poll_motor_index = 0U;
    s_last_command = test_jc2804_command;
    test_jc2804_auto_enter(TEST_JC_AUTO_BOOT);

    (void)test_jc2804_absf(TEST_JC2804_MAX_TORQUE_NM);
}

void test_jc2804_loop(void)
{
    uint32_t now = knx_millis();
    uint8_t id = test_jc2804_id();

    JC_SetTimeoutMs(test_jc2804_period(test_jc2804_timeout_ms, 20U));

    if (test_jc2804_command != s_last_command) {
        s_last_command = test_jc2804_command;
        test_jc2804_run_command(id, (uint8_t)test_jc2804_command);
    }

    if (test_jc2804_auto_enable > 0.0f) {
        test_jc2804_auto_run();
    } else if (now - s_control_tick >= test_jc2804_period(test_jc2804_control_period_ms, 20U)) {
        s_control_tick = now;
        test_jc2804_control_once(id);
    }

    if (test_jc2804_poll_enable > 0.0f &&
        now - s_poll_tick >= test_jc2804_period(test_jc2804_poll_period_ms, 50U)) {
        s_poll_tick = now;
        uint8_t poll_id = (s_poll_motor_index == 0U) ? TEST_JC2804_MOTOR1_ID : TEST_JC2804_MOTOR2_ID;
        test_jc2804_poll_once(poll_id);
    }

    if (now - s_telem_tick >= 100U) {
        s_telem_tick = now;
        test_jc2804_send_telemetry();
    }
}
