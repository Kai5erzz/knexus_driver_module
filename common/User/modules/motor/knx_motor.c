#include "knx_motor.h"
#include "knx_sys.h"
#include "knx_params.h"
#include "knx_project_config.h"
#include "drv8701e.h"
#include "encoder.h"
#include "FreeRTOS.h"
#include "task.h"

static knx_motor_state_t motor_left_state;
static knx_motor_state_t motor_right_state;
volatile uint32_t knx_motor_diag_init_stage;

static float torque_gain_nm_per_a(void)
{
    float gain = g_knx_params.mech.motor_torque_constant_nm_per_a *
                 g_knx_params.mech.motor_gear_ratio *
                 g_knx_params.mech.motor_torque_efficiency;

    if (gain <= 0.000001f) {
        gain = 0.000001f;
    }

    return gain;
}

float knx_motor_current_to_torque(float current_a)
{
    return current_a * torque_gain_nm_per_a();
}

float knx_motor_torque_to_current(float torque_nm)
{
    return torque_nm / torque_gain_nm_per_a();
}

static float duty_to_norm(const DRV8701E_Motor_t *motor)
{
    if (motor == NULL || motor->port == NULL || motor->port->pwm.timer == NULL) {
        return 0.0f;
    }

    uint32_t arr = motor->port->pwm.arr;
    if (arr == 0U) {
        (void)knx_pwm_get_period(&motor->port->pwm, &arr);
    }
    if (arr == 0U) {
        return 0.0f;
    }

    return (float)motor->duty / (float)arr;
}

static void sync_public_state(void)
{
    taskENTER_CRITICAL();
    motor_left_state.current_speed = encoder_left.speed_mps;
    motor_right_state.current_speed = encoder_right.speed_mps;
    motor_left_state.current_filtered = drv8701e_current.current_left_filtered_a;
    motor_right_state.current_filtered = drv8701e_current.current_right_filtered_a;
    motor_left_state.torque_estimated = knx_motor_current_to_torque(motor_left_state.current_filtered);
    motor_right_state.torque_estimated = knx_motor_current_to_torque(motor_right_state.current_filtered);
    motor_left_state.duty = duty_to_norm(&motor_left);
    motor_right_state.duty = duty_to_norm(&motor_right);
    taskEXIT_CRITICAL();
}

knx_status_t knx_motor_init(void)
{
    knx_motor_diag_init_stage = 1U;
    motor_left_state = (knx_motor_state_t){0};
    motor_right_state = (knx_motor_state_t){0};

    DRV8701E_VelocityInit();
    knx_motor_diag_init_stage = 2U;
    DRV8701E_CurrentCtrlInit();
    DRV8701E_Current_Start();
    knx_motor_diag_init_stage = 3U;
    DRV8701E_Current_CalibrateZero(100, 1);
    knx_motor_diag_init_stage = 4U;
    DRV8701E_StopAll();
    sync_public_state();
    knx_motor_diag_init_stage = 5U;

    return KNX_OK;
}

knx_status_t knx_motor_update(void)
{
    float left_target_current;
    float right_target_current;
    float left_target_torque;
    float right_target_torque;
    float left_target_duty;
    float right_target_duty;

    taskENTER_CRITICAL();
    left_target_current = motor_left_state.target_current;
    right_target_current = motor_right_state.target_current;
    left_target_torque = motor_left_state.target_torque;
    right_target_torque = motor_right_state.target_torque;
    left_target_duty = motor_left_state.target_duty;
    right_target_duty = motor_right_state.target_duty;
    taskEXIT_CRITICAL();

    DRV8701E_Current_Poll(1);
    Encoder_CalcSpeed(&encoder_left);
    Encoder_CalcSpeed(&encoder_right);

    knx_run_mode_t mode = knx_sys_get_mode();

    if (mode == KNX_RUN_MODE_TORQUE) {
        float desired_left_current  = knx_motor_torque_to_current(left_target_torque);
        float desired_right_current = knx_motor_torque_to_current(right_target_torque);
        cc_left.target  = desired_left_current  * MOTOR1_TARGET_CURRENT_SIGN;
        cc_right.target = desired_right_current * MOTOR2_TARGET_CURRENT_SIGN;
        taskENTER_CRITICAL();
        motor_left_state.target_current = desired_left_current;
        motor_right_state.target_current = desired_right_current;
        taskEXIT_CRITICAL();
        DRV8701E_CurrentControl(&cc_left, &motor_left, drv8701e_current.current_left_filtered_a);
        DRV8701E_CurrentControl(&cc_right, &motor_right, drv8701e_current.current_right_filtered_a);
    } else if (mode == KNX_RUN_MODE_CURRENT_TEST) {
        cc_left.target  = left_target_current  * MOTOR1_TARGET_CURRENT_SIGN;
        cc_right.target = right_target_current * MOTOR2_TARGET_CURRENT_SIGN;
        DRV8701E_CurrentControl(&cc_left, &motor_left, drv8701e_current.current_left_filtered_a);
        DRV8701E_CurrentControl(&cc_right, &motor_right, drv8701e_current.current_right_filtered_a);
    } else if (mode == KNX_RUN_MODE_SPEED) {
        vc_left.target = motor_left_state.target_speed;
        vc_right.target = motor_right_state.target_speed;
        DRV8701E_VelocityControl(&vc_left, &motor_left, encoder_left.speed_mps);
        DRV8701E_VelocityControl(&vc_right, &motor_right, encoder_right.speed_mps);
    } else if (mode == KNX_RUN_MODE_DUTY) {
        DRV8701E_SetDutyNorm(&motor_left,
                              left_target_duty *
                              (float)MOTOR1_TARGET_CURRENT_SIGN *
                              (float)motor_left.dir_sign);
        DRV8701E_SetDutyNorm(&motor_right,
                              right_target_duty *
                              (float)MOTOR2_TARGET_CURRENT_SIGN *
                              (float)motor_right.dir_sign);
        cc_left.target = 0.0f;
        cc_right.target = 0.0f;
    } else {
        DRV8701E_StopAll();
        cc_left.target = 0.0f;
        cc_right.target = 0.0f;
        vc_left.target = 0.0f;
        vc_right.target = 0.0f;
    }

    sync_public_state();
    return KNX_OK;
}

knx_status_t knx_motor_set_target(float left_mps, float right_mps)
{
    taskENTER_CRITICAL();
    motor_left_state.target_speed = left_mps;
    motor_right_state.target_speed = right_mps;
    taskEXIT_CRITICAL();
    return KNX_OK;
}

knx_status_t knx_motor_set_current_target(float left_a, float right_a)
{
    taskENTER_CRITICAL();
    motor_left_state.target_current = left_a;
    motor_right_state.target_current = right_a;
    motor_left_state.target_torque = knx_motor_current_to_torque(left_a);
    motor_right_state.target_torque = knx_motor_current_to_torque(right_a);
    taskEXIT_CRITICAL();
    return KNX_OK;
}

knx_status_t knx_motor_set_torque_target(float left_nm, float right_nm)
{
    taskENTER_CRITICAL();
    motor_left_state.target_torque = left_nm;
    motor_right_state.target_torque = right_nm;
    motor_left_state.target_current = knx_motor_torque_to_current(left_nm);
    motor_right_state.target_current = knx_motor_torque_to_current(right_nm);
    motor_left_state.target_duty = 0.0f;
    motor_right_state.target_duty = 0.0f;
    taskEXIT_CRITICAL();
    return KNX_OK;
}

knx_status_t knx_motor_set_duty_target(float left_norm, float right_norm)
{
    if (left_norm > 1.0f) {
        left_norm = 1.0f;
    } else if (left_norm < -1.0f) {
        left_norm = -1.0f;
    }
    if (right_norm > 1.0f) {
        right_norm = 1.0f;
    } else if (right_norm < -1.0f) {
        right_norm = -1.0f;
    }

    taskENTER_CRITICAL();
    motor_left_state.target_duty = left_norm;
    motor_right_state.target_duty = right_norm;
    motor_left_state.target_torque = 0.0f;
    motor_right_state.target_torque = 0.0f;
    motor_left_state.target_current = 0.0f;
    motor_right_state.target_current = 0.0f;
    taskEXIT_CRITICAL();
    return KNX_OK;
}

knx_status_t knx_motor_stop(void)
{
    taskENTER_CRITICAL();
    motor_left_state.target_speed = 0.0f;
    motor_right_state.target_speed = 0.0f;
    motor_left_state.target_current = 0.0f;
    motor_right_state.target_current = 0.0f;
    motor_left_state.target_torque = 0.0f;
    motor_right_state.target_torque = 0.0f;
    motor_left_state.target_duty = 0.0f;
    motor_right_state.target_duty = 0.0f;
    cc_left.target = 0.0f;
    cc_right.target = 0.0f;
    vc_left.target = 0.0f;
    vc_right.target = 0.0f;
    taskEXIT_CRITICAL();
    DRV8701E_StopAll();
    sync_public_state();
    return KNX_OK;
}

const knx_motor_state_t *knx_motor_get_left(void)
{
    return &motor_left_state;
}

const knx_motor_state_t *knx_motor_get_right(void)
{
    return &motor_right_state;
}

void knx_motor_snapshot(knx_motor_state_t *left, knx_motor_state_t *right)
{
    taskENTER_CRITICAL();
    if (left != NULL) {
        *left = motor_left_state;
    }
    if (right != NULL) {
        *right = motor_right_state;
    }
    taskEXIT_CRITICAL();
}
