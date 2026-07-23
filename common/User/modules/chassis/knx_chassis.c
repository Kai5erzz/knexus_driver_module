#include "knx_chassis.h"
#include "knx_sys.h"
#include "FreeRTOS.h"
#include "task.h"

static knx_chassis_state_t s_state;

static void refresh_snapshot(void)
{
    knx_drive_snapshot(&s_state.drive);
    knx_motor_snapshot(&s_state.left_motor, &s_state.right_motor);
}

knx_status_t knx_chassis_init(void)
{
    s_state = (knx_chassis_state_t){0};
    s_state.last_status = knx_motor_init();
    if (s_state.last_status != KNX_OK) return s_state.last_status;
    s_state.last_status = knx_drive_init();
    refresh_snapshot();
    return s_state.last_status;
}

knx_status_t knx_chassis_enable(void)
{
    taskENTER_CRITICAL();
    s_state.enabled = true;
    taskEXIT_CRITICAL();
    return KNX_OK;
}

knx_status_t knx_chassis_disable(void)
{
    taskENTER_CRITICAL();
    s_state.enabled = false;
    taskEXIT_CRITICAL();
    return knx_chassis_stop();
}

knx_status_t knx_chassis_set_velocity(float linear_mps, float angular_radps)
{
    if (!s_state.enabled) return KNX_NOT_READY;
    s_state.last_status = knx_drive_set_velocity(linear_mps, angular_radps);
    return s_state.last_status;
}

knx_status_t knx_chassis_set_wheel_speed(float left_mps, float right_mps)
{
    if (!s_state.enabled) return KNX_NOT_READY;
    (void)knx_sys_set_mode(KNX_RUN_MODE_SPEED);
    s_state.last_status = knx_motor_set_target(left_mps, right_mps);
    return s_state.last_status;
}

knx_status_t knx_chassis_update_fast(void)
{
    s_state.fast_updates++;
    s_state.last_status = s_state.enabled ? knx_motor_update() : knx_motor_stop();
    refresh_snapshot();
    return s_state.last_status;
}

knx_status_t knx_chassis_update_control(float dt_s)
{
    s_state.control_updates++;
    s_state.last_status = s_state.enabled ? knx_drive_update(dt_s) : KNX_OK;
    refresh_snapshot();
    return s_state.last_status;
}

knx_status_t knx_chassis_stop(void)
{
    (void)knx_drive_stop();
    s_state.last_status = knx_motor_stop();
    refresh_snapshot();
    return s_state.last_status;
}

void knx_chassis_reset_odometry(void)
{
    knx_drive_reset_odometry();
    refresh_snapshot();
}

void knx_chassis_snapshot(knx_chassis_state_t *out)
{
    if (out == NULL) return;
    taskENTER_CRITICAL();
    *out = s_state;
    taskEXIT_CRITICAL();
}
