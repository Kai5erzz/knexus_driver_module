#include "knx_telemetry.h"
#include "octolinker.h"
#include "knx_motor.h"
#include "knx_imu.h"
#include "knx_drive.h"
#include "knx_sys.h"
#include "bmi088.h"

static Octolinker_Instance_t *s_octo;

void knx_telemetry_set_octo(void *octo)
{
    s_octo = (Octolinker_Instance_t *)octo;
}

knx_status_t knx_telemetry_init(void)
{
    return KNX_OK;
}

knx_status_t knx_telemetry_update(void)
{
    if (s_octo == NULL) {
        return KNX_ERROR;
    }

    knx_motor_state_t left;
    knx_motor_state_t right;
    knx_imu_data_t imu;

    knx_motor_snapshot(&left, &right);
    knx_imu_snapshot(&imu);

    Octolinker_SendF32(s_octo, 1, left.target_current);
    Octolinker_SendF32(s_octo, 2, left.current_filtered);
    Octolinker_SendF32(s_octo, 3, right.target_current);
    Octolinker_SendF32(s_octo, 4, right.current_filtered);

    Octolinker_SendF32(s_octo, 20, imu.roll);
    Octolinker_SendF32(s_octo, 21, imu.pitch);
    Octolinker_SendF32(s_octo, 22, imu.yaw);
    Octolinker_SendF32(s_octo, 23, imu.temperature);
    Octolinker_SendF32(s_octo, 24, bmi088_temp_target_c);
    Octolinker_SendF32(s_octo, 25, bmi088_heater_duty);

    (void)knx_telemetry_update_drive();

    return KNX_OK;
}

knx_status_t knx_telemetry_update_drive(void)
{
    if (s_octo == NULL) {
        return KNX_ERROR;
    }

    knx_motor_state_t left;
    knx_motor_state_t right;
    knx_drive_state_t drive;

    knx_motor_snapshot(&left, &right);
    knx_drive_snapshot(&drive);

    Octolinker_SendF32(s_octo, 10, drive.left_target_mps);
    Octolinker_SendF32(s_octo, 11, drive.right_target_mps);
    Octolinker_SendF32(s_octo, 12, left.current_speed);
    Octolinker_SendF32(s_octo, 13, right.current_speed);
    Octolinker_SendF32(s_octo, 14, drive.target_linear_mps);
    Octolinker_SendF32(s_octo, 15, drive.target_angular_radps);
    Octolinker_SendF32(s_octo, 16, drive.command_active ? 1.0f : 0.0f);
    Octolinker_SendF32(s_octo, 17, drive.profiled_linear_mps);
    Octolinker_SendF32(s_octo, 18, drive.profiled_angular_radps);
    Octolinker_SendF32(s_octo, 19, (float)drive.mode);
    Octolinker_SendF32(s_octo, 30, (float)knx_sys_get_state());
    Octolinker_SendF32(s_octo, 31, (float)knx_sys_get_mode());
    Octolinker_SendF32(s_octo, 32, drive.position_m);
    Octolinker_SendF32(s_octo, 33, drive.heading_rad);
    Octolinker_SendF32(s_octo, 34, drive.measured_linear_mps);
    Octolinker_SendF32(s_octo, 35, drive.measured_angular_radps);
    Octolinker_SendF32(s_octo, 36, drive.position_loop_out_mps);
    Octolinker_SendF32(s_octo, 37, drive.angle_loop_out_radps);
    Octolinker_SendF32(s_octo, 38, drive.linear_speed_loop_out_mps);
    Octolinker_SendF32(s_octo, 39, drive.angular_speed_loop_out_radps);

    return KNX_OK;
}
