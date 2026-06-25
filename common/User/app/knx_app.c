#include "knx_app.h"
#include "knx_board.h"
#include "knx_blackbox.h"
#include "knx_drive.h"
#include "knx_gimbal_ctrl.h"
#include "knx_grayscale.h"
#include "knx_health.h"
#include "knx_imu.h"
#include "knx_motor.h"
#include "knx_param.h"
#include "knx_param_host.h"
#include "knx_safety.h"
#include "knx_sys.h"
#include "knx_telemetry.h"
#include "knx_test_mode.h"
#include "knx_time.h"
#include "knx_vision.h"

static volatile bool s_app_initialized = false;

void knx_app_init(void)
{
    knx_health_init();
    knx_blackbox_init();
    knx_board_init();

    knx_param_init();
    knx_sys_init();
    knx_motor_init();
    knx_imu_init();
    knx_drive_init();
    knx_telemetry_set_octo(knx_board_get_octolinker());
    knx_telemetry_init();
    knx_safety_init();

    knx_grayscale_init();
    knx_vision_init();
    knx_vision_attach_host_comm(knx_board_get_host_comm());
    (void)knx_param_host_attach(knx_board_get_host_comm());
    knx_gimbal_ctrl_init();

    knx_test_mode_init(knx_board_get_octolinker());

    knx_board_post_init();
    s_app_initialized = true;
}

void knx_app_loop(void)
{
    static uint32_t sys_tick = 0;
    uint32_t now = knx_millis();

    if (now - sys_tick >= 5U) {
        sys_tick = now;
        knx_sys_update();
    }

    knx_test_mode_loop();
}

bool knx_app_is_initialized(void)
{
    return s_app_initialized;
}
