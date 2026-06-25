#include "knx_mspm0_app.h"
#include "knx_board.h"
#include "bmi088.h"
#include "drv8701e.h"
#include "encoder.h"
#include "knx_blackbox.h"
#include "knx_can_router.h"
#include "knx_drive.h"
#include "knx_grayscale.h"
#include "knx_health.h"
#include "knx_imu.h"
#include "knx_motor.h"
#include "knx_param.h"
#include "knx_sys.h"
#include "knx_telemetry.h"
#include "track_sensor.h"

static knx_can_router_t s_can_router;
static uint8_t s_upper_ready;

knx_status_t knx_mspm0_app_init(void)
{
    knx_status_t status = knx_board_init();
    if (status != KNX_OK) {
        return status;
    }

    knx_board_post_init();
    return KNX_OK;
}

knx_status_t knx_mspm0_upper_init(void)
{
    if (s_upper_ready != 0U) {
        return KNX_OK;
    }

    knx_param_init();
    knx_blackbox_init();
    knx_health_init();
    (void)knx_sys_init();

    DRV8701E_AttachPorts(knx_board_get_drv_left_port(),
                         knx_board_get_drv_right_port());
    DRV8701E_Init();

    Encoder_AttachPorts(knx_board_get_encoder_left(),
                        knx_board_get_encoder_right());
    Encoder_Init();

    TrackSensor_AttachPorts(knx_board_get_track_sensor_port());
    TrackSensor_Init();
    knx_grayscale_init();

    (void)knx_spi_init((knx_spi_t *)knx_board_get_bmi088_accel_spi());
    (void)knx_spi_init((knx_spi_t *)knx_board_get_bmi088_gyro_spi());
    BMI088_Init((knx_spi_t *)knx_board_get_bmi088_accel_spi(),
                (knx_spi_t *)knx_board_get_bmi088_gyro_spi());
    (void)knx_imu_init();

    (void)knx_can_router_init(&s_can_router, knx_board_get_can());
    (void)knx_can_router_start(&s_can_router);

    (void)knx_motor_init();
    (void)knx_drive_init();

    knx_telemetry_set_octo(knx_board_get_octolinker());
    (void)knx_telemetry_init();

    s_upper_ready = 1U;
    return KNX_OK;
}

knx_status_t knx_mspm0_upper_fast_update(void)
{
    if (s_upper_ready == 0U) {
        return KNX_NOT_READY;
    }

    extern void knx_can_mspm0_poll(void);
    knx_can_mspm0_poll();
    (void)knx_motor_update();
    return KNX_OK;
}

knx_status_t knx_mspm0_upper_medium_update(float dt_s)
{
    if (s_upper_ready == 0U) {
        return KNX_NOT_READY;
    }

    extern void knx_can_mspm0_poll(void);
    knx_can_mspm0_poll();
    (void)knx_imu_update();
    (void)knx_grayscale_update();
    (void)knx_drive_update(dt_s);
    return KNX_OK;
}

knx_status_t knx_mspm0_upper_slow_update(void)
{
    if (s_upper_ready == 0U) {
        return KNX_NOT_READY;
    }

    (void)knx_sys_update();
    (void)knx_telemetry_update();
    return KNX_OK;
}
