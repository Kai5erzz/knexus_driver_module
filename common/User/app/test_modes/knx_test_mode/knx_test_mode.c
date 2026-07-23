/* Test mode dispatcher */
#include "knx_test_mode.h"
#include "knx_debug_config.h"

#include "test_drv8701e.h"
#include "test_bmi088.h"
#include "test_encoder.h"
#include "test_drive_calib.h"
#include "test_bmi088_imu_est.h"
#include "test_grayscale_sample.h"
#include "test_jc2804.h"
#include "test_dm_imu_l1.h"
#include "mini_gimbal_test.h"
#include "test_jc_driver.h"
#include "test_standard_gimbal.h"
#include "test_color_track.h"
#include "test_stm32_board_bringup.h"

#define ACTIVE_MODE  KNX_ACTIVE_TEST_MODE

void knx_test_mode_init(void *octo)
{
#if (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_DRV8701E)
    test_drv8701e_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_BMI088)
    test_bmi088_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_ENCODER)
    test_encoder_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_DRIVE_CALIB)
    test_drive_calib_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_BMI088_IMU_EST)
    test_bmi088_imu_est_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_GRAYSCALE_SAMPLE)
    test_grayscale_sample_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_JC2804)
    test_jc2804_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_DM_IMU_L1)
    test_dm_imu_l1_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_MINI_GIMBAL)
    mini_gimbal_test_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_JC_DRIVER)
    test_jc_driver_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_STANDARD_GIMBAL)
    test_standard_gimbal_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_COLOR_TRACK)
    test_color_track_init(octo);
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_STM32_BOARD_BRINGUP)
    test_stm32_board_bringup_init(octo);
#else
    (void)octo;
#endif
}

void knx_test_mode_loop(void)
{
#if (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_DRV8701E)
    test_drv8701e_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_BMI088)
    test_bmi088_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_ENCODER)
    test_encoder_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_DRIVE_CALIB)
    test_drive_calib_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_BMI088_IMU_EST)
    test_bmi088_imu_est_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_GRAYSCALE_SAMPLE)
    test_grayscale_sample_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_JC2804)
    test_jc2804_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_DM_IMU_L1)
    test_dm_imu_l1_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_MINI_GIMBAL)
    mini_gimbal_test_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_JC_DRIVER)
    test_jc_driver_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_STANDARD_GIMBAL)
    test_standard_gimbal_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_COLOR_TRACK)
    test_color_track_loop();
#elif (ACTIVE_MODE == KNX_ACTIVE_TEST_MODE_STM32_BOARD_BRINGUP)
    test_stm32_board_bringup_loop();
#endif
}
