#ifndef KNX_TEST_MODE_H
#define KNX_TEST_MODE_H

#include "knx_types.h"

/* Test mode identifiers �?values match knx_debug_config.h macros */
typedef enum {
    KNX_TEST_MODE_NONE     = 0,
    KNX_TEST_MODE_DRV8701E = 1,
    KNX_TEST_MODE_BMI088   = 2,
    KNX_TEST_MODE_ENCODER  = 3,
    KNX_TEST_MODE_DRIVE_CALIB = 4,
    KNX_TEST_MODE_BMI088_IMU_EST = 5,
    KNX_TEST_MODE_GRAYSCALE_SAMPLE = 6,
    KNX_TEST_MODE_JC2804 = 7,
    KNX_TEST_MODE_DM_IMU_L1 = 8,
    KNX_TEST_MODE_MINI_GIMBAL = 9,
    KNX_TEST_MODE_JC_DRIVER = 10,
    KNX_TEST_MODE_STANDARD_GIMBAL = 11,
    KNX_TEST_MODE_COLOR_TRACK = 12,
    KNX_TEST_MODE_STM32_BOARD_BRINGUP = 13,
} knx_test_mode_t;

/* Use void* to avoid pulling octolinker.h into the dispatcher header.
 * Each test_*.c includes octolinker.h directly and casts internally. */

/* Initialize the active test mode.
 * octo: shared OctoLink instance for debug output (may be NULL). */
void knx_test_mode_init(void *octo);

/* Run one iteration of the active test mode (non-blocking). */
void knx_test_mode_loop(void);

#endif /* KNX_TEST_MODE_H */
