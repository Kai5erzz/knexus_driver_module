#ifndef KNX_DEBUG_CONFIG_H
#define KNX_DEBUG_CONFIG_H

/* Enable assert checks in debug builds */
#ifdef DEBUG
  #define KNX_USE_ASSERT   1
#endif

/* Debug print via OctoLink */
#define KNX_DEBUG_PRINT_EN   1

/* Fault handler breakpoint */
#define KNX_FAULT_BKPT       1

/* ---- Active test mode ---- */
#define KNX_ACTIVE_TEST_MODE_NONE      0
#define KNX_ACTIVE_TEST_MODE_DRV8701E  1
#define KNX_ACTIVE_TEST_MODE_BMI088    2
#define KNX_ACTIVE_TEST_MODE_ENCODER   3
#define KNX_ACTIVE_TEST_MODE_DRIVE_CALIB 4
#define KNX_ACTIVE_TEST_MODE_BMI088_IMU_EST 5
#define KNX_ACTIVE_TEST_MODE_GRAYSCALE_SAMPLE 6
#define KNX_ACTIVE_TEST_MODE_JC2804 7
#define KNX_ACTIVE_TEST_MODE_DM_IMU_L1 8
#define KNX_ACTIVE_TEST_MODE_MINI_GIMBAL 9
#define KNX_ACTIVE_TEST_MODE_JC_DRIVER 10
#define KNX_ACTIVE_TEST_MODE_STANDARD_GIMBAL 11
#define KNX_ACTIVE_TEST_MODE_COLOR_TRACK 12

/* Set to one of the above to activate a test mode at boot */
#ifdef KNX_TARGET_MSPM0
#define KNX_ACTIVE_TEST_MODE  KNX_ACTIVE_TEST_MODE_NONE
#else
#define KNX_ACTIVE_TEST_MODE  KNX_ACTIVE_TEST_MODE_NONE
#endif

#endif /* KNX_DEBUG_CONFIG_H */
