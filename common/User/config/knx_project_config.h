#ifndef KNX_PROJECT_CONFIG_H
#define KNX_PROJECT_CONFIG_H

/* Project-level configuration */

/* Tick source: TIM17 on STM32, SysTick on MSPM0 */
#define KNX_TICK_SOURCE_TIM17    1

/* Debug output via OctoLink (UART1) */
#define KNX_DEBUG_OCTOLINK       1

/* Module enable flags */
#define KNX_MODULE_MOTOR_EN      1
#define KNX_MODULE_IMU_EN        1
#define KNX_MODULE_TELEMETRY_EN  1
#define KNX_MODULE_SAFETY_EN     1
#define KNX_MODULE_SYS_EN        1
#define KNX_MODULE_DRIVE_EN      1
#define KNX_MODULE_VISION_EN     1
#define KNX_MODULE_GIMBAL_EN     1

#endif /* KNX_PROJECT_CONFIG_H */
