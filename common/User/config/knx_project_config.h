#ifndef KNX_PROJECT_CONFIG_H
#define KNX_PROJECT_CONFIG_H

/* Project-level configuration */

/* Tick source: TIM17 on STM32, SysTick on MSPM0
 * NOTE: KNX_TICK_SOURCE_TIM17 当前无任何文件引用, 保留作为配置预留位 (未使用) */
#define KNX_TICK_SOURCE_TIM17    1

/* Debug output via OctoLink (UART1) */
#define KNX_DEBUG_OCTOLINK       1

/* Module enable flags - DRIVE 与 GIMBAL 可共存, ctrl_task 以独立 #if 分支调度 */
#if defined(KNX_BUILD_CONTEST_2026)
#define KNX_APP_CONTEST_2026       1
#else
#define KNX_APP_CONTEST_2026       0
#endif

#define KNX_MODULE_MOTOR_EN        1
#define KNX_MODULE_IMU_EN          1
#define KNX_MODULE_TELEMETRY_EN    1
#define KNX_MODULE_SAFETY_EN       1
#define KNX_MODULE_SYS_EN          1
#define KNX_MODULE_DRIVE_EN        1
#define KNX_MODULE_TRACK_EN        1
#define KNX_MODULE_INTERSECTION_EN 1
#define KNX_MODULE_CAN_BUS_EN      1
#define KNX_MODULE_COMM_EN         1

#if KNX_APP_CONTEST_2026
#define KNX_MODULE_VISION_EN       0
#define KNX_MODULE_GIMBAL_EN       0
#else
#define KNX_MODULE_VISION_EN       1
#define KNX_MODULE_GIMBAL_EN       1
#endif

#endif /* KNX_PROJECT_CONFIG_H */
