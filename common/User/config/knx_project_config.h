#ifndef KNX_PROJECT_CONFIG_H
#define KNX_PROJECT_CONFIG_H

/* Project-level configuration */

/* Tick source: TIM17 on STM32, SysTick on MSPM0
 * NOTE: KNX_TICK_SOURCE_TIM17 当前无任何文件引用, 保留作为配置预留位 (未使用) */
#define KNX_TICK_SOURCE_TIM17    1

/* Debug output via OctoLink (UART1) */
#define KNX_DEBUG_OCTOLINK       1

/* Module enable flags - DRIVE 与 GIMBAL 可共存, ctrl_task 以独立 #if 分支调度 */
#define KNX_MODULE_MOTOR_EN      1
#define KNX_MODULE_IMU_EN        1
#define KNX_MODULE_TELEMETRY_EN  1
#define KNX_MODULE_SAFETY_EN     1
#define KNX_MODULE_SYS_EN        1
#define KNX_MODULE_DRIVE_EN      1
#define KNX_MODULE_VISION_EN     1
#define KNX_MODULE_GIMBAL_EN     1

#endif /* KNX_PROJECT_CONFIG_H */
