#include "knx_ctrl_task.h"
#include "knx_app.h"
#include "knx_imu.h"
#include "knx_motor.h"
#include "knx_drive.h"
#include "knx_debug_config.h"
#include "knx_gimbal_ctrl.h"
#include "knx_project_config.h"
#include "knx_safety.h"
#include "cmsis_os2.h"

#define KNX_CTRL_TASK_STACK_SIZE  4096U
#define KNX_CTRL_TASK_PRIORITY    osPriorityAboveNormal
#define KNX_CTRL_TASK_PERIOD_MS   1U
#define KNX_IMU_DIVIDER           2U

static osThreadId_t knx_ctrl_task_handle;

__attribute__((noreturn))
static void knx_ctrl_task_entry(void *argument)
{
    (void)argument;

    while (!knx_app_is_initialized()) {
        osDelay(1U);
    }

    uint32_t next_wake = osKernelGetTickCount();
    uint32_t imu_div = 0U;

    for (;;) {
#if (KNX_MODULE_GIMBAL_EN)
        if (knx_safety_get_level() == KNX_SAFETY_LEVEL_FAULT) {
            knx_gimbal_ctrl_stop();
        } else {
            knx_gimbal_ctrl_update((float)KNX_CTRL_TASK_PERIOD_MS * 0.001f);
        }
#elif (KNX_ACTIVE_TEST_MODE == KNX_ACTIVE_TEST_MODE_NONE)
        imu_div++;
        if (imu_div >= KNX_IMU_DIVIDER) {
            imu_div = 0U;
            knx_imu_update();
        }
        knx_drive_update((float)KNX_CTRL_TASK_PERIOD_MS * 0.001f);
        knx_motor_update();
#endif
        next_wake += KNX_CTRL_TASK_PERIOD_MS;
        osDelayUntil(next_wake);
    }
}

void knx_ctrl_task_init(void)
{
    const osThreadAttr_t ctrl_task_attributes = {
        .name = "knx_ctrl",
        .stack_size = KNX_CTRL_TASK_STACK_SIZE,
        .priority = (osPriority_t)KNX_CTRL_TASK_PRIORITY,
    };

    knx_ctrl_task_handle = osThreadNew(knx_ctrl_task_entry, NULL, &ctrl_task_attributes);
}
