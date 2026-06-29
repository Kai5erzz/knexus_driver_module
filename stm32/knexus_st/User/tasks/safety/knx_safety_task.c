#include "knx_safety_task.h"
#include "knx_app.h"
#include "knx_safety.h"
#include "cmsis_os2.h"

#define KNX_SAFETY_TASK_STACK_SIZE  1024U
#define KNX_SAFETY_TASK_PRIORITY    osPriorityHigh
#define KNX_SAFETY_TASK_PERIOD_MS   1U

static osThreadId_t knx_safety_task_handle;

__attribute__((noreturn))
static void knx_safety_task_entry(void *argument)
{
    (void)argument;

    while (!knx_app_is_initialized()) {
        osDelay(1U);
    }

    uint32_t next_wake = osKernelGetTickCount();

    for (;;) {
        (void)knx_safety_update();
        next_wake += KNX_SAFETY_TASK_PERIOD_MS;
        osDelayUntil(next_wake);
    }
}

void knx_safety_task_init(void)
{
    const osThreadAttr_t safety_task_attributes = {
        .name = "knx_safety",
        .stack_size = KNX_SAFETY_TASK_STACK_SIZE,
        .priority = (osPriority_t)KNX_SAFETY_TASK_PRIORITY,
    };

    knx_safety_task_handle = osThreadNew(knx_safety_task_entry, NULL, &safety_task_attributes);
}
