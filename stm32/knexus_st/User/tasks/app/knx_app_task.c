#include "knx_app_task.h"
#include "knx_app.h"
#include "cmsis_os2.h"

#define KNX_APP_TASK_STACK_SIZE  4096U
#define KNX_APP_TASK_PRIORITY    osPriorityNormal
#define KNX_APP_TASK_PERIOD_MS   1U

static osThreadId_t knx_app_task_handle;

__attribute__((noreturn))
static void knx_app_task_entry(void *argument)
{
    (void)argument;

    knx_app_init();

    static uint32_t next_wake;
    if (next_wake == 0) next_wake = osKernelGetTickCount();

    for (;;) {
        knx_app_loop();
        next_wake += KNX_APP_TASK_PERIOD_MS;
        osDelayUntil(next_wake);
    }
}

void knx_app_task_init(void)
{
    const osThreadAttr_t app_task_attributes = {
        .name = "knx_app",
        .stack_size = KNX_APP_TASK_STACK_SIZE,
        .priority = (osPriority_t)KNX_APP_TASK_PRIORITY,
    };

    knx_app_task_handle = osThreadNew(knx_app_task_entry, NULL, &app_task_attributes);
}
