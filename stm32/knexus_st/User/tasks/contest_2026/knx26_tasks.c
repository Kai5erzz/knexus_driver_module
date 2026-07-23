#include "knx26_tasks.h"
#include "knx26_app.h"
#include "knx26_config.h"
#include "knx_app.h"
#include "cmsis_os2.h"

static void wait_ready(void)
{
    while (!knx_app_is_initialized()) osDelay(1U);
}

__attribute__((noreturn)) static void fast_task(void *argument)
{
    (void)argument; wait_ready();
    uint32_t wake = osKernelGetTickCount();
    for (;;) { knx26_fast_update(); wake += KNX26_FAST_PERIOD_MS; osDelayUntil(wake); }
}

__attribute__((noreturn)) static void control_task(void *argument)
{
    (void)argument; wait_ready();
    uint32_t wake = osKernelGetTickCount();
    for (;;) {
        knx26_control_update((float)KNX26_CONTROL_PERIOD_MS * 0.001f);
        wake += KNX26_CONTROL_PERIOD_MS; osDelayUntil(wake);
    }
}

__attribute__((noreturn)) static void track_task(void *argument)
{
    (void)argument; wait_ready();
    uint32_t wake = osKernelGetTickCount();
    for (;;) { knx26_track_update(); wake += KNX26_TRACK_PERIOD_MS; osDelayUntil(wake); }
}

__attribute__((noreturn)) static void comm_task(void *argument)
{
    (void)argument; wait_ready();
    uint32_t wake = osKernelGetTickCount();
    for (;;) { knx26_comm_update(); wake += 1U; osDelayUntil(wake); }
}

__attribute__((noreturn)) static void app_task(void *argument)
{
    (void)argument;
    knx_app_init();
    uint32_t wake = osKernelGetTickCount();
    for (;;) { knx_app_loop(); wake += KNX26_APP_PERIOD_MS; osDelayUntil(wake); }
}

__attribute__((noreturn)) static void debug_task(void *argument)
{
    (void)argument; wait_ready();
    uint32_t wake = osKernelGetTickCount();
    for (;;) { knx26_debug_update(); wake += KNX26_DEBUG_PERIOD_MS; osDelayUntil(wake); }
}

static void create_task(osThreadFunc_t entry, const char *name,
                        uint32_t stack, osPriority_t priority)
{
    const osThreadAttr_t attr = {.name=name, .stack_size=stack, .priority=priority};
    (void)osThreadNew(entry, NULL, &attr);
}

void knx26_tasks_init(void)
{
    create_task(app_task, "knx26_app", 4096U, osPriorityNormal);
    create_task(fast_task, "knx26_fast", 3072U, osPriorityHigh);
    create_task(control_task, "knx26_ctrl", 4096U, osPriorityAboveNormal);
    create_task(track_task, "knx26_track", 3072U, osPriorityNormal);
    create_task(comm_task, "knx26_comm", 3072U, osPriorityAboveNormal);
    create_task(debug_task, "knx26_debug", 3072U, osPriorityLow);
}
