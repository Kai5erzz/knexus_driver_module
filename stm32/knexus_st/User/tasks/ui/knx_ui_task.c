#include "knx_ui_task.h"
#include "knx_app.h"
#include "knx_beep.h"
#include "knx_board.h"
#include "knx_gpio.h"
#include "knx_grayscale.h"
#include "knx_key.h"
#include "knx_safety.h"
#include "cmsis_os2.h"

#define KNX_UI_TASK_STACK_SIZE  2048U
#define KNX_UI_TASK_PRIORITY    osPriorityBelowNormal
#define KNX_UI_TASK_PERIOD_MS   1U

static osThreadId_t knx_ui_task_handle;

__attribute__((noreturn))
static void knx_ui_task_entry(void *argument)
{
    (void)argument;

    while (!knx_app_is_initialized()) {
        osDelay(1U);
    }

    uint32_t next_wake = osKernelGetTickCount();
    uint32_t elapsed_ms = 0U;
    uint32_t led_tick = 0U;

    for (;;) {
        elapsed_ms += KNX_UI_TASK_PERIOD_MS;

        knx_beep_update();

        if ((elapsed_ms % 10U) == 0U) {
            knx_key_update(10U);
        }

        if ((elapsed_ms % 20U) == 0U) {
            knx_grayscale_update();
        }

        const knx_gpio_t *debug_led = knx_board_get_debug_led();
        if (debug_led != NULL) {
            uint32_t led_period = (knx_safety_get_faults() != 0U) ? 100U : 500U;
            if (elapsed_ms - led_tick >= led_period) {
                led_tick = elapsed_ms;
                knx_gpio_toggle(*debug_led);
            }
        }

        next_wake += KNX_UI_TASK_PERIOD_MS;
        osDelayUntil(next_wake);
    }
}

void knx_ui_task_init(void)
{
    const osThreadAttr_t ui_task_attributes = {
        .name = "knx_ui",
        .stack_size = KNX_UI_TASK_STACK_SIZE,
        .priority = (osPriority_t)KNX_UI_TASK_PRIORITY,
    };

    knx_ui_task_handle = osThreadNew(knx_ui_task_entry, NULL, &ui_task_attributes);
}
