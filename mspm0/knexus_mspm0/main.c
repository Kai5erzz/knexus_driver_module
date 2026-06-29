#include "ti_msp_dl_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "knx_mspm0_app.h"
#include "knx_mspm0_tasks.h"
#include "knx_led.h"

int main(void)
{
    SYSCFG_DL_init();

    (void)knx_mspm0_app_init();
    if (knx_mspm0_tasks_init() != KNX_OK) {
        /* Task creation failed — blink LED0 rapidly to signal the fault.
         * Scheduler has not started yet; we never reach vTaskStartScheduler(). */
        for (;;) {
            knx_led_toggle(KNX_LED_0);
            for (volatile uint32_t i = 0; i < 200000U; ++i) { }
        }
    }

    vTaskStartScheduler();

    for (;;) {
    }
}
