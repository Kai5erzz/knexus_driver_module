#include "ti_msp_dl_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "knx_mspm0_app.h"
#include "knx_mspm0_tasks.h"
#include "knx_led.h"

volatile uint32_t knx_mspm0_main_stage;
volatile uint32_t knx_mspm0_main_error;
volatile uint32_t knx_mspm0_reset_cause;

int main(void)
{
    knx_mspm0_reset_cause = (uint32_t)DL_SYSCTL_getResetCause();
    SYSCFG_DL_init();
    knx_mspm0_main_stage = 1U;

    knx_status_t status = knx_mspm0_app_init();
    knx_mspm0_main_stage = 2U;
    if (status != KNX_OK) {
        knx_mspm0_main_error = 0x100U | (uint32_t)status;
    }

    status = knx_mspm0_tasks_init();
    knx_mspm0_main_stage = 3U;
    if (status != KNX_OK) {
        knx_mspm0_main_error = 0x200U | (uint32_t)status;
        /* Task creation failed — blink LED0 rapidly to signal the fault.
         * Scheduler has not started yet; we never reach vTaskStartScheduler(). */
        for (;;) {
            knx_led_toggle(KNX_LED_0);
            for (volatile uint32_t i = 0; i < 200000U; ++i) { }
        }
    }

    knx_mspm0_main_stage = 4U;
    vTaskStartScheduler();

    knx_mspm0_main_error = 0x300U;
    for (;;) {
    }
}
