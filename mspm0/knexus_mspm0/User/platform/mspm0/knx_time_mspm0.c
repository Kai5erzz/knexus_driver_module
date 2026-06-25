#include "knx_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t s_pre_scheduler_ms;

uint32_t knx_millis(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    }

    return s_pre_scheduler_ms++;
}

uint32_t knx_micros(void)
{
    return knx_millis() * 1000U;
}

void knx_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }

    volatile uint32_t loops = (CPUCLK_FREQ / 12000U) * ms;
    while (loops-- > 0U) {
        __NOP();
    }
    s_pre_scheduler_ms += ms;
}
