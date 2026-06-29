/* Time implementation for STM32H743. */
#include "knx_time.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"

static uint8_t dwt_ready = 0U;
static uint32_t dwt_last_cycles = 0U;
static uint32_t dwt_cycle_remainder = 0U;
static uint32_t dwt_micros_accum = 0U;

static void knx_time_dwt_init(void)
{
    if (dwt_ready != 0U) {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_last_cycles = DWT->CYCCNT;
    dwt_ready = 1U;
}

uint32_t knx_millis(void)
{
    if (osKernelGetState() == osKernelRunning) {
        return osKernelGetTickCount();
    }
    return HAL_GetTick();
}

uint32_t knx_micros(void)
{
    knx_time_dwt_init();

    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        return 0U;
    }

    uint32_t now_cycles = DWT->CYCCNT;
    uint32_t delta_cycles = now_cycles - dwt_last_cycles;
    dwt_last_cycles = now_cycles;

    uint64_t total_cycles = (uint64_t)dwt_cycle_remainder + (uint64_t)delta_cycles;
    uint32_t delta_us = (uint32_t)(total_cycles / cycles_per_us);
    dwt_cycle_remainder = (uint32_t)(total_cycles % cycles_per_us);
    dwt_micros_accum += delta_us;

    return dwt_micros_accum;
}

void knx_delay_ms(uint32_t ms)
{
    if (osKernelGetState() == osKernelRunning) {
        osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}
