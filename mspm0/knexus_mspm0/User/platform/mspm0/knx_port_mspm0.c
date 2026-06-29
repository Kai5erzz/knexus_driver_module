#include "knx_port.h"
#include <ti/devices/msp/msp.h>

static volatile uint32_t s_critical_nesting = 0U;

void knx_port_enter_critical(void)
{
    __disable_irq();
    s_critical_nesting++;
}

void knx_port_exit_critical(void)
{
    if (s_critical_nesting == 0U) {
        return;
    }

    s_critical_nesting--;
    if (s_critical_nesting == 0U) {
        __enable_irq();
    }
}

void knx_port_system_reset(void)
{
    NVIC_SystemReset();
}

void knx_port_fault(void)
{
    __disable_irq();
    for (;;) {
    }
}

/* MSPM0 watchdog refresh — stub for now; implement with MSPM0 WDT when available */
void knx_port_watchdog_refresh(void)
{
    /* TODO: implement MSPM0 watchdog refresh */
}

uint8_t knx_port_is_in_isr(void)
{
    /* IPSR[5:0] = exception number; 0 = thread mode, >0 = handler (ISR) */
    uint32_t ipsr = __get_IPSR();
    return (ipsr != 0U) ? 1U : 0U;
}
