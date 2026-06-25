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
