/* Platform port implementation for STM32H743 */
#include "knx_port.h"
#include "stm32h7xx_hal.h"

static uint32_t critical_nesting = 0;

void knx_port_enter_critical(void)
{
    __disable_irq();
    critical_nesting++;
}

void knx_port_exit_critical(void)
{
    if (critical_nesting > 0) {
        critical_nesting--;
    }
    if (critical_nesting == 0) {
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
    while (1) {
        /* Trap — optionally attach debugger here */
    }
}

void knx_port_watchdog_refresh(void)
{
    /* Reload IWDG1 counter — direct register write to avoid cross-layer dependency */
    IWDG1->KR = 0xAAAAU;  /* Reload IWDG counter */
}

uint8_t knx_port_is_in_isr(void)
{
    /* IPSR[5:0] = exception number; 0 = thread mode, >0 = handler (ISR) */
    uint32_t ipsr = __get_IPSR();
    return (ipsr != 0U) ? 1U : 0U;
}
