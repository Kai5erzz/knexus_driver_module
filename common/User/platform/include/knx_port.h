#ifndef KNX_PORT_H
#define KNX_PORT_H

#include "knx_types.h"

/*
 * Platform port abstraction.
 * Each platform provides knx_port_*.c with the actual implementations.
 */

/* Critical section (platform-specific, can be empty for bare-metal superloop) */
void     knx_port_enter_critical(void);
void     knx_port_exit_critical(void);

/* System reset */
void     knx_port_system_reset(void);

/* Fault indication — called on unrecoverable error */
void     knx_port_fault(void);

/* Hardware watchdog refresh — called periodically to prevent watchdog reset */
void     knx_port_watchdog_refresh(void);

/**
 * @brief Returns 1 (true) when called from ISR context, 0 otherwise.
 *        On platforms with FreeRTOS this maps to xPortIsInsideInterrupt();
 *        on bare-metal platforms it can be a compile-time constant (always 0)
 *        or read IPSR (Cortex-M only).
 */
uint8_t  knx_port_is_in_isr(void);

#endif /* KNX_PORT_H */
