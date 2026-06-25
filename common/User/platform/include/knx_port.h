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

#endif /* KNX_PORT_H */
