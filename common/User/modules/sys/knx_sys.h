#ifndef KNX_SYS_H
#define KNX_SYS_H

#include "knx_types.h"

typedef enum {
    KNX_SYS_BOOT = 0,
    KNX_SYS_READY,
    KNX_SYS_RUNNING,
    KNX_SYS_FAULT,
} knx_sys_state_t;

typedef enum {
    KNX_RUN_MODE_IDLE = 0,
    KNX_RUN_MODE_CURRENT_TEST,
    KNX_RUN_MODE_TORQUE,
    KNX_RUN_MODE_SPEED,
    KNX_RUN_MODE_DUTY,
} knx_run_mode_t;

knx_status_t knx_sys_init(void);
knx_status_t knx_sys_update(void);

knx_status_t knx_sys_set_mode(knx_run_mode_t mode);
knx_run_mode_t knx_sys_get_mode(void);

knx_sys_state_t knx_sys_get_state(void);
uint32_t knx_sys_get_state_age_ms(void);

void knx_sys_report_fault(uint32_t fault_mask);
void knx_sys_clear_faults(void);
uint32_t knx_sys_get_fault_latch(void);

#endif /* KNX_SYS_H */
