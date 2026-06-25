#include "knx_sys.h"
#include "knx_health.h"
#include "knx_params.h"
#include "knx_time.h"
#include "FreeRTOS.h"
#include "task.h"

static knx_sys_state_t s_state = KNX_SYS_BOOT;
static knx_run_mode_t s_mode = KNX_RUN_MODE_IDLE;
static uint32_t s_state_enter_ms = 0;
static uint32_t s_fault_latch = 0;

static void report_sys_health(void)
{
    uint32_t flags = (uint32_t)s_state | ((uint32_t)s_mode << 8);
    knx_health_state_t health_state =
        (s_state == KNX_SYS_FAULT || s_fault_latch != 0U)
            ? KNX_HEALTH_STATE_FAULT
            : KNX_HEALTH_STATE_OK;

    (void)knx_health_report(KNX_HEALTH_SOURCE_SYS,
                            health_state,
                            KNX_OK,
                            flags,
                            s_fault_latch);
}

static void set_state(knx_sys_state_t state)
{
    if (s_state == state) {
        return;
    }

    s_state = state;
    s_state_enter_ms = knx_millis();
}

knx_status_t knx_sys_init(void)
{
    s_mode = KNX_RUN_MODE_IDLE;
    s_fault_latch = 0;
    s_state_enter_ms = knx_millis();
    s_state = KNX_SYS_READY;
    report_sys_health();
    return KNX_OK;
}

knx_status_t knx_sys_update(void)
{
    taskENTER_CRITICAL();
    if (s_fault_latch != 0U) {
        set_state(KNX_SYS_FAULT);
        s_mode = KNX_RUN_MODE_IDLE;
        report_sys_health();
        taskEXIT_CRITICAL();
        return KNX_OK;
    }
    taskEXIT_CRITICAL();

    taskENTER_CRITICAL();
    if (s_mode == KNX_RUN_MODE_IDLE) {
        set_state(KNX_SYS_READY);
    } else {
        set_state(KNX_SYS_RUNNING);
    }
    report_sys_health();
    taskEXIT_CRITICAL();

    return KNX_OK;
}

knx_status_t knx_sys_set_mode(knx_run_mode_t mode)
{
    taskENTER_CRITICAL();
    if (s_state == KNX_SYS_FAULT) {
        if (mode != KNX_RUN_MODE_IDLE) {
            taskEXIT_CRITICAL();
            return KNX_NOT_READY;
        }
    }

    s_mode = mode;
    report_sys_health();
    taskEXIT_CRITICAL();
    return KNX_OK;
}

knx_run_mode_t knx_sys_get_mode(void)
{
    knx_run_mode_t mode;
    taskENTER_CRITICAL();
    mode = s_mode;
    taskEXIT_CRITICAL();
    return mode;
}

knx_sys_state_t knx_sys_get_state(void)
{
    knx_sys_state_t state;
    taskENTER_CRITICAL();
    state = s_state;
    taskEXIT_CRITICAL();
    return state;
}

uint32_t knx_sys_get_state_age_ms(void)
{
    return knx_millis() - s_state_enter_ms;
}

void knx_sys_report_fault(uint32_t fault_mask)
{
    taskENTER_CRITICAL();
    s_fault_latch |= fault_mask;
    if (s_fault_latch != 0U) {
        set_state(KNX_SYS_FAULT);
        s_mode = KNX_RUN_MODE_IDLE;
    }
    report_sys_health();
    taskEXIT_CRITICAL();
}

void knx_sys_clear_faults(void)
{
    taskENTER_CRITICAL();
    s_fault_latch = 0;
    set_state(KNX_SYS_READY);
    report_sys_health();
    taskEXIT_CRITICAL();
}

uint32_t knx_sys_get_fault_latch(void)
{
    uint32_t fault_latch;
    taskENTER_CRITICAL();
    fault_latch = s_fault_latch;
    taskEXIT_CRITICAL();
    return fault_latch;
}
