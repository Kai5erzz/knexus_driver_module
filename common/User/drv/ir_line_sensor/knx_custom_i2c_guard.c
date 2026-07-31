#include "knx_custom_i2c_guard.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

static volatile uint8_t s_bus_busy;

bool knx_custom_i2c_try_lock(void)
{
    bool acquired = false;
    taskENTER_CRITICAL();
    if (s_bus_busy == 0U) {
        s_bus_busy = 1U;
        acquired = true;
    }
    taskEXIT_CRITICAL();
    return acquired;
}

void knx_custom_i2c_unlock(void)
{
    taskENTER_CRITICAL();
    s_bus_busy = 0U;
    taskEXIT_CRITICAL();
}
