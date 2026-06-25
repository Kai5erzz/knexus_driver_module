/**
 * @file  knx_pwm_mspm0.c
 * @brief PWM platform layer – MSPM0 DriverLib (Timer_A only)
 *
 * Uses DL_TimerA_* functions exclusively since motor PWM uses TIMA0/TIMA1.
 */

#include "knx_pwm.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/* Map the generic channel index to the DriverLib CC index enum. */
static inline DL_TIMER_CC_INDEX cc_index(uint32_t ch)
{
    switch (ch) {
        case 0:  return DL_TIMER_CC_0_INDEX;
        case 1:  return DL_TIMER_CC_1_INDEX;
        case 2:  return DL_TIMER_CC_2_INDEX;
        case 3:  return DL_TIMER_CC_3_INDEX;
        default: return DL_TIMER_CC_0_INDEX;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

knx_status_t knx_pwm_start(knx_pwm_channel_t *ch)
{
    if (!ch || !ch->timer)
        return KNX_INVALID_ARG;

    DL_TimerA_startCounter((GPTIMER_Regs *)ch->timer);
    return KNX_OK;
}

knx_status_t knx_pwm_stop(knx_pwm_channel_t *ch)
{
    if (!ch || !ch->timer)
        return KNX_INVALID_ARG;

    DL_TimerA_stopCounter((GPTIMER_Regs *)ch->timer);
    return KNX_OK;
}

knx_status_t knx_pwm_set_duty(knx_pwm_channel_t *ch, float duty)
{
    if (!ch || !ch->timer)
        return KNX_INVALID_ARG;

    /* Clamp duty to valid range */
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    /* Get period – use cached arr, or fall back to reading HW */
    uint32_t period = ch->arr;
    if (period == 0) {
        period = DL_TimerA_getCaptureCompareValue(
                     (GPTIMER_Regs *)ch->timer, cc_index(ch->channel));
    }

    uint32_t compare = (uint32_t)(duty * (float)period);

    DL_TimerA_setCaptureCompareValue(
        (GPTIMER_Regs *)ch->timer, compare, cc_index(ch->channel));

    return KNX_OK;
}

knx_status_t knx_pwm_set_compare(knx_pwm_channel_t *ch, uint32_t compare)
{
    if (!ch || !ch->timer)
        return KNX_INVALID_ARG;

    DL_TimerA_setCaptureCompareValue(
        (GPTIMER_Regs *)ch->timer, compare, cc_index(ch->channel));

    return KNX_OK;
}

knx_status_t knx_pwm_get_period(const knx_pwm_channel_t *ch, uint32_t *period)
{
    if (!ch || !period)
        return KNX_INVALID_ARG;

    if (ch->arr > 0) {
        *period = ch->arr;
    } else {
        /* Read period from hardware via capture-compare register */
        *period = DL_TimerA_getCaptureCompareValue(
                      (GPTIMER_Regs *)ch->timer, cc_index(ch->channel));
    }

    return KNX_OK;
}

knx_status_t knx_pwm_get_counter(const knx_pwm_channel_t *ch, uint32_t *counter)
{
    if (!ch || !ch->timer || !counter)
        return KNX_INVALID_ARG;

    *counter = DL_TimerA_getTimerCount((GPTIMER_Regs *)ch->timer);
    return KNX_OK;
}

knx_status_t knx_pwm_start_compare(knx_pwm_channel_t *ch)
{
    if (!ch || !ch->timer)
        return KNX_INVALID_ARG;

    DL_TimerA_startCounter((GPTIMER_Regs *)ch->timer);
    return KNX_OK;
}
