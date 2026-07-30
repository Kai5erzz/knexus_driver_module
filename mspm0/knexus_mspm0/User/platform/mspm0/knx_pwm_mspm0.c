/**
 * @file  knx_pwm_mspm0.c
 * @brief MSPM0 TimerA PWM platform layer.
 */

#include "knx_pwm.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

volatile float knx_pwm_mspm0_tima0_duty;
volatile float knx_pwm_mspm0_tima1_duty;
volatile uint32_t knx_pwm_mspm0_tima0_compare;
volatile uint32_t knx_pwm_mspm0_tima1_compare;
volatile uint32_t knx_pwm_mspm0_tima0_running;
volatile uint32_t knx_pwm_mspm0_tima1_running;
volatile uint32_t knx_pwm_mspm0_tima0_odis;
volatile uint32_t knx_pwm_mspm0_tima1_odis;
volatile uint32_t knx_pwm_mspm0_tima0_ccpd;
volatile uint32_t knx_pwm_mspm0_tima1_ccpd;

/* Multiple logical PWM channels share each timer counter.  Keep output
 * enable state per CC channel so setting the heater on TIMA0/CC2 cannot
 * disable the left motor on TIMA0/CC0 (and vice versa). */
volatile uint32_t knx_pwm_mspm0_tima0_enabled_mask;
static volatile uint32_t s_tima1_enabled_mask;

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

static void force_all_outputs_low(GPTIMER_Regs *timer)
{
    if (timer == TIMA0) {
        /* TIMA0 has four CC channels. DriverLib explicitly requires the
         * advanced ODIS API for this timer class. */
        DL_TimerA_setCCPOutputDisabledAdv(timer,
            DL_TIMER_CCP0_DIS_OUT_ADV_FORCE_LOW |
            DL_TIMER_CCP1_DIS_OUT_ADV_FORCE_LOW |
            DL_TIMER_CCP2_DIS_OUT_ADV_FORCE_LOW |
            DL_TIMER_CCP3_DIS_OUT_ADV_FORCE_LOW);
    } else {
        DL_TimerA_setCCPOutputDisabled(timer,
                                      DL_TIMER_CCP_DIS_OUT_LOW,
                                      DL_TIMER_CCP_DIS_OUT_LOW);
    }
}

static uint32_t *enabled_mask_for(GPTIMER_Regs *timer)
{
    if (timer == TIMA0) return (uint32_t *)&knx_pwm_mspm0_tima0_enabled_mask;
    if (timer == TIMA1) return (uint32_t *)&s_tima1_enabled_mask;
    return NULL;
}

static void apply_enabled_outputs(GPTIMER_Regs *timer, uint32_t mask)
{
    if (timer == TIMA0) {
        uint32_t config =
            ((mask & (1UL << 0)) ? DL_TIMER_CCP0_DIS_OUT_ADV_SET_BY_OCTL
                                 : DL_TIMER_CCP0_DIS_OUT_ADV_FORCE_LOW) |
            ((mask & (1UL << 1)) ? DL_TIMER_CCP1_DIS_OUT_ADV_SET_BY_OCTL
                                 : DL_TIMER_CCP1_DIS_OUT_ADV_FORCE_LOW) |
            ((mask & (1UL << 2)) ? DL_TIMER_CCP2_DIS_OUT_ADV_SET_BY_OCTL
                                 : DL_TIMER_CCP2_DIS_OUT_ADV_FORCE_LOW) |
            ((mask & (1UL << 3)) ? DL_TIMER_CCP3_DIS_OUT_ADV_SET_BY_OCTL
                                 : DL_TIMER_CCP3_DIS_OUT_ADV_FORCE_LOW);
        DL_TimerA_setCCPOutputDisabledAdv(timer, config);
    } else {
        DL_TimerA_setCCPOutputDisabled(timer,
            (mask & (1UL << 0)) ? DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL
                                : DL_TIMER_CCP_DIS_OUT_LOW,
            (mask & (1UL << 1)) ? DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL
                                : DL_TIMER_CCP_DIS_OUT_LOW);
    }
}

static uint32_t set_channel_enabled(GPTIMER_Regs *timer,
                                    uint32_t channel,
                                    bool enabled)
{
    uint32_t *mask_ptr = enabled_mask_for(timer);
    if (mask_ptr == NULL || channel > 3U) return 0U;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t mask = *mask_ptr;
    if (enabled) mask |= (1UL << channel);
    else mask &= ~(1UL << channel);
    *mask_ptr = mask;
    apply_enabled_outputs(timer, mask);
    if (primask == 0U) __enable_irq();
    return mask;
}

static void record_pwm(GPTIMER_Regs *timer, float duty,
                       uint32_t compare, uint32_t running)
{
    if (timer == TIMA0) {
        knx_pwm_mspm0_tima0_duty = duty;
        knx_pwm_mspm0_tima0_compare = compare;
        knx_pwm_mspm0_tima0_running = running;
        knx_pwm_mspm0_tima0_odis = timer->COMMONREGS.ODIS;
        knx_pwm_mspm0_tima0_ccpd = timer->COMMONREGS.CCPD;
    } else if (timer == TIMA1) {
        knx_pwm_mspm0_tima1_duty = duty;
        knx_pwm_mspm0_tima1_compare = compare;
        knx_pwm_mspm0_tima1_running = running;
        knx_pwm_mspm0_tima1_odis = timer->COMMONREGS.ODIS;
        knx_pwm_mspm0_tima1_ccpd = timer->COMMONREGS.CCPD;
    }
}

knx_status_t knx_pwm_start(knx_pwm_channel_t *ch)
{
    if (ch == NULL || ch->timer == NULL) return KNX_INVALID_ARG;
    DL_TimerA_startCounter((GPTIMER_Regs *)ch->timer);
    return KNX_OK;
}

knx_status_t knx_pwm_stop(knx_pwm_channel_t *ch)
{
    if (ch == NULL || ch->timer == NULL) return KNX_INVALID_ARG;

    GPTIMER_Regs *timer = (GPTIMER_Regs *)ch->timer;
    uint32_t period = (ch->arr > 0U) ? ch->arr : DL_TimerA_getLoadValue(timer);
    DL_TimerA_setCaptureCompareValue(timer, period, cc_index(ch->channel));
    uint32_t mask = set_channel_enabled(timer, ch->channel, false);
    if (mask == 0U) DL_TimerA_stopCounter(timer);
    record_pwm(timer, 0.0f, ch->arr, 0U);
    return KNX_OK;
}

knx_status_t knx_pwm_set_duty(knx_pwm_channel_t *ch, float duty)
{
    if (ch == NULL || ch->timer == NULL) return KNX_INVALID_ARG;

    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    GPTIMER_Regs *timer = (GPTIMER_Regs *)ch->timer;
    uint32_t period = ch->arr;
    if (period == 0U) period = DL_TimerA_getLoadValue(timer);

    /* MSPM0 edge-aligned PWM uses active time = LOAD - CC. CC=0 means
     * continuous high, which was the cause of the boot-time motor runaway. */
    if (duty <= 0.0f || period == 0U) {
        DL_TimerA_setCaptureCompareValue(timer, period, cc_index(ch->channel));
        uint32_t mask = set_channel_enabled(timer, ch->channel, false);
        if (mask == 0U) DL_TimerA_stopCounter(timer);
        record_pwm(timer, 0.0f, period, mask != 0U);
        return KNX_OK;
    }

    uint32_t active_counts = (uint32_t)(duty * (float)period);
    if (active_counts > period) active_counts = period;
    uint32_t compare = period - active_counts;

    DL_TimerA_setCaptureCompareValue(timer, compare, cc_index(ch->channel));
    (void)set_channel_enabled(timer, ch->channel, true);
    DL_TimerA_startCounter(timer);
    record_pwm(timer, duty, compare, 1U);
    return KNX_OK;
}

knx_status_t knx_pwm_set_compare(knx_pwm_channel_t *ch, uint32_t compare)
{
    if (ch == NULL || ch->timer == NULL) return KNX_INVALID_ARG;
    DL_TimerA_setCaptureCompareValue((GPTIMER_Regs *)ch->timer,
                                     compare, cc_index(ch->channel));
    return KNX_OK;
}

knx_status_t knx_pwm_get_period(const knx_pwm_channel_t *ch, uint32_t *period)
{
    if (ch == NULL || ch->timer == NULL || period == NULL) return KNX_INVALID_ARG;
    *period = (ch->arr > 0U) ? ch->arr
                             : DL_TimerA_getLoadValue((GPTIMER_Regs *)ch->timer);
    return KNX_OK;
}

knx_status_t knx_pwm_get_counter(const knx_pwm_channel_t *ch, uint32_t *counter)
{
    if (ch == NULL || ch->timer == NULL || counter == NULL) return KNX_INVALID_ARG;
    *counter = DL_TimerA_getTimerCount((GPTIMER_Regs *)ch->timer);
    return KNX_OK;
}

knx_status_t knx_pwm_start_compare(knx_pwm_channel_t *ch)
{
    return knx_pwm_start(ch);
}
