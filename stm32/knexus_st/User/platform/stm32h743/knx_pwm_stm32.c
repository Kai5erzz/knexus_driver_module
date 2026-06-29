/* PWM implementation for STM32H743 */
#include "knx_pwm.h"
#include "stm32h7xx_hal.h"

knx_status_t knx_pwm_start(knx_pwm_channel_t *ch)
{
    if (ch == NULL || ch->timer == NULL) {
        return KNX_INVALID_ARG;
    }
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ch->timer;
    HAL_TIM_PWM_Start(htim, ch->channel);
    return KNX_OK;
}

knx_status_t knx_pwm_stop(knx_pwm_channel_t *ch)
{
    if (ch == NULL || ch->timer == NULL) {
        return KNX_INVALID_ARG;
    }
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ch->timer;
    HAL_TIM_PWM_Stop(htim, ch->channel);
    return KNX_OK;
}

knx_status_t knx_pwm_set_duty(knx_pwm_channel_t *ch, float duty)
{
    if (ch == NULL || ch->timer == NULL) {
        return KNX_INVALID_ARG;
    }

    /* Clamp duty to [0.0, 1.0] */
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ch->timer;

    /* Use ch->arr if set, otherwise read from hardware */
    uint32_t arr = ch->arr;
    if (arr == 0) {
        arr = htim->Instance->ARR;
    }

    /* STM32 ARR=arr counter rolls 0..arr (arr+1 counts total), PWM high when
     * counter < CCR. With duty=1.0 we want compare=arr+1 so CCR > counter
     * always → continuously high.  No explicit +1 corner-case fix needed
     * (see knx_pwm_mspm0.c for Timer-A variant). */
    uint32_t compare = (uint32_t)(duty * (float)(arr + 1));
    if (compare > (arr + 1U)) {  /* belt-and-braces guard for duty>1.0 */
        compare = arr + 1U;
    }
    __HAL_TIM_SET_COMPARE(htim, ch->channel, compare);

    return KNX_OK;
}

knx_status_t knx_pwm_set_compare(knx_pwm_channel_t *ch, uint32_t compare)
{
    if (ch == NULL || ch->timer == NULL) {
        return KNX_INVALID_ARG;
    }

    uint32_t period = 0U;
    if (knx_pwm_get_period(ch, &period) == KNX_OK && period > 0U && compare > period) {
        compare = period;
    }

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ch->timer;
    __HAL_TIM_SET_COMPARE(htim, ch->channel, compare);
    return KNX_OK;
}

knx_status_t knx_pwm_get_period(const knx_pwm_channel_t *ch, uint32_t *period)
{
    if (ch == NULL || ch->timer == NULL || period == NULL) {
        return KNX_INVALID_ARG;
    }

    if (ch->arr != 0U) {
        *period = ch->arr;
        return KNX_OK;
    }

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ch->timer;
    *period = htim->Instance->ARR;
    return KNX_OK;
}

knx_status_t knx_pwm_get_counter(const knx_pwm_channel_t *ch, uint32_t *counter)
{
    if (ch == NULL || ch->timer == NULL || counter == NULL) {
        return KNX_INVALID_ARG;
    }

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ch->timer;
    *counter = __HAL_TIM_GET_COUNTER(htim);
    return KNX_OK;
}

knx_status_t knx_pwm_start_compare(knx_pwm_channel_t *ch)
{
    if (ch == NULL || ch->timer == NULL) {
        return KNX_INVALID_ARG;
    }

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ch->timer;
    return (HAL_TIM_OC_Start(htim, ch->channel) == HAL_OK) ? KNX_OK : KNX_ERROR;
}
