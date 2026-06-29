/* Encoder platform implementation for STM32H743 */
#include "knx_encoder.h"
#include "stm32h7xx_hal.h"

knx_status_t knx_encoder_start(const knx_encoder_port_t *port)
{
    if (port == NULL || port->timer == NULL) {
        return KNX_INVALID_ARG;
    }

    if (port->type == KNX_ENCODER_TIM) {
        HAL_TIM_Encoder_Start((TIM_HandleTypeDef *)port->timer, port->channel);
    } else {
        uint32_t period = port->period ? port->period : 0xFFFF;
        HAL_LPTIM_Encoder_Start((LPTIM_HandleTypeDef *)port->timer, period);
    }

    return KNX_OK;
}

knx_status_t knx_encoder_read_raw(const knx_encoder_port_t *port, uint16_t *raw)
{
    if (port == NULL || port->timer == NULL || raw == NULL) {
        return KNX_INVALID_ARG;
    }

    if (port->type == KNX_ENCODER_TIM) {
        *raw = (uint16_t)__HAL_TIM_GET_COUNTER((TIM_HandleTypeDef *)port->timer);
    } else {
        *raw = (uint16_t)HAL_LPTIM_ReadCounter((LPTIM_HandleTypeDef *)port->timer);
    }

    return KNX_OK;
}
