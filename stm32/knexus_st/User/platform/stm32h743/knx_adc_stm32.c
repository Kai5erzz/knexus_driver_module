/* ADC implementation for STM32H743 */
#include "knx_adc.h"
#include "stm32h7xx_hal.h"

#define KNX_ADC_DMA_PAIR_LEN  2U

static volatile uint16_t adc1_dma_pair[KNX_ADC_DMA_PAIR_LEN] __attribute__((aligned(4)));
static volatile uint32_t adc1_dma_sequence = 0;
static uint8_t adc1_dma_started = 0;

knx_status_t knx_adc_read_raw(const knx_adc_channel_t *ch, uint32_t *raw)
{
    if (ch == NULL || raw == NULL || ch->adc == NULL) {
        return KNX_INVALID_ARG;
    }

    ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)ch->adc;

    if (HAL_ADC_Start(hadc) != HAL_OK) {
        return KNX_ERROR;
    }

    uint32_t timeout = (ch->timeout_ms > 0) ? ch->timeout_ms : 1U;
    if (HAL_ADC_PollForConversion(hadc, timeout) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return KNX_TIMEOUT;
    }

    *raw = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return KNX_OK;
}

knx_status_t knx_adc_start_dma_pair(const knx_adc_channel_t *ch)
{
    if (ch == NULL || ch->adc == NULL) {
        return KNX_INVALID_ARG;
    }

    ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)ch->adc;
    if (hadc->Instance != ADC1) {
        return KNX_INVALID_ARG;
    }

    if (adc1_dma_started) {
        return KNX_OK;
    }

    adc1_dma_pair[0] = 0;
    adc1_dma_pair[1] = 0;
    adc1_dma_sequence = 0;

    if (HAL_ADC_Start_DMA(hadc, (uint32_t *)adc1_dma_pair, KNX_ADC_DMA_PAIR_LEN) != HAL_OK) {
        return KNX_ERROR;
    }

    adc1_dma_started = 1;
    return KNX_OK;
}

knx_status_t knx_adc_stop_dma_pair(const knx_adc_channel_t *ch)
{
    if (ch == NULL || ch->adc == NULL) {
        return KNX_INVALID_ARG;
    }

    ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)ch->adc;
    if (hadc->Instance != ADC1) {
        return KNX_INVALID_ARG;
    }

    if (adc1_dma_started) {
        if (HAL_ADC_Stop_DMA(hadc) != HAL_OK) {
            return KNX_ERROR;
        }
    }

    adc1_dma_started = 0;
    adc1_dma_sequence = 0;
    adc1_dma_pair[0] = 0;
    adc1_dma_pair[1] = 0;
    return KNX_OK;
}

knx_status_t knx_adc_restart_dma_pair(const knx_adc_channel_t *ch)
{
    knx_status_t stop_status = knx_adc_stop_dma_pair(ch);
    if (stop_status != KNX_OK) {
        return stop_status;
    }

    return knx_adc_start_dma_pair(ch);
}

knx_status_t knx_adc_read_dma_pair(const knx_adc_channel_t *ch,
                                   uint16_t *rank1,
                                   uint16_t *rank2,
                                   uint32_t *sequence)
{
    if (ch == NULL || ch->adc == NULL || rank1 == NULL || rank2 == NULL) {
        return KNX_INVALID_ARG;
    }

    ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)ch->adc;
    if (hadc->Instance != ADC1) {
        return KNX_INVALID_ARG;
    }

    if (!adc1_dma_started || adc1_dma_sequence == 0U) {
        return KNX_NOT_READY;
    }

    uint32_t seq_a;
    uint32_t seq_b;
    uint16_t sample0;
    uint16_t sample1;

    do {
        seq_a = adc1_dma_sequence;
        sample0 = adc1_dma_pair[0];
        sample1 = adc1_dma_pair[1];
        seq_b = adc1_dma_sequence;
    } while (seq_a != seq_b);

    *rank1 = sample0;
    *rank2 = sample1;
    if (sequence != NULL) {
        *sequence = seq_b;
    }

    return KNX_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != NULL && hadc->Instance == ADC1) {
        adc1_dma_sequence++;
    }
}
