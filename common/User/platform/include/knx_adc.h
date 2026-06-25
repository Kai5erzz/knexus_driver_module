#ifndef KNX_ADC_H
#define KNX_ADC_H

#include "knx_types.h"

typedef struct {
    void     *adc;
    uint32_t  timeout_ms;
} knx_adc_channel_t;

knx_status_t knx_adc_read_raw(const knx_adc_channel_t *ch, uint32_t *raw);
knx_status_t knx_adc_start_dma_pair(const knx_adc_channel_t *ch);
knx_status_t knx_adc_stop_dma_pair(const knx_adc_channel_t *ch);
knx_status_t knx_adc_restart_dma_pair(const knx_adc_channel_t *ch);
knx_status_t knx_adc_read_dma_pair(const knx_adc_channel_t *ch,
                                   uint16_t *rank1,
                                   uint16_t *rank2,
                                   uint32_t *sequence);

#endif /* KNX_ADC_H */
