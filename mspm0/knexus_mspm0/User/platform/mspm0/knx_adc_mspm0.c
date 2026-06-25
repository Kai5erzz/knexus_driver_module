/**
 * @file  knx_adc_mspm0.c
 * @brief ADC platform layer – MSPM0 DriverLib (ADC12)
 *
 * Implements the knx_adc.h interface for TI MSPM0 devices using the
 * DL_ADC12 driver.  Motor-current sense uses a single-shot conversion
 * on ADC0 / MEM0 (12-bit, VDDA reference = 3.3 V).
 *
 * DMA pair mode is intentionally a no-op on MSPM0 — the single-shot
 * read in knx_adc_read_raw() is sufficient for motor current sampling.
 */

#include "knx_adc.h"
#include "knx_time.h"

#include <stdint.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

knx_status_t knx_adc_read_raw(const knx_adc_channel_t *ch, uint32_t *raw)
{
    if (!ch || !ch->adc || !raw)
        return KNX_INVALID_ARG;

    ADC12_Regs *adc = (ADC12_Regs *)ch->adc;

    /* Ensure conversions are enabled, then trigger a single conversion. */
    DL_ADC12_enableConversions(adc);
    DL_ADC12_startConversion(adc);

    /* Poll for completion with timeout. */
    uint32_t start    = knx_millis();
    uint32_t deadline = start + ch->timeout_ms;

    while (DL_ADC12_getStatus(adc) & DL_ADC12_STATUS_CONVERSION_ACTIVE) {
        if (knx_millis() >= deadline) {
            return KNX_TIMEOUT;
        }
    }

    *raw = (uint32_t)DL_ADC12_getMemResult(adc, DL_ADC12_MEM_IDX_0);

    return KNX_OK;
}

/* DMA pair mode is not used on MSPM0.  Single-shot read_raw() is
   sufficient for motor current sense. */

knx_status_t knx_adc_start_dma_pair(const knx_adc_channel_t *ch)
{
    (void)ch;
    return KNX_OK;
}

knx_status_t knx_adc_stop_dma_pair(const knx_adc_channel_t *ch)
{
    (void)ch;
    return KNX_OK;
}

knx_status_t knx_adc_restart_dma_pair(const knx_adc_channel_t *ch)
{
    (void)ch;
    return KNX_OK;
}

knx_status_t knx_adc_read_dma_pair(const knx_adc_channel_t *ch,
                                   uint16_t *rank1,
                                   uint16_t *rank2,
                                   uint32_t *sequence)
{
    if (!ch || !ch->adc || !rank1 || !rank2 || !sequence)
        return KNX_INVALID_ARG;

    /* Perform a single-shot read and use the same value for both ranks. */
    uint32_t raw = 0;
    knx_status_t st = knx_adc_read_raw(ch, &raw);
    if (st != KNX_OK)
        return st;

    *rank1    = (uint16_t)raw;
    *rank2    = (uint16_t)raw;
    *sequence = 0;

    return KNX_OK;
}
