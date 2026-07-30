/**
 * @file    knx_encoder_mspm0.c
 * @brief   Encoder platform implementation for MSPM0.
 *
 *          Left:  software quadrature via GPIO EXTI (PA8/PA9) — type KNX_ENCODER_MSPM0_EXTI
 *          Right: hardware QEI on TIMG8 (PA21/PA22) — type KNX_ENCODER_MSPM0_QEI
 *
 *          Both are accessed through the same knx_encoder_read_raw() interface.
 *          The left encoder's ISR accumulates into a global 32-bit counter;
 *          read_raw returns its low 16 bits so the common Encoder_CalcDiff()
 *          uint16 wrapping logic works unchanged.
 */

#include "knx_encoder.h"
#include "knx_time.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>

/* ────────────────────────────────────────────────────────
 *  Common interface
 * ──────────────────────────────────────────────────────── */

/* Defined below in the EXTI ISR section; declared here so read_raw can see it. */
extern volatile int32_t s_exti_count;
volatile uint16_t knx_encoder_mspm0_left_raw;
volatile uint16_t knx_encoder_mspm0_right_raw;
volatile uint32_t knx_encoder_mspm0_left_irq_count;
volatile uint32_t knx_encoder_mspm0_left_phase_bits;

knx_status_t knx_encoder_start(const knx_encoder_port_t *port)
{
    if (port == NULL) return KNX_INVALID_ARG;

    if (port->type == KNX_ENCODER_MSPM0_QEI) {
        if (port->timer == NULL) return KNX_NOT_READY;
        DL_TimerG_startCounter((GPTIMER_Regs *)port->timer);
        return KNX_OK;
    }

    if (port->type == KNX_ENCODER_MSPM0_EXTI) {
        /* GPIO encoder is always running — ISR active from board init. */
        return KNX_OK;
    }

    return KNX_NOT_READY;
}

knx_status_t knx_encoder_read_raw(const knx_encoder_port_t *port,
                                  uint16_t *raw)
{
    if (port == NULL || raw == NULL) return KNX_INVALID_ARG;

    if (port->type == KNX_ENCODER_MSPM0_QEI) {
        if (port->timer == NULL) return KNX_NOT_READY;
        *raw = (uint16_t)DL_TimerG_getTimerCount(
                   (GPTIMER_Regs *)port->timer);
        knx_encoder_mspm0_right_raw = *raw;
        return KNX_OK;
    }

    if (port->type == KNX_ENCODER_MSPM0_EXTI) {
        *raw = (uint16_t)(uint32_t)s_exti_count;   /* low 16 bits */
        knx_encoder_mspm0_left_raw = *raw;
        return KNX_OK;
    }

    return KNX_NOT_READY;
}

/* ────────────────────────────────────────────────────────
 *  GPIO quadrature ISR (left encoder, PA8/PA9)
 * ──────────────────────────────────────────────────────── */

volatile int32_t s_exti_count = 0;
static uint8_t   s_prev_phase = 0;

/* Quadrature LUT: index = (old<<2)|new → +1 / -1 / 0 */
static const int8_t s_lut[16] = {
     0, -1, +1,  0,
     1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static inline uint8_t read_phase(void)
{
    uint8_t a = DL_GPIO_readPins(GPIOA, ENC_RA_ENC_RA_PIN_PIN) ? 1U : 0U;
    uint8_t b = DL_GPIO_readPins(GPIOA, ENC_RB_ENC_RB_PIN_PIN) ? 1U : 0U;
    uint8_t phase = (uint8_t)((a << 1) | b);
    knx_encoder_mspm0_left_phase_bits = phase;
    return phase;
}

static void decode_edge(void)
{
    uint8_t new_ph = read_phase();
    int8_t  d      = s_lut[(s_prev_phase << 2) | new_ph];
    s_exti_count  += d;
    s_prev_phase   = new_ph;
}

void GROUP1_IRQHandler(void)
{
    uint32_t mis = DL_GPIO_getEnabledInterruptStatus(
                       GPIOA, ENC_RA_ENC_RA_PIN_PIN | ENC_RB_ENC_RB_PIN_PIN);
    if (mis) {
        knx_encoder_mspm0_left_irq_count++;
        decode_edge();
        DL_GPIO_clearInterruptStatus(GPIOA, mis);
    }
}
