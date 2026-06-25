#include "knx_spi.h"
#include "knx_time.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <stddef.h>

static uint8_t timeout_elapsed(uint32_t start_ms, uint32_t timeout_ms)
{
    if (timeout_ms == 0U) {
        return 0U;
    }

    return ((knx_millis() - start_ms) >= timeout_ms) ? 1U : 0U;
}

static knx_status_t wait_tx_ready(SPI_Regs *regs, uint32_t start_ms, uint32_t timeout_ms)
{
    while (DL_SPI_isTXFIFOFull(regs)) {
        if (timeout_elapsed(start_ms, timeout_ms) != 0U) {
            return KNX_TIMEOUT;
        }
    }
    return KNX_OK;
}

static knx_status_t wait_rx_ready(SPI_Regs *regs, uint32_t start_ms, uint32_t timeout_ms)
{
    while (DL_SPI_isRXFIFOEmpty(regs)) {
        if (timeout_elapsed(start_ms, timeout_ms) != 0U) {
            return KNX_TIMEOUT;
        }
    }
    return KNX_OK;
}

static void flush_rx(SPI_Regs *regs)
{
    uint8_t dummy;
    while (DL_SPI_receiveDataCheck8(regs, &dummy)) {
    }
}

knx_status_t knx_spi_init(knx_spi_t *spi)
{
    if (spi == NULL || spi->handle == NULL) {
        return KNX_NOT_READY;
    }

    knx_spi_cs_high(spi);
    flush_rx((SPI_Regs *)spi->handle);
    return KNX_OK;
}

knx_status_t knx_spi_transmit(knx_spi_t *spi,
                              const uint8_t *data,
                              uint32_t len,
                              uint32_t timeout_ms)
{
    if (spi == NULL || spi->handle == NULL) {
        return KNX_NOT_READY;
    }
    if (data == NULL && len > 0U) {
        return KNX_INVALID_ARG;
    }

    SPI_Regs *regs = (SPI_Regs *)spi->handle;
    uint32_t start_ms = knx_millis();
    flush_rx(regs);

    for (uint32_t i = 0U; i < len; i++) {
        knx_status_t st = wait_tx_ready(regs, start_ms, timeout_ms);
        if (st != KNX_OK) {
            return st;
        }
        DL_SPI_transmitData8(regs, data[i]);

        st = wait_rx_ready(regs, start_ms, timeout_ms);
        if (st != KNX_OK) {
            return st;
        }
        (void)DL_SPI_receiveData8(regs);
    }
    return KNX_OK;
}

knx_status_t knx_spi_receive(knx_spi_t *spi,
                             uint8_t *data,
                             uint32_t len,
                             uint32_t timeout_ms)
{
    if (spi == NULL || spi->handle == NULL) {
        return KNX_NOT_READY;
    }
    if (data == NULL && len > 0U) {
        return KNX_INVALID_ARG;
    }

    SPI_Regs *regs = (SPI_Regs *)spi->handle;
    uint32_t start_ms = knx_millis();
    flush_rx(regs);

    for (uint32_t i = 0U; i < len; i++) {
        knx_status_t st = wait_tx_ready(regs, start_ms, timeout_ms);
        if (st != KNX_OK) {
            return st;
        }
        DL_SPI_transmitData8(regs, 0xFFU);

        st = wait_rx_ready(regs, start_ms, timeout_ms);
        if (st != KNX_OK) {
            return st;
        }
        data[i] = DL_SPI_receiveData8(regs);
    }

    return KNX_OK;
}

knx_status_t knx_spi_transmit_receive(knx_spi_t *spi,
                                      const uint8_t *tx,
                                      uint8_t *rx,
                                      uint32_t len,
                                      uint32_t timeout_ms)
{
    if (spi == NULL || spi->handle == NULL) {
        return KNX_NOT_READY;
    }
    if ((tx == NULL || rx == NULL) && len > 0U) {
        return KNX_INVALID_ARG;
    }

    SPI_Regs *regs = (SPI_Regs *)spi->handle;
    uint32_t start_ms = knx_millis();
    flush_rx(regs);

    for (uint32_t i = 0U; i < len; i++) {
        knx_status_t st = wait_tx_ready(regs, start_ms, timeout_ms);
        if (st != KNX_OK) {
            return st;
        }
        DL_SPI_transmitData8(regs, tx[i]);

        st = wait_rx_ready(regs, start_ms, timeout_ms);
        if (st != KNX_OK) {
            return st;
        }
        rx[i] = DL_SPI_receiveData8(regs);
    }

    return KNX_OK;
}

void knx_spi_cs_low(knx_spi_t *spi)
{
    if (spi != NULL) {
        knx_gpio_write(spi->cs_gpio, KNX_GPIO_LOW);
    }
}

void knx_spi_cs_high(knx_spi_t *spi)
{
    if (spi != NULL) {
        knx_gpio_write(spi->cs_gpio, KNX_GPIO_HIGH);
    }
}
