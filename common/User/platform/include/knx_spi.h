#ifndef KNX_SPI_H
#define KNX_SPI_H

#include "knx_types.h"
#include "knx_gpio.h"

typedef struct {
    void       *handle;
    knx_gpio_t  cs_gpio;
} knx_spi_t;

knx_status_t knx_spi_init(knx_spi_t *spi);
knx_status_t knx_spi_transmit(knx_spi_t *spi, const uint8_t *data, uint32_t len, uint32_t timeout_ms);
knx_status_t knx_spi_receive(knx_spi_t *spi, uint8_t *data, uint32_t len, uint32_t timeout_ms);
knx_status_t knx_spi_transmit_receive(knx_spi_t *spi,
                                       const uint8_t *tx, uint8_t *rx,
                                       uint32_t len, uint32_t timeout_ms);

void knx_spi_cs_low(knx_spi_t *spi);
void knx_spi_cs_high(knx_spi_t *spi);

#endif /* KNX_SPI_H */
