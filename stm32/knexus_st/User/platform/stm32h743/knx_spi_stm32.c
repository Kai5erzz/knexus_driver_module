/* SPI implementation for STM32H743 */
#include "knx_spi.h"
#include "stm32h7xx_hal.h"

knx_status_t knx_spi_init(knx_spi_t *spi)
{
    (void)spi;
    return KNX_OK;
}

knx_status_t knx_spi_transmit(knx_spi_t *spi, const uint8_t *data,
                               uint32_t len, uint32_t timeout_ms)
{
    if (spi == NULL || spi->handle == NULL || data == NULL) {
        return KNX_INVALID_ARG;
    }
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)spi->handle;
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, (uint8_t *)data, len, timeout_ms);
    return (st == HAL_OK) ? KNX_OK : KNX_ERROR;
}

knx_status_t knx_spi_receive(knx_spi_t *spi, uint8_t *data,
                              uint32_t len, uint32_t timeout_ms)
{
    if (spi == NULL || spi->handle == NULL || data == NULL) {
        return KNX_INVALID_ARG;
    }
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)spi->handle;
    HAL_StatusTypeDef st = HAL_SPI_Receive(hspi, data, len, timeout_ms);
    return (st == HAL_OK) ? KNX_OK : KNX_ERROR;
}

knx_status_t knx_spi_transmit_receive(knx_spi_t *spi,
                                       const uint8_t *tx, uint8_t *rx,
                                       uint32_t len, uint32_t timeout_ms)
{
    if (spi == NULL || spi->handle == NULL || tx == NULL || rx == NULL) {
        return KNX_INVALID_ARG;
    }
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)spi->handle;
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, (uint8_t *)tx, rx, len, timeout_ms);
    return (st == HAL_OK) ? KNX_OK : KNX_ERROR;
}

void knx_spi_cs_low(knx_spi_t *spi)
{
    knx_gpio_write(spi->cs_gpio, KNX_GPIO_LOW);
}

void knx_spi_cs_high(knx_spi_t *spi)
{
    knx_gpio_write(spi->cs_gpio, KNX_GPIO_HIGH);
}
