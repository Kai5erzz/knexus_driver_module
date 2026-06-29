/* UART implementation for STM32H743 — stub, drv/ drivers use HAL directly */
#include "knx_uart.h"
#include "stm32h7xx_hal.h"

/* UART is fully initialised by CubeMX HAL_UART_Init() before the board layer
 * runs. knx_uart_init() only validates that the handle is present. */

knx_status_t knx_uart_init(knx_uart_t *uart)
{
    if (uart == NULL || uart->handle == NULL) {
        return KNX_INVALID_ARG;
    }
    return KNX_OK;
}

knx_status_t knx_uart_transmit(knx_uart_t *uart, const uint8_t *data,
                                uint32_t len, uint32_t timeout_ms)
{
    if (uart == NULL || uart->handle == NULL || data == NULL) {
        return KNX_INVALID_ARG;
    }
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)uart->handle;
    HAL_StatusTypeDef st = HAL_UART_Transmit(huart, (uint8_t *)data, len, timeout_ms);
    return (st == HAL_OK) ? KNX_OK : KNX_ERROR;
}

knx_status_t knx_uart_receive(knx_uart_t *uart, uint8_t *data,
                               uint32_t len, uint32_t timeout_ms)
{
    if (uart == NULL || uart->handle == NULL || data == NULL) {
        return KNX_INVALID_ARG;
    }
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)uart->handle;
    HAL_StatusTypeDef st = HAL_UART_Receive(huart, data, len, timeout_ms);
    return (st == HAL_OK) ? KNX_OK : KNX_ERROR;
}
