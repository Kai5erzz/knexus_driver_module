#include "knx_uart.h"
#include "knx_time.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

static uint8_t timeout_elapsed(uint32_t start_ms, uint32_t timeout_ms)
{
    if (timeout_ms == 0U) {
        return 0U;
    }

    return ((knx_millis() - start_ms) >= timeout_ms) ? 1U : 0U;
}

knx_status_t knx_uart_init(knx_uart_t *uart)
{
    if (uart == NULL || uart->handle == NULL) {
        return KNX_NOT_READY;
    }

    return KNX_OK;
}

knx_status_t knx_uart_transmit(knx_uart_t *uart,
                               const uint8_t *data,
                               uint32_t len,
                               uint32_t timeout_ms)
{
    if (uart == NULL || uart->handle == NULL || data == NULL) {
        return KNX_INVALID_ARG;
    }

    UART_Regs *regs = (UART_Regs *)uart->handle;
    uint32_t start_ms = knx_millis();

    for (uint32_t i = 0U; i < len; i++) {
        while (DL_UART_isTXFIFOFull(regs)) {
            if (timeout_elapsed(start_ms, timeout_ms) != 0U) {
                return KNX_TIMEOUT;
            }
        }
        DL_UART_transmitData(regs, data[i]);
    }

    return KNX_OK;
}

knx_status_t knx_uart_receive(knx_uart_t *uart,
                              uint8_t *data,
                              uint32_t len,
                              uint32_t timeout_ms)
{
    if (uart == NULL || uart->handle == NULL || data == NULL) {
        return KNX_INVALID_ARG;
    }

    UART_Regs *regs = (UART_Regs *)uart->handle;
    uint32_t start_ms = knx_millis();

    for (uint32_t i = 0U; i < len; i++) {
        while (!DL_UART_receiveDataCheck(regs, &data[i])) {
            if (timeout_elapsed(start_ms, timeout_ms) != 0U) {
                return KNX_TIMEOUT;
            }
        }
    }

    return KNX_OK;
}
