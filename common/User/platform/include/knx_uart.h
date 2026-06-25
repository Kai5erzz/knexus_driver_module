#ifndef KNX_UART_H
#define KNX_UART_H

#include "knx_types.h"

typedef struct {
    void     *handle;
    uint32_t  baudrate;
} knx_uart_t;

knx_status_t knx_uart_init(knx_uart_t *uart);
knx_status_t knx_uart_transmit(knx_uart_t *uart, const uint8_t *data, uint32_t len, uint32_t timeout_ms);
knx_status_t knx_uart_receive(knx_uart_t *uart, uint8_t *data, uint32_t len, uint32_t timeout_ms);

#endif /* KNX_UART_H */
