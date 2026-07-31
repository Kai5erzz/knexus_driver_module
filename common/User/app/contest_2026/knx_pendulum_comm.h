#ifndef KNX_PENDULUM_COMM_H
#define KNX_PENDULUM_COMM_H

#include <stdbool.h>
#include <stdint.h>
#include "knx_types.h"
#include "knx_uart.h"

#define KNX_PENDULUM_FRAME_SIZE 8U
#define KNX_PENDULUM_STATUS_FRAME_SIZE 3U

typedef enum {
    KNX_PENDULUM_MODE_1 = 1,
    KNX_PENDULUM_MODE_2 = 2,
    KNX_PENDULUM_MODE_3 = 3,
    KNX_PENDULUM_MODE_4 = 4,
    KNX_PENDULUM_MODE_5 = 5,
    KNX_PENDULUM_MODE_6 = 6,
} knx_pendulum_mode_t;

typedef struct {
    float offset; /* 上位机小球偏移量，单位cm，合法范围-12.5~+12.5。 */
    uint32_t last_rx_ms;
    uint32_t rx_count;
    uint32_t crc_error_count;
    uint32_t invalid_value_count;
    uint32_t sync_loss_count;
    uint32_t tx_count;
    uint8_t last_mode;
    knx_status_t last_tx_status;
} knx_pendulum_comm_state_t;

/* Raw upper-computer protocol:
 *   AA 55 | float32 little-endian offset_cm | CRC16/Modbus little-endian
 * CRC covers the first six bytes. */
void knx_pendulum_comm_init(void);
void knx_pendulum_comm_attach_uart(knx_uart_t *uart);
void knx_pendulum_comm_feed(const uint8_t *data, uint16_t len);
/* Raw lower-to-upper mode frame: 55 AA | mode(1..6). */
knx_status_t knx_pendulum_comm_send_mode(knx_pendulum_mode_t mode);
void knx_pendulum_comm_snapshot(knx_pendulum_comm_state_t *out);

#endif
