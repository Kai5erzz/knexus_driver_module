#ifndef KNX_HOST_COMM_H
#define KNX_HOST_COMM_H

#include "knx_types.h"
#include "knx_uart.h"
#include <stdint.h>

#define KNX_HOST_COMM_SOF0              0xA5U
#define KNX_HOST_COMM_SOF1              0x5AU
#define KNX_HOST_COMM_EOF0              0x0DU
#define KNX_HOST_COMM_EOF1              0x0AU
#define KNX_HOST_COMM_MAX_PAYLOAD       256U
#define KNX_HOST_COMM_FRAME_OVERHEAD    8U
#define KNX_HOST_COMM_MAX_FRAME         (KNX_HOST_COMM_MAX_PAYLOAD + KNX_HOST_COMM_FRAME_OVERHEAD)
#define KNX_HOST_COMM_MAX_EXTRA_CALLBACKS 4U

#define KNX_HOST_COMM_CRC_PAYLOAD_ONLY  0U
#define KNX_HOST_COMM_CRC_LEN_PAYLOAD   1U

#ifndef KNX_HOST_COMM_CRC_MODE
#define KNX_HOST_COMM_CRC_MODE          KNX_HOST_COMM_CRC_LEN_PAYLOAD
#endif

typedef void (*knx_host_comm_payload_callback_t)(const uint8_t *payload,
                                                 uint16_t len,
                                                 void *user);

typedef struct {
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t tx_errors;
    uint32_t rx_bytes;
    uint32_t crc_errors;
    uint32_t eof_errors;
    uint32_t overflow_errors;
    uint32_t oversized_errors;
    uint32_t sync_losses;
    knx_status_t last_status;
} knx_host_comm_stats_t;

typedef struct {
    knx_uart_t *uart;
    knx_host_comm_payload_callback_t callback;
    void *callback_user;
    knx_host_comm_payload_callback_t extra_callbacks[KNX_HOST_COMM_MAX_EXTRA_CALLBACKS];
    void *extra_callback_users[KNX_HOST_COMM_MAX_EXTRA_CALLBACKS];
    uint8_t extra_callback_count;

    uint8_t rx_payload[KNX_HOST_COMM_MAX_PAYLOAD];
    uint8_t tx_frame[KNX_HOST_COMM_MAX_FRAME];
    uint16_t rx_len;
    uint16_t rx_index;
    uint16_t rx_crc;
    uint8_t state;

    knx_host_comm_stats_t stats;
} knx_host_comm_t;

void knx_host_comm_init(knx_host_comm_t *comm, knx_uart_t *uart);
void knx_host_comm_set_callback(knx_host_comm_t *comm,
                                knx_host_comm_payload_callback_t callback,
                                void *user);
knx_status_t knx_host_comm_add_callback(knx_host_comm_t *comm,
                                        knx_host_comm_payload_callback_t callback,
                                        void *user);

knx_status_t knx_host_comm_build_frame(const uint8_t *payload,
                                       uint16_t len,
                                       uint8_t *out,
                                       uint16_t out_size,
                                       uint16_t *frame_len);
knx_status_t knx_host_comm_send(knx_host_comm_t *comm,
                                const uint8_t *payload,
                                uint16_t len,
                                uint32_t timeout_ms);

knx_status_t knx_host_comm_feed_byte(knx_host_comm_t *comm, uint8_t byte);
knx_status_t knx_host_comm_feed(knx_host_comm_t *comm,
                                const uint8_t *data,
                                uint16_t len);
knx_status_t knx_host_comm_poll(knx_host_comm_t *comm,
                                uint16_t max_bytes,
                                uint32_t byte_timeout_ms);

void knx_host_comm_get_stats(knx_host_comm_t *comm, knx_host_comm_stats_t *stats);
void knx_host_comm_reset_stats(knx_host_comm_t *comm);
uint16_t knx_host_comm_max_payload(void);

#endif /* KNX_HOST_COMM_H */
