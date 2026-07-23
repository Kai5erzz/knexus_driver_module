#ifndef KNX_COMM_H
#define KNX_COMM_H

#include "knx_host_comm.h"

#define KNX_COMM_VERSION       1U
#define KNX_COMM_MAX_PAYLOAD   120U
#define KNX_COMM_MAX_HANDLERS  8U

typedef enum {
    KNX_COMM_HOST = 0,
    KNX_COMM_BOARD = 1,
    KNX_COMM_PEER = 2,
    KNX_COMM_CHANNEL_COUNT = 3,
} knx_comm_channel_t;

typedef enum {
    KNX_COMM_MSG_HEARTBEAT = 1,
    KNX_COMM_MSG_COMMAND = 2,
    KNX_COMM_MSG_STATE = 3,
    KNX_COMM_MSG_TRACK = 4,
    KNX_COMM_MSG_INTERSECTION = 5,
    KNX_COMM_MSG_USER = 0x40,
} knx_comm_message_type_t;

typedef struct {
    uint8_t type;
    uint8_t source;
    uint8_t destination;
    uint8_t sequence;
    uint8_t flags;
    uint8_t length;
    uint8_t payload[KNX_COMM_MAX_PAYLOAD];
} knx_comm_message_t;

typedef void (*knx_comm_handler_t)(knx_comm_channel_t channel,
                                   const knx_comm_message_t *message,
                                   void *user);

typedef struct {
    bool attached;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t tx_errors;
    uint32_t rx_errors;
    uint32_t last_rx_ms;
    knx_status_t last_status;
} knx_comm_state_t;

void knx_comm_init(uint8_t local_node_id);
knx_status_t knx_comm_attach(knx_comm_channel_t channel, knx_host_comm_t *transport);
knx_status_t knx_comm_subscribe(knx_comm_channel_t channel, uint8_t type,
                                knx_comm_handler_t handler, void *user);
knx_status_t knx_comm_send(knx_comm_channel_t channel, uint8_t type,
                           uint8_t destination, const void *payload, uint8_t length);
void knx_comm_update(uint32_t now_ms);
void knx_comm_snapshot(knx_comm_channel_t channel, knx_comm_state_t *out);

#endif
