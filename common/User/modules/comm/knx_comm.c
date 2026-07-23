#include "knx_comm.h"
#include "knx_time.h"
#include <string.h>

#define KNX_COMM_HEADER_SIZE 7U

typedef struct {
    uint8_t type;
    knx_comm_handler_t handler;
    void *user;
} knx_comm_handler_slot_t;

typedef struct {
    knx_host_comm_t *transport;
    knx_comm_state_t state;
    knx_comm_handler_slot_t handlers[KNX_COMM_MAX_HANDLERS];
    uint8_t handler_count;
    uint8_t tx_sequence;
    knx_comm_channel_t channel;
} knx_comm_endpoint_t;

static knx_comm_endpoint_t s_endpoints[KNX_COMM_CHANNEL_COUNT];
static uint8_t s_local_node;

static knx_comm_endpoint_t *endpoint(knx_comm_channel_t channel)
{
    return (channel < KNX_COMM_CHANNEL_COUNT) ? &s_endpoints[channel] : NULL;
}

static void transport_rx(const uint8_t *payload, uint16_t len, void *user)
{
    knx_comm_endpoint_t *ep = (knx_comm_endpoint_t *)user;
    if (ep == NULL || payload == NULL || len < KNX_COMM_HEADER_SIZE) return;
    if (payload[0] != KNX_COMM_VERSION || payload[6] > KNX_COMM_MAX_PAYLOAD ||
        len != (uint16_t)(KNX_COMM_HEADER_SIZE + payload[6])) {
        ep->state.rx_errors++;
        ep->state.last_status = KNX_ERROR;
        return;
    }

    knx_comm_message_t message = {0};
    message.type = payload[1];
    message.source = payload[2];
    message.destination = payload[3];
    message.sequence = payload[4];
    message.flags = payload[5];
    message.length = payload[6];
    if (message.length > 0U) memcpy(message.payload, &payload[7], message.length);

    if (message.destination != 0xFFU && message.destination != s_local_node) return;
    ep->state.rx_count++;
    ep->state.last_rx_ms = knx_millis();
    ep->state.last_status = KNX_OK;

    for (uint8_t i = 0U; i < ep->handler_count; ++i) {
        if (ep->handlers[i].type == message.type && ep->handlers[i].handler != NULL) {
            ep->handlers[i].handler(ep->channel, &message, ep->handlers[i].user);
        }
    }
}

void knx_comm_init(uint8_t local_node_id)
{
    memset(s_endpoints, 0, sizeof(s_endpoints));
    s_local_node = local_node_id;
    for (uint8_t i = 0U; i < KNX_COMM_CHANNEL_COUNT; ++i) {
        s_endpoints[i].channel = (knx_comm_channel_t)i;
    }
}

knx_status_t knx_comm_attach(knx_comm_channel_t channel, knx_host_comm_t *transport)
{
    knx_comm_endpoint_t *ep = endpoint(channel);
    if (ep == NULL || transport == NULL) return KNX_INVALID_ARG;
    ep->transport = transport;
    ep->state.attached = true;
    ep->state.last_status = knx_host_comm_add_callback(transport, transport_rx, ep);
    return ep->state.last_status;
}

knx_status_t knx_comm_subscribe(knx_comm_channel_t channel, uint8_t type,
                                knx_comm_handler_t handler, void *user)
{
    knx_comm_endpoint_t *ep = endpoint(channel);
    if (ep == NULL || handler == NULL) return KNX_INVALID_ARG;
    if (ep->handler_count >= KNX_COMM_MAX_HANDLERS) return KNX_BUSY;
    ep->handlers[ep->handler_count++] = (knx_comm_handler_slot_t){type, handler, user};
    return KNX_OK;
}

knx_status_t knx_comm_send(knx_comm_channel_t channel, uint8_t type,
                           uint8_t destination, const void *payload, uint8_t length)
{
    knx_comm_endpoint_t *ep = endpoint(channel);
    if (ep == NULL || !ep->state.attached || ep->transport == NULL) return KNX_NOT_READY;
    if (length > KNX_COMM_MAX_PAYLOAD || (length > 0U && payload == NULL)) return KNX_INVALID_ARG;

    uint8_t frame[KNX_COMM_HEADER_SIZE + KNX_COMM_MAX_PAYLOAD];
    frame[0] = KNX_COMM_VERSION;
    frame[1] = type;
    frame[2] = s_local_node;
    frame[3] = destination;
    frame[4] = ep->tx_sequence++;
    frame[5] = 0U;
    frame[6] = length;
    if (length > 0U) memcpy(&frame[7], payload, length);

    ep->state.last_status = knx_host_comm_send(ep->transport, frame,
                                                KNX_COMM_HEADER_SIZE + length, 2U);
    if (ep->state.last_status == KNX_OK) ep->state.tx_count++;
    else ep->state.tx_errors++;
    return ep->state.last_status;
}

void knx_comm_update(uint32_t now_ms)
{
    for (uint8_t i = 0U; i < KNX_COMM_CHANNEL_COUNT; ++i) {
        if (s_endpoints[i].transport != NULL) {
            knx_host_comm_check_timeout(s_endpoints[i].transport, now_ms);
        }
    }
}

void knx_comm_snapshot(knx_comm_channel_t channel, knx_comm_state_t *out)
{
    knx_comm_endpoint_t *ep = endpoint(channel);
    if (ep != NULL && out != NULL) *out = ep->state;
}
