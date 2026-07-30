#include "knx_can_bus.h"
#include "knx_time.h"
#include <string.h>

typedef struct {
    knx_can_t *port;
    knx_can_router_t router;
    knx_can_bus_state_t state;
} knx_can_bus_slot_t;

static knx_can_bus_slot_t s_buses[KNX_CAN_BUS_COUNT];

static knx_can_bus_slot_t *slot(knx_can_bus_id_t bus)
{
    return (bus < KNX_CAN_BUS_COUNT) ? &s_buses[bus] : NULL;
}

void knx_can_bus_init(void)
{
    memset(s_buses, 0, sizeof(s_buses));
}

knx_status_t knx_can_bus_attach(knx_can_bus_id_t bus, knx_can_t *port)
{
    knx_can_bus_slot_t *s = slot(bus);
    if (s == NULL || port == NULL || port->handle == NULL) return KNX_INVALID_ARG;
    s->port = port;
    s->state.attached = true;
    s->state.last_status = knx_can_router_init(&s->router, port);
    return s->state.last_status;
}

knx_status_t knx_can_bus_start(knx_can_bus_id_t bus)
{
    knx_can_bus_slot_t *s = slot(bus);
    if (s == NULL || !s->state.attached) return KNX_NOT_READY;
    if (s->state.started) return KNX_OK;
    s->state.last_status = knx_can_router_start(&s->router);
    s->state.started = (s->state.last_status == KNX_OK);
    return s->state.last_status;
}

knx_status_t knx_can_bus_subscribe(knx_can_bus_id_t bus, uint16_t first_id,
                                   uint16_t last_id,
                                   knx_can_router_handler_t handler, void *user)
{
    knx_can_bus_slot_t *s = slot(bus);
    if (s == NULL || !s->state.attached) return KNX_NOT_READY;
    return knx_can_router_add_range(&s->router, first_id, last_id, handler, user);
}

static knx_status_t send_with_timeout(knx_can_bus_id_t bus, uint16_t std_id,
                                      const uint8_t *data, uint8_t len,
                                      uint32_t timeout_ms)
{
    knx_can_bus_slot_t *s = slot(bus);
    if (s == NULL || !s->state.started) return KNX_NOT_READY;
    s->state.last_status = knx_can_transmit_std(s->port, std_id, data, len,
                                                timeout_ms);
    s->state.last_tx_ms = knx_millis();
    if (s->state.last_status == KNX_OK) s->state.tx_ok++;
    else s->state.tx_error++;
    return s->state.last_status;
}

knx_status_t knx_can_bus_send(knx_can_bus_id_t bus, uint16_t std_id,
                              const uint8_t *data, uint8_t len)
{
    return send_with_timeout(bus, std_id, data, len, 1U);
}

knx_status_t knx_can_bus_try_send(knx_can_bus_id_t bus, uint16_t std_id,
                                  const uint8_t *data, uint8_t len)
{
    return send_with_timeout(bus, std_id, data, len, 0U);
}

void knx_can_bus_snapshot(knx_can_bus_id_t bus, knx_can_bus_state_t *out)
{
    knx_can_bus_slot_t *s = slot(bus);
    if (s == NULL || out == NULL) return;
    knx_can_router_get_stats(&s->router, &s->state.router);
    *out = s->state;
}
