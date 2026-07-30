#ifndef KNX_CAN_BUS_H
#define KNX_CAN_BUS_H

#include "knx_can_router.h"

typedef enum {
    KNX_CAN_BUS_1 = 0,
    KNX_CAN_BUS_2 = 1,
    KNX_CAN_BUS_COUNT = 2,
} knx_can_bus_id_t;

typedef struct {
    bool attached;
    bool started;
    uint32_t tx_ok;
    uint32_t tx_error;
    uint32_t last_tx_ms;
    knx_status_t last_status;
    knx_can_router_stats_t router;
} knx_can_bus_state_t;

void knx_can_bus_init(void);
knx_status_t knx_can_bus_attach(knx_can_bus_id_t bus, knx_can_t *port);
knx_status_t knx_can_bus_start(knx_can_bus_id_t bus);
knx_status_t knx_can_bus_subscribe(knx_can_bus_id_t bus, uint16_t first_id,
                                   uint16_t last_id,
                                   knx_can_router_handler_t handler, void *user);
knx_status_t knx_can_bus_send(knx_can_bus_id_t bus, uint16_t std_id,
                              const uint8_t *data, uint8_t len);
/*
 * Try to enqueue one frame without waiting for TX FIFO/mailbox space.
 * Intended for hard-periodic/high-priority control tasks: a busy CAN bus must
 * drop/skip this control frame instead of blocking the scheduler.
 */
knx_status_t knx_can_bus_try_send(knx_can_bus_id_t bus, uint16_t std_id,
                                  const uint8_t *data, uint8_t len);
void knx_can_bus_snapshot(knx_can_bus_id_t bus, knx_can_bus_state_t *out);

#endif
