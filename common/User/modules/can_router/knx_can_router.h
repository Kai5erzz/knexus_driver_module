#ifndef KNX_CAN_ROUTER_H
#define KNX_CAN_ROUTER_H

#include "knx_can.h"
#include "knx_types.h"
#include <stdint.h>

#define KNX_CAN_ROUTER_MAX_ROUTES 8U

typedef void (*knx_can_router_handler_t)(uint32_t std_id,
                                         const uint8_t *data,
                                         uint8_t len,
                                         void *user);

typedef struct {
    uint32_t rx_count;
    uint32_t matched_count;
    uint32_t unmatched_count;
    uint32_t error_count;
    uint32_t last_rx_id;
    knx_status_t last_status;
} knx_can_router_stats_t;

typedef struct {
    uint16_t first_id;
    uint16_t last_id;
    knx_can_router_handler_t handler;
    void *user;
} knx_can_router_route_t;

typedef struct {
    knx_can_t *can;
    knx_can_router_route_t routes[KNX_CAN_ROUTER_MAX_ROUTES];
    uint8_t route_count;
    knx_can_router_stats_t stats;
} knx_can_router_t;

knx_status_t knx_can_router_init(knx_can_router_t *router, knx_can_t *can);
knx_status_t knx_can_router_add_range(knx_can_router_t *router,
                                      uint16_t first_id,
                                      uint16_t last_id,
                                      knx_can_router_handler_t handler,
                                      void *user);
knx_status_t knx_can_router_add_id(knx_can_router_t *router,
                                   uint16_t std_id,
                                   knx_can_router_handler_t handler,
                                   void *user);
knx_status_t knx_can_router_start(knx_can_router_t *router);
void knx_can_router_get_stats(const knx_can_router_t *router,
                              knx_can_router_stats_t *stats);
void knx_can_router_reset_stats(knx_can_router_t *router);

#endif /* KNX_CAN_ROUTER_H */
