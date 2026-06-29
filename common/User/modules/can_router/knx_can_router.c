#include "knx_can_router.h"
#include "knx_blackbox.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

static void knx_can_router_rx_dispatch(uint32_t std_id,
                                       const uint8_t *data,
                                       uint8_t len,
                                       void *user)
{
    knx_can_router_t *router = (knx_can_router_t *)user;
    if (router == NULL || data == NULL) {
        return;
    }

    router->stats.rx_count++;
    router->stats.last_rx_id = std_id;

    for (uint8_t i = 0U; i < router->route_count; i++) {
        const knx_can_router_route_t *route = &router->routes[i];
        if (std_id >= route->first_id && std_id <= route->last_id) {
            if (route->handler != NULL) {
                route->handler(std_id, data, len, route->user);
                router->stats.matched_count++;
                router->stats.last_status = KNX_OK;
                return;
            }
        }
    }

    router->stats.unmatched_count++;
    router->stats.last_status = KNX_ERROR;
    knx_blackbox_log(KNX_BLACKBOX_CODE_CAN_UNMATCHED,
                     0U,
                     KNX_ERROR,
                     std_id,
                     router->route_count);
}

knx_status_t knx_can_router_init(knx_can_router_t *router, knx_can_t *can)
{
    if (router == NULL || can == NULL) {
        return KNX_INVALID_ARG;
    }

    memset(router, 0, sizeof(*router));
    router->can = can;
    router->stats.last_status = KNX_OK;
    return KNX_OK;
}

knx_status_t knx_can_router_add_range(knx_can_router_t *router,
                                      uint16_t first_id,
                                      uint16_t last_id,
                                      knx_can_router_handler_t handler,
                                      void *user)
{
    if (router == NULL || handler == NULL || first_id > last_id || last_id > 0x7FFU) {
        return KNX_INVALID_ARG;
    }
    if (router->route_count >= KNX_CAN_ROUTER_MAX_ROUTES) {
        router->stats.last_status = KNX_BUSY;
        return KNX_BUSY;
    }

    /* Check for ID range overlap with already-registered routes */
    for (uint8_t i = 0U; i < router->route_count; i++) {
        const knx_can_router_route_t *existing = &router->routes[i];
        if (first_id <= existing->last_id && last_id >= existing->first_id) {
            router->stats.last_status = KNX_BUSY;
            return KNX_BUSY;
        }
    }

    knx_can_router_route_t *route = &router->routes[router->route_count];
    route->first_id = first_id;
    route->last_id = last_id;
    route->handler = handler;
    route->user = user;
    router->route_count++;
    router->stats.last_status = KNX_OK;
    return KNX_OK;
}

knx_status_t knx_can_router_add_id(knx_can_router_t *router,
                                   uint16_t std_id,
                                   knx_can_router_handler_t handler,
                                   void *user)
{
    return knx_can_router_add_range(router, std_id, std_id, handler, user);
}

knx_status_t knx_can_router_start(knx_can_router_t *router)
{
    if (router == NULL || router->can == NULL) {
        return KNX_INVALID_ARG;
    }

    knx_status_t status = knx_can_set_rx_callback(router->can,
                                                  knx_can_router_rx_dispatch,
                                                  router);
    if (status == KNX_OK) {
        status = knx_can_start(router->can);
    }

    router->stats.last_status = status;
    if (status != KNX_OK) {
        router->stats.error_count++;
    }
    return status;
}

void knx_can_router_get_stats(const knx_can_router_t *router,
                              knx_can_router_stats_t *stats)
{
    if (router == NULL || stats == NULL) {
        return;
    }

    /* Atomic structure copy — stats are modified by ISR in rx_dispatch */
    taskENTER_CRITICAL();
    *stats = router->stats;
    taskEXIT_CRITICAL();
}

void knx_can_router_reset_stats(knx_can_router_t *router)
{
    if (router == NULL) {
        return;
    }

    memset(&router->stats, 0, sizeof(router->stats));
}
