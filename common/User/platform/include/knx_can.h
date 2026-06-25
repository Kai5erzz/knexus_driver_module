#ifndef KNX_CAN_H
#define KNX_CAN_H

#include "knx_types.h"
#include <stdint.h>

typedef struct {
    void *handle;
} knx_can_t;

typedef void (*knx_can_rx_callback_t)(uint32_t std_id,
                                      const uint8_t *data,
                                      uint8_t len,
                                      void *user);

knx_status_t knx_can_start(knx_can_t *can);
knx_status_t knx_can_transmit_std(knx_can_t *can,
                                  uint16_t std_id,
                                  const uint8_t *data,
                                  uint8_t len,
                                  uint32_t timeout_ms);
knx_status_t knx_can_set_rx_callback(knx_can_t *can,
                                     knx_can_rx_callback_t callback,
                                     void *user);

#endif /* KNX_CAN_H */
