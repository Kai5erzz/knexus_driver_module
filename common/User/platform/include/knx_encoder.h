#ifndef KNX_ENCODER_H
#define KNX_ENCODER_H

#include "knx_types.h"

typedef enum {
    KNX_ENCODER_TIM,
    KNX_ENCODER_LPTIM,
    KNX_ENCODER_MSPM0_QEI,
    KNX_ENCODER_MSPM0_EXTI,
} knx_encoder_type_t;

typedef struct {
    void               *timer;
    knx_encoder_type_t  type;
    uint32_t            channel;
    uint32_t            period;
} knx_encoder_port_t;

knx_status_t knx_encoder_start(const knx_encoder_port_t *port);
knx_status_t knx_encoder_read_raw(const knx_encoder_port_t *port, uint16_t *raw);

#endif /* KNX_ENCODER_H */
