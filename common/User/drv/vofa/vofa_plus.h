#ifndef VOFA_PLUS_H
#define VOFA_PLUS_H

#include <stdint.h>
#include "knx_uart.h"

#define VOFA_MAX_CHANNELS 16U

typedef struct {
    float channels[VOFA_MAX_CHANNELS];
    uint8_t tail[4];
} Vofa_Packet_t;

typedef struct {
    Vofa_Packet_t packet;
    uint16_t tx_length;
    uint8_t channel_count;
} Vofa_Instance_t;

void Vofa_Init(Vofa_Instance_t *vofa, uint8_t channel_count);
void Vofa_SetData(Vofa_Instance_t *vofa, uint8_t channel_index, float data);
knx_status_t Vofa_Transmit(Vofa_Instance_t *vofa, knx_uart_t *uart);

#endif /* VOFA_PLUS_H */
