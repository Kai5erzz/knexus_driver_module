#include "vofa_plus.h"

#include <string.h>

static const uint8_t VOFA_TAIL[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

void Vofa_Init(Vofa_Instance_t *vofa, uint8_t channel_count)
{
    if (vofa == NULL) {
        return;
    }

    vofa->channel_count = (channel_count > VOFA_MAX_CHANNELS) ? VOFA_MAX_CHANNELS : channel_count;
    memset(&vofa->packet, 0, sizeof(Vofa_Packet_t));
    vofa->tx_length = (uint16_t)(vofa->channel_count * sizeof(float) + sizeof(VOFA_TAIL));
}

void Vofa_SetData(Vofa_Instance_t *vofa, uint8_t channel_index, float data)
{
    if (vofa == NULL || channel_index >= vofa->channel_count) {
        return;
    }

    vofa->packet.channels[channel_index] = data;
}

knx_status_t Vofa_Transmit(Vofa_Instance_t *vofa, knx_uart_t *uart)
{
    if (vofa == NULL || uart == NULL) {
        return KNX_INVALID_ARG;
    }

    uint8_t *tail_ptr = (uint8_t *)(&vofa->packet.channels[vofa->channel_count]);
    tail_ptr[0] = VOFA_TAIL[0];
    tail_ptr[1] = VOFA_TAIL[1];
    tail_ptr[2] = VOFA_TAIL[2];
    tail_ptr[3] = VOFA_TAIL[3];

    return knx_uart_transmit(uart, (const uint8_t *)&vofa->packet, vofa->tx_length, 2U);
}
