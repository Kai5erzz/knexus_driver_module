#include "knx_pendulum_comm.h"
#include "knexus_config.h"
#include "knx_time.h"
#include <math.h>
#include <string.h>

#define PENDULUM_SOF0 0xAAU
#define PENDULUM_SOF1 0x55U
#define PENDULUM_STATUS_SOF0 0x55U
#define PENDULUM_STATUS_SOF1 0xAAU

static uint8_t s_frame[KNX_PENDULUM_FRAME_SIZE];
static uint8_t s_index;
static volatile uint32_t s_sequence;
static knx_pendulum_comm_state_t s_state;
static knx_uart_t *s_uart;

static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U
                ? (uint16_t)((crc >> 1U) ^ 0xA001U)
                : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static void state_write_begin(void)
{
    ++s_sequence;
    __sync_synchronize();
}

static void state_write_end(void)
{
    __sync_synchronize();
    ++s_sequence;
}

static void resync_after(uint8_t byte)
{
    s_index = 0U;
    if (byte == PENDULUM_SOF0) {
        s_frame[0] = byte;
        s_index = 1U;
    }
}

static void accept_frame(void)
{
    uint16_t received_crc = (uint16_t)s_frame[6] |
                            ((uint16_t)s_frame[7] << 8U);
    uint16_t expected_crc = crc16_modbus(s_frame, 6U);
    if (received_crc != expected_crc) {
        state_write_begin();
        ++s_state.crc_error_count;
        state_write_end();
        resync_after(s_frame[7]);
        return;
    }

    float offset;
    memcpy(&offset, &s_frame[2], sizeof(offset));
    if (!isfinite(offset) ||
        fabsf(offset) > KNEXUS_OFFSET_INPUT_ABS_MAX) {
        state_write_begin();
        ++s_state.invalid_value_count;
        state_write_end();
        resync_after(s_frame[7]);
        return;
    }

    state_write_begin();
    s_state.offset = offset;
    s_state.last_rx_ms = knx_millis();
    ++s_state.rx_count;
    state_write_end();
    resync_after(s_frame[7]);
}

void knx_pendulum_comm_init(void)
{
    s_index = 0U;
    s_sequence = 0U;
    memset(s_frame, 0, sizeof(s_frame));
    memset(&s_state, 0, sizeof(s_state));
    s_state.last_tx_status = KNX_NOT_READY;
}

void knx_pendulum_comm_attach_uart(knx_uart_t *uart)
{
    s_uart = uart;
}

void knx_pendulum_comm_feed(const uint8_t *data, uint16_t len)
{
    if (data == NULL) return;

    for (uint16_t i = 0U; i < len; ++i) {
        uint8_t byte = data[i];
        if (s_index == 0U) {
            if (byte == PENDULUM_SOF0) {
                s_frame[s_index++] = byte;
            }
            continue;
        }
        if (s_index == 1U && byte != PENDULUM_SOF1) {
            state_write_begin();
            ++s_state.sync_loss_count;
            state_write_end();
            resync_after(byte);
            continue;
        }

        s_frame[s_index++] = byte;
        if (s_index >= KNX_PENDULUM_FRAME_SIZE) accept_frame();
    }
}

knx_status_t knx_pendulum_comm_send_mode(knx_pendulum_mode_t mode)
{
    if (s_uart == NULL) return KNX_NOT_READY;
    if (mode < KNX_PENDULUM_MODE_1 || mode > KNX_PENDULUM_MODE_6) {
        return KNX_INVALID_ARG;
    }
    const uint8_t frame[KNX_PENDULUM_STATUS_FRAME_SIZE] = {
        PENDULUM_STATUS_SOF0, PENDULUM_STATUS_SOF1, (uint8_t)mode
    };
    knx_status_t status = knx_uart_transmit(
        s_uart, frame, sizeof(frame), 10U);
    state_write_begin();
    s_state.last_mode = (uint8_t)mode;
    s_state.last_tx_status = status;
    if (status == KNX_OK) ++s_state.tx_count;
    state_write_end();
    return status;
}

void knx_pendulum_comm_snapshot(knx_pendulum_comm_state_t *out)
{
    if (out == NULL) return;
    for (;;) {
        uint32_t before = s_sequence;
        if ((before & 1U) != 0U) continue;
        __sync_synchronize();
        *out = s_state;
        __sync_synchronize();
        uint32_t after = s_sequence;
        if (before == after && (after & 1U) == 0U) return;
    }
}
