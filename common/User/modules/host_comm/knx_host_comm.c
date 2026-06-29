#include "knx_host_comm.h"
#include "knx_health.h"
#include "knx_time.h"
#include <stddef.h>
#include <string.h>

typedef enum {
    KNX_HOST_COMM_WAIT_SOF0 = 0,
    KNX_HOST_COMM_WAIT_SOF1,
    KNX_HOST_COMM_LEN0,
    KNX_HOST_COMM_LEN1,
    KNX_HOST_COMM_PAYLOAD,
    KNX_HOST_COMM_CRC0,
    KNX_HOST_COMM_CRC1,
    KNX_HOST_COMM_EOF0_STATE,
    KNX_HOST_COMM_EOF1_STATE,
} knx_host_comm_rx_state_t;

static uint16_t knx_host_comm_crc16_ccitt_false_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8;
    for (uint8_t i = 0U; i < 8U; i++) {
        if ((crc & 0x8000U) != 0U) {
            crc = (uint16_t)((crc << 1) ^ 0x1021U);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t knx_host_comm_crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    if (data == NULL && len != 0U) {
        return crc;
    }

    for (uint16_t i = 0U; i < len; i++) {
        crc = knx_host_comm_crc16_ccitt_false_update(crc, data[i]);
    }
    return crc;
}

static uint16_t knx_host_comm_calc_crc(uint16_t len, const uint8_t *payload)
{
    uint8_t len_bytes[2];
    len_bytes[0] = (uint8_t)(len & 0xFFU);
    len_bytes[1] = (uint8_t)(len >> 8);

#if (KNX_HOST_COMM_CRC_MODE == KNX_HOST_COMM_CRC_PAYLOAD_ONLY)
    return knx_host_comm_crc16_ccitt_false(payload, len);
#else
    uint16_t crc = 0xFFFFU;
    crc = knx_host_comm_crc16_ccitt_false_update(crc, len_bytes[0]);
    crc = knx_host_comm_crc16_ccitt_false_update(crc, len_bytes[1]);
    for (uint16_t i = 0U; i < len; i++) {
        crc = knx_host_comm_crc16_ccitt_false_update(crc, payload[i]);
    }
    return crc;
#endif
}

static void knx_host_comm_reset_rx(knx_host_comm_t *comm)
{
    comm->state = (uint8_t)KNX_HOST_COMM_WAIT_SOF0;
    comm->rx_len = 0U;
    comm->rx_index = 0U;
    comm->rx_crc = 0U;
}

static uint32_t knx_host_comm_error_total(const knx_host_comm_t *comm)
{
    return comm->stats.tx_errors +
           comm->stats.crc_errors +
           comm->stats.eof_errors +
           comm->stats.overflow_errors +
           comm->stats.oversized_errors +
           comm->stats.sync_losses;
}

static void knx_host_comm_report_health(knx_host_comm_t *comm,
                                        knx_health_state_t state,
                                        knx_status_t status)
{
    (void)knx_health_report(KNX_HEALTH_SOURCE_HOST_COMM,
                            state,
                            status,
                            comm->stats.rx_frames,
                            knx_host_comm_error_total(comm));
}

static void knx_host_comm_dispatch(knx_host_comm_t *comm)
{
    if (comm->callback != NULL) {
        comm->callback(comm->rx_payload, comm->rx_len, comm->callback_user);
    }

    for (uint8_t i = 0U; i < comm->extra_callback_count; i++) {
        if (comm->extra_callbacks[i] != NULL) {
            comm->extra_callbacks[i](comm->rx_payload,
                                     comm->rx_len,
                                     comm->extra_callback_users[i]);
        }
    }
}

void knx_host_comm_init(knx_host_comm_t *comm, knx_uart_t *uart)
{
    if (comm == NULL) {
        return;
    }

    memset(comm, 0, sizeof(*comm));
    comm->uart = uart;
    knx_host_comm_reset_rx(comm);
    knx_host_comm_report_health(comm, KNX_HEALTH_STATE_OK, KNX_OK);
}

void knx_host_comm_set_callback(knx_host_comm_t *comm,
                                knx_host_comm_payload_callback_t callback,
                                void *user)
{
    if (comm == NULL) {
        return;
    }

    comm->callback = callback;
    comm->callback_user = user;
}

knx_status_t knx_host_comm_add_callback(knx_host_comm_t *comm,
                                        knx_host_comm_payload_callback_t callback,
                                        void *user)
{
    if (comm == NULL || callback == NULL) {
        return KNX_INVALID_ARG;
    }

    if (comm->callback == callback && comm->callback_user == user) {
        return KNX_OK;
    }

    for (uint8_t i = 0U; i < comm->extra_callback_count; i++) {
        if (comm->extra_callbacks[i] == callback &&
            comm->extra_callback_users[i] == user) {
            return KNX_OK;
        }
    }

    if (comm->extra_callback_count >= KNX_HOST_COMM_MAX_EXTRA_CALLBACKS) {
        return KNX_BUSY;
    }

    uint8_t index = comm->extra_callback_count;
    comm->extra_callbacks[index] = callback;
    comm->extra_callback_users[index] = user;
    comm->extra_callback_count++;
    return KNX_OK;
}

knx_status_t knx_host_comm_build_frame(const uint8_t *payload,
                                       uint16_t len,
                                       uint8_t *out,
                                       uint16_t out_size,
                                       uint16_t *frame_len)
{
    if ((payload == NULL && len != 0U) || out == NULL || frame_len == NULL) {
        return KNX_INVALID_ARG;
    }
    if (len > KNX_HOST_COMM_MAX_PAYLOAD) {
        return KNX_INVALID_ARG;
    }

    uint16_t total = (uint16_t)(KNX_HOST_COMM_FRAME_OVERHEAD + len);
    if (out_size < total) {
        return KNX_INVALID_ARG;
    }

    uint16_t crc = knx_host_comm_calc_crc(len, payload);

    out[0] = KNX_HOST_COMM_SOF0;
    out[1] = KNX_HOST_COMM_SOF1;
    out[2] = (uint8_t)(len & 0xFFU);
    out[3] = (uint8_t)(len >> 8);
    if (len != 0U) {
        memcpy(&out[4], payload, len);
    }
    out[4U + len] = (uint8_t)(crc & 0xFFU);
    out[5U + len] = (uint8_t)(crc >> 8);
    out[6U + len] = KNX_HOST_COMM_EOF0;
    out[7U + len] = KNX_HOST_COMM_EOF1;
    *frame_len = total;
    return KNX_OK;
}

knx_status_t knx_host_comm_send(knx_host_comm_t *comm,
                                const uint8_t *payload,
                                uint16_t len,
                                uint32_t timeout_ms)
{
    if (comm == NULL || comm->uart == NULL) {
        return KNX_INVALID_ARG;
    }

    uint16_t frame_len = 0U;
    knx_status_t status = knx_host_comm_build_frame(payload,
                                                    len,
                                                    comm->tx_frame,
                                                    sizeof(comm->tx_frame),
                                                    &frame_len);
    if (status != KNX_OK) {
        comm->stats.tx_errors++;
        comm->stats.last_status = status;
        return status;
    }

    status = knx_uart_transmit(comm->uart, comm->tx_frame, frame_len, timeout_ms);
    comm->stats.last_status = status;
    if (status == KNX_OK) {
        comm->stats.tx_frames++;
    } else {
        comm->stats.tx_errors++;
    }
    knx_host_comm_report_health(comm,
                                (status == KNX_OK) ? KNX_HEALTH_STATE_OK : KNX_HEALTH_STATE_WARN,
                                status);
    return status;
}

knx_status_t knx_host_comm_feed_byte(knx_host_comm_t *comm, uint8_t byte)
{
    if (comm == NULL) {
        return KNX_INVALID_ARG;
    }

    comm->stats.rx_bytes++;
    comm->last_byte_ms = knx_millis();

    switch ((knx_host_comm_rx_state_t)comm->state) {
    case KNX_HOST_COMM_WAIT_SOF0:
        if (byte == KNX_HOST_COMM_SOF0) {
            comm->state = (uint8_t)KNX_HOST_COMM_WAIT_SOF1;
        }
        break;

    case KNX_HOST_COMM_WAIT_SOF1:
        if (byte == KNX_HOST_COMM_SOF1) {
            comm->state = (uint8_t)KNX_HOST_COMM_LEN0;
        } else {
            comm->stats.sync_losses++;
            knx_host_comm_report_health(comm, KNX_HEALTH_STATE_WARN, KNX_ERROR);
            comm->state = (byte == KNX_HOST_COMM_SOF0)
                        ? (uint8_t)KNX_HOST_COMM_WAIT_SOF1
                        : (uint8_t)KNX_HOST_COMM_WAIT_SOF0;
        }
        break;

    case KNX_HOST_COMM_LEN0:
        comm->rx_len = byte;
        comm->state = (uint8_t)KNX_HOST_COMM_LEN1;
        break;

    case KNX_HOST_COMM_LEN1:
        comm->rx_len |= (uint16_t)byte << 8;
        comm->rx_index = 0U;
        if (comm->rx_len > KNX_HOST_COMM_MAX_PAYLOAD) {
            comm->stats.oversized_errors++;
            knx_host_comm_report_health(comm, KNX_HEALTH_STATE_WARN, KNX_INVALID_ARG);
            knx_host_comm_reset_rx(comm);
        } else if (comm->rx_len == 0U) {
            comm->state = (uint8_t)KNX_HOST_COMM_CRC0;
        } else {
            comm->state = (uint8_t)KNX_HOST_COMM_PAYLOAD;
        }
        break;

    case KNX_HOST_COMM_PAYLOAD:
        if (comm->rx_index < KNX_HOST_COMM_MAX_PAYLOAD) {
            comm->rx_payload[comm->rx_index++] = byte;
            if (comm->rx_index >= comm->rx_len) {
                comm->state = (uint8_t)KNX_HOST_COMM_CRC0;
            }
        } else {
            comm->stats.overflow_errors++;
            knx_host_comm_report_health(comm, KNX_HEALTH_STATE_WARN, KNX_BUSY);
            knx_host_comm_reset_rx(comm);
        }
        break;

    case KNX_HOST_COMM_CRC0:
        comm->rx_crc = byte;
        comm->state = (uint8_t)KNX_HOST_COMM_CRC1;
        break;

    case KNX_HOST_COMM_CRC1:
        comm->rx_crc |= (uint16_t)byte << 8;
        comm->state = (uint8_t)KNX_HOST_COMM_EOF0_STATE;
        break;

    case KNX_HOST_COMM_EOF0_STATE:
        if (byte == KNX_HOST_COMM_EOF0) {
            comm->state = (uint8_t)KNX_HOST_COMM_EOF1_STATE;
        } else {
            comm->stats.eof_errors++;
            knx_host_comm_report_health(comm, KNX_HEALTH_STATE_WARN, KNX_ERROR);
            knx_host_comm_reset_rx(comm);
        }
        break;

    case KNX_HOST_COMM_EOF1_STATE:
        if (byte == KNX_HOST_COMM_EOF1) {
            uint16_t expected = knx_host_comm_calc_crc(comm->rx_len, comm->rx_payload);
            if (expected == comm->rx_crc) {
                comm->stats.rx_frames++;
                comm->stats.last_status = KNX_OK;
                comm->last_rx_frame_ms = knx_millis();
                knx_host_comm_report_health(comm, KNX_HEALTH_STATE_OK, KNX_OK);
                knx_host_comm_dispatch(comm);
            } else {
                comm->stats.crc_errors++;
                comm->stats.last_status = KNX_ERROR;
                knx_host_comm_report_health(comm, KNX_HEALTH_STATE_WARN, KNX_ERROR);
            }
        } else {
            comm->stats.eof_errors++;
            comm->stats.last_status = KNX_ERROR;
            knx_host_comm_report_health(comm, KNX_HEALTH_STATE_WARN, KNX_ERROR);
        }
        knx_host_comm_reset_rx(comm);
        break;

    default:
        knx_host_comm_reset_rx(comm);
        break;
    }

    return KNX_OK;
}

knx_status_t knx_host_comm_feed(knx_host_comm_t *comm,
                                const uint8_t *data,
                                uint16_t len)
{
    if (comm == NULL || (data == NULL && len != 0U)) {
        return KNX_INVALID_ARG;
    }

    for (uint16_t i = 0U; i < len; i++) {
        (void)knx_host_comm_feed_byte(comm, data[i]);
    }
    return KNX_OK;
}

knx_status_t knx_host_comm_poll(knx_host_comm_t *comm,
                                uint16_t max_bytes,
                                uint32_t byte_timeout_ms)
{
    if (comm == NULL || comm->uart == NULL) {
        return KNX_INVALID_ARG;
    }

    uint8_t byte = 0U;
    for (uint16_t i = 0U; i < max_bytes; i++) {
        knx_status_t status = knx_uart_receive(comm->uart, &byte, 1U, byte_timeout_ms);
        if (status != KNX_OK) {
            comm->stats.last_status = status;
            return status;
        }
        (void)knx_host_comm_feed_byte(comm, byte);
    }

    comm->stats.last_status = KNX_OK;
    return KNX_OK;
}

void knx_host_comm_get_stats(knx_host_comm_t *comm, knx_host_comm_stats_t *stats)
{
    if (comm == NULL || stats == NULL) {
        return;
    }

    *stats = comm->stats;
}

void knx_host_comm_reset_stats(knx_host_comm_t *comm)
{
    if (comm == NULL) {
        return;
    }

    memset(&comm->stats, 0, sizeof(comm->stats));
}

uint16_t knx_host_comm_max_payload(void)
{
    return KNX_HOST_COMM_MAX_PAYLOAD;
}

void knx_host_comm_check_timeout(knx_host_comm_t *comm, uint32_t now_ms)
{
    if (comm == NULL) {
        return;
    }

    if (comm->state != (uint8_t)KNX_HOST_COMM_WAIT_SOF0 &&
        (now_ms - comm->last_byte_ms) > KNX_HOST_COMM_BYTE_TIMEOUT_MS) {
        knx_host_comm_reset_rx(comm);
        comm->stats.sync_losses++;
    }
}

uint32_t knx_host_comm_age_ms(knx_host_comm_t *comm)
{
    if (comm == NULL || comm->last_rx_frame_ms == 0U) {
        return 0xFFFFFFFFU;
    }
    return knx_millis() - comm->last_rx_frame_ms;
}
