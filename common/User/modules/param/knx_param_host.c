#include "knx_param_host.h"
#include "knx_health.h"
#include "knx_param.h"
#include <stddef.h>
#include <string.h>

#define PARAM_HOST_TX_TIMEOUT_MS  2U
#define PARAM_HOST_MAX_NAME_LEN   48U

static knx_host_comm_t *s_comm;

uint32_t knx_param_host_rx_count = 0U;
uint32_t knx_param_host_tx_count = 0U;
uint32_t knx_param_host_error_count = 0U;
int32_t knx_param_host_last_status = 0;
uint16_t knx_param_host_last_id = 0U;
uint8_t knx_param_host_last_msg = 0U;

static uint16_t rd_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void wr_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8);
}

static void wr_i32_le(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8) & 0xFFU);
    data[2] = (uint8_t)((raw >> 16) & 0xFFU);
    data[3] = (uint8_t)((raw >> 24) & 0xFFU);
}

static uint32_t rd_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void wr_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static knx_param_value_t rd_value_le(knx_param_type_t type, const uint8_t *data)
{
    knx_param_value_t value = {0};

    switch (type) {
    case KNX_PARAM_TYPE_F32:
        memcpy(&value.f32, data, sizeof(value.f32));
        break;

    case KNX_PARAM_TYPE_U32:
        value.u32 = rd_u32_le(data);
        break;

    case KNX_PARAM_TYPE_I32:
        value.i32 = (int32_t)rd_u32_le(data);
        break;

    case KNX_PARAM_TYPE_U8:
        value.u8 = data[0];
        break;

    default:
        break;
    }

    return value;
}

static void wr_value_le(uint8_t *data,
                        knx_param_type_t type,
                        knx_param_value_t value)
{
    switch (type) {
    case KNX_PARAM_TYPE_F32:
        memcpy(data, &value.f32, sizeof(value.f32));
        break;

    case KNX_PARAM_TYPE_U32:
        wr_u32_le(data, value.u32);
        break;

    case KNX_PARAM_TYPE_I32:
        wr_i32_le(data, value.i32);
        break;

    case KNX_PARAM_TYPE_U8:
        data[0] = value.u8;
        data[1] = 0U;
        data[2] = 0U;
        data[3] = 0U;
        break;

    default:
        data[0] = 0U;
        data[1] = 0U;
        data[2] = 0U;
        data[3] = 0U;
        break;
    }
}

static void send_payload(const uint8_t *payload, uint16_t len)
{
    knx_status_t status = KNX_NOT_READY;

    if (s_comm != NULL) {
        status = knx_host_comm_send(s_comm, payload, len, PARAM_HOST_TX_TIMEOUT_MS);
    }

    knx_param_host_last_status = status;
    if (status == KNX_OK) {
        knx_param_host_tx_count++;
    } else {
        knx_param_host_error_count++;
    }
    (void)knx_health_report(KNX_HEALTH_SOURCE_PARAM_HOST,
                            (status == KNX_OK) ? KNX_HEALTH_STATE_OK : KNX_HEALTH_STATE_WARN,
                            status,
                            knx_param_host_last_id,
                            knx_param_host_error_count);
}

static void send_ack(uint8_t request_msg, uint16_t id, knx_status_t status)
{
    uint8_t rsp[6];
    rsp[0] = KNX_PARAM_HOST_MSG_ACK_RSP;
    rsp[1] = request_msg;
    rsp[2] = (uint8_t)((int8_t)status);
    wr_u16_le(&rsp[3], id);
    rsp[5] = (uint8_t)knx_param_count();
    send_payload(rsp, sizeof(rsp));
}

static void send_value(uint8_t request_msg,
                       uint16_t id,
                       const knx_param_entry_t *entry,
                       knx_status_t status)
{
    uint8_t rsp[14] = {0};
    knx_param_value_t value = {0};

    if (entry != NULL && status == KNX_OK) {
        status = knx_param_get(id, &value);
    }

    rsp[0] = KNX_PARAM_HOST_MSG_VALUE_RSP;
    rsp[1] = request_msg;
    rsp[2] = (uint8_t)((int8_t)status);
    wr_u16_le(&rsp[3], id);
    rsp[5] = (entry != NULL) ? (uint8_t)entry->type : 0xFFU;
    rsp[6] = (entry != NULL) ? entry->flags : 0U;
    rsp[7] = 0U;
    rsp[8] = 0U;
    rsp[9] = 0U;

    if (entry != NULL && status == KNX_OK) {
        wr_value_le(&rsp[10], entry->type, value);
    }

    send_payload(rsp, sizeof(rsp));
}

static void send_info(uint16_t index)
{
    uint8_t rsp[72] = {0};
    const knx_param_entry_t *entry = knx_param_entry_at(index);
    knx_status_t status = (entry != NULL) ? KNX_OK : KNX_INVALID_ARG;
    uint8_t name_len = 0U;

    rsp[0] = KNX_PARAM_HOST_MSG_INFO_RSP;
    rsp[1] = (uint8_t)((int8_t)status);
    wr_u16_le(&rsp[2], index);
    wr_u16_le(&rsp[4], (entry != NULL) ? entry->id : 0U);
    rsp[6] = (entry != NULL) ? (uint8_t)entry->type : 0xFFU;
    rsp[7] = (entry != NULL) ? entry->flags : 0U;

    if (entry != NULL) {
        wr_value_le(&rsp[8], entry->type, entry->default_value);
        wr_value_le(&rsp[12], entry->type, entry->min_value);
        wr_value_le(&rsp[16], entry->type, entry->max_value);

        while (entry->name[name_len] != '\0' && name_len < PARAM_HOST_MAX_NAME_LEN) {
            rsp[21U + name_len] = (uint8_t)entry->name[name_len];
            name_len++;
        }
    }

    rsp[20] = name_len;
    send_payload(rsp, (uint16_t)(21U + name_len));
}

static void handle_get(const uint8_t *payload, uint16_t len)
{
    if (len != 3U) {
        send_ack(KNX_PARAM_HOST_MSG_GET_REQ, 0U, KNX_INVALID_ARG);
        return;
    }

    uint16_t id = rd_u16_le(&payload[1]);
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    knx_param_host_last_id = id;
    send_value(KNX_PARAM_HOST_MSG_GET_REQ,
               id,
               entry,
               (entry != NULL) ? KNX_OK : KNX_INVALID_ARG);
}

static void handle_set(const uint8_t *payload, uint16_t len)
{
    if (len != 8U) {
        send_ack(KNX_PARAM_HOST_MSG_SET_REQ, 0U, KNX_INVALID_ARG);
        return;
    }

    uint16_t id = rd_u16_le(&payload[1]);
    knx_param_type_t type = (knx_param_type_t)payload[3];
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    knx_param_host_last_id = id;

    if (entry == NULL || entry->type != type) {
        send_value(KNX_PARAM_HOST_MSG_SET_REQ, id, entry, KNX_INVALID_ARG);
        return;
    }

    knx_param_value_t value = rd_value_le(type, &payload[4]);
    knx_status_t status = knx_param_set(id, value);
    send_value(KNX_PARAM_HOST_MSG_SET_REQ, id, entry, status);
}

static void handle_reset(const uint8_t *payload, uint16_t len)
{
    if (len != 3U) {
        send_ack(KNX_PARAM_HOST_MSG_RESET_REQ, 0U, KNX_INVALID_ARG);
        return;
    }

    uint16_t id = rd_u16_le(&payload[1]);
    const knx_param_entry_t *entry = knx_param_find_by_id(id);
    knx_param_host_last_id = id;
    knx_status_t status = (entry != NULL) ? knx_param_reset(id) : KNX_INVALID_ARG;
    send_value(KNX_PARAM_HOST_MSG_RESET_REQ, id, entry, status);
}

static void handle_info(const uint8_t *payload, uint16_t len)
{
    if (len != 3U) {
        send_ack(KNX_PARAM_HOST_MSG_INFO_REQ, 0U, KNX_INVALID_ARG);
        return;
    }

    send_info(rd_u16_le(&payload[1]));
}

static void param_host_callback(const uint8_t *payload, uint16_t len, void *user)
{
    (void)user;

    if (payload == NULL || len == 0U) {
        return;
    }

    switch (payload[0]) {
    case KNX_PARAM_HOST_MSG_GET_REQ:
        knx_param_host_rx_count++;
        knx_param_host_last_msg = payload[0];
        handle_get(payload, len);
        break;

    case KNX_PARAM_HOST_MSG_SET_REQ:
        knx_param_host_rx_count++;
        knx_param_host_last_msg = payload[0];
        handle_set(payload, len);
        break;

    case KNX_PARAM_HOST_MSG_RESET_REQ:
        knx_param_host_rx_count++;
        knx_param_host_last_msg = payload[0];
        handle_reset(payload, len);
        break;

    case KNX_PARAM_HOST_MSG_INFO_REQ:
        knx_param_host_rx_count++;
        knx_param_host_last_msg = payload[0];
        handle_info(payload, len);
        break;

    default:
        break;
    }
}

knx_status_t knx_param_host_attach(knx_host_comm_t *comm)
{
    if (comm == NULL) {
        return KNX_INVALID_ARG;
    }

    s_comm = comm;
    knx_status_t status = knx_host_comm_add_callback(comm, param_host_callback, NULL);
    (void)knx_health_report(KNX_HEALTH_SOURCE_PARAM_HOST,
                            (status == KNX_OK) ? KNX_HEALTH_STATE_OK : KNX_HEALTH_STATE_WARN,
                            status,
                            0U,
                            knx_param_host_error_count);
    return status;
}
