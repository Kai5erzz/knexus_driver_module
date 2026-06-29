#include "octolinker.h"

#include <stdio.h>
#include <string.h>

static uint8_t Octolinker_CalcChecksum8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0U;
    for (uint16_t i = 0U; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

static knx_status_t Octolinker_SendBytes(Octolinker_Instance_t *ol,
                                         const uint8_t *data,
                                         uint16_t len)
{
    if (ol == NULL || ol->uart == NULL || data == NULL || len == 0U) {
        return KNX_INVALID_ARG;
    }

    return knx_uart_transmit(ol->uart, data, len, OCTOLINKER_TX_TIMEOUT_MS);
}

void Octolinker_Init(Octolinker_Instance_t *ol, knx_uart_t *uart)
{
    if (ol == NULL) {
        return;
    }

    memset(ol, 0, sizeof(*ol));
    ol->uart = uart;
}

knx_status_t Octolinker_SendFrame(Octolinker_Instance_t *ol,
                                  uint16_t var_id,
                                  uint8_t value_type,
                                  const uint8_t *shape,
                                  uint8_t shape_len,
                                  const uint8_t *payload,
                                  uint16_t payload_len)
{
    if (ol == NULL || ol->uart == NULL || payload == NULL) {
        return KNX_INVALID_ARG;
    }
    if (shape_len > OCTOLINKER_MAX_SHAPE_DIMS) {
        return KNX_INVALID_ARG;
    }
    if (shape_len > 0U && shape == NULL) {
        return KNX_INVALID_ARG;
    }
    if (payload_len > OCTOLINKER_MAX_PAYLOAD) {
        return KNX_INVALID_ARG;
    }

    uint16_t pos = 0U;
    ol->tx_buf[pos++] = OCTOLINKER_SOF0;
    ol->tx_buf[pos++] = OCTOLINKER_SOF1;
    ol->tx_buf[pos++] = OCTOLINKER_VERSION;
    ol->tx_buf[pos++] = (uint8_t)(var_id & 0xFFU);
    ol->tx_buf[pos++] = (uint8_t)((var_id >> 8) & 0xFFU);
    ol->tx_buf[pos++] = value_type;
    ol->tx_buf[pos++] = shape_len;

    for (uint8_t i = 0U; i < shape_len; i++) {
        ol->tx_buf[pos++] = shape[i];
    }

    ol->tx_buf[pos++] = (uint8_t)(payload_len & 0xFFU);
    ol->tx_buf[pos++] = (uint8_t)((payload_len >> 8) & 0xFFU);
    memcpy(&ol->tx_buf[pos], payload, payload_len);
    pos = (uint16_t)(pos + payload_len);

    ol->tx_buf[pos++] = Octolinker_CalcChecksum8(&ol->tx_buf[2], (uint16_t)(pos - 2U));
    ol->tx_len = pos;

    return Octolinker_SendBytes(ol, ol->tx_buf, ol->tx_len);
}

knx_status_t Octolinker_SendF32(Octolinker_Instance_t *ol, uint16_t var_id, float value)
{
    uint8_t payload[sizeof(float)];
    memcpy(payload, &value, sizeof(value));
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_F32, NULL, 0U, payload, sizeof(payload));
}

knx_status_t Octolinker_SendI32(Octolinker_Instance_t *ol, uint16_t var_id, int32_t value)
{
    uint8_t payload[sizeof(int32_t)];
    memcpy(payload, &value, sizeof(value));
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_I32, NULL, 0U, payload, sizeof(payload));
}

knx_status_t Octolinker_SendU32(Octolinker_Instance_t *ol, uint16_t var_id, uint32_t value)
{
    uint8_t payload[sizeof(uint32_t)];
    memcpy(payload, &value, sizeof(value));
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_U32, NULL, 0U, payload, sizeof(payload));
}

knx_status_t Octolinker_SendI16(Octolinker_Instance_t *ol, uint16_t var_id, int16_t value)
{
    uint8_t payload[sizeof(int16_t)];
    memcpy(payload, &value, sizeof(value));
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_I16, NULL, 0U, payload, sizeof(payload));
}

knx_status_t Octolinker_SendU16(Octolinker_Instance_t *ol, uint16_t var_id, uint16_t value)
{
    uint8_t payload[sizeof(uint16_t)];
    memcpy(payload, &value, sizeof(value));
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_U16, NULL, 0U, payload, sizeof(payload));
}

knx_status_t Octolinker_SendI8(Octolinker_Instance_t *ol, uint16_t var_id, int8_t value)
{
    uint8_t payload = (uint8_t)value;
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_I8, NULL, 0U, &payload, sizeof(payload));
}

knx_status_t Octolinker_SendU8(Octolinker_Instance_t *ol, uint16_t var_id, uint8_t value)
{
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_U8, NULL, 0U, &value, sizeof(value));
}

knx_status_t Octolinker_SendF32Array(Octolinker_Instance_t *ol,
                                     uint16_t var_id,
                                     const float *values,
                                     uint8_t count)
{
    if (values == NULL || count == 0U) {
        return KNX_INVALID_ARG;
    }
    uint16_t payload_len = (uint16_t)count * (uint16_t)sizeof(float);
    if (payload_len > OCTOLINKER_MAX_PAYLOAD) {
        return KNX_INVALID_ARG;
    }
    uint8_t shape[1] = {count};
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_F32_ARRAY, shape, 1U,
                                (const uint8_t *)values, payload_len);
}

knx_status_t Octolinker_SendU16Array(Octolinker_Instance_t *ol,
                                     uint16_t var_id,
                                     const uint16_t *values,
                                     uint8_t count)
{
    if (values == NULL || count == 0U) {
        return KNX_INVALID_ARG;
    }
    uint16_t payload_len = (uint16_t)count * (uint16_t)sizeof(uint16_t);
    if (payload_len > OCTOLINKER_MAX_PAYLOAD) {
        return KNX_INVALID_ARG;
    }
    uint8_t shape[1] = {count};
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_U16_ARRAY, shape, 1U,
                                (const uint8_t *)values, payload_len);
}

knx_status_t Octolinker_SendU8Matrix(Octolinker_Instance_t *ol,
                                     uint16_t var_id,
                                     const uint8_t *values,
                                     uint8_t rows,
                                     uint8_t cols)
{
    if (values == NULL || rows == 0U || cols == 0U) {
        return KNX_INVALID_ARG;
    }
    uint16_t payload_len = (uint16_t)rows * (uint16_t)cols;
    if (payload_len > OCTOLINKER_MAX_PAYLOAD) {
        return KNX_INVALID_ARG;
    }
    uint8_t shape[2] = {rows, cols};
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_U8_MATRIX, shape, 2U,
                                values, payload_len);
}

knx_status_t Octolinker_SendRaw(Octolinker_Instance_t *ol,
                                uint16_t var_id,
                                const uint8_t *data,
                                uint16_t len)
{
    if (data == NULL || len == 0U || len > OCTOLINKER_MAX_PAYLOAD) {
        return KNX_INVALID_ARG;
    }
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_RAW, NULL, 0U, data, len);
}

knx_status_t Octolinker_SendBitMatrix(Octolinker_Instance_t *ol,
                                      uint16_t var_id,
                                      const uint8_t *data,
                                      uint8_t rows,
                                      uint8_t cols)
{
    if (data == NULL || rows == 0U || cols == 0U) {
        return KNX_INVALID_ARG;
    }
    uint16_t bit_count = (uint16_t)rows * (uint16_t)cols;
    uint16_t bytes = (uint16_t)((bit_count + 7U) / 8U);
    if (bytes > OCTOLINKER_MAX_PAYLOAD) {
        return KNX_INVALID_ARG;
    }
    uint8_t shape[2] = {rows, cols};
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_BIT_MATRIX, shape, 2U, data, bytes);
}

knx_status_t Octolinker_SendLiteFrame(Octolinker_Instance_t *ol,
                                      uint8_t var_id,
                                      uint8_t type_shape,
                                      uint8_t seq,
                                      const uint8_t *payload,
                                      uint8_t payload_len)
{
    if (ol == NULL || ol->uart == NULL || payload == NULL || payload_len == 0U) {
        return KNX_INVALID_ARG;
    }
    if (payload_len > OCTOLINKER_LITE_MAX_PAYLOAD) {
        return KNX_INVALID_ARG;
    }

    uint16_t pos = 0U;
    ol->tx_buf[pos++] = OCTOLINKER_LITE_SOF;
    ol->tx_buf[pos++] = var_id;
    ol->tx_buf[pos++] = type_shape;
    ol->tx_buf[pos++] = seq;
    ol->tx_buf[pos++] = payload_len;
    memcpy(&ol->tx_buf[pos], payload, payload_len);
    pos = (uint16_t)(pos + payload_len);
    ol->tx_buf[pos++] = Octolinker_CalcChecksum8(&ol->tx_buf[1], (uint16_t)(pos - 1U));
    ol->tx_len = pos;

    return Octolinker_SendBytes(ol, ol->tx_buf, ol->tx_len);
}

static knx_status_t Octolinker_SendLiteChunked(Octolinker_Instance_t *ol,
                                               uint8_t var_id,
                                               uint8_t type_shape,
                                               const uint8_t *payload,
                                               uint16_t payload_len)
{
    if (payload == NULL || payload_len == 0U) {
        return KNX_INVALID_ARG;
    }

    uint8_t chunk_count = (uint8_t)((payload_len + OCTOLINKER_LITE_MAX_PAYLOAD - 1U) /
                                    OCTOLINKER_LITE_MAX_PAYLOAD);
    if (chunk_count == 0U || chunk_count > OCTOLINKER_LITE_CHUNK_MAX) {
        return KNX_INVALID_ARG;
    }

    knx_status_t final_status = KNX_OK;
    uint16_t offset = 0U;
    for (uint8_t chunk = 0U; chunk < chunk_count; chunk++) {
        uint16_t remain = (uint16_t)(payload_len - offset);
        uint8_t n = (uint8_t)((remain > OCTOLINKER_LITE_MAX_PAYLOAD) ?
                              OCTOLINKER_LITE_MAX_PAYLOAD : remain);
        uint8_t seq = (uint8_t)(((chunk & 0x0FU) << 4) | ((chunk_count - 1U) & 0x0FU));
        knx_status_t st = Octolinker_SendLiteFrame(ol, var_id, type_shape, seq,
                                                   &payload[offset], n);
        if (st != KNX_OK && final_status == KNX_OK) {
            final_status = st;
        }
        offset = (uint16_t)(offset + n);
    }
    return final_status;
}

static knx_status_t Octolinker_SendLiteMatrixChunked(Octolinker_Instance_t *ol,
                                                     uint8_t var_id,
                                                     uint8_t type_shape,
                                                     const uint8_t *payload,
                                                     uint16_t payload_len,
                                                     uint8_t rows,
                                                     uint8_t cols)
{
    if (payload == NULL || payload_len == 0U || rows == 0U || cols == 0U) {
        return KNX_INVALID_ARG;
    }

    uint16_t first_data = payload_len;
    if (first_data > (OCTOLINKER_LITE_MAX_PAYLOAD - 2U)) {
        first_data = (uint16_t)(OCTOLINKER_LITE_MAX_PAYLOAD - 2U);
    }

    uint16_t remain_after_first = (uint16_t)(payload_len - first_data);
    uint8_t chunk_count = (uint8_t)(1U + ((remain_after_first + OCTOLINKER_LITE_MAX_PAYLOAD - 1U) /
                                          OCTOLINKER_LITE_MAX_PAYLOAD));
    if (chunk_count == 0U || chunk_count > OCTOLINKER_LITE_CHUNK_MAX) {
        return KNX_INVALID_ARG;
    }

    uint8_t first_payload[OCTOLINKER_LITE_MAX_PAYLOAD];
    first_payload[0] = rows;
    first_payload[1] = cols;
    memcpy(&first_payload[2], payload, first_data);

    uint8_t seq0 = (uint8_t)((chunk_count - 1U) & 0x0FU);
    knx_status_t final_status = Octolinker_SendLiteFrame(ol, var_id, type_shape, seq0,
                                                         first_payload,
                                                         (uint8_t)(first_data + 2U));

    uint16_t offset = first_data;
    for (uint8_t chunk = 1U; chunk < chunk_count; chunk++) {
        uint16_t remain = (uint16_t)(payload_len - offset);
        uint8_t n = (uint8_t)((remain > OCTOLINKER_LITE_MAX_PAYLOAD) ?
                              OCTOLINKER_LITE_MAX_PAYLOAD : remain);
        uint8_t seq = (uint8_t)(((chunk & 0x0FU) << 4) | ((chunk_count - 1U) & 0x0FU));
        knx_status_t st = Octolinker_SendLiteFrame(ol, var_id, type_shape, seq,
                                                   &payload[offset], n);
        if (st != KNX_OK && final_status == KNX_OK) {
            final_status = st;
        }
        offset = (uint16_t)(offset + n);
    }
    return final_status;
}

knx_status_t Octolinker_SendLiteF32(Octolinker_Instance_t *ol, uint8_t var_id, float value)
{
    return Octolinker_SendLiteFrame(ol, var_id,
                                    OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_F32,
                                                               OCTOLINKER_LITE_SHAPE_SCALAR),
                                    0U, (const uint8_t *)&value, sizeof(value));
}

knx_status_t Octolinker_SendLiteI32(Octolinker_Instance_t *ol, uint8_t var_id, int32_t value)
{
    return Octolinker_SendLiteFrame(ol, var_id,
                                    OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_I32,
                                                               OCTOLINKER_LITE_SHAPE_SCALAR),
                                    0U, (const uint8_t *)&value, sizeof(value));
}

knx_status_t Octolinker_SendLiteU32(Octolinker_Instance_t *ol, uint8_t var_id, uint32_t value)
{
    return Octolinker_SendLiteFrame(ol, var_id,
                                    OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_U32,
                                                               OCTOLINKER_LITE_SHAPE_SCALAR),
                                    0U, (const uint8_t *)&value, sizeof(value));
}

knx_status_t Octolinker_SendLiteI16(Octolinker_Instance_t *ol, uint8_t var_id, int16_t value)
{
    return Octolinker_SendLiteFrame(ol, var_id,
                                    OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_I16,
                                                               OCTOLINKER_LITE_SHAPE_SCALAR),
                                    0U, (const uint8_t *)&value, sizeof(value));
}

knx_status_t Octolinker_SendLiteU16(Octolinker_Instance_t *ol, uint8_t var_id, uint16_t value)
{
    return Octolinker_SendLiteFrame(ol, var_id,
                                    OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_U16,
                                                               OCTOLINKER_LITE_SHAPE_SCALAR),
                                    0U, (const uint8_t *)&value, sizeof(value));
}

knx_status_t Octolinker_SendLiteI8(Octolinker_Instance_t *ol, uint8_t var_id, int8_t value)
{
    return Octolinker_SendLiteFrame(ol, var_id,
                                    OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_I8,
                                                               OCTOLINKER_LITE_SHAPE_SCALAR),
                                    0U, (const uint8_t *)&value, sizeof(value));
}

knx_status_t Octolinker_SendLiteU8(Octolinker_Instance_t *ol, uint8_t var_id, uint8_t value)
{
    return Octolinker_SendLiteFrame(ol, var_id,
                                    OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_U8,
                                                               OCTOLINKER_LITE_SHAPE_SCALAR),
                                    0U, &value, sizeof(value));
}

knx_status_t Octolinker_SendLiteF32Array(Octolinker_Instance_t *ol, uint8_t var_id, const float *values, uint16_t count)
{
    if (values == NULL || count == 0U || count > (UINT16_MAX / sizeof(float))) {
        return KNX_INVALID_ARG;
    }
    return Octolinker_SendLiteChunked(ol, var_id,
                                      OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_F32,
                                                                 OCTOLINKER_LITE_SHAPE_ARRAY),
                                      (const uint8_t *)values,
                                      (uint16_t)(count * sizeof(float)));
}

knx_status_t Octolinker_SendLiteI32Array(Octolinker_Instance_t *ol, uint8_t var_id, const int32_t *values, uint16_t count)
{
    if (values == NULL || count == 0U || count > (UINT16_MAX / sizeof(int32_t))) {
        return KNX_INVALID_ARG;
    }
    return Octolinker_SendLiteChunked(ol, var_id,
                                      OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_I32,
                                                                 OCTOLINKER_LITE_SHAPE_ARRAY),
                                      (const uint8_t *)values,
                                      (uint16_t)(count * sizeof(int32_t)));
}

knx_status_t Octolinker_SendLiteU32Array(Octolinker_Instance_t *ol, uint8_t var_id, const uint32_t *values, uint16_t count)
{
    if (values == NULL || count == 0U || count > (UINT16_MAX / sizeof(uint32_t))) {
        return KNX_INVALID_ARG;
    }
    return Octolinker_SendLiteChunked(ol, var_id,
                                      OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_U32,
                                                                 OCTOLINKER_LITE_SHAPE_ARRAY),
                                      (const uint8_t *)values,
                                      (uint16_t)(count * sizeof(uint32_t)));
}

knx_status_t Octolinker_SendLiteI16Array(Octolinker_Instance_t *ol, uint8_t var_id, const int16_t *values, uint16_t count)
{
    if (values == NULL || count == 0U || count > (UINT16_MAX / sizeof(int16_t))) {
        return KNX_INVALID_ARG;
    }
    return Octolinker_SendLiteChunked(ol, var_id,
                                      OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_I16,
                                                                 OCTOLINKER_LITE_SHAPE_ARRAY),
                                      (const uint8_t *)values,
                                      (uint16_t)(count * sizeof(int16_t)));
}

knx_status_t Octolinker_SendLiteU16Array(Octolinker_Instance_t *ol, uint8_t var_id, const uint16_t *values, uint16_t count)
{
    if (values == NULL || count == 0U || count > (UINT16_MAX / sizeof(uint16_t))) {
        return KNX_INVALID_ARG;
    }
    return Octolinker_SendLiteChunked(ol, var_id,
                                      OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_U16,
                                                                 OCTOLINKER_LITE_SHAPE_ARRAY),
                                      (const uint8_t *)values,
                                      (uint16_t)(count * sizeof(uint16_t)));
}

knx_status_t Octolinker_SendLiteI8Array(Octolinker_Instance_t *ol, uint8_t var_id, const int8_t *values, uint16_t count)
{
    if (values == NULL || count == 0U) {
        return KNX_INVALID_ARG;
    }
    return Octolinker_SendLiteChunked(ol, var_id,
                                      OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_I8,
                                                                 OCTOLINKER_LITE_SHAPE_ARRAY),
                                      (const uint8_t *)values, count);
}

knx_status_t Octolinker_SendLiteU8Array(Octolinker_Instance_t *ol, uint8_t var_id, const uint8_t *values, uint16_t count)
{
    if (values == NULL || count == 0U) {
        return KNX_INVALID_ARG;
    }
    return Octolinker_SendLiteChunked(ol, var_id,
                                      OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_U8,
                                                                 OCTOLINKER_LITE_SHAPE_ARRAY),
                                      values, count);
}

knx_status_t Octolinker_SendLiteU8Matrix(Octolinker_Instance_t *ol, uint8_t var_id, const uint8_t *values, uint8_t rows, uint8_t cols)
{
    if (values == NULL || rows == 0U || cols == 0U) {
        return KNX_INVALID_ARG;
    }
    return Octolinker_SendLiteMatrixChunked(ol, var_id,
                                            OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_U8,
                                                                       OCTOLINKER_LITE_SHAPE_MATRIX),
                                            values, (uint16_t)rows * (uint16_t)cols,
                                            rows, cols);
}

knx_status_t Octolinker_SendLiteBitMatrix(Octolinker_Instance_t *ol, uint8_t var_id, const uint8_t *data, uint8_t rows, uint8_t cols)
{
    if (data == NULL || rows == 0U || cols == 0U) {
        return KNX_INVALID_ARG;
    }
    uint16_t bit_count = (uint16_t)rows * (uint16_t)cols;
    uint16_t bytes = (uint16_t)((bit_count + 7U) / 8U);
    return Octolinker_SendLiteMatrixChunked(ol, var_id,
                                            OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_BITS,
                                                                       OCTOLINKER_LITE_SHAPE_BIT_MATRIX),
                                            data, bytes, rows, cols);
}

knx_status_t Octolinker_SendLiteRaw(Octolinker_Instance_t *ol, uint8_t var_id, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return KNX_INVALID_ARG;
    }
    return Octolinker_SendLiteChunked(ol, var_id,
                                      OCTOLINKER_LITE_TYPE_SHAPE(OCTOLINKER_LITE_TYPE_RAW,
                                                                 OCTOLINKER_LITE_SHAPE_ARRAY),
                                      data, len);
}

knx_status_t Octolinker_PrintLine(Octolinker_Instance_t *ol, const char *line)
{
    if (ol == NULL || ol->uart == NULL || line == NULL) {
        return KNX_INVALID_ARG;
    }

    char buf[128];
    size_t len = strlen(line);
    if (len > 0U && (line[len - 1U] == '\n' || line[len - 1U] == '\r')) {
        if (len >= sizeof(buf)) {
            len = sizeof(buf) - 1U;
        }
        memcpy(buf, line, len);
        buf[len] = '\0';
    } else {
        if (len >= (sizeof(buf) - 2U)) {
            len = sizeof(buf) - 3U;
        }
        memcpy(buf, line, len);
        buf[len++] = '\r';
        buf[len++] = '\n';
        buf[len] = '\0';
    }

    return Octolinker_SendBytes(ol, (const uint8_t *)buf, (uint16_t)len);
}

knx_status_t Octolinker_SendKV_F32(Octolinker_Instance_t *ol, const char *key, float value)
{
    if (ol == NULL || ol->uart == NULL || key == NULL) {
        return KNX_INVALID_ARG;
    }

    char buf[64];
    int len = snprintf(buf, sizeof(buf), "%s=%.3f\r\n", key, (double)value);
    if (len < 0 || (size_t)len >= sizeof(buf)) {
        return KNX_ERROR;
    }
    return Octolinker_SendBytes(ol, (const uint8_t *)buf, (uint16_t)len);
}

knx_status_t Octolinker_SendKV_I32(Octolinker_Instance_t *ol, const char *key, int32_t value)
{
    if (ol == NULL || ol->uart == NULL || key == NULL) {
        return KNX_INVALID_ARG;
    }

    char buf[48];
    int len = snprintf(buf, sizeof(buf), "%s=%ld\r\n", key, (long)value);
    if (len < 0 || (size_t)len >= sizeof(buf)) {
        return KNX_ERROR;
    }
    return Octolinker_SendBytes(ol, (const uint8_t *)buf, (uint16_t)len);
}

knx_status_t Octolinker_SendKV_U32(Octolinker_Instance_t *ol, const char *key, uint32_t value)
{
    if (ol == NULL || ol->uart == NULL || key == NULL) {
        return KNX_INVALID_ARG;
    }

    char buf[48];
    int len = snprintf(buf, sizeof(buf), "%s=%lu\r\n", key, (unsigned long)value);
    if (len < 0 || (size_t)len >= sizeof(buf)) {
        return KNX_ERROR;
    }
    return Octolinker_SendBytes(ol, (const uint8_t *)buf, (uint16_t)len);
}
