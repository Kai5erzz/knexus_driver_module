#ifndef OCTOLINKER_H
#define OCTOLINKER_H

#include <stddef.h>
#include <stdint.h>
#include "knx_uart.h"

#define OCTOLINKER_VERSION          1U
#define OCTOLINKER_SOF0             0xA5U
#define OCTOLINKER_SOF1             0x5AU
#define OCTOLINKER_MAX_PAYLOAD      128U
#define OCTOLINKER_MAX_SHAPE_DIMS   4U
#define OCTOLINKER_TX_TIMEOUT_MS    2U
#define OCTOLINKER_TX_FRAME_MAX     (2U + 1U + 2U + 1U + 1U + OCTOLINKER_MAX_SHAPE_DIMS + 2U + OCTOLINKER_MAX_PAYLOAD + 1U)

#define OCTOLINKER_TYPE_I32         1U
#define OCTOLINKER_TYPE_U32         2U
#define OCTOLINKER_TYPE_F32         3U
#define OCTOLINKER_TYPE_I16         4U
#define OCTOLINKER_TYPE_U16         5U
#define OCTOLINKER_TYPE_I8          6U
#define OCTOLINKER_TYPE_U8          7U
#define OCTOLINKER_TYPE_RAW         8U
#define OCTOLINKER_TYPE_F32_ARRAY   9U
#define OCTOLINKER_TYPE_U16_ARRAY   10U
#define OCTOLINKER_TYPE_U8_MATRIX   11U
#define OCTOLINKER_TYPE_BIT_MATRIX  12U

#define OCTOLINKER_LITE_SOF         0xAAU
#define OCTOLINKER_LITE_MAX_PAYLOAD 128U
#define OCTOLINKER_LITE_CHUNK_MAX   16U

#define OCTOLINKER_LITE_TYPE_U8     0U
#define OCTOLINKER_LITE_TYPE_I8     1U
#define OCTOLINKER_LITE_TYPE_U16    2U
#define OCTOLINKER_LITE_TYPE_I16    3U
#define OCTOLINKER_LITE_TYPE_U32    4U
#define OCTOLINKER_LITE_TYPE_I32    5U
#define OCTOLINKER_LITE_TYPE_F32    6U
#define OCTOLINKER_LITE_TYPE_BITS   7U
#define OCTOLINKER_LITE_TYPE_RAW    8U

#define OCTOLINKER_LITE_SHAPE_SCALAR     0U
#define OCTOLINKER_LITE_SHAPE_ARRAY      1U
#define OCTOLINKER_LITE_SHAPE_MATRIX     2U
#define OCTOLINKER_LITE_SHAPE_BIT_MATRIX 3U

#define OCTOLINKER_LITE_TYPE_SHAPE(type, shape) \
    ((uint8_t)((((type) & 0x0FU) << 4) | ((shape) & 0x0FU)))

typedef struct {
    knx_uart_t *uart;
    uint8_t tx_buf[OCTOLINKER_TX_FRAME_MAX];
    uint16_t tx_len;
} Octolinker_Instance_t;

void Octolinker_Init(Octolinker_Instance_t *ol, knx_uart_t *uart);

knx_status_t Octolinker_SendFrame(Octolinker_Instance_t *ol,
                                  uint16_t var_id,
                                  uint8_t value_type,
                                  const uint8_t *shape,
                                  uint8_t shape_len,
                                  const uint8_t *payload,
                                  uint16_t payload_len);

knx_status_t Octolinker_SendF32(Octolinker_Instance_t *ol, uint16_t var_id, float value);
knx_status_t Octolinker_SendI32(Octolinker_Instance_t *ol, uint16_t var_id, int32_t value);
knx_status_t Octolinker_SendU32(Octolinker_Instance_t *ol, uint16_t var_id, uint32_t value);
knx_status_t Octolinker_SendI16(Octolinker_Instance_t *ol, uint16_t var_id, int16_t value);
knx_status_t Octolinker_SendU16(Octolinker_Instance_t *ol, uint16_t var_id, uint16_t value);
knx_status_t Octolinker_SendI8(Octolinker_Instance_t *ol, uint16_t var_id, int8_t value);
knx_status_t Octolinker_SendU8(Octolinker_Instance_t *ol, uint16_t var_id, uint8_t value);

knx_status_t Octolinker_SendF32Array(Octolinker_Instance_t *ol,
                                     uint16_t var_id,
                                     const float *values,
                                     uint8_t count);
knx_status_t Octolinker_SendU16Array(Octolinker_Instance_t *ol,
                                     uint16_t var_id,
                                     const uint16_t *values,
                                     uint8_t count);
knx_status_t Octolinker_SendU8Matrix(Octolinker_Instance_t *ol,
                                     uint16_t var_id,
                                     const uint8_t *values,
                                     uint8_t rows,
                                     uint8_t cols);
knx_status_t Octolinker_SendRaw(Octolinker_Instance_t *ol,
                                uint16_t var_id,
                                const uint8_t *data,
                                uint16_t len);
knx_status_t Octolinker_SendBitMatrix(Octolinker_Instance_t *ol,
                                      uint16_t var_id,
                                      const uint8_t *data,
                                      uint8_t rows,
                                      uint8_t cols);

knx_status_t Octolinker_SendLiteFrame(Octolinker_Instance_t *ol,
                                      uint8_t var_id,
                                      uint8_t type_shape,
                                      uint8_t seq,
                                      const uint8_t *payload,
                                      uint8_t payload_len);

knx_status_t Octolinker_SendLiteF32(Octolinker_Instance_t *ol, uint8_t var_id, float value);
knx_status_t Octolinker_SendLiteI32(Octolinker_Instance_t *ol, uint8_t var_id, int32_t value);
knx_status_t Octolinker_SendLiteU32(Octolinker_Instance_t *ol, uint8_t var_id, uint32_t value);
knx_status_t Octolinker_SendLiteI16(Octolinker_Instance_t *ol, uint8_t var_id, int16_t value);
knx_status_t Octolinker_SendLiteU16(Octolinker_Instance_t *ol, uint8_t var_id, uint16_t value);
knx_status_t Octolinker_SendLiteI8(Octolinker_Instance_t *ol, uint8_t var_id, int8_t value);
knx_status_t Octolinker_SendLiteU8(Octolinker_Instance_t *ol, uint8_t var_id, uint8_t value);
knx_status_t Octolinker_SendLiteF32Array(Octolinker_Instance_t *ol, uint8_t var_id, const float *values, uint16_t count);
knx_status_t Octolinker_SendLiteI32Array(Octolinker_Instance_t *ol, uint8_t var_id, const int32_t *values, uint16_t count);
knx_status_t Octolinker_SendLiteU32Array(Octolinker_Instance_t *ol, uint8_t var_id, const uint32_t *values, uint16_t count);
knx_status_t Octolinker_SendLiteI16Array(Octolinker_Instance_t *ol, uint8_t var_id, const int16_t *values, uint16_t count);
knx_status_t Octolinker_SendLiteU16Array(Octolinker_Instance_t *ol, uint8_t var_id, const uint16_t *values, uint16_t count);
knx_status_t Octolinker_SendLiteI8Array(Octolinker_Instance_t *ol, uint8_t var_id, const int8_t *values, uint16_t count);
knx_status_t Octolinker_SendLiteU8Array(Octolinker_Instance_t *ol, uint8_t var_id, const uint8_t *values, uint16_t count);
knx_status_t Octolinker_SendLiteU8Matrix(Octolinker_Instance_t *ol, uint8_t var_id, const uint8_t *values, uint8_t rows, uint8_t cols);
knx_status_t Octolinker_SendLiteBitMatrix(Octolinker_Instance_t *ol, uint8_t var_id, const uint8_t *data, uint8_t rows, uint8_t cols);
knx_status_t Octolinker_SendLiteRaw(Octolinker_Instance_t *ol, uint8_t var_id, const uint8_t *data, uint16_t len);

knx_status_t Octolinker_PrintLine(Octolinker_Instance_t *ol, const char *line);
knx_status_t Octolinker_SendKV_F32(Octolinker_Instance_t *ol, const char *key, float value);
knx_status_t Octolinker_SendKV_I32(Octolinker_Instance_t *ol, const char *key, int32_t value);
knx_status_t Octolinker_SendKV_U32(Octolinker_Instance_t *ol, const char *key, uint32_t value);

#endif /* OCTOLINKER_H */
