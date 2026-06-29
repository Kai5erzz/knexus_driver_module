#include "encoder.h"
#include "knx_time.h"
#include <stddef.h>

static float speed_lpf_left = 0.0f;
static float speed_lpf_right = 0.0f;

Encoder_t encoder_left = {
    .port = NULL,
    .is_lptim = 0,
    .dir_sign = ENC1_DIR_SIGN,
    .total_count = 0,
    .last_raw = 0,
};

Encoder_t encoder_right = {
    .port = NULL,
    .is_lptim = 1,
    .dir_sign = ENC2_DIR_SIGN,
    .total_count = 0,
    .last_raw = 0,
};

knx_status_t Encoder_AttachPorts(const knx_encoder_port_t *left,
                         const knx_encoder_port_t *right)
{
    if (left == NULL || right == NULL) {
        return KNX_INVALID_ARG;
    }

    encoder_left.port = left;
    encoder_left.is_lptim = (left->type == KNX_ENCODER_LPTIM) ? 1U : 0U;

    encoder_right.port = right;
    encoder_right.is_lptim = (right->type == KNX_ENCODER_LPTIM) ? 1U : 0U;
    return KNX_OK;
}

static uint16_t Encoder_ReadRaw(Encoder_t *enc)
{
    uint16_t raw = 0;
    knx_encoder_read_raw(enc->port, &raw);
    return raw;
}

static int16_t Encoder_CalcDiff(uint16_t now, uint16_t last)
{
    return (int16_t)(now - last);
}

static float Encoder_CountsPerRev(const Encoder_t *enc)
{
    return enc->is_lptim ?
        (float)ENC_COUNTS_PER_WHEEL_REV_LPTIM :
        (float)ENC_COUNTS_PER_WHEEL_REV_TIM;
}

static float Encoder_CountToDistance(const Encoder_t *enc, int32_t count)
{
    return ((float)count / Encoder_CountsPerRev(enc)) *
           3.14159265f * (WHEEL_DIAMETER_MM / 1000.0f);
}

static void Encoder_ApplyDelta(Encoder_t *enc, int32_t signed_count)
{
    enc->total_count += signed_count;
    enc->delta_m = Encoder_CountToDistance(enc, signed_count);
    enc->position_m += enc->delta_m;
}

knx_status_t Encoder_Init(void)
{
    if (encoder_left.port == NULL || encoder_right.port == NULL) {
        return KNX_INVALID_ARG;
    }

    knx_encoder_start(encoder_left.port);
    knx_encoder_start(encoder_right.port);

    encoder_left.last_raw = Encoder_ReadRaw(&encoder_left);
    encoder_right.last_raw = Encoder_ReadRaw(&encoder_right);

    Encoder_Reset(&encoder_left);
    Encoder_Reset(&encoder_right);

    speed_lpf_left = 0.0f;
    speed_lpf_right = 0.0f;
    encoder_left.last_calc_tick = knx_millis();
    encoder_right.last_calc_tick = knx_millis();
    return KNX_OK;
}

void Encoder_Update(void)
{
    uint16_t raw_left = Encoder_ReadRaw(&encoder_left);
    int16_t diff_left = Encoder_CalcDiff(raw_left, encoder_left.last_raw);
    Encoder_ApplyDelta(&encoder_left, (int32_t)diff_left * encoder_left.dir_sign);
    encoder_left.last_raw = raw_left;

    uint16_t raw_right = Encoder_ReadRaw(&encoder_right);
    int16_t diff_right = Encoder_CalcDiff(raw_right, encoder_right.last_raw);
    Encoder_ApplyDelta(&encoder_right, (int32_t)diff_right * encoder_right.dir_sign);
    encoder_right.last_raw = raw_right;
}

int32_t Encoder_GetCount(Encoder_t *enc)
{
    return enc->total_count;
}

void Encoder_Reset(Encoder_t *enc)
{
    enc->total_count = 0;
    enc->last_raw = Encoder_ReadRaw(enc);
    enc->speed_rpm = 0.0f;
    enc->speed_mps = 0.0f;
    enc->speed_mps_raw = 0.0f;
    enc->delta_m = 0.0f;
    enc->position_m = 0.0f;
}

void Encoder_CalcSpeed(Encoder_t *enc)
{
    uint16_t raw = Encoder_ReadRaw(enc);
    int16_t diff = Encoder_CalcDiff(raw, enc->last_raw);
    Encoder_ApplyDelta(enc, (int32_t)diff * enc->dir_sign);
    enc->last_raw = raw;

    uint32_t now = knx_millis();
    uint32_t dt_ms = now - enc->last_calc_tick;
    if (dt_ms < ENC_SPEED_UPDATE_MS) {
        return;
    }
    enc->last_calc_tick = now;

    int32_t delta = enc->total_count;
    enc->total_count = 0;

    float dt_sec = (float)dt_ms * 0.001f;
    float rev_per_sec = (float)delta / (Encoder_CountsPerRev(enc) * dt_sec);

    enc->speed_rpm = rev_per_sec * 60.0f;
    enc->speed_mps_raw = rev_per_sec * 3.14159265f * (WHEEL_DIAMETER_MM / 1000.0f);

    float *lpf_state = (enc == &encoder_left) ? &speed_lpf_left : &speed_lpf_right;
    float rc = 1.0f / (2.0f * 3.14159265f * ENC_SPEED_LPF_HZ);
    float alpha = dt_sec / (rc + dt_sec);
    *lpf_state += alpha * (enc->speed_mps_raw - *lpf_state);
    enc->speed_mps = *lpf_state;
}
