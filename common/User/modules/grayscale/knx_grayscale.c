/**
 * @file knx_grayscale.c
 * @brief Adapter from the MCU-based digital line sensor to the existing
 *        chassis tracking interface.
 */

#include "knx_grayscale.h"
#include "knx_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static const float s_line_weights[KNX_GRAYSCALE_CH_NUM] = {
    -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f
};

static knx_grayscale_data_t s_data;
static ir_line_sensor_t *s_sensor;
static uint32_t s_last_poll_ms;
static uint8_t s_has_sample;

void knx_grayscale_init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        s_data.cal_min[i] = 0U;
        s_data.cal_max[i] = 1U;
    }
    /* Thresholding is completed inside the new sensor module. */
    s_data.is_calibrated = 1U;
    s_last_poll_ms = 0U;
    s_has_sample = 0U;
}

void knx_grayscale_attach_sensor(ir_line_sensor_t *sensor)
{
    taskENTER_CRITICAL();
    s_sensor = sensor;
    s_last_poll_ms = 0U;
    s_has_sample = 0U;
    taskEXIT_CRITICAL();
}

void knx_grayscale_set_calibration(const uint16_t *cal_min,
                                   const uint16_t *cal_max)
{
    /* Compatibility no-op: applications written for the old ADC board may
     * still call this function, but the digital module needs no calibration. */
    (void)cal_min;
    (void)cal_max;
    s_data.is_calibrated = 1U;
}

knx_status_t knx_grayscale_update(void)
{
    if (s_sensor == NULL) return KNX_NOT_READY;

    uint32_t now = knx_millis();
    if (s_has_sample != 0U &&
        (now - s_last_poll_ms) < IR_LINE_SENSOR_UPDATE_PERIOD_MS) {
        return KNX_OK;
    }

    knx_status_t status = ir_line_sensor_read(s_sensor);
    /* OLED may own the shared bus for one short transaction.  Retain the last
     * complete sample and retry next period instead of declaring sensor loss. */
    if (status == KNX_BUSY && s_has_sample != 0U) return KNX_OK;
    if (status != KNX_OK) return status;

    const ir_line_sensor_sample_t *sample = ir_line_sensor_get_sample(s_sensor);
    if (sample == NULL || sample->valid == 0U) return KNX_ERROR;

    knx_grayscale_data_t next = s_data;
    next.digital_byte = sample->digital_bits;
    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        uint8_t detected = (uint8_t)((sample->digital_bits >> i) & 1U);
        next.raw[i] = detected;
        next.normalized[i] = (float)detected;
        next.digital[i] = detected;
    }

    float weighted_sum = 0.0f;
    float strength = 0.0f;
    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        weighted_sum += s_line_weights[i] * next.normalized[i];
        strength += next.normalized[i];
    }
    next.line_error = strength > 0.01f ? weighted_sum / strength : 0.0f;
    next.is_calibrated = 1U;

    taskENTER_CRITICAL();
    s_data = next;
    s_last_poll_ms = now;
    s_has_sample = 1U;
    taskEXIT_CRITICAL();
    return KNX_OK;
}

void knx_grayscale_calibrate(void)
{
    /* Kept for source compatibility with legacy modes. */
    s_data.is_calibrated = 1U;
}

void knx_grayscale_snapshot(knx_grayscale_data_t *data)
{
    if (data == NULL) return;
    taskENTER_CRITICAL();
    *data = s_data;
    taskEXIT_CRITICAL();
}

float knx_grayscale_get_line_error(void)
{
    return s_data.line_error;
}

uint8_t knx_grayscale_get_digital_byte(void)
{
    return s_data.digital_byte;
}

uint8_t knx_grayscale_is_calibrated(void)
{
    return 1U;
}

void knx_grayscale_debug_octo(Octolinker_Instance_t *octo)
{
    if (octo == NULL) return;
    (void)Octolinker_SendF32(octo, 50U, s_data.line_error);
    (void)Octolinker_SendU8(octo, 51U, s_data.digital_byte);
    for (uint8_t i = 0U; i < KNX_GRAYSCALE_CH_NUM; ++i) {
        (void)Octolinker_SendF32(octo, (uint16_t)(52U + i),
                                 s_data.normalized[i]);
    }
}
