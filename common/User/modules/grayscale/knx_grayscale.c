/**
 * @file    knx_grayscale.c
 * @author  kaiser
 * @version V2.0.0
 * @date    2026-06-07
 * @brief   灰度巡线模块实现
 *
 * 数据流:
 *   TrackSensor_Scan() → raw[8]
 *     → 归一化: normalized = (cal_max - raw) / (cal_max - cal_min)
 *     → 数字化: digital = (normalized > 0.5) ? 1 : 0
 *     → 巡线偏差: line_error = Σ(i × normalized) / Σ(normalized) - 3.5
 */

#include "knx_grayscale.h"
#include "track_sensor.h"
#include "knx_time.h"
#include "octolinker.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ==================== 巡线偏差权重表 ==================== */

static const float s_line_weights[KNX_GRAYSCALE_CH_NUM] = {
    -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f
};

/* ==================== 模块数据 ==================== */

static knx_grayscale_data_t s_data;

/* ==================== 接口函数实现 ==================== */

void knx_grayscale_init(void)
{
    memset(&s_data, 0, sizeof(s_data));

    /* 校准值初始化为满量程 (等效于不校准) */
    for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
        s_data.cal_min[i] = 0;
        s_data.cal_max[i] = 65535;
    }
    s_data.is_calibrated = 0;
}

void knx_grayscale_set_calibration(const uint16_t *cal_min, const uint16_t *cal_max)
{
    if (cal_min == NULL || cal_max == NULL) return;

    for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
        s_data.cal_min[i] = cal_min[i];
        s_data.cal_max[i] = cal_max[i];
        if (s_data.cal_max[i] <= s_data.cal_min[i]) {
            s_data.cal_max[i] = s_data.cal_min[i] + 1;
        }
    }
    s_data.is_calibrated = 1;
}

knx_status_t knx_grayscale_update(void)
{
    /* 读取原始数据 (复用 TrackSensor 驱动) */
    knx_status_t status = TrackSensor_Scan();
    if (status != KNX_OK) {
        return status;
    }

    /* 复制原始值 */
    memcpy(s_data.raw, track_raw, sizeof(s_data.raw));

    s_data.digital_byte = 0;

    for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
        /* 归一化: (cal_max - raw) / (cal_max - cal_min)
         * 黑线上 ADC 低 → cal_max - raw 大 → normalized ≈ 1
         * 白底上 ADC 高 → cal_max - raw 小 → normalized ≈ 0
         */
        float span = (float)(s_data.cal_max[i] - s_data.cal_min[i]);
        if (span < 1.0f) span = 1.0f;

        float val = ((float)s_data.cal_max[i] - (float)s_data.raw[i]) / span;
        if (val < 0.0f) val = 0.0f;
        if (val > 1.0f) val = 1.0f;
        s_data.normalized[i] = val;

        /* 数字化 */
        s_data.digital[i] = (val > KNX_GRAYSCALE_DIGITAL_TH) ? 1 : 0;
        s_data.digital_byte |= (s_data.digital[i] << i);
    }

    /* 巡线偏差: 加权平均 */
    float weighted_sum = 0.0f;
    float weight_sum   = 0.0f;

    for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
        weighted_sum += s_line_weights[i] * s_data.normalized[i];
        weight_sum   += s_data.normalized[i];
    }

    s_data.line_error = (weight_sum > 0.01f) ? (weighted_sum / weight_sum) : 0.0f;

    return KNX_OK;
}

void knx_grayscale_calibrate(void)
{
    uint16_t sample;

    /* ===== 阶段 1: 采样黑线 ===== */
    /* 提示: 将传感器放到黑线上 */
    knx_delay_ms(2000);

    /* 初始化为极值 */
    for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
        s_data.cal_min[i] = 65535;
    }

    /* 采样, 取每通道最小值 */
    for (uint16_t n = 0; n < KNX_GRAYSCALE_CAL_SAMPLES; n++) {
        TrackSensor_Scan();
        for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
            sample = track_raw[i];
            if (sample < s_data.cal_min[i]) {
                s_data.cal_min[i] = sample;
            }
        }
    }

    /* ===== 阶段 2: 采样白底 ===== */
    /* 提示: 将传感器放到白底上 */
    knx_delay_ms(2000);

    /* 初始化为极值 */
    for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
        s_data.cal_max[i] = 0;
    }

    /* 采样, 取每通道最大值 */
    for (uint16_t n = 0; n < KNX_GRAYSCALE_CAL_SAMPLES; n++) {
        TrackSensor_Scan();
        for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
            sample = track_raw[i];
            if (sample > s_data.cal_max[i]) {
                s_data.cal_max[i] = sample;
            }
        }
    }

    /* 防御: 确保 max > min */
    for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
        if (s_data.cal_max[i] <= s_data.cal_min[i]) {
            s_data.cal_max[i] = s_data.cal_min[i] + 1;
        }
    }

    s_data.is_calibrated = 1;
}

void knx_grayscale_snapshot(knx_grayscale_data_t *data)
{
    if (data == NULL) return;
    taskENTER_CRITICAL();
    memcpy(data, &s_data, sizeof(knx_grayscale_data_t));
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
    return s_data.is_calibrated;
}

/* ==================== OctoLink 调试输出 ==================== */

void knx_grayscale_debug_octo(Octolinker_Instance_t *octo)
{
    if (octo == NULL) return;

    /* 巡线偏差 + 数字化字节 */
    Octolinker_SendF32(octo, 50, s_data.line_error);
    Octolinker_SendU8(octo,  51, s_data.digital_byte);

    /* 8 通道归一化值 */
    for (uint8_t i = 0; i < KNX_GRAYSCALE_CH_NUM; i++) {
        Octolinker_SendF32(octo, 52 + i, s_data.normalized[i]);
    }
}
