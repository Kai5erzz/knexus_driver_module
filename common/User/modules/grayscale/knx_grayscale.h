/**
 * @file    knx_grayscale.h
 * @author  kaiser
 * @version V2.0.0
 * @date    2026-06-07
 * @brief   灰度巡线模块 - 校准 + 归一化 + 二值化 + 连续偏差 (platform-backed)
 *
 * 功能:
 *   - 基于 TrackSensor 驱动读取 8 通道原始 ADC 值
 *   - 黑/白校准, 自动计算每通道上下界
 *   - 归一化插值巡线 (连续位置输出)
 *   - 数字化 (二值化) 输出
 *
 * 校准流程:
 *   knx_grayscale_calibrate()
 *     → 提示放黑线 → 延时 → 采样 (每通道取 min)
 *     → 提示放白底 → 延时 → 采样 (每通道取 max)
 *     → 计算归一化参数
 *
 * 归一化算法:
 *   normalized[i] = (cal_max[i] - raw[i]) / (cal_max[i] - cal_min[i])
 *   黑线上 ADC 低 → normalized ≈ 1
 *   白底上 ADC 高 → normalized ≈ 0
 *
 * 巡线偏差:
 *   line_error = Σ(i × normalized[i]) / Σ(normalized[i]) - 3.5
 *   全通道加权, 中间过渡通道自然产生亚像素插值效果
 */

#ifndef KNX_GRAYSCALE_H
#define KNX_GRAYSCALE_H

#include "knx_types.h"
#include "octolinker.h"

/* ==================== 配置参数 ==================== */

#define KNX_GRAYSCALE_CH_NUM        8       /**< 传感器通道数 */
#define KNX_GRAYSCALE_CAL_SAMPLES   100     /**< 每次校准采样次数 */
#define KNX_GRAYSCALE_DIGITAL_TH    0.5f    /**< 数字化阈值 */

/* ==================== 数据结构 ==================== */

typedef struct {
    /* 原始数据 */
    uint16_t raw[KNX_GRAYSCALE_CH_NUM];         /**< 各通道 ADC 原始值 */
    float    normalized[KNX_GRAYSCALE_CH_NUM];  /**< 归一化值 [0, 1], 1=黑线, 0=白底 */
    uint8_t  digital[KNX_GRAYSCALE_CH_NUM];     /**< 数字化结果: 1=黑线, 0=白底 */
    uint8_t  digital_byte;                      /**< 8 通道打包为 1 字节 (bit0=ch0) */

    /* 巡线偏差 */
    float    line_error;                        /**< 连续偏差, 0=居中, <0偏左, >0偏右 */

    /* 校准数据 */
    uint16_t cal_min[KNX_GRAYSCALE_CH_NUM];     /**< 黑线 ADC 值 (每通道最小值) */
    uint16_t cal_max[KNX_GRAYSCALE_CH_NUM];     /**< 白底 ADC 值 (每通道最大值) */
    uint8_t  is_calibrated;                     /**< 校准完成标志 */
} knx_grayscale_data_t;

/* ==================== 接口函数 ==================== */

/**
 * @brief  初始化灰度模块 (清零数据, 校准值设为默认满量程)
 */
void knx_grayscale_init(void);

/**
 * @brief  读取全部 8 通道, 计算归一化 + 数字化 + 巡线偏差
 * @return knx_status_t
 * @note   完整扫描约 0.5ms, 建议 5-10ms 周期调用
 */
knx_status_t knx_grayscale_update(void);

/**
 * @brief  设置校准数据 (由外部校准流程调用)
 * @param  cal_min  黑线 ADC 值数组 (每通道最小值)
 * @param  cal_max  白底 ADC 值数组 (每通道最大值)
 * @note   自动处理 max <= min 的防御
 */
void knx_grayscale_set_calibration(const uint16_t *cal_min, const uint16_t *cal_max);

/**
 * @brief  一键校准 (阻塞, 约 5 秒)
 *
 * 流程:
 *   1. 延时 2 秒 (放置黑线)
 *   2. 采样 N 次, 每通道取最小值
 *   3. 延时 2 秒 (放置白底)
 *   4. 采样 N 次, 每通道取最大值
 *   5. 标记校准完成
 *
 * @note   需要在 RTOS 任务中调用 (使用 knx_delay_ms)
 */
void knx_grayscale_calibrate(void);

/**
 * @brief  获取完整数据快照 (临界区保护)
 * @param  data  输出指针
 */
void knx_grayscale_snapshot(knx_grayscale_data_t *data);

/**
 * @brief  获取巡线偏差
 * @return 连续偏差值, 0=居中, <0偏左, >0偏右, 全白返回 0
 */
float knx_grayscale_get_line_error(void);

/**
 * @brief  获取 8 通道打包字节
 * @return bit0=ch0, bit7=ch7
 */
uint8_t knx_grayscale_get_digital_byte(void);

/**
 * @brief  获取校准状态
 * @return 1=已校准, 0=未校准
 */
uint8_t knx_grayscale_is_calibrated(void);

/**
 * @brief  通过 OctoLink 输出灰度调试数据
 *
 * VAR_ID 分配 (50-59):
 *   50       - line_error (f32)
 *   51       - digital_byte (u8)
 *   52-59    - normalized[0..7] (f32)
 *   60-67    - raw[0..7] (u16, 仅校准时输出)
 *
 * @param  octo  OctoLink 实例指针
 */
void knx_grayscale_debug_octo(Octolinker_Instance_t *octo);

#endif /* KNX_GRAYSCALE_H */
