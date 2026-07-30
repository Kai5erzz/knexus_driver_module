/**
 * @file    track_sensor.h
 * @author  kaiser
 * @version V2.0.0
 * @date    2026-06-02
 * @brief   循迹传感器驱动 - 8通道模拟多路复用器 + ADC读取 (platform-backed)
 *
 * 硬件连接:
 *   多路复用器地址线:
 *     - AD0/AD1/AD2: 由各平台 board 层绑定
 *   ADC输入:
 *     - 由各平台 board 层绑定到独立循迹 ADC
 *
 * 说明:
 *   - 3根地址线选择8个通道 (0-7)
 *   - 每个通道对应一个灰度传感器
 *   - 循迹 ADC 与电机电流 ADC 由平台层分别配置
 *   - 原始ADC值 (16-bit, 0-65535) 直接存储
 *   - 不做归一化处理
 */

#ifndef __TRACK_SENSOR_H
#define __TRACK_SENSOR_H

#include "knx_types.h"
#include "knx_gpio.h"
#include "knx_adc.h"
#include "octolinker.h"

/* ==================== 配置参数 ==================== */
#define TRACK_SENSOR_CHANNELS       8       /**< 传感器通道数 */
#define TRACK_MUX_SETTLE_US         10      /**< 多路复用器稳定时间 (us) */

/* ==================== 端口描述 ==================== */

/**
 * @brief  循迹传感器平台端口描述
 *
 * board 层在 knx_board.c 中实例化, 通过 TrackSensor_AttachPorts() 绑定。
 * 驱动层只通过 knx_gpio_write / knx_adc_read_raw 访问硬件。
 */
typedef struct {
    knx_gpio_t       mux_ad0;     /**< 多路复用器地址线0 */
    knx_gpio_t       mux_ad1;     /**< 多路复用器地址线1 */
    knx_gpio_t       mux_ad2;     /**< 多路复用器地址线2 */
    knx_adc_channel_t adc;        /**< 平台循迹 ADC */
} track_sensor_port_t;

/* ==================== 全局数据 ==================== */

/**
 * @brief  8通道原始ADC值数组
 * @note   track_raw[0] = 通道0 (AD0=0, AD1=0, AD2=0)
 *         track_raw[1] = 通道1 (AD0=1, AD1=0, AD2=0)
 *         ...
 *         track_raw[7] = 通道7 (AD0=1, AD1=1, AD2=1)
 *         值域: 0-65535 (16-bit ADC原始值)
 */
extern uint16_t track_raw[TRACK_SENSOR_CHANNELS];
extern volatile int32_t track_scan_last_status;
extern volatile uint32_t track_scan_ok_count;
extern volatile uint32_t track_scan_error_count;
extern volatile uint32_t track_scan_last_completed_channel;

/* ==================== 接口函数 ==================== */

/**
 * @brief  绑定平台端口 (board层调用, 在 Init 之前)
 * @param  port: 端口描述指针 (board层静态实例)
 */
void TrackSensor_AttachPorts(const track_sensor_port_t *port);

/**
 * @brief  初始化循迹传感器 (清零数据, 设置MUX初始状态)
 * @retval None
 * @note   必须在 AttachPorts 之后调用
 */
void TrackSensor_Init(void);

/**
 * @brief  扫描所有8个通道并更新track_raw数组
 * @retval knx_status_t
 * @note   ADC 实例由 board 层绑定
 */
knx_status_t TrackSensor_Scan(void);

/**
 * @brief  输出调试信息到OctoLink
 * @param  octo: OctoLink实例指针
 * @retval None
 */
void TrackSensor_DebugOcto(Octolinker_Instance_t *octo);

#endif /* __TRACK_SENSOR_H */
