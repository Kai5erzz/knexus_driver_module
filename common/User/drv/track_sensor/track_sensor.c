/**
 * @file    track_sensor.c
 * @author  kaiser
 * @version V2.0.0
 * @date    2026-06-02
 * @brief   循迹传感器驱动实现 - 8通道模拟多路复用器 + ADC读取 (platform-backed)
 *
 * 实现说明:
 *   - 通过3根地址线选择多路复用器通道
 *   - 使用 knx_gpio_write 设置 MUX 地址
 *   - 使用 knx_adc_read_raw 读取 ADC2
 *   - 原始16-bit ADC值直接存入全局数组
 */

#include "track_sensor.h"
#include <string.h>

/* ==================== 全局变量 ==================== */
uint16_t track_raw[TRACK_SENSOR_CHANNELS] = {0};

/* ==================== 私有数据 ==================== */
static const track_sensor_port_t *s_port;

/* ==================== 私有函数 ==================== */

/**
 * @brief  设置多路复用器地址线
 * @param  ch: 通道号 (0-7)
 */
static void TrackSensor_SetMux(uint8_t ch)
{
    knx_gpio_write(s_port->mux_ad0, (ch & 0x01) ? KNX_GPIO_HIGH : KNX_GPIO_LOW);
    knx_gpio_write(s_port->mux_ad1, (ch & 0x02) ? KNX_GPIO_HIGH : KNX_GPIO_LOW);
    knx_gpio_write(s_port->mux_ad2, (ch & 0x04) ? KNX_GPIO_HIGH : KNX_GPIO_LOW);
}

/**
 * @brief  微秒级延时 (忙等待)
 * @param  us: 延时微秒数
 * @note   基于480MHz系统时钟, 粗略延时
 */
static void TrackSensor_DelayUs(uint32_t us)
{
    /* 480MHz -> 约 480 cycles/us, 考虑循环开销, 用 120 次迭代/us */
    volatile uint32_t count = us * 120;
    while (count--) {
        __asm volatile ("nop");
    }
}

/* ==================== 接口函数实现 ==================== */

void TrackSensor_AttachPorts(const track_sensor_port_t *port)
{
    s_port = port;
}

void TrackSensor_Init(void)
{
    /* 清零原始数据 */
    memset(track_raw, 0, sizeof(track_raw));

    /* 地址线初始状态: 全低 (选择通道0) */
    if (s_port) {
        TrackSensor_SetMux(0);
    }
}

knx_status_t TrackSensor_Scan(void)
{
    knx_status_t status;
    uint32_t raw;

    if (!s_port) {
        return KNX_INVALID_ARG;
    }

    /* 逐通道扫描 */
    for (uint8_t ch = 0; ch < TRACK_SENSOR_CHANNELS; ch++) {
        /* 设置多路复用器地址 */
        TrackSensor_SetMux(ch);

        /* 等待多路复用器输出稳定 */
        TrackSensor_DelayUs(TRACK_MUX_SETTLE_US);

        /* 读取ADC2值 (ADC2_INP8 = PC5) */
        status = knx_adc_read_raw(&s_port->adc, &raw);
        if (status != KNX_OK) {
            return status;
        }

        track_raw[ch] = (uint16_t)raw;
    }

    return KNX_OK;
}

void TrackSensor_DebugOcto(Octolinker_Instance_t *octo)
{
    if (octo == NULL) return;

    /* 发送8通道原始数据作为uint16数组 */
    Octolinker_SendU16Array(octo, 30, track_raw, TRACK_SENSOR_CHANNELS);
}
