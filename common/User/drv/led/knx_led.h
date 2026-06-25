/**
 * @file    knx_led.h
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-06-07
 * @brief   LED 驱动 - 支持多路 LED (platform-backed)
 *
 * 硬件连接:
 *   LED0: PH7 (已由 board_debug_led 管理)
 *   LED1: PD3
 *   LED2: PD4
 *
 * 说明:
 *   - 通过 knx_gpio_write 控制, 低电平点亮 (假设共阳)
 *   - LED0 已在 knx_app.c 中作为心跳指示, 本驱动不重复管理
 */

#ifndef KNX_LED_H
#define KNX_LED_H

#include "knx_types.h"
#include "knx_gpio.h"

/* ==================== 配置参数 ==================== */

#define KNX_LED_MAX_LEDS        3       /**< 最大 LED 数量 */

/* ==================== LED ID 定义 ==================== */

typedef enum {
    KNX_LED_0 = 0,      /**< PH7 - 心跳 LED (由 app 管理) */
    KNX_LED_1 = 1,      /**< PD3 */
    KNX_LED_2 = 2,      /**< PD4 */
} knx_led_id_t;

/* ==================== 端口描述 ==================== */

/**
 * @brief  LED 平台端口描述
 *
 * board 层在 knx_board.c 中实例化, 通过 knx_led_attach_ports() 绑定。
 * 驱动层只通过 knx_gpio_write / knx_gpio_toggle 访问硬件。
 */
typedef struct {
    knx_gpio_t  gpio;           /**< LED GPIO 引脚 */
    uint8_t     active_low;     /**< 1 = 低电平点亮, 0 = 高电平点亮 */
} knx_led_port_t;

/* ==================== 接口函数 ==================== */

/**
 * @brief  绑定平台端口 (board 层调用, 在 init 之前)
 * @param  port  端口描述数组指针
 * @param  count LED 数量
 */
void knx_led_attach_ports(const knx_led_port_t *port, uint8_t count);

/**
 * @brief  初始化 LED (全部熄灭)
 */
void knx_led_init(void);

/**
 * @brief  点亮指定 LED
 * @param  id  LED ID
 */
void knx_led_on(knx_led_id_t id);

/**
 * @brief  熄灭指定 LED
 * @param  id  LED ID
 */
void knx_led_off(knx_led_id_t id);

/**
 * @brief  翻转指定 LED
 * @param  id  LED ID
 */
void knx_led_toggle(knx_led_id_t id);

/**
 * @brief  设置 LED 状态
 * @param  id     LED ID
 * @param  state  true=点亮, false=熄灭
 */
void knx_led_set(knx_led_id_t id, bool state);

/**
 * @brief  获取 LED 当前状态
 * @param  id  LED ID
 * @return true=点亮, false=熄灭
 */
bool knx_led_get(knx_led_id_t id);

#endif /* KNX_LED_H */
