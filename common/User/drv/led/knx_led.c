/**
 * @file    knx_led.c
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-06-07
 * @brief   LED 驱动实现
 */

#include "knx_led.h"
#include <string.h>

/* ==================== 私有数据 ==================== */

static const knx_led_port_t *s_port;
static uint8_t s_count;
static bool s_state[KNX_LED_MAX_LEDS];

/* ==================== 接口函数实现 ==================== */

void knx_led_attach_ports(const knx_led_port_t *port, uint8_t count)
{
    s_port  = port;
    s_count = (count > KNX_LED_MAX_LEDS) ? KNX_LED_MAX_LEDS : count;
}

void knx_led_init(void)
{
    memset(s_state, 0, sizeof(s_state));

    /* 全部熄灭 */
    for (uint8_t i = 0; i < s_count; i++) {
        if (s_port[i].active_low) {
            knx_gpio_write(s_port[i].gpio, KNX_GPIO_HIGH);   /* 高 = 灭 */
        } else {
            knx_gpio_write(s_port[i].gpio, KNX_GPIO_LOW);    /* 低 = 灭 */
        }
    }
}

void knx_led_on(knx_led_id_t id)
{
    if (id >= s_count) return;
    knx_gpio_write(s_port[id].gpio, s_port[id].active_low ? KNX_GPIO_LOW : KNX_GPIO_HIGH);
    s_state[id] = true;
}

void knx_led_off(knx_led_id_t id)
{
    if (id >= s_count) return;
    knx_gpio_write(s_port[id].gpio, s_port[id].active_low ? KNX_GPIO_HIGH : KNX_GPIO_LOW);
    s_state[id] = false;
}

void knx_led_toggle(knx_led_id_t id)
{
    if (id >= s_count) return;
    knx_gpio_toggle(s_port[id].gpio);
    s_state[id] = !s_state[id];
}

void knx_led_set(knx_led_id_t id, bool state)
{
    if (state) {
        knx_led_on(id);
    } else {
        knx_led_off(id);
    }
}

bool knx_led_get(knx_led_id_t id)
{
    if (id >= s_count) return false;
    return s_state[id];
}
