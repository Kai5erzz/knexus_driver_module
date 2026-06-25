/**
 * @file    knx_beep.c
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-06-07
 * @brief   蜂鸣器驱动实现 - 非阻塞
 */

#include "knx_beep.h"

static knx_gpio_t s_gpio;
static uint32_t s_remaining_ms;

void knx_beep_attach_port(knx_gpio_t gpio)
{
    s_gpio = gpio;
}

void knx_beep_init(void)
{
    s_remaining_ms = 0;
    knx_gpio_write(s_gpio, KNX_GPIO_LOW);  /* 默认不响 */
}

void knx_beep_update(void)
{
    if (s_remaining_ms > 0) {
        s_remaining_ms--;
        knx_gpio_write(s_gpio, KNX_GPIO_HIGH);  /* 响 */
    } else {
        knx_gpio_write(s_gpio, KNX_GPIO_LOW);   /* 停 */
    }
}

void knx_beep_beep(uint32_t duration_ms)
{
    s_remaining_ms = duration_ms;
}
