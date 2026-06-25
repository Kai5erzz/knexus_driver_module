/**
 * @file    knx_key_debug.c
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-06-07
 * @brief   按键/LED OctoLink 调试输出实现
 */

#include "knx_key_debug.h"
#include "knx_key.h"
#include "knx_led.h"

void knx_key_debug_octo(Octolinker_Instance_t *octo)
{
    if (octo == NULL) return;

    /* KEY 状态 */
    Octolinker_SendI32(octo, 40, knx_key_is_pressed(KNX_KEY_0) ? 1 : 0);
    Octolinker_SendI32(octo, 41, knx_key_is_pressed(KNX_KEY_1) ? 1 : 0);
    Octolinker_SendU32(octo, 42, knx_key_get_hold_ms(KNX_KEY_0));
    Octolinker_SendU32(octo, 43, knx_key_get_hold_ms(KNX_KEY_1));
    Octolinker_SendI32(octo, 44, knx_key_is_long_press(KNX_KEY_0) ? 1 : 0);
    Octolinker_SendI32(octo, 45, knx_key_is_long_press(KNX_KEY_1) ? 1 : 0);

    /* LED 状态 */
    Octolinker_SendI32(octo, 46, knx_led_get(KNX_LED_1) ? 1 : 0);
    Octolinker_SendI32(octo, 47, knx_led_get(KNX_LED_2) ? 1 : 0);
}
