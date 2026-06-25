/**
 * @file    knx_key_debug.h
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-06-07
 * @brief   按键/LED OctoLink 调试输出
 *
 * OctoLink VAR_ID 分配 (40-49):
 *   40 - KEY0 按下状态 (0/1)
 *   41 - KEY1 按下状态 (0/1)
 *   42 - KEY0 按住时长 (ms)
 *   43 - KEY1 按住时长 (ms)
 *   44 - KEY0 长按标志 (0/1)
 *   45 - KEY1 长按标志 (0/1)
 *   46 - LED1 状态 (0/1)
 *   47 - LED2 状态 (0/1)
 */

#ifndef KNX_KEY_DEBUG_H
#define KNX_KEY_DEBUG_H

#include "octolinker.h"

/**
 * @brief  通过 OctoLink 输出 KEY/LED 状态
 * @param  octo  OctoLink 实例指针
 */
void knx_key_debug_octo(Octolinker_Instance_t *octo);

#endif /* KNX_KEY_DEBUG_H */
