/**
 * @file    knx_beep.h
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-06-07
 * @brief   蜂鸣器驱动 - 非阻塞式 (platform-backed)
 *
 * 硬件: PA10 (BEEP), 推挽输出, 外部上拉
 *   HIGH = 响, LOW = 不响
 *
 * 使用方式:
 *   1. board 层调用 knx_beep_attach_port() 绑定引脚
 *   2. knx_beep_init() 初始化
 *   3. app loop 中以 1ms 周期调用 knx_beep_update()
 *   4. 任意位置调用 knx_beep_beep(ms) 触发响声
 */

#ifndef KNX_BEEP_H
#define KNX_BEEP_H

#include "knx_types.h"
#include "knx_gpio.h"

void knx_beep_attach_port(knx_gpio_t gpio);
void knx_beep_init(void);
void knx_beep_update(void);
void knx_beep_beep(uint32_t duration_ms);

#endif /* KNX_BEEP_H */
