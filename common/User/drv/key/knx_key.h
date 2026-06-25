/**
 * @file    knx_key.h
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-06-07
 * @brief   按键驱动 - 支持多路按键, 消抖 + 边沿检测 + 长按计时 (platform-backed)
 *
 * 硬件连接:
 *   KEY0: PE3 - 下拉, 高电平有效 (按下=HIGH)
 *   KEY1: PG10 - 上拉, 低电平有效 (按下=LOW)
 *
 * 使用方式:
 *   1. board 层调用 knx_key_attach_ports() 绑定引脚
 *   2. 初始化时调用 knx_key_init()
 *   3. 周期性调用 knx_key_update() 进行采样和消抖 (建议 5-10ms)
 *   4. 应用层通过 knx_key_is_pressed() / knx_key_get_hold_ms() 查询状态
 */

#ifndef KNX_KEY_H
#define KNX_KEY_H

#include "knx_types.h"
#include "knx_gpio.h"

/* ==================== 配置参数 ==================== */

#define KNX_KEY_MAX_KEYS        4       /**< 最大按键数量 */
#define KNX_KEY_DEBOUNCE_CNT    3       /**< 消抖连续采样次数 (× 调用周期) */
#define KNX_KEY_LONG_PRESS_MS   1000    /**< 长按判定阈值 (ms) */

/* ==================== 按键 ID 定义 ==================== */

typedef enum {
    KNX_KEY_0 = 0,      /**< PE3 - 下拉, 高有效 */
    KNX_KEY_1 = 1,      /**< PG10 - 上拉, 低有效 */
} knx_key_id_t;

/* ==================== 端口描述 ==================== */

/**
 * @brief  按键平台端口描述
 *
 * board 层在 knx_board.c 中实例化, 通过 knx_key_attach_ports() 绑定。
 */
typedef struct {
    knx_gpio_t  gpio;           /**< 按键 GPIO 引脚 */
    uint8_t     active_low;     /**< 1 = 低电平有效 (按下=LOW), 0 = 高电平有效 (按下=HIGH) */
} knx_key_port_t;

/* ==================== 按键状态 ==================== */

typedef struct {
    bool     pressed;           /**< 当前是否按下 (消抖后) */
    bool     just_pressed;      /**< 本次 update 中检测到按下边沿 (脉冲) */
    bool     just_released;     /**< 本次 update 中检测到释放边沿 (脉冲) */
    bool     long_press;        /**< 是否达到长按阈值 */
    uint32_t hold_ms;           /**< 当前按住时长 (ms) */
} knx_key_state_t;

/* ==================== 接口函数 ==================== */

/**
 * @brief  绑定平台端口 (board 层调用, 在 init 之前)
 * @param  port  端口描述数组指针
 * @param  count 按键数量
 */
void knx_key_attach_ports(const knx_key_port_t *port, uint8_t count);

/**
 * @brief  初始化按键 (清零状态)
 */
void knx_key_init(void);

/**
 * @brief  周期性更新 - 采样 + 消抖 + 边沿检测 + 长按计时
 * @param  dt_ms  距上次调用的时间间隔 (ms), 通常 5-10ms
 * @note   必须由 app 层以固定周期调用
 */
void knx_key_update(uint32_t dt_ms);

/**
 * @brief  查询按键是否按下 (消抖后)
 * @param  id  按键 ID
 * @return true = 按下
 */
bool knx_key_is_pressed(knx_key_id_t id);

/**
 * @brief  查询按键是否刚按下 (边沿, 单次脉冲)
 * @param  id  按键 ID
 * @return true = 本次 update 检测到按下
 */
bool knx_key_just_pressed(knx_key_id_t id);

/**
 * @brief  查询按键是否刚释放 (边沿, 单次脉冲)
 * @param  id  按键 ID
 * @return true = 本次 update 检测到释放
 */
bool knx_key_just_released(knx_key_id_t id);

/**
 * @brief  查询按键是否达到长按阈值
 * @param  id  按键 ID
 * @return true = 长按
 */
bool knx_key_is_long_press(knx_key_id_t id);

/**
 * @brief  获取按键当前按住时长 (ms)
 * @param  id  按键 ID
 * @return 按住时长, 释放时归零
 */
uint32_t knx_key_get_hold_ms(knx_key_id_t id);

/**
 * @brief  获取指定按键完整状态快照
 * @param  id     按键 ID
 * @param  state  输出状态指针
 * @return KNX_OK / KNX_INVALID_ARG
 */
knx_status_t knx_key_get_state(knx_key_id_t id, knx_key_state_t *state);

#endif /* KNX_KEY_H */
