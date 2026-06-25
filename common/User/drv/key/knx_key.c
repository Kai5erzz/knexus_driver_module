/**
 * @file    knx_key.c
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-06-07
 * @brief   按键驱动实现 - 消抖 + 边沿检测 + 长按计时
 *
 * 消抖策略:
 *   每次 update() 读取 GPIO, 连续 N 次 (KNX_KEY_DEBOUNCE_CNT) 相同采样值
 *   才确认状态变化。确认后产生 just_pressed / just_released 边沿脉冲。
 *
 * 长按检测:
 *   按下后持续累计 hold_ms, 达到 KNX_KEY_LONG_PRESS_MS 后置 long_press 标志。
 *   释放时清零。
 */

#include "knx_key.h"
#include <string.h>

/* ==================== 私有数据 ==================== */

static const knx_key_port_t *s_port;
static uint8_t s_count;

/* 每个按键的内部状态 */
typedef struct {
    bool     raw_level;         /**< 当前原始电平 (消抖前) */
    bool     stable_level;      /**< 消抖后的稳定电平 */
    uint8_t  debounce_cnt;      /**< 连续相同采样计数 */
    bool     pressed;           /**< 当前按下状态 (消抖后) */
    bool     just_pressed;      /**< 边沿脉冲: 刚按下 */
    bool     just_released;     /**< 边沿脉冲: 刚释放 */
    bool     long_press;        /**< 长按标志 */
    uint32_t hold_ms;           /**< 按住累计时长 */
} knx_key_internal_t;

static knx_key_internal_t s_key[KNX_KEY_MAX_KEYS];

/* ==================== 私有函数 ==================== */

/**
 * @brief  读取指定按键的物理电平, 转换为逻辑状态 (true=按下)
 */
static bool knx_key_read_raw(uint8_t idx)
{
    if (idx >= s_count || s_port == NULL) return false;

    knx_gpio_state_t level = knx_gpio_read(s_port[idx].gpio);

    if (s_port[idx].active_low) {
        return (level == KNX_GPIO_LOW);     /* 低有效: LOW = 按下 */
    } else {
        return (level == KNX_GPIO_HIGH);    /* 高有效: HIGH = 按下 */
    }
}

/* ==================== 接口函数实现 ==================== */

void knx_key_attach_ports(const knx_key_port_t *port, uint8_t count)
{
    s_port  = port;
    s_count = (count > KNX_KEY_MAX_KEYS) ? KNX_KEY_MAX_KEYS : count;
}

void knx_key_init(void)
{
    memset(s_key, 0, sizeof(s_key));

    /* 读取一次初始状态, 作为消抖基准 */
    for (uint8_t i = 0; i < s_count; i++) {
        bool raw = knx_key_read_raw(i);
        s_key[i].raw_level    = raw;
        s_key[i].stable_level = raw;
        s_key[i].pressed      = raw;
        s_key[i].debounce_cnt = KNX_KEY_DEBOUNCE_CNT;  /* 已稳定 */
    }
}

void knx_key_update(uint32_t dt_ms)
{
    for (uint8_t i = 0; i < s_count; i++) {
        knx_key_internal_t *k = &s_key[i];

        /* 清除边沿脉冲 (仅持续一个周期) */
        k->just_pressed  = false;
        k->just_released = false;

        /* 读取原始电平 */
        bool raw = knx_key_read_raw(i);

        /* 消抖: 连续 N 次相同才确认 */
        if (raw != k->stable_level) {
            /* 电平发生了变化 */
            if (raw == k->raw_level) {
                /* 与上次原始值相同, 持续计数 */
                k->debounce_cnt++;
                if (k->debounce_cnt >= KNX_KEY_DEBOUNCE_CNT) {
                    /* 消抖确认, 更新稳定状态 */
                    k->stable_level = raw;
                    k->debounce_cnt = 0;

                    /* 产生边沿事件 */
                    if (raw && !k->pressed) {
                        k->just_pressed = true;
                        k->pressed      = true;
                        k->long_press   = false;
                        k->hold_ms      = 0;
                    } else if (!raw && k->pressed) {
                        k->just_released = true;
                        k->pressed       = false;
                        k->long_press    = false;
                        k->hold_ms       = 0;
                    }
                }
            } else {
                /* 电平抖动, 重新计数 */
                k->raw_level    = raw;
                k->debounce_cnt = 1;
            }
        } else {
            /* 电平与稳定值相同, 保持 */
            k->raw_level    = raw;
            k->debounce_cnt = 0;
        }

        /* 按住时长累计 */
        if (k->pressed) {
            k->hold_ms += dt_ms;
            if (k->hold_ms >= KNX_KEY_LONG_PRESS_MS) {
                k->long_press = true;
            }
        }
    }
}

bool knx_key_is_pressed(knx_key_id_t id)
{
    if (id >= s_count) return false;
    return s_key[id].pressed;
}

bool knx_key_just_pressed(knx_key_id_t id)
{
    if (id >= s_count) return false;
    return s_key[id].just_pressed;
}

bool knx_key_just_released(knx_key_id_t id)
{
    if (id >= s_count) return false;
    return s_key[id].just_released;
}

bool knx_key_is_long_press(knx_key_id_t id)
{
    if (id >= s_count) return false;
    return s_key[id].long_press;
}

uint32_t knx_key_get_hold_ms(knx_key_id_t id)
{
    if (id >= s_count) return 0;
    return s_key[id].hold_ms;
}

knx_status_t knx_key_get_state(knx_key_id_t id, knx_key_state_t *state)
{
    if (id >= s_count || state == NULL) return KNX_INVALID_ARG;

    state->pressed       = s_key[id].pressed;
    state->just_pressed  = s_key[id].just_pressed;
    state->just_released = s_key[id].just_released;
    state->long_press    = s_key[id].long_press;
    state->hold_ms       = s_key[id].hold_ms;

    return KNX_OK;
}
