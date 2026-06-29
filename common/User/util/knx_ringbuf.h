#ifndef KNX_RINGBUF_H
#define KNX_RINGBUF_H

#include "knx_types.h"
#include <stdint.h>

typedef struct {
    uint8_t *buffer;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t push_count;
    volatile uint32_t pop_count;
    volatile uint32_t overflow_count;
} knx_ringbuf_t;

knx_status_t knx_ringbuf_init(knx_ringbuf_t *rb, uint8_t *buffer, uint16_t size);
void knx_ringbuf_reset(knx_ringbuf_t *rb);
uint16_t knx_ringbuf_available(const knx_ringbuf_t *rb);
uint16_t knx_ringbuf_free(const knx_ringbuf_t *rb);
uint8_t knx_ringbuf_push_byte(knx_ringbuf_t *rb, uint8_t byte);
uint16_t knx_ringbuf_push(knx_ringbuf_t *rb, const uint8_t *data, uint16_t len);
uint8_t knx_ringbuf_pop_byte(knx_ringbuf_t *rb, uint8_t *byte);
uint16_t knx_ringbuf_pop(knx_ringbuf_t *rb, uint8_t *out, uint16_t max_len);

/**
 * @brief 窥视（不消费）缓冲区头部 N 字节。供协议解析 lookahead / 调试使用。
 *        多次 peek 返回相同数据直到 pop 才推进 tail。
 * @param rb  缓冲区
 * @param dst 目标内存（非 NULL，容量 >= n）
 * @param n   要窥视的字节数
 * @return    实际复制的字节数（若 <n 则表示缓冲区内可用数据不足）
 */
uint16_t knx_ringbuf_peek(const knx_ringbuf_t *rb, void *dst, uint16_t n);

/**
 * @brief 复制缓冲区内最新 N 字节（tail-peek，从写入端倒数 N 个）。
 *        主要用于调试协议栈 / 上位机查看"刚收到什么"。
 *        注意 n 不应超过 rb->size - 1（环形缓冲有 1 字节空位）。
 * @param rb  缓冲区
 * @param dst 目标内存
 * @param n   要查看的字节数（≤ 实际可用数据量）
 * @return    实际复制的字节数
 */
uint16_t knx_ringbuf_peek_tail(const knx_ringbuf_t *rb, void *dst, uint16_t n);

/**
 * @brief 查询头部连续可读字节数（到 buffer 末尾，未跨回绕点）。
 *        配合 knx_ringbuf_available() 一起使用：先调本函数处理第一段，
 *        若不足再读剩余。适合 DMA / 大块协议解析。
 * @param rb  缓冲区
 * @return    头部连续可读字节数 [0, capacity-1]
 */
uint16_t knx_ringbuf_readable_linear(const knx_ringbuf_t *rb);

uint32_t knx_ringbuf_push_count(const knx_ringbuf_t *rb);
uint32_t knx_ringbuf_pop_count(const knx_ringbuf_t *rb);
uint32_t knx_ringbuf_overflow_count(const knx_ringbuf_t *rb);

#endif /* KNX_RINGBUF_H */
