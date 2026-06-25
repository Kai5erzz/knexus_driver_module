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
uint32_t knx_ringbuf_push_count(const knx_ringbuf_t *rb);
uint32_t knx_ringbuf_pop_count(const knx_ringbuf_t *rb);
uint32_t knx_ringbuf_overflow_count(const knx_ringbuf_t *rb);

#endif /* KNX_RINGBUF_H */
