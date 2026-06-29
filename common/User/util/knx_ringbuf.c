#include "knx_ringbuf.h"
#include <stddef.h>

static uint16_t next_index(const knx_ringbuf_t *rb, uint16_t index)
{
    index = (uint16_t)(index + 1U);
    if (index >= rb->size) {
        index = 0U;
    }
    return index;
}

knx_status_t knx_ringbuf_init(knx_ringbuf_t *rb, uint8_t *buffer, uint16_t size)
{
    if (rb == NULL || buffer == NULL || size < 2U) {
        return KNX_INVALID_ARG;
    }

    rb->buffer = buffer;
    rb->size = size;
    knx_ringbuf_reset(rb);
    return KNX_OK;
}

void knx_ringbuf_reset(knx_ringbuf_t *rb)
{
    if (rb == NULL) {
        return;
    }

    rb->head = 0U;
    rb->tail = 0U;
    rb->push_count = 0U;
    rb->pop_count = 0U;
    rb->overflow_count = 0U;
}

uint16_t knx_ringbuf_available(const knx_ringbuf_t *rb)
{
    if (rb == NULL || rb->buffer == NULL || rb->size == 0U) {
        return 0U;
    }

    uint16_t head = rb->head;
    uint16_t tail = rb->tail;
    if (head >= tail) {
        return (uint16_t)(head - tail);
    }
    return (uint16_t)(rb->size - tail + head);
}

uint16_t knx_ringbuf_free(const knx_ringbuf_t *rb)
{
    if (rb == NULL || rb->size < 2U) {
        return 0U;
    }

    return (uint16_t)(rb->size - 1U - knx_ringbuf_available(rb));
}

uint8_t knx_ringbuf_push_byte(knx_ringbuf_t *rb, uint8_t byte)
{
    if (rb == NULL || rb->buffer == NULL || rb->size < 2U) {
        return 0U;
    }

    uint16_t head = rb->head;
    uint16_t next = next_index(rb, head);
    if (next == rb->tail) {
        rb->overflow_count++;
        return 0U;
    }

    rb->buffer[head] = byte;
    rb->head = next;
    rb->push_count++;
    return 1U;
}

uint16_t knx_ringbuf_push(knx_ringbuf_t *rb, const uint8_t *data, uint16_t len)
{
    if (rb == NULL || (data == NULL && len != 0U)) {
        return 0U;
    }

    uint16_t count = 0U;
    while (count < len) {
        if (knx_ringbuf_push_byte(rb, data[count]) == 0U) {
            break;
        }
        count++;
    }
    return count;
}

uint8_t knx_ringbuf_pop_byte(knx_ringbuf_t *rb, uint8_t *byte)
{
    if (rb == NULL || rb->buffer == NULL || byte == NULL ||
        rb->tail == rb->head) {
        return 0U;
    }

    uint16_t tail = rb->tail;
    *byte = rb->buffer[tail];
    rb->tail = next_index(rb, tail);
    rb->pop_count++;
    return 1U;
}

uint16_t knx_ringbuf_pop(knx_ringbuf_t *rb, uint8_t *out, uint16_t max_len)
{
    if (rb == NULL || out == NULL || max_len == 0U) {
        return 0U;
    }

    uint16_t count = 0U;
    while (count < max_len) {
        if (knx_ringbuf_pop_byte(rb, &out[count]) == 0U) {
            break;
        }
        count++;
    }
    return count;
}

uint32_t knx_ringbuf_push_count(const knx_ringbuf_t *rb)
{
    return (rb != NULL) ? rb->push_count : 0U;
}

uint16_t knx_ringbuf_peek(const knx_ringbuf_t *rb, void *dst, uint16_t n)
{
    if (rb == NULL || rb->buffer == NULL || dst == NULL || n == 0U) {
        return 0U;
    }

    uint8_t *out = (uint8_t *)dst;
    uint16_t head = rb->head;
    uint16_t tail = rb->tail;
    uint16_t available;

    if (head >= tail) {
        available = (uint16_t)(head - tail);
    } else {
        available = (uint16_t)(rb->size - tail + head);
    }

    if (n > available) {
        n = available;
    }
    if (n == 0U) {
        return 0U;
    }

    /* First segment: from tail to end of buffer (or end of data) */
    uint16_t first_chunk = (uint16_t)(rb->size - tail);
    if (first_chunk > n) {
        first_chunk = n;
    }
    for (uint16_t i = 0U; i < first_chunk; i++) {
        out[i] = rb->buffer[(uint16_t)(tail + i)];
    }

    /* Second segment: wrapped data from beginning of buffer */
    uint16_t second_chunk = (uint16_t)(n - first_chunk);
    for (uint16_t i = 0U; i < second_chunk; i++) {
        out[(uint16_t)(first_chunk + i)] = rb->buffer[i];
    }

    return n;
}

uint16_t knx_ringbuf_peek_tail(const knx_ringbuf_t *rb, void *dst, uint16_t n)
{
    if (rb == NULL || rb->buffer == NULL || dst == NULL || n == 0U) {
        return 0U;
    }

    uint16_t head = rb->head;
    uint16_t tail = rb->tail;
    uint16_t available;

    if (head >= tail) {
        available = (uint16_t)(head - tail);
    } else {
        available = (uint16_t)(rb->size - tail + head);
    }

    if (n > available) {
        n = available;
    }
    if (n == 0U) {
        return 0U;
    }

    /* Start position: n bytes back from head (handles wrap) */
    uint16_t start;
    if (head >= n) {
        start = (uint16_t)(head - n);
    } else {
        start = (uint16_t)(rb->size - (n - head));
    }

    uint8_t *out = (uint8_t *)dst;
    /* First segment: from start to end of buffer */
    uint16_t first_chunk = (uint16_t)(rb->size - start);
    if (first_chunk > n) {
        first_chunk = n;
    }
    for (uint16_t i = 0U; i < first_chunk; i++) {
        out[i] = rb->buffer[(uint16_t)(start + i)];
    }

    /* Second segment: wrapped data */
    uint16_t second_chunk = (uint16_t)(n - first_chunk);
    for (uint16_t i = 0U; i < second_chunk; i++) {
        out[(uint16_t)(first_chunk + i)] = rb->buffer[i];
    }

    return n;
}

uint16_t knx_ringbuf_readable_linear(const knx_ringbuf_t *rb)
{
    if (rb == NULL || rb->buffer == NULL || rb->size == 0U) {
        return 0U;
    }

    uint16_t head = rb->head;
    uint16_t tail = rb->tail;
    if (head >= tail) {
        return (uint16_t)(head - tail);
    }
    /* Wrapped: contiguous from tail to end of buffer */
    return (uint16_t)(rb->size - tail);
}

uint32_t knx_ringbuf_pop_count(const knx_ringbuf_t *rb)
{
    return (rb != NULL) ? rb->pop_count : 0U;
}

uint32_t knx_ringbuf_overflow_count(const knx_ringbuf_t *rb)
{
    return (rb != NULL) ? rb->overflow_count : 0U;
}
