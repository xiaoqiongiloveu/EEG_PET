#include "circular_buffer.h"

static uint8_t internal_buffer[CIRCULAR_BUFFER_MAX_SIZE];

void circular_buffer_init(CircularBuffer *cb, uint32_t size) {
    cb->data = internal_buffer;
    cb->size = (size > CIRCULAR_BUFFER_MAX_SIZE) ? CIRCULAR_BUFFER_MAX_SIZE : size;
    cb->head = 0;
    cb->tail = 0;
}

void circular_buffer_clear(CircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
}

int circular_buffer_push(CircularBuffer *cb, uint8_t data) {
    uint32_t next_head = (cb->head + 1) % cb->size;
    if (next_head == cb->tail) {
        return -1; // Buffer full
    }
    cb->data[cb->head] = data;
    cb->head = next_head;
    return 0;
}

int circular_buffer_pop(CircularBuffer *cb, uint8_t *data) {
    if (cb->head == cb->tail) {
        return -1; // Buffer empty
    }
    *data = cb->data[cb->tail];
    cb->tail = (cb->tail + 1) % cb->size;
    return 0;
}

int circular_buffer_read(CircularBuffer *cb, uint8_t *data, uint32_t len) {
    if (circular_buffer_available(cb) < len) {
        return -1; // Not enough data
    }
    for (uint32_t i = 0; i < len; i++) {
        circular_buffer_pop(cb, &data[i]);
    }
    return 0;
}

uint32_t circular_buffer_available(CircularBuffer *cb) {
    if (cb->head >= cb->tail) {
        return cb->head - cb->tail;
    } else {
        return cb->size + cb->head - cb->tail;
    }
}

uint32_t circular_buffer_get_data_count(CircularBuffer *cb) {
    return circular_buffer_available(cb);
}
