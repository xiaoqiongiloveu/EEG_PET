#ifndef __CIRCULAR_BUFFER_H
#define __CIRCULAR_BUFFER_H

#include <stdint.h>

#define CIRCULAR_BUFFER_MAX_SIZE 16384 // 16KB

typedef struct {
    uint8_t *data;
    uint32_t size;
    volatile uint32_t head;
    volatile uint32_t tail;
} CircularBuffer;

void circular_buffer_init(CircularBuffer *cb, uint32_t size);
void circular_buffer_clear(CircularBuffer *cb);
int circular_buffer_push(CircularBuffer *cb, uint8_t data);
int circular_buffer_pop(CircularBuffer *cb, uint8_t *data);
int circular_buffer_read(CircularBuffer *cb, uint8_t *data, uint32_t len);
uint32_t circular_buffer_available(CircularBuffer *cb);
uint32_t circular_buffer_get_data_count(CircularBuffer *cb);

#endif
