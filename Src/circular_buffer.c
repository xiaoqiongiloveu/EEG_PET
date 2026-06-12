#include "circular_buffer.h"

/* 静态分配的内部缓冲区，最大容量由 CIRCULAR_BUFFER_MAX_SIZE 决定 */
static uint8_t internal_buffer[CIRCULAR_BUFFER_MAX_SIZE];

/**
 * @brief  初始化环形缓冲区
 * @param  cb: 指向 CircularBuffer 结构体的指针
 * @param  size: 期望的缓冲区大小
 * @note   如果传入的 size 大于 CIRCULAR_BUFFER_MAX_SIZE，将被限制为最大允许值
 */
void circular_buffer_init(CircularBuffer *cb, uint32_t size) {
    cb->data = internal_buffer;
    cb->size = (size > CIRCULAR_BUFFER_MAX_SIZE) ? CIRCULAR_BUFFER_MAX_SIZE : size;
    cb->head = 0;
    cb->tail = 0;
}

/**
 * @brief  清空环形缓冲区
 * @param  cb: 指向 CircularBuffer 结构体的指针
 * @note   仅重置头尾指针，不清除实际内存数据
 */
void circular_buffer_clear(CircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
}

/**
 * @brief  向环形缓冲区压入一个字节的数据
 * @param  cb: 指向 CircularBuffer 结构体的指针
 * @param  data: 要压入的字节数据
 * @retval 0: 成功; -1: 失败 (缓冲区已满)
 */
int circular_buffer_push(CircularBuffer *cb, uint8_t data) {
    uint32_t next_head = (cb->head + 1) % cb->size;
    if (next_head == cb->tail) {
        return -1; // Buffer full (缓冲区已满)
    }
    cb->data[cb->head] = data;
    cb->head = next_head;
    return 0;
}

/**
 * @brief  从环形缓冲区弹出一个字节的数据
 * @param  cb: 指向 CircularBuffer 结构体的指针
 * @param  data: 用于接收弹出数据的指针
 * @retval 0: 成功; -1: 失败 (缓冲区为空)
 */
int circular_buffer_pop(CircularBuffer *cb, uint8_t *data) {
    if (cb->head == cb->tail) {
        return -1; // Buffer empty (缓冲区为空)
    }
    *data = cb->data[cb->tail];
    cb->tail = (cb->tail + 1) % cb->size;
    return 0;
}

/**
 * @brief  从环形缓冲区连续读取多个字节的数据
 * @param  cb: 指向 CircularBuffer 结构体的指针
 * @param  data: 用于接收读取数据的缓冲区指针
 * @param  len: 期望读取的字节长度
 * @retval 0: 成功; -1: 失败 (缓冲区内的数据量不足)
 */
int circular_buffer_read(CircularBuffer *cb, uint8_t *data, uint32_t len) {
    if (circular_buffer_available(cb) < len) {
        return -1; // Not enough data (数据量不足)
    }
    for (uint32_t i = 0; i < len; i++) {
        circular_buffer_pop(cb, &data[i]);
    }
    return 0;
}

/**
 * @brief  获取环形缓冲区中当前可用的数据字节数
 * @param  cb: 指向 CircularBuffer 结构体的指针
 * @retval 当前有效数据的字节数
 */
uint32_t circular_buffer_available(CircularBuffer *cb) {
    if (cb->head >= cb->tail) {
        return cb->head - cb->tail;
    } else {
        return cb->size + cb->head - cb->tail;
    }
}

/**
 * @brief  获取环形缓冲区中当前可用的数据字节数 (与 available 功能相同)
 * @param  cb: 指向 CircularBuffer 结构体的指针
 * @retval 当前有效数据的字节数
 */
uint32_t circular_buffer_get_data_count(CircularBuffer *cb) {
    return circular_buffer_available(cb);
}
