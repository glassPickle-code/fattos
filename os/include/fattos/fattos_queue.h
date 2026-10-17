#ifndef FATTOS_QUEUE_H
#define FATTOS_QUEUE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Fixed-capacity, copy-in/copy-out message queue backed by caller-provided
 * storage. Safe for producer/consumer across an interrupt boundary when the
 * platform implements fattos_platform_irq_lock()/unlock() for real.
 * Never allocates. Send fails with FATTOS_ERR_FULL, receive with FATTOS_ERR_EMPTY.
 */
typedef struct {
    uint8_t *storage;
    size_t item_size;
    size_t capacity;
    size_t head;  /* dequeue index */
    size_t tail;  /* enqueue index */
    size_t count;
} FattosQueue;

FattosStatus fattos_queue_init(FattosQueue *queue, void *storage,
                             size_t item_size, size_t capacity);
FattosStatus fattos_queue_send(FattosQueue *queue, const void *item);
FattosStatus fattos_queue_receive(FattosQueue *queue, void *out_item);
FattosStatus fattos_queue_peek(const FattosQueue *queue, void *out_item);
size_t fattos_queue_count(const FattosQueue *queue);
bool fattos_queue_is_empty(const FattosQueue *queue);
bool fattos_queue_is_full(const FattosQueue *queue);

#ifdef __cplusplus
}
#endif

#endif
