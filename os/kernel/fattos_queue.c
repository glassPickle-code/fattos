#include "fattos/fattos_queue.h"
#include "fattos/fattos_platform.h"
#include <string.h>

FattosStatus fattos_queue_init(FattosQueue *queue, void *storage,
                             size_t item_size, size_t capacity)
{
    if (queue == NULL || storage == NULL || item_size == 0U || capacity == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }
    queue->storage = (uint8_t *)storage;
    queue->item_size = item_size;
    queue->capacity = capacity;
    queue->head = 0U;
    queue->tail = 0U;
    queue->count = 0U;
    return FATTOS_OK;
}

FattosStatus fattos_queue_send(FattosQueue *queue, const void *item)
{
    if (queue == NULL || item == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    uint32_t key = fattos_platform_irq_lock();
    if (queue->count >= queue->capacity) {
        fattos_platform_irq_unlock(key);
        return FATTOS_ERR_FULL;
    }
    memcpy(queue->storage + queue->tail * queue->item_size, item, queue->item_size);
    queue->tail = (queue->tail + 1U) % queue->capacity;
    queue->count++;
    fattos_platform_irq_unlock(key);
    return FATTOS_OK;
}

FattosStatus fattos_queue_receive(FattosQueue *queue, void *out_item)
{
    if (queue == NULL || out_item == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    uint32_t key = fattos_platform_irq_lock();
    if (queue->count == 0U) {
        fattos_platform_irq_unlock(key);
        return FATTOS_ERR_EMPTY;
    }
    memcpy(out_item, queue->storage + queue->head * queue->item_size, queue->item_size);
    queue->head = (queue->head + 1U) % queue->capacity;
    queue->count--;
    fattos_platform_irq_unlock(key);
    return FATTOS_OK;
}

FattosStatus fattos_queue_peek(const FattosQueue *queue, void *out_item)
{
    if (queue == NULL || out_item == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    /* Const view of a shared queue: lock against a concurrent consumer. */
    FattosQueue *mutable_queue = (FattosQueue *)queue;
    uint32_t key = fattos_platform_irq_lock();
    if (mutable_queue->count == 0U) {
        fattos_platform_irq_unlock(key);
        return FATTOS_ERR_EMPTY;
    }
    memcpy(out_item, mutable_queue->storage + mutable_queue->head * mutable_queue->item_size,
           mutable_queue->item_size);
    fattos_platform_irq_unlock(key);
    return FATTOS_OK;
}

size_t fattos_queue_count(const FattosQueue *queue)
{
    return queue != NULL ? queue->count : 0U;
}

bool fattos_queue_is_empty(const FattosQueue *queue)
{
    return queue == NULL || queue->count == 0U;
}

bool fattos_queue_is_full(const FattosQueue *queue)
{
    return queue != NULL && queue->count >= queue->capacity;
}
