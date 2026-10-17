#ifndef FATTOS_MEM_H
#define FATTOS_MEM_H

#include <stddef.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Fixed-block memory pool: O(1) alloc/free, no fragmentation, no malloc.
 * Storage is caller-provided; each block must be pointer-aligned and at
 * least sizeof(void*) bytes (the free list is stored in free blocks).
 * Alloc/free use critical sections so an ISR producer and task consumer
 * cannot corrupt the free list.
 */
typedef struct {
    uint8_t *base;
    size_t block_size;
    size_t block_count;
    void *free_list;
    size_t free_count;
} FattosPool;

FattosStatus fattos_pool_init(FattosPool *pool, void *storage,
                            size_t block_size, size_t block_count);
void *fattos_pool_alloc(FattosPool *pool);
void fattos_pool_free(FattosPool *pool, void *block);
size_t fattos_pool_free_count(const FattosPool *pool);

#ifdef __cplusplus
}
#endif

#endif
