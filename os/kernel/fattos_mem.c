#include "fattos/fattos_mem.h"
#include "fattos/fattos_platform.h"
#include <string.h>

/* The next pointer lives in the first bytes of each free block. memcpy keeps
   this strict-aliasing-safe and tolerates any pointer-aligned storage. */
static void write_next(uint8_t *block, void *next)
{
    memcpy(block, &next, sizeof(next));
}

static void *read_next(const uint8_t *block)
{
    void *next;
    memcpy(&next, block, sizeof(next));
    return next;
}

FattosStatus fattos_pool_init(FattosPool *pool, void *storage,
                            size_t block_size, size_t block_count)
{
    if (pool == NULL || storage == NULL || block_count == 0U ||
        block_size < sizeof(void *)) {
        return FATTOS_ERR_INVALID_ARG;
    }
    pool->base = (uint8_t *)storage;
    pool->block_size = block_size;
    pool->block_count = block_count;
    pool->free_count = block_count;
    pool->free_list = NULL;
    /* Chain in reverse so the first alloc returns the first block. */
    for (size_t i = block_count; i-- > 0;) {
        uint8_t *block = pool->base + i * block_size;
        write_next(block, pool->free_list);
        pool->free_list = block;
    }
    return FATTOS_OK;
}

void *fattos_pool_alloc(FattosPool *pool)
{
    if (pool == NULL) {
        return NULL;
    }
    uint32_t key = fattos_platform_irq_lock();
    if (pool->free_list == NULL) {
        fattos_platform_irq_unlock(key);
        return NULL;
    }
    uint8_t *block = (uint8_t *)pool->free_list;
    pool->free_list = read_next(block);
    pool->free_count--;
    fattos_platform_irq_unlock(key);
    return block;
}

void fattos_pool_free(FattosPool *pool, void *block)
{
    if (pool == NULL || block == NULL) {
        return;
    }
    uint8_t *candidate = (uint8_t *)block;
    if (candidate < pool->base ||
        candidate >= pool->base + pool->block_size * pool->block_count) {
        return; /* Not owned by this pool; ignore rather than corrupt. */
    }
    uint32_t key = fattos_platform_irq_lock();
    write_next(candidate, pool->free_list);
    pool->free_list = candidate;
    pool->free_count++;
    fattos_platform_irq_unlock(key);
}

size_t fattos_pool_free_count(const FattosPool *pool)
{
    return pool != NULL ? pool->free_count : 0U;
}
