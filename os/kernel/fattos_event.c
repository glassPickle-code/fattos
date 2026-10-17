#include "fattos/fattos_event.h"
#include "fattos/fattos_platform.h"

void fattos_event_init(FattosEventGroup *group)
{
    if (group != NULL) {
        group->flags = 0U;
    }
}

void fattos_event_set(FattosEventGroup *group, uint32_t bits)
{
    if (group == NULL) {
        return;
    }
    uint32_t key = fattos_platform_irq_lock();
    group->flags |= bits;
    fattos_platform_irq_unlock(key);
}

void fattos_event_clear(FattosEventGroup *group, uint32_t bits)
{
    if (group == NULL) {
        return;
    }
    uint32_t key = fattos_platform_irq_lock();
    group->flags &= ~bits;
    fattos_platform_irq_unlock(key);
}

uint32_t fattos_event_get(const FattosEventGroup *group)
{
    if (group == NULL) {
        return 0U;
    }
    uint32_t key = fattos_platform_irq_lock();
    uint32_t flags = group->flags;
    fattos_platform_irq_unlock(key);
    return flags;
}

bool fattos_event_poll(const FattosEventGroup *group, uint32_t bits, bool wait_all)
{
    uint32_t flags = fattos_event_get(group);
    if (wait_all) {
        return (flags & bits) == bits;
    }
    return (flags & bits) != 0U;
}
