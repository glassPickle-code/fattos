#ifndef FATTOS_EVENT_H
#define FATTOS_EVENT_H

#include <stdbool.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 32-bit event flag groups for cooperative task coordination.
 * Non-blocking by design (a cooperative scheduler cannot block a task):
 * tasks poll with fattos_event_poll() and continue when it returns false.
 */
typedef struct {
    volatile uint32_t flags;
} FattosEventGroup;

void fattos_event_init(FattosEventGroup *group);
void fattos_event_set(FattosEventGroup *group, uint32_t bits);
void fattos_event_clear(FattosEventGroup *group, uint32_t bits);
uint32_t fattos_event_get(const FattosEventGroup *group);

/* True when any (wait_all=false) or all (wait_all=true) bits are set. */
bool fattos_event_poll(const FattosEventGroup *group, uint32_t bits, bool wait_all);

#ifdef __cplusplus
}
#endif

#endif
