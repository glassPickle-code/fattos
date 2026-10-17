#ifndef FATTOS_TIMER_H
#define FATTOS_TIMER_H

#include <stdbool.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Static software timers (one-shot or auto-reload) driven by
 * fattos_timer_service(). Timers fire in the context of whichever task
 * calls fattos_timer_service(); keep timer callbacks short and non-blocking.
 * Register the service task with fattos_timers_register() once after
 * fattos_init().
 */
typedef void (*FattosTimerFn)(void *context);

typedef struct FattosTimer {
    const char *name;
    FattosTimerFn function;
    void *context;
    uint32_t period_ticks;
    uint32_t deadline;
    bool active;
    bool autoreload;
} FattosTimer;

FattosStatus fattos_timer_init(FattosTimer *timer, const char *name,
                             FattosTimerFn function, void *context);
FattosStatus fattos_timer_start(FattosTimer *timer, uint32_t period_ms, bool autoreload);
FattosStatus fattos_timer_stop(FattosTimer *timer);
bool fattos_timer_is_active(const FattosTimer *timer);

/* Fire all expired timers. Call from a periodic task. */
void fattos_timer_service(void);

/* Convenience: create the timer service task (period 1 ms granularity). */
FattosStatus fattos_timers_register(uint32_t service_period_ms, uint8_t priority,
                                  uint8_t *out_task_id);
size_t fattos_timer_count(void);

#ifdef __cplusplus
}
#endif

#endif
