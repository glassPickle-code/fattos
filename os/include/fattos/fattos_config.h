#ifndef FATTOS_CONFIG_H
#define FATTOS_CONFIG_H

#include <stdint.h>

#ifndef FATTOS_MAX_TASKS
#define FATTOS_MAX_TASKS 8U
#endif

#ifndef FATTOS_TICK_HZ
#define FATTOS_TICK_HZ 1000U
#endif

#ifndef FATTOS_TENSOR_ARENA_SIZE
#define FATTOS_TENSOR_ARENA_SIZE 16384U
#endif

#ifndef FATTOS_TENSOR_ARENA_ALIGNMENT
#define FATTOS_TENSOR_ARENA_ALIGNMENT 16U
#endif

#ifndef FATTOS_MAX_TIMERS
#define FATTOS_MAX_TIMERS 8U
#endif

#ifndef FATTOS_MAX_WDT_ITEMS
#define FATTOS_MAX_WDT_ITEMS 8U
#endif

/* Scheduler hardening: a task that overruns its period this many times in
   a row is auto-quarantined (disabled) so one runaway cannot sink the
   whole system's real-time behavior. Rehab with fattos_task_rehab(). */
#ifndef FATTOS_QUARANTINE_LIMIT
#define FATTOS_QUARANTINE_LIMIT 3U
#endif

#if FATTOS_MAX_TIMERS == 0
#error "FATTOS_MAX_TIMERS must be greater than zero"
#endif

#if FATTOS_MAX_WDT_ITEMS == 0
#error "FATTOS_MAX_WDT_ITEMS must be greater than zero"
#endif

#if FATTOS_MAX_TASKS == 0
#error "FATTOS_MAX_TASKS must be greater than zero"
#endif

#if FATTOS_TICK_HZ == 0
#error "FATTOS_TICK_HZ must be greater than zero"
#endif

#if FATTOS_QUARANTINE_LIMIT == 0
#error "FATTOS_QUARANTINE_LIMIT must be greater than zero"
#endif

#endif
