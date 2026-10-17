#ifndef FATTOS_H
#define FATTOS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fattos_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FATTOS_OK = 0,
    FATTOS_ERR_INVALID_ARG,
    FATTOS_ERR_NOT_INITIALIZED,
    FATTOS_ERR_NO_TASK_SLOTS,
    FATTOS_ERR_PLATFORM,
    FATTOS_ERR_NOT_IMPLEMENTED,
    FATTOS_ERR_TIMEOUT,
    FATTOS_ERR_FULL,
    FATTOS_ERR_EMPTY,
    FATTOS_ERR_FAULT /* safety supervisor refused the request (latched/timeout) */
} FattosStatus;

typedef void (*FattosTaskFn)(void *context);

typedef struct {
    const char *name;
    FattosTaskFn function;
    void *context;
    uint32_t period_ticks;
    uint32_t next_release;
    uint32_t run_count;
    uint8_t priority;
    bool enabled;
    /* Execution profiling, measured at tick resolution. */
    uint32_t exec_last_ticks;
    uint32_t exec_max_ticks;
    uint32_t missed_releases;
    uint32_t overrun_count;
    uint32_t jitter_max_ticks; /* worst lateness between due time and dispatch */
    /* Hardening: consecutive overruns since the last clean run, and the
       quarantine flag set once that streak hit FATTOS_QUARANTINE_LIMIT. */
    uint32_t overrun_streak;
    bool quarantined;
} FattosTaskInfo;

/* Initialize platform and scheduler state. Call once before task registration. */
FattosStatus fattos_init(void);

/* Register a periodic callback. Higher numeric priority wins among due tasks. */
FattosStatus fattos_task_create(const char *name,
                              FattosTaskFn function,
                              void *context,
                              uint32_t period_ms,
                              uint8_t priority,
                              uint8_t *out_task_id);

/* Call exactly FATTOS_TICK_HZ times per second from a hardware timer ISR. */
void fattos_tick_isr(void);

/* Execute one due task, if any. No due task is a normal condition:
   FATTOS_OK with *ran = false. One callback per call, always. */
FattosStatus fattos_run_once(bool *ran);

/* Run at most max_dispatches due callbacks (priority order) and report how
   many ran in *dispatched (may be NULL). The bound guarantees a drain loop
   can never spin forever, so a runaway callback cannot freeze the caller. */
FattosStatus fattos_run_pending(uint32_t max_dispatches, uint32_t *dispatched);

/* Run scheduler forever. Platform idle hook may sleep until next interrupt. */
_Noreturn void fattos_run(void);

/* Auto-quarantine state: true once the task was disabled for a sustained
   overrun streak. Half of "turn limitations into strengths": instead of a
   slow death by jitter, the scheduler isolates the offenders itself. */
bool fattos_task_is_quarantined(uint8_t task_id);
/* Number of tasks currently quarantined (diagnostics/telemetry). */
size_t fattos_task_quarantined_count(void);
/* Release a quarantined task back to duty after it was inspected/fixed.
   Clears the streak and re-enables it. Never re-enables a task that was
   merely disabled by the app (that stays disabled until it asks). */
FattosStatus fattos_task_rehab(uint8_t task_id);

uint32_t fattos_millis(void);

/* Convert a millisecond duration to ticks at FATTOS_TICK_HZ (rounds up,
   never returns 0 for a non-zero duration). Public durations are always ms. */
uint32_t fattos_ms_to_ticks(uint32_t ms);

FattosStatus fattos_task_enable(uint8_t task_id, bool enabled);
size_t fattos_task_count(void);
FattosStatus fattos_task_get_info(uint8_t task_id, FattosTaskInfo *out_info);

/* Called from the scheduler right after a callback ran longer than its
   period. Use it (safety-first) to latch an e-stop or raise an alarm; it
   runs in task context with interrupts still enabled. */
typedef void (*FattosFaultFn)(uint8_t task_id, uint32_t exec_ticks,
                             uint32_t period_ticks);
void fattos_set_fault_hook(FattosFaultFn hook);

/* Optional platform hook called when no task is due. */
void fattos_platform_idle(void);

#ifdef __cplusplus
}
#endif

#endif
