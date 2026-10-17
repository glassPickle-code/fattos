#include "fattos/fattos.h"
#include "fattos/fattos_platform.h"

static volatile uint32_t g_ticks;
static FattosTaskInfo g_tasks[FATTOS_MAX_TASKS];
static size_t g_task_count;
static size_t g_scan_cursor; /* round-robin tie breaker among equal priority */
static bool g_scheduler_initialized;
static FattosFaultFn g_fault_hook;

/* Implemented in fattos.c. */
FattosStatus fattos_scheduler_init_internal(void);

uint32_t fattos_ms_to_ticks(uint32_t ms)
{
    if (ms == 0U) {
        return 0U;
    }
    uint64_t ticks = ((uint64_t)ms * (uint64_t)FATTOS_TICK_HZ + 999U) / 1000U;
    if (ticks == 0U) {
        ticks = 1U; /* a non-zero duration is never sub-tick */
    }
    if (ticks > 0xFFFFFFFFULL) {
        ticks = 0xFFFFFFFFULL;
    }
    return (uint32_t)ticks;
}

FattosStatus fattos_scheduler_init_internal(void)
{
    g_ticks = 0U;
    g_task_count = 0U;
    g_scan_cursor = 0U;
    g_fault_hook = NULL;
    g_scheduler_initialized = true;
    for (size_t i = 0U; i < FATTOS_MAX_TASKS; ++i) {
        g_tasks[i].name = NULL;
        g_tasks[i].function = NULL;
        g_tasks[i].context = NULL;
        g_tasks[i].period_ticks = 0U;
        g_tasks[i].next_release = 0U;
        g_tasks[i].run_count = 0U;
        g_tasks[i].priority = 0U;
        g_tasks[i].enabled = false;
        g_tasks[i].exec_last_ticks = 0U;
        g_tasks[i].exec_max_ticks = 0U;
        g_tasks[i].missed_releases = 0U;
        g_tasks[i].overrun_count = 0U;
        g_tasks[i].jitter_max_ticks = 0U;
        g_tasks[i].overrun_streak = 0U;
        g_tasks[i].quarantined = false;
    }
    return FATTOS_OK;
}

void fattos_set_fault_hook(FattosFaultFn hook)
{
    uint32_t key = fattos_platform_irq_lock();
    g_fault_hook = hook;
    fattos_platform_irq_unlock(key);
}

uint32_t fattos_millis(void)
{
    /*
     * Tick counter may run at any FATTOS_TICK_HZ; the public clock is always
     * milliseconds. On 32-bit MCU targets an aligned 32-bit read is atomic;
     * ports where it is not must guard this in fattos_platform_irq_lock().
     */
    uint32_t ticks = g_ticks;
    return (uint32_t)(((uint64_t)ticks * 1000ULL) / (uint64_t)FATTOS_TICK_HZ);
}

void fattos_tick_isr(void)
{
    g_ticks++;
}

FattosStatus fattos_task_create(const char *name,
                              FattosTaskFn function,
                              void *context,
                              uint32_t period_ms,
                              uint8_t priority,
                              uint8_t *out_task_id)
{
    if (!g_scheduler_initialized) {
        return FATTOS_ERR_NOT_INITIALIZED;
    }
    if (function == NULL || period_ms == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }
    if (g_task_count >= FATTOS_MAX_TASKS) {
        return FATTOS_ERR_NO_TASK_SLOTS;
    }

    uint32_t period_ticks = fattos_ms_to_ticks(period_ms);
    if (period_ticks == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }

    size_t id = g_task_count++;
    g_tasks[id].name = name;
    g_tasks[id].function = function;
    g_tasks[id].context = context;
    g_tasks[id].period_ticks = period_ticks;
    g_tasks[id].next_release = g_ticks;
    g_tasks[id].run_count = 0U;
    g_tasks[id].priority = priority;
    g_tasks[id].enabled = true;
    g_tasks[id].exec_last_ticks = 0U;
    g_tasks[id].exec_max_ticks = 0U;
    g_tasks[id].missed_releases = 0U;
    g_tasks[id].overrun_count = 0U;
    g_tasks[id].jitter_max_ticks = 0U;
    g_tasks[id].overrun_streak = 0U;
    g_tasks[id].quarantined = false;

    if (out_task_id != NULL) {
        *out_task_id = (uint8_t)id;
    }
    return FATTOS_OK;
}

FattosStatus fattos_task_enable(uint8_t task_id, bool enabled)
{
    if ((size_t)task_id >= g_task_count) {
        return FATTOS_ERR_INVALID_ARG;
    }
    /* SAFETY-FIRST: a quarantined task stays down. Only an explicit
       rehabilitation says "we looked at it, it is fixed". */
    if (enabled && g_tasks[task_id].quarantined) {
        return FATTOS_ERR_FAULT;
    }
    g_tasks[task_id].enabled = enabled;
    if (enabled) {
        /* Rejoining tasks start fresh instead of replaying a backlog. */
        g_tasks[task_id].next_release = g_ticks;
    }
    return FATTOS_OK;
}

size_t fattos_task_count(void)
{
    return g_task_count;
}

FattosStatus fattos_task_get_info(uint8_t task_id, FattosTaskInfo *out_info)
{
    if (out_info == NULL || (size_t)task_id >= g_task_count) {
        return FATTOS_ERR_INVALID_ARG;
    }
    uint32_t key = fattos_platform_irq_lock();
    *out_info = g_tasks[task_id];
    fattos_platform_irq_unlock(key);
    return FATTOS_OK;
}

/* Select the highest-priority due task; equal priorities rotate through the
   scan so no task is systematically executed last every cycle. */
static int select_due_task(uint32_t now)
{
    if (g_task_count == 0U) {
        return -1;
    }
    size_t start = g_scan_cursor % g_task_count;
    int selected = -1;
    uint8_t best_priority = 0U;

    for (size_t k = 0U; k < g_task_count; ++k) {
        size_t i = (start + k) % g_task_count;
        FattosTaskInfo *task = &g_tasks[i];
        if (!task->enabled || task->function == NULL ||
            (int32_t)(now - task->next_release) < 0) {
            continue;
        }
        if (selected < 0 || task->priority > best_priority) {
            selected = (int)i;
            best_priority = task->priority;
        }
    }
    if (selected >= 0) {
        g_scan_cursor = (size_t)selected + 1U;
    }
    return selected;
}

FattosStatus fattos_run_once(bool *ran)
{
    if (!g_scheduler_initialized) {
        return FATTOS_ERR_NOT_INITIALIZED;
    }
    if (ran == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }

    *ran = false;
    int selected = select_due_task(g_ticks);
    if (selected < 0) {
        return FATTOS_OK;
    }

    FattosTaskInfo *task = &g_tasks[selected];
    uint32_t elapsed = g_ticks - task->next_release;
    /* Release jitter: lateness between when this task was due and when it
       actually dispatched. On target this is your scheduling-latency WCET. */
    if (elapsed > task->jitter_max_ticks) {
        task->jitter_max_ticks = elapsed;
    }
    uint32_t releases = elapsed / task->period_ticks + 1U;
    task->next_release += releases * task->period_ticks;
    if (releases > 1U) {
        task->missed_releases += releases - 1U;
    }
    task->run_count++;

    uint32_t start = g_ticks;
    /* Callbacks must be bounded and non-blocking; no preemption is provided. */
    task->function(task->context);
    uint32_t exec = g_ticks - start;

    task->exec_last_ticks = exec;
    if (exec > task->exec_max_ticks) {
        task->exec_max_ticks = exec;
    }
    if (exec > task->period_ticks) {
        task->overrun_count++;
        task->overrun_streak++;
        FattosFaultFn hook = g_fault_hook;
        if (hook != NULL) {
            hook((uint8_t)selected, exec, task->period_ticks);
        }
        /* Sustained overruns: quarantine — disable the offender so the
           remaining tasks keep their deadlines, record why, and keep the
           counters. One unbounded callback must never sink the ship. */
        if (task->overrun_streak >= FATTOS_QUARANTINE_LIMIT &&
            !task->quarantined) {
            task->quarantined = true;
            task->enabled = false;
        }
    } else {
        /* A clean run ends the streak: a transient spike is forgiven, only
           sustained misbehavior quarantines. */
        task->overrun_streak = 0U;
    }
    *ran = true;
    return FATTOS_OK;
}

bool fattos_task_is_quarantined(uint8_t task_id)
{
    if ((size_t)task_id >= g_task_count) {
        return false;
    }
    return g_tasks[task_id].quarantined;
}

size_t fattos_task_quarantined_count(void)
{
    size_t n = 0U;
    for (size_t i = 0U; i < g_task_count; ++i) {
        if (g_tasks[i].quarantined) {
            n++;
        }
    }
    return n;
}

FattosStatus fattos_task_rehab(uint8_t task_id)
{
    if ((size_t)task_id >= g_task_count) {
        return FATTOS_ERR_INVALID_ARG;
    }
    FattosTaskInfo *task = &g_tasks[task_id];
    if (!task->quarantined) {
        return FATTOS_OK; /* nothing to release */
    }
    task->quarantined = false;
    task->overrun_streak = 0U;
    task->enabled = true;
    task->next_release = g_ticks; /* rejoin fresh, no backlog replay */
    return FATTOS_OK;
}

FattosStatus fattos_run_pending(uint32_t max_dispatches, uint32_t *dispatched)
{
    uint32_t count = 0U;
    if (dispatched != NULL) {
        *dispatched = 0U;
    }
    if (!g_scheduler_initialized) {
        return FATTOS_ERR_NOT_INITIALIZED;
    }
    /* Bounded drain: a caller can never be spun forever by a broken app. */
    while (count < max_dispatches) {
        bool ran = false;
        if (fattos_run_once(&ran) != FATTOS_OK) {
            break;
        }
        if (!ran) {
            break;
        }
        count++;
    }
    if (dispatched != NULL) {
        *dispatched = count;
    }
    return FATTOS_OK;
}

_Noreturn void fattos_run(void)
{
    for (;;) {
        bool ran = false;
        (void)fattos_run_once(&ran);
        if (!ran) {
            fattos_platform_idle();
        }
    }
}