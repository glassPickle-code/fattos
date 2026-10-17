#include "fattos/fattos_timer.h"
#include "fattos/fattos_platform.h"

static FattosTimer *g_timers[FATTOS_MAX_TIMERS];
static size_t g_timer_count;

static void remove_from_registry(FattosTimer *timer)
{
    for (size_t i = 0U; i < g_timer_count; ++i) {
        if (g_timers[i] != timer) {
            continue;
        }
        for (size_t j = i; j + 1U < g_timer_count; ++j) {
            g_timers[j] = g_timers[j + 1U];
        }
        g_timer_count--;
        return;
    }
}

static bool is_registered(const FattosTimer *timer)
{
    for (size_t i = 0U; i < g_timer_count; ++i) {
        if (g_timers[i] == timer) {
            return true;
        }
    }
    return false;
}

FattosStatus fattos_timer_init(FattosTimer *timer, const char *name,
                             FattosTimerFn function, void *context)
{
    if (timer == NULL || function == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    timer->name = name;
    timer->function = function;
    timer->context = context;
    timer->period_ticks = 0U;
    timer->deadline = 0U;
    timer->active = false;
    timer->autoreload = false;
    return FATTOS_OK;
}

FattosStatus fattos_timer_start(FattosTimer *timer, uint32_t period_ms, bool autoreload)
{
    if (timer == NULL || timer->function == NULL || period_ms == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }
    uint32_t key = fattos_platform_irq_lock();
    if (!is_registered(timer)) {
        if (g_timer_count >= FATTOS_MAX_TIMERS) {
            fattos_platform_irq_unlock(key);
            return FATTOS_ERR_FULL;
        }
        g_timers[g_timer_count++] = timer;
    }
    timer->period_ticks = period_ms;
    timer->deadline = fattos_millis() + period_ms;
    timer->autoreload = autoreload;
    timer->active = true;
    fattos_platform_irq_unlock(key);
    return FATTOS_OK;
}

FattosStatus fattos_timer_stop(FattosTimer *timer)
{
    if (timer == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    uint32_t key = fattos_platform_irq_lock();
    timer->active = false;
    remove_from_registry(timer);
    fattos_platform_irq_unlock(key);
    return FATTOS_OK;
}

bool fattos_timer_is_active(const FattosTimer *timer)
{
    return timer != NULL && timer->active;
}

void fattos_timer_service(void)
{
    uint32_t now = fattos_millis();
    /* Fires each due timer exactly once per service call in caller context. */
    for (size_t i = 0U; i < g_timer_count;) {
        FattosTimer *timer = g_timers[i];
        if (!timer->active) {
            remove_from_registry(timer);
            continue;
        }
        if ((int32_t)(now - timer->deadline) < 0) {
            ++i;
            continue;
        }
        if (timer->autoreload) {
            /* Skip missed periods instead of bursting to catch up. */
            uint32_t late = now - timer->deadline;
            timer->deadline += (late / timer->period_ticks + 1U) * timer->period_ticks;
        } else {
            timer->active = false;
            remove_from_registry(timer);
        }
        timer->function(timer->context);
    }
}

static void timer_service_task(void *context)
{
    (void)context;
    fattos_timer_service();
}

FattosStatus fattos_timers_register(uint32_t service_period_ms, uint8_t priority,
                                  uint8_t *out_task_id)
{
    return fattos_task_create("fattos_timers", timer_service_task, NULL,
                             service_period_ms, priority, out_task_id);
}

size_t fattos_timer_count(void)
{
    return g_timer_count;
}
