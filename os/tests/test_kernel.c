#include <assert.h>
#include <stdio.h>
#include "fattos/fattos.h"
#include "fattos/fattos_event.h"
#include "fattos/fattos_mem.h"
#include "fattos/fattos_platform.h"
#include "fattos/fattos_queue.h"
#include "fattos/fattos_timer.h"
#include "fattos/fattos_watchdog.h"
#include "fattos_host.h"
#include <stdlib.h>

/* On Windows the debug-CRT assert can block on a dialog, hanging automated
   runs. Redefine assert so a failed check always prints and exits. */
#undef assert
#define assert(cond)                                                            \
    do {                                                                        \
        if (!(cond)) {                                                          \
            fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__,    \
                    #cond);                                                     \
            exit(1);                                                            \
        }                                                                       \
    } while (0)

/* ---- Queue -------------------------------------------------------------- */
static void test_queue(void)
{
    FattosQueue queue;
    uint32_t storage[4];
    assert(fattos_queue_init(&queue, storage, sizeof(uint32_t), 4U) == FATTOS_OK);
    assert(fattos_queue_is_empty(&queue));
    assert(!fattos_queue_is_full(&queue));

    for (uint32_t i = 1U; i <= 4U; ++i) {
        assert(fattos_queue_send(&queue, &i) == FATTOS_OK);
    }
    assert(fattos_queue_is_full(&queue));
    uint32_t overflow = 99U;
    assert(fattos_queue_send(&queue, &overflow) == FATTOS_ERR_FULL);
    assert(fattos_queue_count(&queue) == 4U);

    uint32_t peeked = 0U;
    assert(fattos_queue_peek(&queue, &peeked) == FATTOS_OK && peeked == 1U);
    assert(fattos_queue_count(&queue) == 4U); /* peek must not consume */

    for (uint32_t expected = 1U; expected <= 4U; ++expected) {
        uint32_t value = 0U;
        assert(fattos_queue_receive(&queue, &value) == FATTOS_OK);
        assert(value == expected); /* FIFO order */
    }
    uint32_t underflow = 0U;
    assert(fattos_queue_receive(&queue, &underflow) == FATTOS_ERR_EMPTY);
    assert(fattos_queue_is_empty(&queue));
}

/* ---- Event flags -------------------------------------------------------- */
static void test_events(void)
{
    FattosEventGroup group;
    fattos_event_init(&group);
    assert(!fattos_event_poll(&group, 0x1U, false));

    fattos_event_set(&group, 0x5U);
    assert(fattos_event_get(&group) == 0x5U);
    assert(fattos_event_poll(&group, 0x1U, false));  /* any: bit0 set */
    assert(!fattos_event_poll(&group, 0x2U, false)); /* any: bit1 clear */
    assert(!fattos_event_poll(&group, 0x3U, true));  /* all: bit1 clear */
    fattos_event_set(&group, 0x2U);
    assert(fattos_event_poll(&group, 0x3U, true)); /* all: bits0+1 set */

    fattos_event_clear(&group, 0x1U);
    assert(fattos_event_get(&group) == 0x6U); /* 0x7 minus bit0 */
}

/* ---- Memory pool -------------------------------------------------------- */
static void test_pool(void)
{
    FattosPool pool;
    uint64_t storage[4][4]; /* 32-byte blocks, pointer-aligned */
    assert(fattos_pool_init(&pool, storage, sizeof(storage[0]), 4U) == FATTOS_OK);
    assert(fattos_pool_free_count(&pool) == 4U);

    void *blocks[4];
    for (int i = 0; i < 4; ++i) {
        blocks[i] = fattos_pool_alloc(&pool);
        assert(blocks[i] != NULL);
    }
    assert(fattos_pool_alloc(&pool) == NULL); /* exhausted */
    assert(fattos_pool_free_count(&pool) == 0U);

    fattos_pool_free(&pool, blocks[2]);
    assert(fattos_pool_free_count(&pool) == 1U);
    void *reused = fattos_pool_alloc(&pool);
    assert(reused == blocks[2]); /* LIFO reuse */

    for (int i = 0; i < 4; ++i) {
        fattos_pool_free(&pool, blocks[i]);
    }
    assert(fattos_pool_free_count(&pool) == 4U);
}

/* ---- Software timers ---------------------------------------------------- */
static int timer_a_fires;
static int timer_b_fires;
static int timer_c_fires;
static uint8_t timer_task_id;

static void timer_a_cb(void *ctx) { (void)ctx; timer_a_fires++; }
static void timer_b_cb(void *ctx) { (void)ctx; timer_b_fires++; }
static void timer_c_cb(void *ctx) { (void)ctx; timer_c_fires++; }

static void test_timers(void)
{
    FattosTimer timer_a;
    FattosTimer timer_b;
    assert(fattos_timer_init(&timer_a, "a", timer_a_cb, NULL) == FATTOS_OK);
    assert(fattos_timer_init(&timer_b, "b", timer_b_cb, NULL) == FATTOS_OK);
    assert(fattos_timer_start(&timer_a, 10U, false) == FATTOS_OK); /* one-shot */
    assert(fattos_timer_start(&timer_b, 5U, true) == FATTOS_OK);   /* reload */
    assert(fattos_timer_count() == 2U);

    fattos_host_advance(5U); /* t=5 */
    fattos_timer_service();
    assert(timer_a_fires == 0 && timer_b_fires == 1);

    fattos_host_advance(5U); /* t=10 */
    fattos_timer_service();
    assert(timer_a_fires == 1 && timer_b_fires == 2);

    fattos_host_advance(5U); /* t=15 */
    fattos_timer_service();
    assert(timer_a_fires == 1); /* one-shot stays quiet */
    assert(timer_b_fires == 3);
    assert(!fattos_timer_is_active(&timer_a));
    assert(fattos_timer_count() == 1U); /* one-shot unregistered itself */

    assert(fattos_timer_stop(&timer_b) == FATTOS_OK);
    fattos_host_advance(5U);
    fattos_timer_service();
    assert(timer_b_fires == 3); /* stopped timer never fires */
    assert(fattos_timer_count() == 0U);

    /* Service task integration: fires through the scheduler. */
    FattosTimer timer_c;
    assert(fattos_timers_register(5U, 1U, &timer_task_id) == FATTOS_OK);
    assert(fattos_timer_init(&timer_c, "c", timer_c_cb, NULL) == FATTOS_OK);
    assert(fattos_timer_start(&timer_c, 10U, false) == FATTOS_OK);
    fattos_host_advance(10U);
    bool ran = false;
    assert(fattos_run_once(&ran) == FATTOS_OK && ran);
    assert(timer_c_fires == 1);
    (void)fattos_timer_stop(&timer_c);
}

/* ---- Watchdog ----------------------------------------------------------- */
static void test_watchdog(void)
{
    uint8_t id_a = 0xFFU;
    uint8_t id_b = 0xFFU;
    assert(fattos_wdt_register("a", 50U, &id_a) == FATTOS_OK);
    assert(fattos_wdt_register("b", 80U, &id_b) == FATTOS_OK);
    assert(fattos_wdt_count() == 2U);
    assert(fattos_wdt_is_healthy());
    assert(fattos_wdt_kick_hardware() == FATTOS_OK);

    (void)fattos_wdt_feed(id_a);
    fattos_host_advance(49U);
    assert(fattos_wdt_poll() == FATTOS_OK); /* within 50 ms of last feed */

    fattos_host_advance(2U); /* past item a's timeout, before b's */
    assert(fattos_wdt_poll() == FATTOS_ERR_TIMEOUT);
    assert(!fattos_wdt_is_healthy());
    assert(fattos_wdt_fault_id() == id_a);
    assert(fattos_wdt_kick_hardware() == FATTOS_ERR_TIMEOUT); /* starve HW WDT */

    (void)fattos_wdt_feed(id_a);
    fattos_host_advance(70U); /* now item b is stale too; feed both */
    (void)fattos_wdt_poll();
    (void)fattos_wdt_feed(id_a);
    (void)fattos_wdt_feed(id_b);
    assert(fattos_wdt_poll() == FATTOS_OK);
    assert(fattos_wdt_kick_hardware() == FATTOS_OK);

    FattosWdtItem info;
    assert(fattos_wdt_get_info(id_b, &info) == FATTOS_OK);
    assert(info.timeout_ticks == 80U);
    assert(fattos_wdt_get_info(99U, &info) == FATTOS_ERR_INVALID_ARG);
}

/* ---- Task profiling ----------------------------------------------------- */
static uint8_t g_spinner_id;
static int g_fault_calls;
static uint8_t g_fault_task;

static void record_fault(uint8_t task_id, uint32_t exec_ticks,
                         uint32_t period_ticks)
{
    (void)exec_ticks;
    (void)period_ticks;
    g_fault_calls++;
    g_fault_task = task_id;
}

static void spinner_task(void *context)
{
    (void)context;
    fattos_host_advance(3U); /* simulate a task that burns 3 ticks */
}

static void test_task_profiling(void)
{
    assert(fattos_task_create("spinner", spinner_task, NULL, 2U, 7U,
                             &g_spinner_id) == FATTOS_OK);
    fattos_set_fault_hook(record_fault);

    fattos_host_advance(1U);
    bool ran = false;
    assert(fattos_run_once(&ran) == FATTOS_OK && ran);

    FattosTaskInfo info;
    assert(fattos_task_get_info(g_spinner_id, &info) == FATTOS_OK);
    assert(info.exec_last_ticks == 3U);
    assert(info.exec_max_ticks == 3U);
    assert(info.overrun_count == 1U); /* 3 ms ran > 2 ms period */
    assert(info.missed_releases == 0U);
    assert(info.jitter_max_ticks == 1U); /* dispatched 1 tick after due */

    /* The overrun fired the fault hook (safety-first escalation path). */
    assert(g_fault_calls == 1);
    assert(g_fault_task == g_spinner_id);

    /* Starve the scheduler: releases accumulate while ticks advance. */
    fattos_host_advance(30U);
    assert(fattos_run_once(&ran) == FATTOS_OK && ran);
    assert(fattos_task_get_info(g_spinner_id, &info) == FATTOS_OK);
    assert(info.missed_releases > 0U);
    /* Lateness at second dispatch: now T+34 (1 + 3 burned in the callback
       + 30 idle) against the scheduled release T+2 -> 32 ticks. */
    assert(info.jitter_max_ticks == 32U);

    fattos_set_fault_hook(NULL);
}

/* ---- Fairness and bounded drain ---------------------------------------- */
static int rr_a_runs;
static int rr_b_runs;

static void rr_a_task(void *ctx) { (void)ctx; rr_a_runs++; }
static void rr_b_task(void *ctx) { (void)ctx; rr_b_runs++; }

static void test_fairness_and_drain(void)
{
    /* Silence every other registered task so only the pair is due. */
    assert(fattos_task_enable(g_spinner_id, false) == FATTOS_OK);
    assert(fattos_task_enable(timer_task_id, false) == FATTOS_OK);

    uint8_t id_a = 0U;
    uint8_t id_b = 0U;
    assert(fattos_task_create("rr_a", rr_a_task, NULL, 1000U, 2U, &id_a) ==
           FATTOS_OK);
    assert(fattos_task_create("rr_b", rr_b_task, NULL, 1000U, 2U, &id_b) ==
           FATTOS_OK);

    fattos_host_advance(1U);
    uint32_t dispatched = 0U;
    assert(fattos_run_pending(8U, &dispatched) == FATTOS_OK);
    /* Equal priorities: both due tasks run — neither is starved by index. */
    assert(dispatched == 2U);
    assert(rr_a_runs == 1 && rr_b_runs == 1);

    /* Bounded drain: nothing due now, and the budget was never hit. */
    dispatched = 99U;
    assert(fattos_run_pending(8U, &dispatched) == FATTOS_OK);
    assert(dispatched == 0U);

    /* A zero budget is a legal no-op, not an error. */
    assert(fattos_run_pending(0U, NULL) == FATTOS_OK);
}

int main(void)
{
    assert(fattos_init() == FATTOS_OK);

    test_queue();
    test_events();
    test_pool();
    test_timers();
    test_watchdog();
    test_task_profiling();
    test_fairness_and_drain();

    puts("FattOS kernel tests passed");
    return 0;
}
