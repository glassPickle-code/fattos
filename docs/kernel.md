# The FattOS kernel

The kernel is two things: a **clock** and a **cooperative, fixed-priority
scheduler**, plus a set of allocation-free IPC/instrumentation services
built on top of them.

Contents:
- [Concepts](#concepts)
- [The scheduler](#the-scheduler)
- [Task profiling and the fault hook](#task-profiling-and-the-fault-hook)
- [Quarantine: self-healing under overruns](#quarantine-self-healing-under-overruns)
- [Queues](#queues)
- [Software timers](#software-timers)
- [Event flags](#event-flags)
- [Memory pool](#memory-pool)
- [Deadman watchdog](#deadman-watchdog)
- [Critical sections and ISR rules](#critical-sections-and-isr-rules)
- [Configuration knobs](#configuration-knobs)

Header: `fattos/fattos.h`, `fattos/fattos_config.h`, plus
`fattos_queue.h / fattos_timer.h / fattos_event.h / fattos_mem.h / fattos_watchdog.h`.

---

## Concepts

- **Tick** — the scheduler's heartbeat. `FATTOS_TICK_HZ` ticks happen per
  second (default 1000), driven by `fattos_tick_isr()` from a hardware timer
  ISR on target, or `fattos_host_advance(n)` in host tests. Public APIs take
  **milliseconds**; `fattos_ms_to_ticks(ms)` converts with round-up and never
  returns 0 for a non-zero duration.
- **Task** — a periodic callback with a period, a priority, and an optional
  context pointer. Tasks never "block": there is no semaphore to wait on by
  design. A task does its work and returns; if it can't do work this cycle,
  it returns early.
- **Due** — `now >= next_release`. After being dispatched, the task's
  `next_release` advances by whole periods, so a task that is late does not
  replay a burst of missed periods (misses are counted instead).
- **Millisecond clock** — `fattos_millis()` wraps safely at 2^32 ms
  (~49.7 days). Always compare with the `(int32_t)(now - deadline) < 0`
  idiom (the kernel does this internally).

---

## The scheduler

```c
FattosStatus fattos_init(void);

FattosStatus fattos_task_create(const char *name, FattosTaskFn fn,
                                void *context, uint32_t period_ms,
                                uint8_t priority, uint8_t *out_task_id);

uint32_t fattos_millis(void);
uint32_t fattos_ms_to_ticks(uint32_t ms);
void     fattos_tick_isr(void);            /* exactly FATTOS_TICK_HZ / s */
```

Dispatch:

```c
FattosStatus fattos_run_once(bool *ran);                    /* one callback */
FattosStatus fattos_run_pending(uint32_t budget,
                                uint32_t *dispatched);      /* bounded drain */
_Noreturn void fattos_run(void);                            /* run forever  */
```

Properties you can rely on:

1. **Highest numeric priority first** among all due tasks.
2. **Fair ties** — equal-priority due tasks rotate through the scan order
   (`g_scan_cursor`), so no task is systematically last every cycle.
3. **One callback per `fattos_run_once`** — your main loop stays in control.
4. **Bounded drain** — `fattos_run_pending(budget, …)` runs at most `budget`
   callbacks and reports how many ran. A runaway application callback can
   never turn a drain loop into an infinite spin.
5. **No due task is not an error** — `FATTOS_OK` with `*ran == false`.

```c
/* MCU main loop */
for (;;) {
    bool ran = false;
    fattos_run_once(&ran);
    if (!ran) fattos_platform_idle();
}

/* Host / cooperative integration into someone else's loop */
uint32_t n = 0;
fattos_run_pending(8, &n);   /* run at most 8 due callbacks */
```

Enable/disable:

```c
fattos_task_enable(id, false);   /* pause; re-enabling restarts cleanly:
                                    next_release = now (no backlog replay) */
size_t n = fattos_task_count();  /* slots used */
```

Tasks can be **disabled while quarantined** — see below.

---

## Task profiling and the fault hook

Every task carries a `FattosTaskInfo`:

```c
typedef struct {
    const char *name;
    FattosTaskFn function;
    void *context;
    uint32_t period_ticks, next_release, run_count;
    uint8_t priority;
    bool enabled;
    uint32_t exec_last_ticks;    /* last execution time, tick resolution */
    uint32_t exec_max_ticks;     /* WCET observed so far */
    uint32_t missed_releases;    /* periods that elapsed without dispatch */
    uint32_t overrun_count;      /* executions longer than the period */
    uint32_t jitter_max_ticks;   /* worst lateness: due time -> dispatch */
    uint32_t overrun_streak;     /* consecutive overruns since a clean run */
    bool quarantined;            /* auto-disabled by sustained overruns */
} FattosTaskInfo;
```

- `exec_max_ticks` versus your period is the **schedulability check**: if
  `exec_max + jitter_max ≥ period`, the schedule is infeasible at that load.
- `jitter_max_ticks` is your real scheduling-latency WCET on target.
- Sub-tick executions read as 0; use a cycle counter in the port if you need
  sub-millisecond WCET.

```c
FattosTaskInfo info;
fattos_task_get_info(id, &info);
```

**Fault hook** — called *in task context, interrupts enabled*, right after
a callback overruns:

```c
void on_overrun(uint8_t task_id, uint32_t exec_ticks, uint32_t period_ticks)
{
    (void)exec_ticks; (void)period_ticks;
    /* Safety-first: a late task becomes a stop, not a surprise. */
    fattos_vehicle_estop(&vehicle);
}
fattos_set_fault_hook(on_overrun);
```

---

## Quarantine: self-healing under overruns

The cooperative scheduler cannot preempt a bad callback — so it **isolates**
it instead. This is the kernel turning the classic RTOS "one task can sink
the ship" limitation into a managed capability:

- Each task counts `overrun_streak`: overruns in a row since its last clean
  run. A single clean run resets the streak — transient spikes are forgiven.
- When the streak reaches `FATTOS_QUARANTINE_LIMIT` (default **3**,
  configurable in `fattos_config.h`), the task is **quarantined**: flagged
  and disabled. The rest of the system keeps its deadlines.
- Quarantine still calls the fault hook on each overrun, so your e-stop
  policy runs first, isolation second.
- A quarantined task **cannot be re-enabled by `fattos_task_enable`** —
  that returns `FATTOS_ERR_FAULT`. Only an explicit rehabilitation says
  "we looked at it, it is fixed":

```c
if (fattos_task_is_quarantined(id)) {
    /* inspect: why did it overrun? fix, then release it. */
    fattos_task_rehab(id);       /* clears streak, re-enables fresh */
}
size_t isolated = fattos_task_quarantined_count();   /* for telemetry */
```

Design rule: `fattos_set_fault_hook` + vehicle e-stop handles the *safety*
response; quarantine handles the *availability* response. Use both.

---

## Queues

Fixed-capacity, copy-in/copy-out, caller-provided storage. Internally
protected by `fattos_platform_irq_lock()`, so they are valid ISR→task
channels on ports that implement the irq lock properly.

```c
typedef struct { /* ... */ } FattosQueue;

static uint8_t q_storage[8 * sizeof(float)];

FattosQueue q;
fattos_queue_init(&q, q_storage, sizeof(float), 8);

fattos_queue_send(&q, &value);        /* FATTOS_ERR_FULL when full   */
fattos_queue_receive(&q, &out);       /* FATTOS_ERR_EMPTY when empty */
fattos_queue_peek(&q, &out);          /* copy without consuming      */
size_t n    = fattos_queue_count(&q);
bool empty  = fattos_queue_is_empty(&q);
bool full   = fattos_queue_is_full(&q);
```

Guidance:
- Never loop waiting for space — check the return and move on (or drop the
  oldest by receiving first, if your protocol tolerates it).
- `item_size` is bytes; the storage must hold `item_size * capacity` bytes.

---

## Software timers

Static one-shot or auto-reload timers, serviced by whichever task calls
`fattos_timer_service()`. Missed periods are **skipped, not burst**.

```c
static FattosTimer blink;

fattos_timer_init(&blink, "blink", on_blink, NULL);
fattos_timer_start(&blink, 500 /* ms */, true /* autoreload */);
fattos_timer_stop(&blink);
bool active = fattos_timer_is_active(&blink);
```

Convenience: create the service task once at startup —

```c
uint8_t tid;
fattos_timers_register(10 /* service period ms */, 1, &tid);
```

Timer callbacks run in the service task's context: keep them short,
non-blocking — same rules as task callbacks. `fattos_timer_count()`
reports registered timers.

---

## Event flags

32-bit flag groups with **non-blocking** semantics (a cooperative scheduler
cannot block a task):

```c
static FattosEventGroup ev;
fattos_event_init(&ev);

fattos_event_set(&ev, 1u << 3);      /* producer (task or ISR)     */
fattos_event_clear(&ev,1u << 3);
if (fattos_event_poll(&ev, 1u << 3, false)) { ... }   /* ANY of bits */
if (fattos_event_poll(&ev, 0x5u, true))       { ... } /* ALL of bits */
uint32_t current = fattos_event_get(&ev);
```

Pattern: producer sets bits; consumer polls each cycle and continues when
the poll returns false. Nothing ever waits.

---

## Memory pool

Fixed-block allocator: O(1) alloc/free, zero fragmentation, free list lives
inside the free blocks themselves. ISR-safe via critical sections.

```c
static uint8_t pool_storage[16 * 128];

FattosPool pool;
fattos_pool_init(&pool, pool_storage, 128 /* block size */, 16);

void *block = fattos_pool_alloc(&pool);   /* NULL when exhausted */
fattos_pool_free(&pool, block);
size_t free_n = fattos_pool_free_count(&pool);
```

Rules: every block must be pointer-aligned and ≥ `sizeof(void*)` bytes
(the free list is threaded through free blocks). Allocation failure is a
normal, expected condition — handle NULL.

---

## Deadman watchdog

Each registered item must be fed within its timeout. `fattos_wdt_poll()`
re-evaluates all items; **`fattos_wdt_kick_hardware()` only kicks the
hardware watchdog while every item is healthy**, so a stalled task starves
the IWDG and the MCU resets itself.

```c
uint8_t control_wdt;
fattos_wdt_register("control", 50 /* ms */, &control_wdt);

/* in the control task, every cycle: */
fattos_wdt_feed(control_wdt);

/* in the diagnostics task: */
if (fattos_wdt_poll() != FATTOS_OK) {
    /* which item died? */
    uint8_t victim = fattos_wdt_fault_id();
}
fattos_wdt_kick_hardware();   /* no-op while any item is expired */
```

API: `fattos_wdt_poll()` (`FATTOS_OK` healthy / `FATTOS_ERR_TIMEOUT`),
`fattos_wdt_is_healthy()`, `fattos_wdt_fault_id()` (`0xFF` = none),
`fattos_wdt_count()`, `fattos_wdt_get_info(id, &item)`.

Layer the soft deadman (this) under the hardware watchdog: soft catches
*slow* stalls with a diagnosable name; hardware catches a hard scheduler
hang no software can see.

---

## Critical sections and ISR rules

```c
uint32_t key = fattos_platform_irq_lock();
/* ... critical section ... */
fattos_platform_irq_unlock(key);
```

- MCU ports must save/restore full interrupt state (e.g. PRIMASK).
- Never allocate the key yourself; never unlock with a different key.
- ISRs should only: call `fattos_tick_isr()`, push bytes into queues, and
  set event bits. **Never call task functions, timers, or the logger's
  printf path from an ISR.**

---

## Configuration knobs

All in `fattos/fattos_config.h` (override with compiler `-D`):

| Macro | Default | Meaning |
|---|---|---|
| `FATTOS_MAX_TASKS` | 8 | task table size |
| `FATTOS_TICK_HZ` | 1000 | tick rate; public durations stay in ms |
| `FATTOS_MAX_TIMERS` | 8 | software timer registry |
| `FATTOS_MAX_WDT_ITEMS` | 8 | deadman items |
| `FATTOS_QUARANTINE_LIMIT` | 3 | consecutive overruns before isolation |
| `FATTOS_TENSOR_ARENA_SIZE` | 16384 | static AI arena bytes |
| `FATTOS_TENSOR_ARENA_ALIGNMENT` | 16 | arena alignment |

Zero values for tasks/timers/wdt/tick are compile-time errors, so a bad
configuration fails the build, not the flight.

---

## Checklist: writing a good task

1. Period and priority chosen from the control requirements, not guessed.
2. Bounded work — worst-case path returns quickly; no waiting, no I/O
   stalls, no `printf` in the hot path (log into the ring instead).
3. Reads sensors by name (`fattos_sensor_read`), computes, writes actuators
   through the safety supervisor, feeds its own watchdog item, returns.
4. Fault-hook registered so its own overruns e-stop instead of degrading.
5. `exec_max + jitter_max < period` verified on target before shipping.