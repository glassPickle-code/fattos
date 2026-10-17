#include <assert.h>
#include <stdio.h>
#include "fattos/fattos.h"
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

static int low_count;
static int high_count;

static void low_task(void *context)
{
    (void)context;
    low_count++;
}

static void high_task(void *context)
{
    (void)context;
    high_count++;
}

int main(void)
{
    assert(fattos_init() == FATTOS_OK);

    uint8_t low_id = 0U;
    uint8_t high_id = 0U;
    assert(fattos_task_create("low", low_task, NULL, 2U, 1U, &low_id) == FATTOS_OK);
    assert(fattos_task_create("high", high_task, NULL, 2U, 5U, &high_id) == FATTOS_OK);
    assert(fattos_task_count() == 2U);

    bool ran = false;
    assert(fattos_run_once(&ran) == FATTOS_OK && ran);
    assert(high_count == 1);
    assert(low_count == 0);

    /* The low-priority task is still due and runs on the next dispatch. */
    assert(fattos_run_once(&ran) == FATTOS_OK && ran);
    assert(low_count == 1);

    fattos_host_advance(2U);
    assert(fattos_run_once(&ran) == FATTOS_OK && ran);
    assert(high_count == 2);

    FattosTaskInfo info;
    assert(fattos_task_get_info(low_id, &info) == FATTOS_OK);
    assert(info.run_count == 1U);

    /* Bounded drain runs every due task (high first, then low) and stops
       cleanly when nothing is due — no spin, no starvation. */
    fattos_host_advance(2U);
    uint32_t dispatched = 0U;
    assert(fattos_run_pending(4U, &dispatched) == FATTOS_OK);
    assert(dispatched == 2U);
    assert(high_count == 3 && low_count == 2);
    assert(fattos_run_pending(4U, &dispatched) == FATTOS_OK);
    assert(dispatched == 0U);

    assert(fattos_task_enable(high_id, false) == FATTOS_OK);

    puts("FattOS tests passed");
    return 0;
}
