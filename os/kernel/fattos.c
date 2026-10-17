#include "fattos/fattos.h"
#include "fattos/fattos_platform.h"

/* Private scheduler initializer. */
FattosStatus fattos_scheduler_init_internal(void);

static bool g_initialized;

FattosStatus fattos_init(void)
{
    if (g_initialized) {
        return FATTOS_OK;
    }

    FattosStatus status = fattos_platform_init();
    if (status != FATTOS_OK) {
        return status;
    }

    status = fattos_scheduler_init_internal();
    if (status != FATTOS_OK) {
        return status;
    }

    status = fattos_platform_timer_start(FATTOS_TICK_HZ);
    if (status != FATTOS_OK) {
        return status;
    }

    g_initialized = true;
    return FATTOS_OK;
}
