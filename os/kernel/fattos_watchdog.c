#include "fattos/fattos_watchdog.h"
#include "fattos/fattos_platform.h"

static FattosWdtItem g_items[FATTOS_MAX_WDT_ITEMS];
static size_t g_count;

FattosStatus fattos_wdt_register(const char *name, uint32_t timeout_ms,
                               uint8_t *out_id)
{
    if (timeout_ms == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }
    if (g_count >= FATTOS_MAX_WDT_ITEMS) {
        return FATTOS_ERR_FULL;
    }
    size_t id = g_count++;
    g_items[id].name = name;
    g_items[id].timeout_ticks = timeout_ms;
    g_items[id].last_feed = fattos_millis();
    g_items[id].used = true;
    g_items[id].expired = false;
    if (out_id != NULL) {
        *out_id = (uint8_t)id;
    }
    return FATTOS_OK;
}

FattosStatus fattos_wdt_feed(uint8_t id)
{
    if ((size_t)id >= g_count || !g_items[id].used) {
        return FATTOS_ERR_INVALID_ARG;
    }
    uint32_t key = fattos_platform_irq_lock();
    g_items[id].last_feed = fattos_millis();
    g_items[id].expired = false; /* Feeding clears this item's fault. */
    fattos_platform_irq_unlock(key);
    return FATTOS_OK;
}

FattosStatus fattos_wdt_poll(void)
{
    uint32_t now = fattos_millis();
    bool healthy = true;
    for (size_t i = 0U; i < g_count; ++i) {
        if (!g_items[i].used) {
            continue;
        }
        if ((int32_t)(now - (g_items[i].last_feed + g_items[i].timeout_ticks)) >= 0) {
            g_items[i].expired = true;
        }
        if (g_items[i].expired) {
            healthy = false;
        }
    }
    return healthy ? FATTOS_OK : FATTOS_ERR_TIMEOUT;
}

bool fattos_wdt_is_healthy(void)
{
    return fattos_wdt_poll() == FATTOS_OK;
}

uint8_t fattos_wdt_fault_id(void)
{
    for (size_t i = 0U; i < g_count; ++i) {
        if (g_items[i].used && g_items[i].expired) {
            return (uint8_t)i;
        }
    }
    return 0xFFU;
}

size_t fattos_wdt_count(void)
{
    return g_count;
}

FattosStatus fattos_wdt_get_info(uint8_t id, FattosWdtItem *out_item)
{
    if (out_item == NULL || (size_t)id >= g_count || !g_items[id].used) {
        return FATTOS_ERR_INVALID_ARG;
    }
    *out_item = g_items[id];
    return FATTOS_OK;
}

FattosStatus fattos_wdt_kick_hardware(void)
{
    FattosStatus status = fattos_wdt_poll();
    if (status != FATTOS_OK) {
        /* Starve the hardware watchdog on purpose: the system is unhealthy. */
        return status;
    }
    return fattos_platform_watchdog_kick();
}
