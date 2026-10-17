#ifndef FATTOS_WATCHDOG_H
#define FATTOS_WATCHDOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Cooperative deadman monitor: each registered item must be fed within its
 * timeout. fattos_wdt_poll() re-evaluates all items (call it from a periodic
 * task). fattos_wdt_kick_hardware() forwards to the platform hardware
 * watchdog only while every item is healthy, so a stalled task starves the
 * hardware watchdog and triggers a reset on MCU ports.
 */
typedef struct {
    const char *name;
    uint32_t timeout_ticks;
    uint32_t last_feed;
    bool used;
    bool expired;
} FattosWdtItem;

FattosStatus fattos_wdt_register(const char *name, uint32_t timeout_ms,
                               uint8_t *out_id);
FattosStatus fattos_wdt_feed(uint8_t id);

/* FATTOS_OK while healthy, FATTOS_ERR_TIMEOUT once any item expired. */
FattosStatus fattos_wdt_poll(void);
bool fattos_wdt_is_healthy(void);
uint8_t fattos_wdt_fault_id(void); /* id of first expired item, or 0xFF */
size_t fattos_wdt_count(void);
FattosStatus fattos_wdt_get_info(uint8_t id, FattosWdtItem *out_item);

/* Kick hardware watchdog iff all items are healthy. */
FattosStatus fattos_wdt_kick_hardware(void);

#ifdef __cplusplus
}
#endif

#endif
