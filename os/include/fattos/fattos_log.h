#ifndef FATTOS_LOG_H
#define FATTOS_LOG_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Leveled, non-blocking ring logger. Logging never allocates and never
 * blocks: entries land in a fixed ring with a timestamp, and overflow
 * drops the oldest (counted, never silent). On target, drain the ring to
 * your telemetry link from a low-priority task — a full log buffer can
 * never stall a control loop.
 *
 * Uses vsnprintf: available on newlib/ARM-none-eabi and host builds; for
 * fully freestanding ports provide a tiny vsnprintf or route
 * fattos_log_str() instead.
 */
typedef enum {
    FATTOS_LOG_DEBUG = 0,
    FATTOS_LOG_INFO = 1,
    FATTOS_LOG_WARN = 2,
    FATTOS_LOG_ERROR = 3
} FattosLogLevel;

#ifndef FATTOS_LOG_RING
#define FATTOS_LOG_RING 16U
#endif

#ifndef FATTOS_LOG_TEXT_LEN
#define FATTOS_LOG_TEXT_LEN 48U
#endif

typedef struct {
    FattosLogLevel level;
    uint32_t timestamp_ms;
    char text[FATTOS_LOG_TEXT_LEN];
} FattosLogEntry;

typedef struct {
    FattosLogEntry entries[FATTOS_LOG_RING];
    size_t head;         /* next write slot */
    size_t count;        /* entries currently held (<= ring) */
    FattosLogLevel min_level;
    uint32_t total;      /* entries accepted at min_level or above */
    uint32_t dropped;    /* entries evicted by ring overflow */
    uint32_t filtered;   /* entries below min_level */
} FattosLogger;

void fattos_log_init(FattosLogger *logger, FattosLogLevel min_level);

/* printf-style entry. Safe from tasks; from ISRs prefer the str variant
   and keep messages short. */
void fattos_log(FattosLogger *logger, FattosLogLevel level, const char *fmt,
                ...);
void fattos_log_v(FattosLogger *logger, FattosLogLevel level,
                  const char *fmt, va_list args);
void fattos_log_str(FattosLogger *logger, FattosLogLevel level,
                    const char *text);

/* index 0 = newest entry; NULL when out of range. */
const FattosLogEntry *fattos_log_at(const FattosLogger *logger, size_t index);
const char *fattos_log_level_name(FattosLogLevel level);

#ifdef __cplusplus
}
#endif

#endif