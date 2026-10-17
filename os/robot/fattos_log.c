#include "fattos/fattos_log.h"
#include "fattos/fattos_platform.h"
#include <stdio.h>
#include <string.h>

const char *fattos_log_level_name(FattosLogLevel level)
{
    switch (level) {
    case FATTOS_LOG_DEBUG:
        return "DEBUG";
    case FATTOS_LOG_INFO:
        return "INFO";
    case FATTOS_LOG_WARN:
        return "WARN";
    case FATTOS_LOG_ERROR:
        return "ERROR";
    default:
        return "?";
    }
}

void fattos_log_init(FattosLogger *logger, FattosLogLevel min_level)
{
    if (logger == NULL) {
        return;
    }
    memset(logger, 0, sizeof(*logger));
    logger->min_level = min_level;
}

void fattos_log_str(FattosLogger *logger, FattosLogLevel level,
                    const char *text)
{
    if (logger == NULL) {
        return;
    }
    if (level < logger->min_level) {
        logger->filtered++;
        return;
    }

    uint32_t key = fattos_platform_irq_lock(); /* one producer at a time */
    if (logger->count == FATTOS_LOG_RING) {
        logger->dropped++; /* oldest entry is evicted */
    } else {
        logger->count++;
    }
    FattosLogEntry *entry = &logger->entries[logger->head];
    entry->level = level;
    entry->timestamp_ms = fattos_millis();
    if (text != NULL) {
        size_t i = 0U;
        for (; i + 1U < FATTOS_LOG_TEXT_LEN && text[i] != '\0'; ++i) {
            entry->text[i] = text[i];
        }
        entry->text[i] = '\0';
    } else {
        entry->text[0] = '\0';
    }
    logger->head = (logger->head + 1U) % FATTOS_LOG_RING;
    logger->total++;
    fattos_platform_irq_unlock(key);
}

void fattos_log_v(FattosLogger *logger, FattosLogLevel level,
                  const char *fmt, va_list args)
{
    if (logger == NULL || fmt == NULL || level < logger->min_level) {
        if (logger != NULL && fmt != NULL && level < logger->min_level) {
            logger->filtered++;
        }
        return;
    }
    char text[FATTOS_LOG_TEXT_LEN];
    (void)vsnprintf(text, sizeof(text), fmt, args);
    fattos_log_str(logger, level, text);
}

void fattos_log(FattosLogger *logger, FattosLogLevel level, const char *fmt,
                ...)
{
    if (logger == NULL || fmt == NULL) {
        return;
    }
    if (level < logger->min_level) {
        logger->filtered++;
        return;
    }
    char text[FATTOS_LOG_TEXT_LEN];
    va_list args;
    va_start(args, fmt);
    (void)vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    fattos_log_str(logger, level, text);
}

const FattosLogEntry *fattos_log_at(const FattosLogger *logger, size_t index)
{
    if (logger == NULL || index >= logger->count) {
        return NULL;
    }
    /* 0 = newest: walk backwards from the write head. */
    size_t slot = (logger->head + FATTOS_LOG_RING - 1U - index) % FATTOS_LOG_RING;
    return &logger->entries[slot];
}