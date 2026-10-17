#include "fattos/fattos_safety.h"
#include "fattos/fattos_platform.h"
#include <string.h>

static void drive_safe_state(FattosSafety *safety)
{
    for (size_t i = 0U; i < safety->count; ++i) {
        if (safety->channels[i].active) {
            (void)fattos_platform_pwm_write(safety->channels[i].pwm_channel, 0U);
            safety->channels[i].last_duty = 0U;
        }
    }
}

/* Freshly evaluate timeouts and the e-stop latch. Caller holds the irq lock
   so state, fault channel and the decision are always consistent. */
static FattosSafetyState evaluate_locked(FattosSafety *safety, uint32_t now)
{
    bool timed_out = false;
    uint8_t fault = 0xFFU;

    for (size_t i = 0U; i < safety->count; ++i) {
        FattosSafetyChannel *channel = &safety->channels[i];
        if (!channel->active) {
            continue;
        }
        if ((int32_t)(now - (channel->last_beat + channel->timeout_ticks)) >= 0) {
            channel->timed_out = true;
        }
        if (channel->timed_out && !timed_out) {
            timed_out = true;
            fault = (uint8_t)i;
        }
    }

    if (timed_out) {
        safety->state = FATTOS_SAFETY_HEARTBEAT_TIMEOUT;
        safety->fault_channel = fault;
    } else if (safety->estop) {
        safety->state = FATTOS_SAFETY_ESTOP_LATCHED;
    } else {
        safety->state = FATTOS_SAFETY_OK;
        safety->fault_channel = 0xFFU;
    }
    return safety->state;
}

FattosStatus fattos_safety_init(FattosSafety *safety)
{
    if (safety == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    safety->count = 0U;
    safety->estop = false;
    safety->state = FATTOS_SAFETY_OK;
    safety->fault_channel = 0xFFU;
    return FATTOS_OK;
}

FattosStatus fattos_safety_register_channel(FattosSafety *safety, const char *name,
                                          uint8_t pwm_channel,
                                          uint32_t timeout_ms,
                                          uint8_t *out_id)
{
    if (safety == NULL || timeout_ms == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }
    if (safety->count >= FATTOS_MAX_SAFETY_CHANNELS) {
        return FATTOS_ERR_FULL;
    }
    size_t id = safety->count++;
    safety->channels[id].name = name;
    safety->channels[id].pwm_channel = pwm_channel;
    safety->channels[id].timeout_ticks = timeout_ms;
    safety->channels[id].last_beat = fattos_millis();
    safety->channels[id].active = true;
    safety->channels[id].timed_out = false;
    safety->channels[id].last_duty = 0U;
    safety->channels[id].slew_limit = 0U;
    /* A freshly registered actuator starts at zero duty, always. */
    (void)fattos_platform_pwm_write(pwm_channel, 0U);
    if (out_id != NULL) {
        *out_id = (uint8_t)id;
    }
    return FATTOS_OK;
}

FattosStatus fattos_safety_heartbeat(FattosSafety *safety, uint8_t id)
{
    if (safety == NULL || (size_t)id >= safety->count) {
        return FATTOS_ERR_INVALID_ARG;
    }
    uint32_t key = fattos_platform_irq_lock();
    safety->channels[id].last_beat = fattos_millis();
    safety->channels[id].timed_out = false;
    fattos_platform_irq_unlock(key);
    return FATTOS_OK;
}

FattosStatus fattos_safety_estop(FattosSafety *safety)
{
    if (safety == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    uint32_t key = fattos_platform_irq_lock();
    safety->estop = true;
    safety->state = FATTOS_SAFETY_ESTOP_LATCHED;
    fattos_platform_irq_unlock(key);
    drive_safe_state(safety);
    return FATTOS_OK;
}

FattosStatus fattos_safety_estop_clear(FattosSafety *safety)
{
    if (safety == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    /* Refuse to clear while any channel is still faulted. */
    for (size_t i = 0U; i < safety->count; ++i) {
        if (safety->channels[i].timed_out) {
            return FATTOS_ERR_TIMEOUT;
        }
    }
    uint32_t key = fattos_platform_irq_lock();
    safety->estop = false;
    safety->state = FATTOS_SAFETY_OK;
    safety->fault_channel = 0xFFU;
    fattos_platform_irq_unlock(key);
    return FATTOS_OK;
}

FattosSafetyState fattos_safety_poll(FattosSafety *safety)
{
    if (safety == NULL) {
        return FATTOS_SAFETY_HEARTBEAT_TIMEOUT;
    }
    uint32_t key = fattos_platform_irq_lock();
    FattosSafetyState result = evaluate_locked(safety, fattos_millis());
    fattos_platform_irq_unlock(key);

    if (result != FATTOS_SAFETY_OK) {
        drive_safe_state(safety);
    }
    return result;
}

FattosStatus fattos_safety_command(FattosSafety *safety, uint8_t id,
                                 uint16_t duty_permille)
{
    if (safety == NULL || (size_t)id >= safety->count ||
        !safety->channels[id].active) {
        return FATTOS_ERR_INVALID_ARG;
    }
    if (duty_permille > 1000U) {
        duty_permille = 1000U; /* clamp requests into the valid duty range */
    }

    uint8_t pwm_channel = safety->channels[id].pwm_channel;
    uint32_t key = fattos_platform_irq_lock();
    /* Decide on fresh state, not on whatever the last poll happened to see. */
    FattosSafetyState state = evaluate_locked(safety, fattos_millis());
    if (state == FATTOS_SAFETY_OK) {
        /* Actuator protection: slew-limit duty changes so a corrupted
           command cannot slam a motor from 0 to full in one cycle. */
        uint16_t applied = duty_permille;
        uint16_t limit = safety->channels[id].slew_limit;
        uint16_t last = safety->channels[id].last_duty;
        if (limit > 0U) {
            if ((uint32_t)applied > (uint32_t)last + limit) {
                applied = (uint16_t)((uint32_t)last + limit);
            } else if ((uint32_t)applied + limit < (uint32_t)last) {
                applied = (uint16_t)(last > limit ? last - limit : 0U);
            }
        }
        safety->channels[id].last_duty = applied;
        (void)fattos_platform_pwm_write(pwm_channel, applied);
        fattos_platform_irq_unlock(key);
        return FATTOS_OK;
    }
    /* Faults bypass slew entirely: zero now, smoothness is irrelevant. */
    safety->channels[id].last_duty = 0U;
    (void)fattos_platform_pwm_write(pwm_channel, 0U);
    fattos_platform_irq_unlock(key);
    return FATTOS_ERR_FAULT;
}

bool fattos_safety_is_safe(const FattosSafety *safety)
{
    return safety != NULL && safety->state == FATTOS_SAFETY_OK;
}

FattosSafetyState fattos_safety_state(const FattosSafety *safety)
{
    return safety != NULL ? safety->state : FATTOS_SAFETY_HEARTBEAT_TIMEOUT;
}

int fattos_safety_find(const FattosSafety *safety, const char *name)
{
    if (safety == NULL || name == NULL) {
        return -1;
    }
    for (size_t i = 0U; i < safety->count; ++i) {
        if (safety->channels[i].active && safety->channels[i].name != NULL &&
            strcmp(safety->channels[i].name, name) == 0) {
            return (int)i;
        }
    }
    return -1;
}

FattosStatus fattos_safety_set_slew(FattosSafety *safety, uint8_t id,
                                    uint16_t max_delta)
{
    if (safety == NULL || (size_t)id >= safety->count) {
        return FATTOS_ERR_INVALID_ARG;
    }
    safety->channels[id].slew_limit = max_delta;
    return FATTOS_OK;
}

uint16_t fattos_safety_last_duty(const FattosSafety *safety, uint8_t id)
{
    if (safety == NULL || (size_t)id >= safety->count) {
        return 0U;
    }
    return safety->channels[id].last_duty;
}

uint8_t fattos_safety_fault_channel(const FattosSafety *safety)
{
    return safety != NULL ? safety->fault_channel : 0xFFU;
}
