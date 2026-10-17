#ifndef FATTOS_SAFETY_H
#define FATTOS_SAFETY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FATTOS_MAX_SAFETY_CHANNELS
#define FATTOS_MAX_SAFETY_CHANNELS 8U
#endif

/*
 * Actuator safety supervisor. Every actuator channel must be heartbeated
 * within its timeout (typically by the control task). On timeout or a
 * latched e-stop, the supervisor immediately drives every registered PWM
 * channel to zero (fail-safe) and reports a fault state. Recovery requires
 * explicit fattos_safety_estop_clear() after conditions are healthy.
 */
typedef enum {
    FATTOS_SAFETY_OK = 0,
    FATTOS_SAFETY_ESTOP_LATCHED,
    FATTOS_SAFETY_HEARTBEAT_TIMEOUT
} FattosSafetyState;

typedef struct {
    const char *name;
    uint8_t pwm_channel;
    uint32_t timeout_ticks;
    uint32_t last_beat;
    bool active;
    bool timed_out;
    uint16_t last_duty;   /* what the supervisor believes is applied */
    uint16_t slew_limit;  /* max duty change per command, 0 = disabled */
} FattosSafetyChannel;

typedef struct {
    FattosSafetyChannel channels[FATTOS_MAX_SAFETY_CHANNELS];
    size_t count;
    bool estop;
    FattosSafetyState state;
    uint8_t fault_channel; /* index of first faulting channel, 0xFF if none */
} FattosSafety;

FattosStatus fattos_safety_init(FattosSafety *safety);
FattosStatus fattos_safety_register_channel(FattosSafety *safety, const char *name,
                                          uint8_t pwm_channel,
                                          uint32_t timeout_ms,
                                          uint8_t *out_id);
FattosStatus fattos_safety_heartbeat(FattosSafety *safety, uint8_t id);

/* Latch emergency stop; drives all channels to zero duty immediately. */
FattosStatus fattos_safety_estop(FattosSafety *safety);
/* Clear the latch; refuses while any channel is still timed out. */
FattosStatus fattos_safety_estop_clear(FattosSafety *safety);

/* Scan heartbeats (call from a periodic task). Returns the current state and
   applies safe state (zero duty) on any fault. */
FattosSafetyState fattos_safety_poll(FattosSafety *safety);

/* SAFETY-FIRST actuator path: validate + write one channel's duty.
   - Re-evaluates heartbeat timeouts/latch freshly (does not trust the last poll).
   - Any non-OK state forces zero duty and returns FATTOS_ERR_FAULT.
   - duty_permille is clamped into [0, 1000].
   Route every motor command through this instead of the raw platform HAL. */
FattosStatus fattos_safety_command(FattosSafety *safety, uint8_t id,
                                 uint16_t duty_permille);

bool fattos_safety_is_safe(const FattosSafety *safety);
/* Last evaluated supervisor state (see fattos_vehicle for the gate above it). */
FattosSafetyState fattos_safety_state(const FattosSafety *safety);

/* Channel id by name (-1 if not registered) — lets board manifests drive
   actuator writes by name instead of by index. */
int fattos_safety_find(const FattosSafety *safety, const char *name);

/* Actuator protection: limit how fast duty may change per command
   (0 disables). Faults and e-stops bypass the limit and go straight to 0 —
   safety beats smoothness, always. */
FattosStatus fattos_safety_set_slew(FattosSafety *safety, uint8_t id,
                                    uint16_t max_delta);
/* What the supervisor last applied (post-slew), 0 on any fault. */
uint16_t fattos_safety_last_duty(const FattosSafety *safety, uint8_t id);
uint8_t fattos_safety_fault_channel(const FattosSafety *safety);

#ifdef __cplusplus
}
#endif

#endif
