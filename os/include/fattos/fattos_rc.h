#ifndef FATTOS_RC_H
#define FATTOS_RC_H

#include <stdbool.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * SBUS radio-control input decoder with signal-loss failsafe. Feed raw
 * serial bytes (one per call); a valid frame unpacks 16 channels of
 * 11-bit values plus the frame-lost/failsafe flags. Use
 * fattos_rc_signal_ok() as the RC half of the vehicle pre-arm condition
 * and feed the arm switch into fattos_vehicle_set_rc().
 *
 * Channel value range: 172 (min) .. 1811 (max), 992 center (SBUS native).
 */
#define FATTOS_RC_MAX_CHANNELS 16U
#define FATTOS_RC_FRAME_LEN 25U
#define FATTOS_RC_SOF 0x0FU

typedef struct {
    uint16_t channels[FATTOS_RC_MAX_CHANNELS];
    bool frame_lost;      /* SBUS frame-lost flag from the receiver */
    bool failsafe;        /* SBUS failsafe flag (receiver lost transmitter) */
    bool valid;           /* at least one good frame decoded */
    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t last_frame_ms;
    /* decoder state */
    uint8_t state;
    uint8_t index;
    uint8_t buffer[FATTOS_RC_FRAME_LEN];
} FattosRCState;

void fattos_rc_init(FattosRCState *rc);

/* Feed one serial byte. Returns FATTOS_OK exactly when a frame just
   completed and was validated; frames_bad counts rejected frames. */
FattosStatus fattos_rc_push(FattosRCState *rc, uint8_t byte);

/* True when frames arrived recently and neither lost nor failsafe. */
bool fattos_rc_signal_ok(const FattosRCState *rc, uint32_t now_ms,
                         uint32_t timeout_ms);

/* Raw 11-bit channel value, 0 if out of range. */
uint16_t fattos_rc_channel_raw(const FattosRCState *rc, uint8_t channel);
/* Normalized stick: -1.0 .. +1.0 around center. */
float fattos_rc_channel_norm(const FattosRCState *rc, uint8_t channel);
/* Throttle-style: 0.0 .. 1.0. */
float fattos_rc_channel_throttle(const FattosRCState *rc, uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif