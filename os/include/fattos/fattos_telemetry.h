#ifndef FATTOS_TELEMETRY_H
#define FATTOS_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Binary telemetry framing for host/ground-station links:
 *   [0xA5][type][seq][len][payload...][crc16-lo][crc16-hi]
 * CRC-16/CCITT covers type, seq, len and payload. The decoder is an
 * explicit state machine with resync: a corrupted frame never wedges the
 * stream, and a new SOF restarts parsing.
 */
#define FATTOS_TELEMETRY_SOF 0xA5U
#define FATTOS_TELEMETRY_MAX_PAYLOAD 64U

/* Status frame: one packed health snapshot of the whole machine. */
#define FATTOS_TELEMETRY_TYPE_STATUS 0x10U
/*
 * Status payload (produced by fattos_telemetry_encode_status):
 *   [0]       sensor count
 *   [1..4*n]  per sensor: kind, health, errors u16 (little endian)
 *   [last]    quarantined-task count
 * so a ground station can see which chip died and whether the scheduler
 * had to isolate a runaway task — without the receiver knowing anything
 * about the manifest.
 */

typedef struct {
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    const uint8_t *payload; /* valid only during the FATTOS_OK delivery call */
} FattosTelemetryFrame;

/* Encodes one frame into out; returns bytes written, 0 on error. */
size_t fattos_telemetry_encode(uint8_t type, uint8_t seq, const void *payload,
                              uint8_t len, uint8_t *out, size_t out_cap);

/* Encode a complete status frame (sensor health + quarantined-task count).
   Returns bytes written, 0 on error (output buffer too small). */
size_t fattos_telemetry_encode_status(uint8_t seq, uint8_t *out,
                                     size_t out_cap);

typedef enum {
    FATTOS_TELEMETRY_DEC_IDLE = 0,
    FATTOS_TELEMETRY_DEC_TYPE,
    FATTOS_TELEMETRY_DEC_SEQ,
    FATTOS_TELEMETRY_DEC_LEN,
    FATTOS_TELEMETRY_DEC_PAYLOAD,
    FATTOS_TELEMETRY_DEC_CRC_LO,
    FATTOS_TELEMETRY_DEC_CRC_HI
} FattosTelemetryDecState;

typedef struct {
    uint8_t state;
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t index;
    uint8_t crc_lo;
    uint8_t payload[FATTOS_TELEMETRY_MAX_PAYLOAD];
    uint32_t frames_ok;
    uint32_t frames_rejected;
} FattosTelemetryDecoder;

void fattos_telemetry_decoder_init(FattosTelemetryDecoder *decoder);

/* Feed one byte. Returns FATTOS_OK with *out_frame populated when a complete
   valid frame has just been received; FATTOS_ERR_EMPTY while mid-frame or
   idle; FATTOS_ERR_TIMEOUT is never returned here — CRC failures increment
   frames_rejected and restart at idle. */
FattosStatus fattos_telemetry_decoder_push(FattosTelemetryDecoder *decoder,
                                         uint8_t byte,
                                         FattosTelemetryFrame *out_frame);

#ifdef __cplusplus
}
#endif

#endif
