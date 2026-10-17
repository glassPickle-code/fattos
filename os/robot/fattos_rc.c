#include "fattos/fattos_rc.h"
#include "fattos/fattos_platform.h"

#define SBUS_MIN 172U
#define SBUS_MAX 1811U
#define SBUS_CENTER 992U

enum {
    RC_STATE_IDLE = 0,
    RC_STATE_COLLECT
};

void fattos_rc_init(FattosRCState *rc)
{
    if (rc == NULL) {
        return;
    }
    for (size_t i = 0U; i < FATTOS_RC_MAX_CHANNELS; ++i) {
        rc->channels[i] = SBUS_CENTER;
    }
    rc->frame_lost = false;
    rc->failsafe = false;
    rc->valid = false;
    rc->frames_ok = 0U;
    rc->frames_bad = 0U;
    rc->last_frame_ms = 0U;
    rc->state = RC_STATE_IDLE;
    rc->index = 0U;
}

static bool footer_valid(uint8_t byte)
{
    /* SBUS2 tails vary; accept the documented set. */
    return byte == 0x00U || byte == 0x04U || byte == 0x14U ||
           byte == 0x24U || byte == 0x34U;
}

static void unpack_channels(FattosRCState *rc)
{
    /* 16 channels of 11 bits packed little-endian across 22 bytes. */
    const uint8_t *d = &rc->buffer[1];
    for (size_t ch = 0U; ch < FATTOS_RC_MAX_CHANNELS; ++ch) {
        size_t bit = ch * 11U;
        size_t byte_index = bit / 8U;
        uint32_t value = ((uint32_t)d[byte_index] |
                          ((uint32_t)d[byte_index + 1U] << 8) |
                          ((uint32_t)d[byte_index + 2U] << 16)) >>
                         (bit % 8U);
        rc->channels[ch] = (uint16_t)(value & 0x07FFU);
    }
    rc->frame_lost = (rc->buffer[23] & 0x04U) != 0U;
    rc->failsafe = (rc->buffer[23] & 0x08U) != 0U;
}

FattosStatus fattos_rc_push(FattosRCState *rc, uint8_t byte)
{
    if (rc == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    if (rc->state == RC_STATE_IDLE) {
        if (byte == FATTOS_RC_SOF) {
            rc->buffer[0] = byte;
            rc->index = 1U;
            rc->state = RC_STATE_COLLECT;
        }
        return FATTOS_ERR_EMPTY;
    }

    /* Collect the fixed 25-byte frame. Channel data can legitimately
       contain the SOF byte, so never restart mid-frame; a misaligned
       start is caught by the footer check and the next SOF resyncs. */
    rc->buffer[rc->index++] = byte;
    if (rc->index < FATTOS_RC_FRAME_LEN) {
        return FATTOS_ERR_EMPTY;
    }
    rc->state = RC_STATE_IDLE;

    if (!footer_valid(rc->buffer[24])) {
        rc->frames_bad++;
        return FATTOS_ERR_EMPTY;
    }
    unpack_channels(rc);
    rc->valid = true;
    rc->frames_ok++;
    uint32_t key = fattos_platform_irq_lock();
    rc->last_frame_ms = fattos_millis();
    fattos_platform_irq_unlock(key);
    return FATTOS_OK;
}

bool fattos_rc_signal_ok(const FattosRCState *rc, uint32_t now_ms,
                         uint32_t timeout_ms)
{
    if (rc == NULL || !rc->valid || rc->frame_lost || rc->failsafe) {
        return false;
    }
    return (int32_t)(now_ms - (rc->last_frame_ms + timeout_ms)) < 0;
}

uint16_t fattos_rc_channel_raw(const FattosRCState *rc, uint8_t channel)
{
    if (rc == NULL || channel >= FATTOS_RC_MAX_CHANNELS) {
        return 0U;
    }
    return rc->channels[channel];
}

float fattos_rc_channel_norm(const FattosRCState *rc, uint8_t channel)
{
    uint16_t raw = fattos_rc_channel_raw(rc, channel);
    if (rc == NULL || channel >= FATTOS_RC_MAX_CHANNELS) {
        return 0.0f;
    }
    if (raw >= SBUS_CENTER) {
        return (float)(raw - SBUS_CENTER) / (float)(SBUS_MAX - SBUS_CENTER);
    }
    return -(float)(SBUS_CENTER - raw) / (float)(SBUS_CENTER - SBUS_MIN);
}

float fattos_rc_channel_throttle(const FattosRCState *rc, uint8_t channel)
{
    uint16_t raw = fattos_rc_channel_raw(rc, channel);
    if (rc == NULL || channel >= FATTOS_RC_MAX_CHANNELS) {
        return 0.0f;
    }
    if (raw <= SBUS_MIN) {
        return 0.0f;
    }
    if (raw >= SBUS_MAX) {
        return 1.0f;
    }
    return (float)(raw - SBUS_MIN) / (float)(SBUS_MAX - SBUS_MIN);
}