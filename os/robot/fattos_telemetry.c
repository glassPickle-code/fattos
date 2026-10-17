#include "fattos/fattos_telemetry.h"
#include "fattos/fattos_crc.h"
#include "fattos/fattos_sensor.h"
#include <string.h>

size_t fattos_telemetry_encode(uint8_t type, uint8_t seq, const void *payload,
                              uint8_t len, uint8_t *out, size_t out_cap)
{
    if (out == NULL || len > FATTOS_TELEMETRY_MAX_PAYLOAD ||
        (len > 0U && payload == NULL)) {
        return 0U;
    }
    /* SOF + type + seq + len + payload + crc16 */
    size_t frame_len = 4U + (size_t)len + 2U;
    if (out_cap < frame_len) {
        return 0U;
    }

    out[0] = FATTOS_TELEMETRY_SOF;
    out[1] = type;
    out[2] = seq;
    out[3] = len;
    if (len > 0U) {
        memcpy(&out[4], payload, len);
    }
    uint16_t crc = fattos_crc16(&out[1], 3U + (size_t)len); /* type..payload */
    out[4U + len] = (uint8_t)(crc & 0xFFU);
    out[5U + len] = (uint8_t)(crc >> 8);
    return frame_len;
}

size_t fattos_telemetry_encode_status(uint8_t seq, uint8_t *out,
                                     size_t out_cap)
{
    uint8_t payload[FATTOS_TELEMETRY_MAX_PAYLOAD];
    size_t n = fattos_sensor_count();
    if (n > 14U) {
        n = 14U; /* payload budget: 1 + 4*14 + 1 = 58 <= 64 */
    }
    size_t len = 1U + n * 4U + 1U;

    payload[0] = (uint8_t)fattos_sensor_count();
    for (size_t i = 0; i < n; i++) {
        const char *name = NULL;
        uint8_t kind = 0U, health = 0U;
        uint32_t errors = 0U;
        (void)fattos_sensor_get_info(i, &name, &kind, &health, &errors);
        payload[1U + i * 4U + 0U] = kind;
        payload[1U + i * 4U + 1U] = health;
        uint16_t e16 = errors > 0xFFFFU ? 0xFFFFU : (uint16_t)errors;
        payload[1U + i * 4U + 2U] = (uint8_t)(e16 & 0xFFU);
        payload[1U + i * 4U + 3U] = (uint8_t)(e16 >> 8);
    }
    payload[len - 1U] = (uint8_t)(fattos_task_quarantined_count() > 15U
                                      ? 15U
                                      : fattos_task_quarantined_count());

    return fattos_telemetry_encode(FATTOS_TELEMETRY_TYPE_STATUS, seq, payload,
                                   (uint8_t)len, out, out_cap);
}

void fattos_telemetry_decoder_init(FattosTelemetryDecoder *decoder)
{
    if (decoder == NULL) {
        return;
    }
    memset(decoder, 0, sizeof(*decoder));
    decoder->state = FATTOS_TELEMETRY_DEC_IDLE;
}

FattosStatus fattos_telemetry_decoder_push(FattosTelemetryDecoder *decoder,
                                         uint8_t byte,
                                         FattosTelemetryFrame *out_frame)
{
    if (decoder == NULL || out_frame == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }

    switch ((FattosTelemetryDecState)decoder->state) {
    case FATTOS_TELEMETRY_DEC_IDLE:
        if (byte == FATTOS_TELEMETRY_SOF) {
            decoder->state = FATTOS_TELEMETRY_DEC_TYPE;
        }
        return FATTOS_ERR_EMPTY;
    case FATTOS_TELEMETRY_DEC_TYPE:
        decoder->type = byte;
        decoder->state = FATTOS_TELEMETRY_DEC_SEQ;
        return FATTOS_ERR_EMPTY;
    case FATTOS_TELEMETRY_DEC_SEQ:
        decoder->seq = byte;
        decoder->state = FATTOS_TELEMETRY_DEC_LEN;
        return FATTOS_ERR_EMPTY;
    case FATTOS_TELEMETRY_DEC_LEN:
        if (byte > FATTOS_TELEMETRY_MAX_PAYLOAD) {
            decoder->frames_rejected++;
            decoder->state = FATTOS_TELEMETRY_DEC_IDLE;
            return FATTOS_ERR_EMPTY;
        }
        decoder->len = byte;
        decoder->index = 0U;
        decoder->state = byte > 0U ? FATTOS_TELEMETRY_DEC_PAYLOAD
                                   : FATTOS_TELEMETRY_DEC_CRC_LO;
        return FATTOS_ERR_EMPTY;
    case FATTOS_TELEMETRY_DEC_PAYLOAD:
        decoder->payload[decoder->index++] = byte;
        if (decoder->index >= decoder->len) {
            decoder->state = FATTOS_TELEMETRY_DEC_CRC_LO;
        }
        return FATTOS_ERR_EMPTY;
    case FATTOS_TELEMETRY_DEC_CRC_LO:
        decoder->crc_lo = byte;
        decoder->state = FATTOS_TELEMETRY_DEC_CRC_HI;
        return FATTOS_ERR_EMPTY;
    case FATTOS_TELEMETRY_DEC_CRC_HI: {
        decoder->state = FATTOS_TELEMETRY_DEC_IDLE;
        uint16_t received = (uint16_t)decoder->crc_lo | ((uint16_t)byte << 8);

        uint8_t covered[3U + FATTOS_TELEMETRY_MAX_PAYLOAD];
        covered[0] = decoder->type;
        covered[1] = decoder->seq;
        covered[2] = decoder->len;
        if (decoder->len > 0U) {
            memcpy(&covered[3], decoder->payload, decoder->len);
        }
        uint16_t computed = fattos_crc16(covered, 3U + (size_t)decoder->len);
        if (computed != received) {
            decoder->frames_rejected++;
            return FATTOS_ERR_EMPTY;
        }
        decoder->frames_ok++;
        out_frame->type = decoder->type;
        out_frame->seq = decoder->seq;
        out_frame->len = decoder->len;
        out_frame->payload = decoder->payload;
        return FATTOS_OK;
    }
    default:
        decoder->state = FATTOS_TELEMETRY_DEC_IDLE;
        return FATTOS_ERR_EMPTY;
    }
}
