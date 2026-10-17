#include "fattos/fattos_command.h"

FattosStatus fattos_cmd_registry_init(FattosCommandRegistry *registry)
{
    if (registry == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    registry->count = 0U;
    registry->handled = 0U;
    registry->unknown = 0U;
    registry->malformed = 0U;
    return FATTOS_OK;
}

FattosStatus fattos_cmd_register(FattosCommandRegistry *registry, uint8_t id,
                                 FattosCmdHandler handler, void *context)
{
    if (registry == NULL || handler == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    if (registry->count >= FATTOS_MAX_COMMANDS) {
        return FATTOS_ERR_FULL;
    }
    /* Re-registering an id replaces its handler (idempotent setup). */
    for (size_t i = 0U; i < registry->count; ++i) {
        if (registry->commands[i].id == id) {
            registry->commands[i].handler = handler;
            registry->commands[i].context = context;
            return FATTOS_OK;
        }
    }
    registry->commands[registry->count].id = id;
    registry->commands[registry->count].handler = handler;
    registry->commands[registry->count].context = context;
    registry->count++;
    return FATTOS_OK;
}

static void encode_reply(const FattosTelemetryFrame *frame, uint8_t seq,
                         uint8_t cmd_id, uint8_t status, const uint8_t *body,
                         uint8_t body_len, uint8_t *out, size_t out_cap,
                         size_t *out_len)
{
    uint8_t payload[3U + FATTOS_TELEMETRY_MAX_PAYLOAD];
    payload[0] = cmd_id;
    payload[1] = status;
    for (uint8_t i = 0U; i < body_len; ++i) {
        payload[2U + i] = body[i];
    }
    size_t written = fattos_telemetry_encode(
        FATTOS_CMD_REPLY, seq, payload, (uint8_t)(2U + body_len), out, out_cap);
    *out_len = written;
    (void)frame;
}

FattosStatus fattos_cmd_dispatch(FattosCommandRegistry *registry,
                                 const FattosTelemetryFrame *frame,
                                 uint8_t seq, uint8_t *out, size_t out_cap,
                                 size_t *out_len)
{
    if (registry == NULL || frame == NULL || out == NULL || out_len == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    *out_len = 0U;
    if (frame->type != FATTOS_CMD_REQUEST) {
        return FATTOS_ERR_EMPTY; /* not a command frame; carry on */
    }
    if (frame->len == 0U || frame->payload == NULL) {
        /* Malformed command: NACK so the host sees the failure. */
        registry->malformed++;
        encode_reply(frame, seq, 0U, FATTOS_CMD_STATUS_BAD_ARGS, NULL, 0U,
                     out, out_cap, out_len);
        return FATTOS_OK;
    }

    uint8_t cmd_id = frame->payload[0];
    const uint8_t *args = &frame->payload[1];
    uint8_t args_len = (uint8_t)(frame->len - 1U);

    for (size_t i = 0U; i < registry->count; ++i) {
        if (registry->commands[i].id != cmd_id) {
            continue;
        }
        uint8_t reply[FATTOS_TELEMETRY_MAX_PAYLOAD];
        uint8_t reply_len = 0U;
        uint8_t status = registry->commands[i].handler(
            registry->commands[i].context, args, args_len, reply, &reply_len,
            (uint8_t)sizeof(reply));
        if (status != FATTOS_CMD_STATUS_OK) {
            reply_len = 0U;
        }
        registry->handled++;
        encode_reply(frame, seq, cmd_id, status, reply, reply_len, out,
                     out_cap, out_len);
        return FATTOS_OK;
    }

    registry->unknown++;
    encode_reply(frame, seq, cmd_id, FATTOS_CMD_STATUS_UNKNOWN, NULL, 0U, out,
                 out_cap, out_len);
    return FATTOS_OK;
}