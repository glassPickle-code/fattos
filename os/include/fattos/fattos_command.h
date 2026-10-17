#ifndef FATTOS_COMMAND_H
#define FATTOS_COMMAND_H

#include <stddef.h>
#include <stdint.h>
#include "fattos.h"
#include "fattos_telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Typed command registry over the telemetry link — the uplink half of a
 * ground-station protocol. A host sends a command frame (type
 * FATTOS_CMD_REQUEST, payload = [cmd_id][args...]); the registry
 * dispatches to the registered handler and encodes a reply frame
 * (type FATTOS_CMD_REPLY, payload = [cmd_id][status][reply...]).
 * Unknown ids and bad payloads get an explicit NACK — never silence.
 */
#define FATTOS_CMD_REQUEST 0x20U
#define FATTOS_CMD_REPLY 0x21U
#define FATTOS_CMD_STATUS_OK 0x00U
#define FATTOS_CMD_STATUS_UNKNOWN 0x01U
#define FATTOS_CMD_STATUS_BAD_ARGS 0x02U
#define FATTOS_CMD_STATUS_FAILED 0x03U

#ifndef FATTOS_MAX_COMMANDS
#define FATTOS_MAX_COMMANDS 16U
#endif

/* args: payload after cmd_id. Write the reply body into reply and set
   *reply_len (bytes written, excluding the cmd_id+status header). */
typedef uint8_t (*FattosCmdHandler)(void *context, const uint8_t *args,
                                    uint8_t args_len, uint8_t *reply,
                                    uint8_t *reply_len, uint8_t reply_cap);

typedef struct {
    uint8_t id;
    FattosCmdHandler handler;
    void *context;
} FattosCommand;

typedef struct {
    FattosCommand commands[FATTOS_MAX_COMMANDS];
    size_t count;
    uint32_t handled;
    uint32_t unknown;
    uint32_t malformed;
} FattosCommandRegistry;

FattosStatus fattos_cmd_registry_init(FattosCommandRegistry *registry);
FattosStatus fattos_cmd_register(FattosCommandRegistry *registry, uint8_t id,
                                 FattosCmdHandler handler, void *context);

/* Feed a decoded telemetry frame. Returns FATTOS_OK when the frame was a
   command request (handled, unknown or malformed — see *out_len for the
   encoded reply) and FATTOS_ERR_EMPTY when it is not a command frame. */
FattosStatus fattos_cmd_dispatch(FattosCommandRegistry *registry,
                                 const FattosTelemetryFrame *frame,
                                 uint8_t seq, uint8_t *out, size_t out_cap,
                                 size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif