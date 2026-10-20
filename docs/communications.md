# Communications — telemetry, commands, RC input, and logging

FattOS treats every off-board link as a **framed, CRC-checked byte stream**
and gives you the pieces to build a ground station, a mission computer
link, or a pilot's RC input without writing a protocol from scratch.

Three independent subsystems live here:

| Subsystem | Header | Purpose |
|---|---|---|
| Telemetry framing | `fattos/fattos_telemetry.h` | Binary downlink frames with CRC-16 and a resyncing decoder |
| Command registry | `fattos/fattos_command.h` | Typed uplink: request/reply dispatch with explicit NACKs |
| RC (SBUS) input | `fattos/fattos_rc.h` | 16-channel SBUS decoder with signal-loss failsafe |
| Logger | `fattos/fattos_log.h` | Non-blocking ring logger that can never stall a control loop |
| CRC | `fattos/fattos_crc.h` | CRC-16/CCITT and CRC-32 primitives |

---

## 1. Telemetry framing

The wire format is a fixed 4-byte header, the payload, and a 2-byte CRC:

```
[0xA5][type][seq][len][payload ... len bytes][crc16-lo][crc16-hi]
```

- `SOF` is always `0xA5` (`FATTOS_TELEMETRY_SOF`).
- `type` is your frame id (`0x00`–`0xFF`; `0x10` is reserved for the status
  frame, `0x20`/`0x21` for command request/reply).
- `seq` is a free-running sequence number you choose; use it on the host
  to detect dropped frames.
- `len` is the payload length, 0–64 (`FATTOS_TELEMETRY_MAX_PAYLOAD`).
- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) covers `type`, `seq`,
  `len`, and the payload — **not** the SOF. The CRC is transmitted
  little-endian (lo byte first).

### Encoding

```c
uint8_t wire[72];
uint8_t payload[2] = {0x42, 0x07};

size_t n = fattos_telemetry_encode(0x30 /* your type */, seq++, payload, 2,
                                   wire, sizeof wire);
if (n == 0) {
    /* payload > 64 bytes or output buffer too small — handle it, never guess */
}
uart_write(port, wire, n);
```

`fattos_telemetry_encode` returns the number of bytes written, or `0` on
error (payload too large for the frame, output buffer too small, NULL
arguments). It never writes past `out_cap`, ever.

### Decoding — byte at a time, self-resyncing

The decoder is an explicit state machine fed one byte at a time — exactly
how a UART ISR or your `uart_read` drain loop wants it:

```c
static FattosTelemetryDecoder decoder;   /* zero-init is fine; call init */

void on_uart_byte(uint8_t b)
{
    FattosTelemetryFrame frame;
    if (fattos_telemetry_decoder_push(&decoder, b, &frame) == FATTOS_OK) {
        /* frame.type, frame.seq, frame.len, frame.payload are valid NOW.
           frame.payload points into the decoder's internal buffer — copy
           anything you need to keep before returning. */
        handle_frame(&frame);
    }
}
```

Key properties you can rely on:

- **A corrupted frame never wedges the stream.** Any CRC failure or an
  over-long `len` byte increments `frames_rejected` and restarts the
  state machine at idle; the next `0xA5` starts a fresh parse.
- **`frame.payload` is only valid during the `FATTOS_OK` call.** It points
  into decoder-owned memory that the next frame will overwrite.
- Return contract: `FATTOS_OK` when a complete valid frame just arrived;
  `FATTOS_ERR_EMPTY` while mid-frame (this is *normal*, not an error);
  `FATTOS_ERR_INVALID_ARG` for NULL arguments. CRC failures also return
  `FATTOS_ERR_EMPTY` and bump `frames_rejected`.
- Inspect `decoder.frames_ok` and `decoder.frames_rejected` for link
  quality; ship them in your own telemetry if you like.

### The status frame — the whole machine in one packet

FattOS pre-encodes one frame type for you: `FATTOS_TELEMETRY_TYPE_STATUS`
(`0x10`). It packs every sensor's health plus the scheduler's quarantine
count, so a ground station sees which chip died and whether the scheduler
isolated a runaway task — **without knowing anything about your manifest**:

```c
uint8_t wire[72];
size_t n = fattos_telemetry_encode_status(seq++, wire, sizeof wire);
if (n > 0) {
    link_write(wire, n);
}
```

Payload layout (little-endian, decoded on the host):

```
[0]        sensor count (n; report payload caps n at 14)
[1..4*n]   per sensor, 4 bytes: kind, health, errors u16 LE
[last]     quarantined-task count (capped at 15)
```

`kind` is the `FattosSensorKind` value and `health` the
`FattosSensorHealth` value — publish the enum meanings next to your host
code (they are stable, see [api-reference.md](api-reference.md)). Errors
above `0xFFFF` saturate rather than wrap. A `kind` of `FATTOS_SENSOR_*`
that your host has never heard of is still printable as "unknown kind" —
forward compatibility is by design.

---

## 2. Command registry — the uplink half

The command registry is the ground-station *request/reply* protocol on top
of the same framing. A host sends a frame of type `FATTOS_CMD_REQUEST`
(`0x20`) whose payload is `[cmd_id][args...]`; the registry dispatches to
your handler and encodes a reply frame of type `FATTOS_CMD_REPLY` (`0x21`)
with payload `[cmd_id][status][reply...]`.

Four status codes exist and **every** request gets exactly one reply:

| Status | Meaning |
|---|---|
| `0x00` `FATTOS_CMD_STATUS_OK` | handler ran, reply body follows |
| `0x01` `FATTOS_CMD_STATUS_UNKNOWN` | no handler registered for that cmd_id |
| `0x02` `FATTOS_CMD_STATUS_BAD_ARGS` | handler rejected the arg bytes |
| `0x03` `FATTOS_CMD_STATUS_FAILED` | handler ran but could not complete |

Unknown ids are never silent — that's the whole point. A ground station
can always distinguish "lost on the link" from "robot said no".

### Registering and dispatching

```c
#include "fattos/fattos_command.h"

#define CMD_SET_GOAL 0x01

static uint8_t handle_set_goal(void *ctx, const uint8_t *args, uint8_t args_len,
                               uint8_t *reply, uint8_t *reply_len,
                               uint8_t reply_cap)
{
    (void)ctx;
    if (args_len != 8) {
        return FATTOS_CMD_STATUS_BAD_ARGS;   /* two little-endian float32 */
    }
    float gx, gy;
    memcpy(&gx, &args[0], 4);
    memcpy(&gy, &args[4], 4);
    app_set_goal(gx, gy);

    if (reply_cap >= 1) {
        reply[0] = 1;                        /* accepted */
        *reply_len = 1;
    }
    return FATTOS_CMD_STATUS_OK;
}

static FattosCommandRegistry registry;

void comm_init(void)
{
    fattos_cmd_registry_init(&registry);
    fattos_cmd_register(&registry, CMD_SET_GOAL, handle_set_goal, NULL);
}

/* Called from your telemetry byte loop, alongside the frame decoder: */
void on_frame(const FattosTelemetryFrame *frame, uint8_t seq)
{
    uint8_t wire[72];
    size_t n = 0;
    if (fattos_cmd_dispatch(&registry, frame, seq, wire, sizeof wire, &n)
            == FATTOS_OK) {
        link_write(wire, n);     /* the reply frame, ready to send */
    }
}
```

Handler contract:

- `args`/`args_len` are the payload after the `cmd_id` byte.
- Write the reply **body** into `reply` and set `*reply_len` to the bytes
  written (excluding the cmd_id+status header the registry adds).
  `reply_cap` is the space available — respect it, the registry checks
  the resulting frame length too.
- Return one of the four status codes.
- The handler runs in whatever context calls `fattos_cmd_dispatch` —
  keep it bounded and non-blocking, like a task.
- Registry counters: `handled`, `unknown`, `malformed` — put them on your
  health telemetry to watch for a mismatched host build.

`fattos_cmd_dispatch` returns `FATTOS_OK` when the frame was a command
request (whether it was handled, unknown, or malformed — check `*out_len`
for the encoded reply, which is 0 when the request was not a command
frame at all) and `FATTOS_ERR_EMPTY` when `frame->type` is not
`FATTOS_CMD_REQUEST`, so a dispatcher over mixed traffic is one `if`.

The registry holds `FATTOS_MAX_COMMANDS` (default 16, override with the
preprocessor) handlers, fixed at compile time — no allocation anywhere.

---

## 3. RC input (SBUS)

`fattos_rc.h` decodes the SBUS serial protocol — 16 channels of 11-bit
values plus frame-lost and failsafe flags — from a raw byte stream. It
feeds the vehicle state machine's "is the pilot there?" input directly.

Native channel range is 172 (min) .. 992 (center) .. 1811 (max).

```c
#include "fattos/fattos_rc.h"

static FattosRCState rc;
fattos_rc_init(&rc);

/* In your UART RX path — one byte per call: */
void on_sbus_byte(uint8_t b)
{
    if (fattos_rc_push(&rc, b) == FATTOS_OK) {
        /* a complete, validated frame just landed */
    }
}

/* In your control task: */
void control_step(void)
{
    uint32_t now = fattos_millis();
    bool pilot_there = fattos_rc_signal_ok(&rc, now, 100 /* ms */);

    float steer = fattos_rc_channel_norm(&rc, 0);      /* -1..+1 */
    float drive = fattos_rc_channel_throttle(&rc, 1);  /*  0..1  */
    uint16_t raw = fattos_rc_channel_raw(&rc, 2);      /* 172..1811 */

    fattos_vehicle_set_rc(&veh, pilot_there, raw > 1200 /* arm switch */);
}
```

API summary:

- `fattos_rc_push(rc, byte)` — feed one serial byte; returns `FATTOS_OK`
  exactly when a frame completed and validated. `frames_bad` counts
  rejections; `last_frame_ms` records arrival time.
- `fattos_rc_signal_ok(rc, now_ms, timeout_ms)` — true when frames
  arrived recently **and** neither the frame-lost nor the failsafe flag
  is set. This is the boolean the vehicle state machine wants.
- `fattos_rc_channel_raw(rc, ch)` — 11-bit value, 0 if out of range.
- `fattos_rc_channel_norm(rc, ch)` — −1.0..+1.0 around center (sticks).
- `fattos_rc_channel_throttle(rc, ch)` — 0.0..1.0 (throttle-style).

The decoder is strictly feed-driven and allocation-free; wire it to any
UART that produces SBUS (inverted UART on most receivers — that inversion
is the port's job, not the decoder's).

---

## 4. Logging

`fattos_log.h` is a leveled ring logger with two rules: **never allocate,
never block.** Entries land in a fixed ring; overflow evicts the oldest
and counts the eviction (`dropped`) — a full log buffer can never stall a
control loop, and nothing is ever dropped silently.

```c
#include "fattos/fattos_log.h"

static FattosLogger logger;

void app_init(void)
{
    fattos_log_init(&logger, FATTOS_LOG_INFO);  /* below-INFO is filtered */
}

void on_event(void)
{
    fattos_log(&logger, FATTOS_LOG_WARN, "motor %s timeout after %u ms",
               name, (unsigned)elapsed);
}
```

- Levels: `DEBUG`, `INFO`, `WARN`, `ERROR`. Entries below `min_level` are
  counted in `filtered` and discarded.
- `fattos_log()` is printf-style (via `vsnprintf`); `fattos_log_v()` takes
  a `va_list`; `fattos_log_str()` takes a plain string and is the
  ISR-friendly variant (no formatting, keep it short).
- Entries carry a `timestamp_ms` from `fattos_millis()`.
- Read back with `fattos_log_at(logger, index)` — index 0 is the **newest**
  entry; returns `NULL` out of range. Each entry is
  `{level, timestamp_ms, text[48]}` (`FATTOS_LOG_TEXT_LEN`).
- Size knobs: `FATTOS_LOG_RING` (default 16 entries) and
  `FATTOS_LOG_TEXT_LEN` (default 48) — override with the preprocessor
  before including the header, or globally with `-D`.
- On target, drain the ring to your telemetry link from a **low-priority
  task** — the logger itself is safe from anywhere, but the link write
  is not.
- Freestanding ports: the printf variant needs `vsnprintf`. If your
  toolchain lacks one, route everything through `fattos_log_str()`.

---

## 5. CRC primitives

Both CRCs are exposed standalone (`fattos/fattos_crc.h`) so host tools and
custom frames use the same math as the wire format:

```c
uint16_t crc16 = fattos_crc16(data, len);   /* CCITT-FALSE: poly 0x1021,
                                               init 0xFFFF,
                                               check("123456789") = 0x29B1 */
uint32_t crc32 = fattos_crc32(data, len);   /* reflected 0xEDB88320,
                                               check("123456789") = 0xCBF43926 */
```

The check values above are what a host implementation must reproduce —
use them as the conformance test when you write the ground station in
another language.

---

## 6. Design rules for your own frame types

When you add custom telemetry types (anything besides `0x10`/`0x20`/`0x21`):

1. **Reserve a type id table in one header shared with the host tool.**
   The `type` byte is the only multiplexer; drift here is the classic
   ground-station bug.
2. **Little-endian everything.** FattOS encoders write little-endian; keep
   your host parser the same.
3. **Prefer fixed-size payloads.** The decoder is length-prefixed so
   variable sizes work, but fixed layouts are trivially parseable in any
   language.
4. **Cap at 64 bytes per frame.** Larger data goes in multiple frames with
   a sequence, not a bigger buffer.
5. **Never block the sender.** Telemetry is best-effort by nature; the
   safety supervisor and control loops outrank it, always.

Next: [writing-drivers.md](writing-drivers.md) explains how to make new
chips and buses appear in this same "declare it, the OS knows it" model.
