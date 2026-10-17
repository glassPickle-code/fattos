# Safety: supervisor, vehicle gate, and the layers below it

FattOS treats actuator safety as a **stack of independent layers**, each
of which fails to the safe direction:

```
  application control loop
        │  only writes when fattos_vehicle_can_actuate()
        ▼
  vehicle state manager          (arming policy: DISARMED/ARMED/FAILSAFE/ESTOP)
        │  state == ARMED?
        ▼
  safety supervisor              (heartbeats, timeouts, latched e-stop, slew)
        │  fattos_safety_command() re-checks everything, per write
        ▼
  platform PWM HAL               (never called directly by app code)
```

Underneath everything: the **deadman watchdog** (soft, named) and the
**hardware watchdog** (MCU reset), plus the **scheduler fault hook** and
**sensor health** feeding prearm from two more directions.

Headers: `fattos/fattos_safety.h`, `fattos/fattos_vehicle.h`,
`fattos/fattos_watchdog.h`, `fattos/fattos_board.h`, `fattos/fattos.h`.

---

## 1. The safety supervisor (`fattos_safety.h`)

One instance per vehicle. Every actuator channel you register must be
**heartbeated** — normally by the control task, every cycle.

```c
FattosSafety safety;
fattos_safety_init(&safety);

uint8_t left_id, right_id;
fattos_safety_register_channel(&safety, "left_motor",  0 /* PWM ch */,
                               60 /* ms timeout */, &left_id);
fattos_safety_register_channel(&safety, "right_motor", 1,
                               60, &right_id);
```

### States

```c
typedef enum {
    FATTOS_SAFETY_OK = 0,
    FATTOS_SAFETY_ESTOP_LATCHED,      /* e-stop pressed (latched) */
    FATTOS_SAFETY_HEARTBEAT_TIMEOUT   /* a channel missed its window */
} FattosSafetyState;
```

On any non-OK state every registered channel is driven to **zero duty**
(fail-safe) and the state is reported. `fattos_safety_fault_channel()`
names the first offender.

### The actuator path — `fattos_safety_command`

This is the **only** function application code uses to move anything:

```c
FattosStatus st = fattos_safety_command(&safety, left_id, 400 /* permille */);
if (st == FATTOS_ERR_FAULT) {
    /* supervisor refused: latched e-stop or heartbeat timeout.
       Duty was forced to 0. Treat as a real event (log, failsafe). */
}
```

Per write it:

1. Re-evaluates heartbeat timeouts and the latch **freshly** — it never
   trusts the last poll, so the gate reacts within one control cycle even
   if nobody else polled.
2. Any non-OK state → zero duty + `FATTOS_ERR_FAULT`.
3. Clamps duty into [0, 1000] permille.
4. Applies the per-channel **slew-rate limit** (if configured) — a
   corrupted command can never slam a motor 0→full in one cycle.
5. Faults and e-stops bypass slew and go straight to 0 — *safety beats
   smoothness, always*.

```c
fattos_safety_set_slew(&safety, left_id, 100); /* ≤100 permille/cycle */
uint16_t applied = fattos_safety_last_duty(&safety, left_id);
```

### Heartbeats, e-stop, polling

```c
fattos_safety_heartbeat(&safety, left_id);      /* every control cycle */

FattosSafetyState ss = fattos_safety_poll(&safety);   /* from a periodic task;
                                                          applies zero on fault */

fattos_safety_estop(&safety);        /* latch: all channels zero immediately */
fattos_safety_estop_clear(&safety);  /* refuses while any channel still timed
                                        out — recovery must be earned */

bool safe  = fattos_safety_is_safe(&safety);
int found  = fattos_safety_find(&safety, "left_motor");  /* by name, -1 none */
```

API summary:

| Function | Purpose |
|---|---|
| `fattos_safety_init` | zero-count supervisor, state OK |
| `fattos_safety_register_channel(name, ch, timeout_ms, &id)` | add a channel |
| `fattos_safety_heartbeat(id)` | feed one channel's timeout |
| `fattos_safety_poll` | evaluate all, fail-safe on fault, return state |
| `fattos_safety_command(id, duty)` | **the** validated actuator write |
| `fattos_safety_estop` / `estop_clear` | latch / (healthy-only) clear |
| `fattos_safety_set_slew(id, max_delta)` | duty change limit per command |
| `fattos_safety_last_duty(id)` | what the supervisor actually applied |
| `fattos_safety_is_safe` / `fattos_safety_state` | predicate + last state |
| `fattos_safety_find(name)` | channel id by name (-1 if absent) |
| `fattos_safety_fault_channel` | index of first fault, 0xFF if none |

Up to `FATTOS_MAX_SAFETY_CHANNELS` (default 8).

---

## 2. The vehicle state manager (`fattos_vehicle.h`)

Flight-controller arming semantics in front of the supervisor:

```
INIT ──► DISARMED ──arm conditions held arm_hold_ms──► ARMED
              ▲                                            │
              │                          ┌─────────────────┤
              │                 RC loss / supervisor fault │  arm switch off
              │                          ▼                 │  (pilot disarm)
              │                      FAILSAFE ◄────────────┤
              │                          │                 ▼
              ├── estop_clear ──── ESTOP ◄── latched e-stop (any state)
              └──── FAILSAFE recovers only to DISARMED (must re-arm)
```

```c
FattosVehicle veh;
fattos_vehicle_init(&veh, &safety,
                    100 /* arm switch held this long, ms */,
                    100 /* RC loss timeout, ms */);
```

**To arm:** RC present and recent (`rc_ok`), pilot's arm switch ON,
`prearm_ok`, sensor manifest healthy (when declared), supervisor OK —
*all of them held continuously for `arm_hold_ms`*. A deliberate hold,
not a flap.

**To FAILSAFE:** while ARMED, RC loss or a supervisor fault. (The arm
switch being turned off is different: that is a deliberate *pilot disarm*
straight to DISARMED.) Recovery from FAILSAFE requires everything healthy
*and* goes to DISARMED — a full re-arm cycle, never a silent jump.

**To ESTOP:** only from a latched e-stop; out only via explicit
`fattos_vehicle_estop_clear()`, and only if the supervisor is healthy.

```c
fattos_vehicle_set_rc(&veh, rc_present, rc_arm_switch);  /* each cycle */
fattos_vehicle_set_prearm(&veh, app_health_ok);          /* app-level bit */

FattosVehicleState st = fattos_vehicle_update(&veh);      /* each cycle */
const char *name = fattos_vehicle_state_name(st);         /* "ARMED" etc */

if (!fattos_vehicle_can_actuate(&veh)) {
    /* THE single "may I move?" question — gate every actuator write. */
}
```

### Sensor health is wired in automatically

If a sensor manifest exists (`fattos_sensor_count() > 0`), the vehicle
gate ANDs `fattos_sensors_prearm_ok()` into its arming decision: every
declared sensor must be present, fresh and plausible to arm, and a sensor
that dies in flight drops the vehicle to FAILSAFE through the same
condition. With no manifest, only your `prearm_ok` bit gates arming.

### When the gate says no

```c
if (!fattos_vehicle_can_actuate(&veh)) {
    /* zero your commands, and seed controllers against the CURRENT
       measurement so re-enabling later starts bumpless: */
    fattos_pid_seed(&yaw_pid, yaw_now, yaw_now, 0.0f);
}
```

Never bypass this with direct HAL writes — that is the one rule the whole
stack exists to enforce.

---

## 3. The fault hook: late task → stop

```c
fattos_set_fault_hook(on_overrun);

static void on_overrun(uint8_t task_id, uint32_t exec_ticks,
                       uint32_t period_ticks)
{
    (void)task_id; (void)exec_ticks; (void)period_ticks;
    fattos_vehicle_estop(&veh);    /* a late task becomes a stop */
}
```

Runs in task context with interrupts enabled, immediately after a
callback exceeds its period. Combined with
[scheduler quarantine](kernel.md#quarantine-self-healing-under-overruns),
overruns first *stop safely* (hook), then get *isolated* (quarantine).

---

## 4. The deadman watchdog chain

```c
uint8_t wd_control;
fattos_wdt_register("control", 50 /* ms */, &wd_control);
fattos_wdt_feed(wd_control);              /* control task, every cycle */

/* diagnostics task: */
if (fattos_wdt_poll() != FATTOS_OK) {
    uint8_t victim = fattos_wdt_fault_id();   /* whose feed went missing */
    /* escalate: estop, latch, log */
}
fattos_wdt_kick_hardware();   /* kicks ONLY while all items are healthy */
```

Layering:

- **Soft deadman** (`fattos_wdt_*`) — catches a slow/stalled *specific*
  task by name, diagnosable, no reset needed to know who.
- **Hardware watchdog** (`fattos_platform_watchdog_kick`, forwarded by
  `fattos_wdt_kick_hardware`) — catches a total scheduler hang, resets
  the MCU. Because the kick is *conditional* on soft health, a stalled
  task starves the hardware watchdog too: soft detection first, hard
  reset as the floor.
- **Independent hardware motor-disable** (kill switch / driver enable
  pin) remains *your* porting responsibility — software layers never
  replace a physical e-stop (see [docs/porting-stm32.md](porting-stm32.md)).

---

## 5. Board-managed safety (`fattos_board.h`)

The board manifest turns declaration into supervision automatically:

```c
static const FattosDevice devices[] = {
    {"left_motor",  FATTOS_DEV_MOTOR, .pin = 0, .param = 60}, /* ch, timeout */
    {"right_motor", FATTOS_DEV_MOTOR, .pin = 1, .param = 60},
    /* ... */
};
static const FattosBoard board = {"my-bot", devices, 4};

fattos_board_setup_safety(&board, &safety);   /* every MOTOR -> a channel */
fattos_board_setup_vehicle(&board, &veh, &safety, 100, 100);

/* actuator write by NAME (validates kind, then the full supervisor path): */
fattos_board_write_motor(&board, &safety, "left_motor", 400);
```

`fattos_board_write_motor` = kind validation + `fattos_safety_command`
(the entire section 1 path). An optional `FATTOS_DEV_ESTOP` GPIO device
exists for wiring a hardware e-stop input into your port.

---

## 6. Safety checklist (ship this, literally)

- [ ] Every motor registered with a heartbeat timeout ≤ its
      worst-observed outage tolerance.
- [ ] Control task heartbeats **all** channels every cycle — even when
      doing nothing, even when disarmed.
- [ ] All actuator writes go through `fattos_safety_command` /
      `fattos_board_write_motor` — verified by grep for
      `fattos_platform_pwm_write` outside `platform/`.
- [ ] `fattos_set_fault_hook` → vehicle estop registered at startup.
- [ ] `fattos_wdt_register` + feed per critical task; diagnostics task
      calls `fattos_wdt_poll` + `fattos_wdt_kick_hardware`.
- [ ] Sensor manifest set up **before** expecting to arm; boot waits for
      `FATTOS_CAL_DONE`.
- [ ] Main loop drains with `fattos_run_pending(budget, …)`, never an
      unbounded `while (ran)`.
- [ ] FAILSAFE/ESTOP paths manually verified on the bench: pull the RC
      receiver, stall a task, unplug a sensor — each must stop motion.