# FattOS API reference

Complete function reference for every public header, in dependency order.
Signatures are copied from the headers; behavior notes capture what the
implementation actually guarantees. For tutorials see
[getting-started.md](getting-started.md); for design see
[architecture.md](architecture.md).

**Conventions used throughout:**

- All functions are reentrancy-safe unless noted; per-instance state lives
  in caller-owned structs, and the few singletons (sensors, watchdog) say
  so explicitly.
- Durations are milliseconds in public APIs; the kernel converts to ticks
  internally with `fattos_ms_to_ticks()`.
- "Static" means: no `malloc`, ever. All storage is caller-provided or
  compile-time. Every `*_init` function documents who owns the buffers.
- Return type `FattosStatus`: `FATTOS_OK` is the only success; every error
  is distinct and checked (see `fattos.h`).
- An empty struct-out pointer or `NULL` where documented is ignored, not
  dereferenced.

---

## 1. `fattos/fattos.h` — kernel core

### Status codes (`FattosStatus`)

| Value | Meaning |
|---|---|
| `FATTOS_OK` | success |
| `FATTOS_ERR_INVALID_ARG` | NULL or out-of-range argument |
| `FATTOS_ERR_NOT_INITIALIZED` | used before `fattos_init()` |
| `FATTOS_ERR_NO_TASK_SLOTS` | `FATTOS_MAX_TASKS` reached |
| `FATTOS_ERR_PLATFORM` | platform HAL call failed |
| `FATTOS_ERR_NOT_IMPLEMENTED` | valid call, feature not wired on this build/bus |
| `FATTOS_ERR_TIMEOUT` | deadline passed without the condition (also "not fresh" for sensors) |
| `FATTOS_ERR_FULL` | queue/pool full |
| `FATTOS_ERR_EMPTY` | queue empty, or "no frame yet" in byte-stream parsers |
| `FATTOS_ERR_FAULT` | safety supervisor refused the request (latched/timeout) |

### Task info (`FattosTaskInfo`)

Filled by `fattos_task_get_info`. Fields:

- `name`, `function`, `context`, `period_ticks`, `next_release`,
  `run_count`, `priority`, `enabled`
- Profiling: `exec_last_ticks`, `exec_max_ticks`, `missed_releases`,
  `overrun_count`, `jitter_max_ticks`
- Hardening: `overrun_streak` (consecutive overruns since last clean run),
  `quarantined` (set once the streak hit `FATTOS_QUARANTINE_LIMIT`)

### Lifecycle

```c
FattosStatus fattos_init(void);
```
Initializes platform and scheduler state. Call once, before task
registration. Returns `FATTOS_OK` on success, `FATTOS_ERR_PLATFORM` if the
platform init fails.

```c
FattosStatus fattos_task_create(const char *name, FattosTaskFn function,
                                void *context, uint32_t period_ms,
                                uint8_t priority, uint8_t *out_task_id);
```
Register a periodic callback. Higher numeric priority wins among due
tasks. `period_ms` is converted to ticks (rounds up, minimum 1 tick).
`name` must outlive the task (string literal or static). Returns
`FATTOS_ERR_NO_TASK_SLOTS` when full, `FATTOS_ERR_INVALID_ARG` on NULL
function or NULL out_task_id. The task starts enabled.

```c
_Noreturn void fattos_run(void);
```
The normal run loop: executes due tasks and calls
`fattos_platform_idle()` when none are due (the port may sleep until the
next tick interrupt).

```c
FattosStatus fattos_run_once(bool *ran);
```
Execute one due task, if any. No due task is normal:
`FATTOS_OK` with `*ran = false`. Exactly one callback per call, always.

```c
FattosStatus fattos_run_pending(uint32_t max_dispatches, uint32_t *dispatched);
```
Run at most `max_dispatches` due callbacks in priority order; `*dispatched`
(may be NULL) reports how many ran. The bound guarantees a drain loop can
never spin forever — a runaway callback cannot freeze the caller. When
several tasks are due, the highest priority runs first; tasks that become
due during the drain are picked up on the next call.

### Tick source

```c
void fattos_tick_isr(void);
```
Call exactly `FATTOS_TICK_HZ` times per second from a hardware timer ISR.
Sets the periodic tasks free; keep the ISR itself short (the kernel work
is a few counter increments and a priority pick).

### Time

```c
uint32_t fattos_millis(void);        /* ms since boot, from tick count */
uint32_t fattos_ms_to_ticks(uint32_t ms);  /* rounds up; 0 ms -> 0 ticks */
```
`fattos_ms_to_ticks` never returns 0 for a non-zero duration, so a 1 ms
duration at 100 Hz still costs a tick.

### Task control and introspection

```c
FattosStatus fattos_task_enable(uint8_t task_id, bool enabled);
size_t        fattos_task_count(void);
FattosStatus fattos_task_get_info(uint8_t task_id, FattosTaskInfo *out_info);
```
Enable/disable a registered task; inspect profiling counters. Enabling an
already-enabled task (or disabling a disabled one) is `FATTOS_OK`.

### Quarantine (fault isolation)

```c
bool          fattos_task_is_quarantined(uint8_t task_id);
size_t        fattos_task_quarantined_count(void);
FattosStatus  fattos_task_rehab(uint8_t task_id);
```
A task that overruns its period `FATTOS_QUARANTINE_LIMIT` times in a row
is auto-disabled so one runaway cannot sink the whole system's real-time
behavior. Query the count for telemetry (it appears in the status frame),
and call `rehab` after inspecting/fixing the offender — which clears the
streak and re-enables it. Rehab never re-enables a task the application
merely disabled; that stays disabled until the app re-enables it.

### Fault hook

```c
typedef void (*FattosFaultFn)(uint8_t task_id, uint32_t exec_ticks,
                              uint32_t period_ticks);
void fattos_set_fault_hook(FattosFaultFn hook);
```
Called from the scheduler immediately after a callback runs longer than
its period. Use it (safety-first) to latch an e-stop or raise an alarm.
It runs in task context with interrupts still enabled, so it may call
most FattOS APIs. Pass NULL to remove.

### Platform hook

```c
void fattos_platform_idle(void);
```
Optional platform hook called when no task is due; weak-linked default is
a no-op.

---

## 2. `fattos/fattos_config.h` — compile-time knobs

All are `#ifndef`-guarded: override with `-D` or before the include.

| Macro | Default | Meaning |
|---|---|---|
| `FATTOS_MAX_TASKS` | 8 | task table size |
| `FATTOS_TICK_HZ` | 1000 | tick ISR frequency |
| `FATTOS_TENSOR_ARENA_SIZE` | 16384 | static AI arena bytes |
| `FATTOS_TENSOR_ARENA_ALIGNMENT` | 16 | arena alignment |
| `FATTOS_MAX_TIMERS` | 8 | software timer cap (must be > 0) |
| `FATTOS_MAX_WDT_ITEMS` | 8 | watchdog item cap (must be > 0) |
| `FATTOS_QUARANTINE_LIMIT` | 3 | consecutive overruns before auto-disable |
| `FATTOS_MAX_SAFETY_CHANNELS` | 8 | safety supervisor channels (in `fattos_safety.h`) |
| `FATTOS_MAX_COMMANDS` | 16 | command registry size (in `fattos_command.h`) |
| `FATTOS_LOG_RING` / `FATTOS_LOG_TEXT_LEN` | 16 / 48 | logger capacity (in `fattos_log.h`) |
| `FATTOS_BT_MAX_DEPTH` | 16 | behavior-tree depth cap (in `fattos_bt.h`) |

Zero values for the four guarded macros are a compile-time `#error`, not a
runtime surprise.

---

## 3. `fattos/fattos_platform.h` — the port seam

Implement these in an MCU port; the host build provides stubs.

```c
FattosStatus fattos_platform_init(void);
FattosStatus fattos_platform_timer_start(uint32_t tick_hz);
void         fattos_platform_gpio_write(uint16_t pin, bool high);
bool         fattos_platform_gpio_read(uint16_t pin);
FattosStatus fattos_platform_pwm_write(uint8_t channel, uint16_t duty_permille);
FattosStatus fattos_platform_read_imu(int16_t *ax, int16_t *ay, int16_t *az);
FattosStatus fattos_platform_imu_read6(float accel_g[3], float gyro_rads[3]);
void         fattos_platform_idle(void);
uint32_t     fattos_platform_irq_lock(void);
void         fattos_platform_irq_unlock(uint32_t key);
FattosStatus fattos_platform_watchdog_kick(void);
```

Notes:

- `fattos_platform_imu_read6` is the board-managed 6-axis sample for the
  device layer: accel in g, gyro in rad/s, already scaled by the port.
  Ports with raw-only sensors convert here; board manifests reference it
  through `FATTOS_DEV_IMU`.
- `irq_lock`/`unlock`: disable interrupts, return an opaque key, restore
  on unlock. MCU ports must save/restore full interrupt state (e.g.
  PRIMASK). Nested pairs with the same key must be balanced by the caller.
- `watchdog_kick`: the kernel calls this only while all health checks
  pass; ports may reset the MCU when this is starved.

---

## 4. `fattos/fattos_timer.h` — software timers

```c
FattosStatus fattos_timer_init(FattosTimer *timer, const char *name,
                               FattosTimerFn function, void *context);
FattosStatus fattos_timer_start(FattosTimer *timer, uint32_t period_ms,
                                bool autoreload);
FattosStatus fattos_timer_stop(FattosTimer *timer);
bool         fattos_timer_is_active(const FattosTimer *timer);
void         fattos_timer_service(void);     /* fire expired timers */
FattosStatus fattos_timers_register(uint32_t service_period_ms,
                                    uint8_t priority, uint8_t *out_task_id);
size_t       fattos_timer_count(void);
```

One-shot or auto-reload. Timers fire in the context of whichever task
calls `fattos_timer_service()` — keep callbacks short and non-blocking.
Convenience: `fattos_timers_register` creates the service task for you at
1 ms granularity.

---

## 5. `fattos/fattos_queue.h` — message queues

```c
FattosStatus fattos_queue_init(FattosQueue *queue, void *storage,
                               size_t item_size, size_t capacity);
FattosStatus fattos_queue_send(FattosQueue *queue, const void *item);
FattosStatus fattos_queue_receive(FattosQueue *queue, void *out_item);
FattosStatus fattos_queue_peek(const FattosQueue *queue, void *out_item);
size_t       fattos_queue_count(const FattosQueue *queue);
bool         fattos_queue_is_empty(const FattosQueue *queue);
bool         fattos_queue_is_full(const FattosQueue *queue);
```

Fixed-capacity, copy-in/copy-out, caller-provided storage. Send fails
`FATTOS_ERR_FULL`, receive `FATTOS_ERR_EMPTY`. Safe for producer/consumer
across an interrupt boundary when the platform implements
`irq_lock`/`unlock` for real.

---

## 6. `fattos/fattos_event.h` — event flags

```c
void     fattos_event_init(FattosEventGroup *group);
void     fattos_event_set(FattosEventGroup *group, uint32_t bits);
void     fattos_event_clear(FattosEventGroup *group, uint32_t bits);
uint32_t fattos_event_get(const FattosEventGroup *group);
bool     fattos_event_poll(const FattosEventGroup *group, uint32_t bits,
                           bool wait_all);
```

32-bit flag groups. Non-blocking by design (a cooperative scheduler
cannot block a task): poll and continue. `wait_all=true` requires every
listed bit; false requires any one.

---

## 7. `fattos/fattos_mem.h` — fixed-block pool

```c
FattosStatus fattos_pool_init(FattosPool *pool, void *storage,
                              size_t block_size, size_t block_count);
void        *fattos_pool_alloc(FattosPool *pool);
void         fattos_pool_free(FattosPool *pool, void *block);
size_t       fattos_pool_free_count(const FattosPool *pool);
```

O(1) alloc/free, no fragmentation. Caller-provided storage; each block
must be pointer-aligned and at least `sizeof(void*)` (the free list lives
in free blocks). Alloc/free use critical sections, so an ISR producer and
task consumer cannot corrupt the free list. `alloc` returns NULL when
exhausted — check it.

---

## 8. `fattos/fattos_watchdog.h` — cooperative deadman

```c
FattosStatus fattos_wdt_register(const char *name, uint32_t timeout_ms,
                                 uint8_t *out_id);
FattosStatus fattos_wdt_feed(uint8_t id);
FattosStatus fattos_wdt_poll(void);
bool         fattos_wdt_is_healthy(void);
uint8_t      fattos_wdt_fault_id(void);    /* first expired id, or 0xFF */
size_t       fattos_wdt_count(void);
FattosStatus fattos_wdt_get_info(uint8_t id, FattosWdtItem *out_item);
FattosStatus fattos_wdt_kick_hardware(void);
```

Each item must be fed within its timeout. `fattos_wdt_poll` re-evaluates
all items (call from a periodic task).
`fattos_wdt_kick_hardware()` forwards to
`fattos_platform_watchdog_kick()` **only while every item is healthy**, so
a stalled task starves the hardware watchdog and triggers a reset on MCU
ports. Single global registry (there is one per system).

---

## 9. `fattos/fattos_bus.h` — peripheral bus vtable

```c
typedef struct {
    FattosStatus (*i2c_write_read)(void *ctx, uint8_t addr7,
                                   const uint8_t *wr, size_t wr_len,
                                   uint8_t *rd, size_t rd_len);
    FattosStatus (*spi_transfer)(void *ctx, uint16_t cs_pin,
                                 const uint8_t *tx, uint8_t *rx, size_t len);
    FattosStatus (*uart_write)(void *ctx, uint8_t port,
                               const uint8_t *buf, size_t len);
    int          (*uart_read)(void *ctx, uint8_t port, uint8_t *buf, size_t cap);
    FattosStatus (*can_send)(void *ctx, uint32_t id,
                             const uint8_t *data, size_t len);
    int          (*can_recv)(void *ctx, uint32_t *id, uint8_t *data, size_t cap);
    void *ctx;
} FattosBusOps;

typedef struct { const char *name; const FattosBusOps *ops; } FattosBus;
```

One vtable per bus instance, so a board can mix several I2C buses, an SPI
bus and a UART without global state. NULL ops for a class means "not
wired": helpers return `FATTOS_ERR_NOT_IMPLEMENTED`, never a crash.
`i2c_write_read` is a write then repeated-start read in one transaction.
UART/CAN `*_read`/`*_recv` return bytes stored (0 = empty, < 0 = error).

Helpers:

```c
FattosStatus fattos_i2c_write(const FattosBus *bus, uint8_t addr7,
                              const uint8_t *data, size_t len);
FattosStatus fattos_i2c_read(const FattosBus *bus, uint8_t addr7, uint8_t reg,
                             uint8_t *data, size_t len);
FattosStatus fattos_i2c_write_u8(const FattosBus *bus, uint8_t addr7,
                                 uint8_t reg, uint8_t value);
FattosStatus fattos_i2c_read_u8(const FattosBus *bus, uint8_t addr7,
                                uint8_t reg, uint8_t *value);
FattosStatus fattos_spi_transfer(const FattosBus *bus, uint16_t cs_pin,
                                 const uint8_t *tx, uint8_t *rx, size_t len);
FattosStatus fattos_uart_write(const FattosBus *bus, uint8_t port,
                               const uint8_t *buf, size_t len);
int          fattos_uart_read(const FattosBus *bus, uint8_t port,
                              uint8_t *buf, size_t cap);
FattosStatus fattos_can_send(const FattosBus *bus, uint32_t id,
                             const uint8_t *data, size_t len);
int          fattos_can_recv(const FattosBus *bus, uint32_t *id,
                             uint8_t *data, size_t cap);
```

---

## 10. `fattos/fattos_sensor.h` — the sensor subsystem

### Kinds and health

```c
typedef enum {
    FATTOS_SENSOR_NONE = 0, FATTOS_SENSOR_MPU6050, FATTOS_SENSOR_QMC5883,
    FATTOS_SENSOR_BMP280, FATTOS_SENSOR_VL53L0X, FATTOS_SENSOR_COUNT
} FattosSensorKind;

typedef enum {
    FATTOS_SENSOR_OK = 0,       /* present, fresh, plausible */
    FATTOS_SENSOR_MISSING,      /* not in manifest / probe failed */
    FATTOS_SENSOR_ERROR,        /* consecutive bus errors */
    FATTOS_SENSOR_STALE,        /* no fresh sample within 3x its period */
    FATTOS_SENSOR_PLAUSIBLE     /* sample out of physical bounds */
} FattosSensorHealth;
```

### Sample and manifest entry

```c
typedef struct {
    uint32_t timestamp_ms;
    uint8_t valid;              /* 1=accel 2=gyro 4=mag 8=pressure 16=temp 32=range */
    float accel_g[3];           /* g */
    float gyro_rads[3];         /* rad/s */
    float mag_ut[3];            /* microtesla */
    float pressure_pa;          /* Pa */
    float temp_c;               /* C */
    float range_m;              /* meters */
} FattosSensorSample;

typedef struct {
    const char *name;           /* unique, used by fattos_sensor_read */
    uint8_t kind;               /* FattosSensorKind */
    uint8_t bus;                /* index into bus table passed to setup */
    uint8_t addr7;              /* 7-bit I2C address */
    uint16_t rate_hz;           /* requested output data rate */
    float range_g;              /* IMU accel full-scale (g), 0 = driver default */
    float range_dps;            /* IMU gyro full-scale (deg/s), 0 = driver default */
} FattosSensorEntry;
```

Limits: `FATTOS_SENSOR_NAME_MAX` 12 chars, `FATTOS_MAX_SENSORS` 8.

### Functions

```c
size_t       fattos_sensor_setup(const FattosSensorEntry *entries,
                                 size_t count, const FattosBus *buses,
                                 size_t bus_count);
```
Probe + configure + start every sensor in the manifest; returns how many
came up OK. A sensor that fails probe is marked MISSING and does not abort
the others. `buses` may be NULL when the manifest is empty.

```c
size_t       fattos_sensor_poll(uint32_t now_ms);
```
Poll all sensors: reads those due at their configured rate, applies
auto-calibration, checks plausibility, updates health. Returns the number
of sensors read this call.

```c
FattosStatus fattos_sensor_read(const char *name, FattosSensorSample *out);
```
Latest converted sample by name. `FATTOS_ERR_INVALID_ARG` for unknown
name; `FATTOS_ERR_TIMEOUT` when stale/faulted — the sample is still filled
with last good data. `FATTOS_OK` only for present + fresh + plausible.

```c
bool         fattos_sensors_prearm_ok(void);
```
True when every declared sensor is present, fresh and plausible. Wire
straight into `fattos_vehicle_set_prearm()`. False when the manifest is
empty.

```c
size_t       fattos_sensor_count(void);
FattosStatus fattos_sensor_get_info(size_t index, const char **out_name,
                                    uint8_t *out_kind, uint8_t *out_health,
                                    uint32_t *out_errors);
const char  *fattos_sensor_kind_name(FattosSensorKind kind);
const char  *fattos_sensor_health_name(FattosSensorHealth health);
```

Health table for telemetry/diagnostics.

```c
FattosStatus fattos_sensor_calibrate(const char *name, FattosCalState what,
                                     uint16_t samples);
FattosCalState fattos_sensor_cal_state(const char *name);
```
Auto-calibration. `FATTOS_CAL_GYRO_BIAS` averages gyro bias at rest
(default 200 samples; MPU6050 only today). `FATTOS_CAL_IDLE` cancels.
`cal_state` returns `FATTOS_CAL_FAILED` for an unknown name.

```c
void         fattos_sensor_reset(void);
```
Test/host hook: replace the bus table (e.g. with a simulated bus) and
reset all driver state. Passing NULL clears everything.

---

## 11. `fattos/fattos_imu.h` — attitude estimation

```c
void         fattos_imu_init(FattosIMUState *state);
void         fattos_imu_update(FattosIMUState *state, const float gyro[3],
                               const float accel[3], float dt_seconds);
void         fattos_imu_update_mag(FattosIMUState *state, const float gyro[3],
                                   const float accel[3], const float mag[3],
                                   float dt_seconds);
void         fattos_imu_yaw_correct(FattosIMUState *state, float yaw_meas,
                                    float dt_seconds);
FattosStatus fattos_imu_euler(const FattosIMUState *state, float *roll,
                              float *pitch, float *yaw);
```

Mahony-style complementary filter with gyro-bias integral feedback. Float
math (Cortex-M4F/M7F class). Gyro rad/s; accel any consistent unit
(normalized internally, treated as gravity direction while quasi-static).

`fattos_imu_update_mag` adds tilt-compensated magnetometer heading fusion
at `kp_mag`, removing gyro-integration yaw drift.
`fattos_imu_yaw_correct` rotates attitude yaw toward an absolute
observation (GPS heading, landmark fix) at `kp_mag`.

`fattos_imu_euler` returns radians: roll (x), pitch (y), yaw (z).
Struct fields: quaternion `q0..q3`, `integral_fb[3]` (bias estimate),
gains `kp`, `ki`, `kp_mag`, counters `updates`, `mag_updates`.

---

## 12. `fattos/fattos_odometry.h` — diff-drive pose + mixer

```c
FattosStatus fattos_odometry_init(FattosOdometry *odom, float wheel_radius_m,
                                  float wheel_base_m, float ticks_per_rev);
void         fattos_odometry_set_pose(FattosOdometry *odom, float x, float y,
                                      float theta);
void         fattos_odometry_update(FattosOdometry *odom, int32_t left_delta,
                                    int32_t right_delta);
FattosStatus fattos_diff_mix(float linear, float angular, float max_wheel_speed,
                             float *left_speed, float *right_speed);
```

Integrates encoder tick deltas into a planar pose `(x m, y m, theta rad
wrapped to [-pi, pi])`. `fattos_diff_mix` is the inverse mixer:
`linear` m/s forward, `angular` is omega·base/2 in m/s of differential
speed; outputs are clamped to ±`max_wheel_speed`.

---

## 13. `fattos/fattos_nav.h` — go-to-goal

```c
FattosStatus fattos_nav_init(FattosNav *nav, float k_linear,
                             float max_linear, float arrive_radius);
bool         fattos_nav_step(FattosNav *nav, float pose_x, float pose_y,
                             float pose_theta, float goal_x, float goal_y,
                             float *linear_out, float *heading_out);
float        fattos_wrap_pi(float angle);
```

Defaults on init: `k=1.0` m/s per m capped at 0.4, arrive within 5 cm.
`nav_step` returns true once inside the arrival radius (`linear_out` 0);
`heading_out` is the wrapped bearing to goal (rad). Speed scales with
remaining distance and fades as heading error grows, so the robot pivots
before charging. Struct counters `arrived`, `goals_reached`.

---

## 14. `fattos/fattos_pid.h` — PID controller

```c
FattosStatus fattos_pid_init(FattosPID *pid, float kp, float ki, float kd,
                             float out_min, float out_max, float d_alpha);
void         fattos_pid_reset(FattosPID *pid);
float        fattos_pid_step(FattosPID *pid, float setpoint, float measurement,
                             float dt_seconds);
float        fattos_pid_step_ff(FattosPID *pid, float setpoint,
                                float measurement, float dt_seconds,
                                float feedforward);
void         fattos_pid_seed(FattosPID *pid, float setpoint, float measurement,
                             float output);
```

Derivative acts on measurement (no setpoint-kick); `d_alpha` (0..1) is a
first-order low-pass on the derivative term; conditional-integration
anti-windup against output clamping. `dt_seconds` must be > 0 and
consistent. `step_ff` adds a feedforward term **before** clamping
(anti-windup accounts for it) — use it for known plant effort
(ff = mass·accel + friction), letting the PID trim model error.
`seed` enables bumpless transfer: the first step after enabling outputs
`output` with no integral windup.

---

## 15. `fattos/fattos_motion.h` — trapezoidal profile

```c
FattosStatus fattos_motion_init(FattosMotionProfile *profile,
                                float max_speed, float accel);
void         fattos_motion_set_target(FattosMotionProfile *profile, float target);
void         fattos_motion_set_state(FattosMotionProfile *profile,
                                     float position, float velocity);
float        fattos_motion_step(FattosMotionProfile *profile, float dt_seconds);
bool         fattos_motion_done(const FattosMotionProfile *profile);
```

Accel-limited reference generator for point-to-point moves. Step at the
control rate; feed `step`'s position as setpoint and `velocity` as
feedforward to `fattos_pid_step_ff`. Handles retargeting mid-profile and
never overshoots the target. `set_state` syncs external state (measured
pose handed back in). `done` = target reached and stopped.

---

## 16. `fattos/fattos_safety.h` — actuator supervisor

### Types

```c
typedef enum {
    FATTOS_SAFETY_OK = 0,
    FATTOS_SAFETY_ESTOP_LATCHED,
    FATTOS_SAFETY_HEARTBEAT_TIMEOUT
} FattosSafetyState;
```

`FattosSafetyChannel`: `{name, pwm_channel, timeout_ticks, last_beat,
active, timed_out, last_duty, slew_limit}`.
`FattosSafety`: channel array (cap `FATTOS_MAX_SAFETY_CHANNELS`, 8),
`count`, `estop`, `state`, `fault_channel` (0xFF if none).

### Functions

```c
FattosStatus fattos_safety_init(FattosSafety *safety);
FattosStatus fattos_safety_register_channel(FattosSafety *safety, const char *name,
                                            uint8_t pwm_channel,
                                            uint32_t timeout_ms, uint8_t *out_id);
FattosStatus fattos_safety_heartbeat(FattosSafety *safety, uint8_t id);
FattosStatus fattos_safety_estop(FattosSafety *safety);
FattosStatus fattos_safety_estop_clear(FattosSafety *safety);
FattosSafetyState fattos_safety_poll(FattosSafety *safety);
FattosStatus fattos_safety_command(FattosSafety *safety, uint8_t id,
                                   uint16_t duty_permille);
bool         fattos_safety_is_safe(const FattosSafety *safety);
FattosSafetyState fattos_safety_state(const FattosSafety *safety);
int          fattos_safety_find(const FattosSafety *safety, const char *name);
FattosStatus fattos_safety_set_slew(FattosSafety *safety, uint8_t id,
                                    uint16_t max_delta);
uint16_t     fattos_safety_last_duty(const FattosSafety *safety, uint8_t id);
uint8_t      fattos_safety_fault_channel(const FattosSafety *safety);
```

Every actuator channel must be heartbeated within its timeout (typically
by the control task). On timeout or latched e-stop, the supervisor
immediately drives every registered PWM channel to zero (fail-safe) and
reports fault. Recovery requires explicit `estop_clear` after conditions
are healthy (refused while any channel is still timed out).

`fattos_safety_command` is the **SAFETY-FIRST actuator path**: it
re-evaluates timeouts/latch freshly (does not trust the last poll), forces
zero duty and returns `FATTOS_ERR_FAULT` on any non-OK state, and clamps
duty into [0,1000]. Route every motor command through it — never the raw
platform HAL. `set_slew` limits duty change per command (0 disables);
faults and e-stops bypass the limit and go straight to 0 — safety beats
smoothness.

---

## 17. `fattos/fattos_vehicle.h` — arming state machine

```c
typedef enum {
    FATTOS_VEHICLE_INIT = 0, FATTOS_VEHICLE_DISARMED, FATTOS_VEHICLE_ARMED,
    FATTOS_VEHICLE_FAILSAFE, FATTOS_VEHICLE_ESTOP
} FattosVehicleState;

FattosStatus      fattos_vehicle_init(FattosVehicle *vehicle, FattosSafety *safety,
                                      uint32_t arm_hold_ms, uint32_t rc_timeout_ms);
void              fattos_vehicle_set_rc(FattosVehicle *vehicle, bool present,
                                        bool arm_switch);
void              fattos_vehicle_set_prearm(FattosVehicle *vehicle, bool ok);
FattosVehicleState fattos_vehicle_update(FattosVehicle *vehicle);
bool              fattos_vehicle_can_actuate(const FattosVehicle *vehicle);
FattosStatus      fattos_vehicle_estop(FattosVehicle *vehicle);
FattosStatus      fattos_vehicle_estop_clear(FattosVehicle *vehicle);
const char       *fattos_vehicle_state_name(FattosVehicleState state);
```

Transitions: `INIT → DISARMED → (RC ok + arm switch held arm_hold_ms +
prearm ok + supervisor healthy) → ARMED`. `ARMED → FAILSAFE` on RC loss,
RC failsafe flag, or supervisor fault. `ARMED/FAILSAFE → ESTOP` on latched
e-stop. `FAILSAFE → DISARMED` when healthy again (pilot must re-arm).
`ESTOP → DISARMED` only via explicit `estop_clear`.

`can_actuate` is the single "may I move?" question: true only when ARMED
**and** the safety supervisor is healthy. Gate every actuator command on
it. `update` advances the state machine one control cycle and returns the
new state. Struct counters: `state_changes` (diagnostics/telemetry).

---

## 18. `fattos/fattos_board.h` — board manifest

```c
typedef enum {
    FATTOS_DEV_NONE = 0, FATTOS_DEV_MOTOR = 1, FATTOS_DEV_ODOMETRY = 2,
    FATTOS_DEV_IMU = 3, FATTOS_DEV_RC = 4, FATTOS_DEV_ESTOP = 5
} FattosDeviceKind;

typedef struct {
    const char *name;   /* unique id used by lookups and safety channels */
    uint8_t kind;       /* FattosDeviceKind */
    uint16_t pin;       /* primary pin / PWM channel / bus index */
    uint16_t pin2;      /* secondary pin (SCL, address, dir) */
    uint32_t param;     /* kind-specific integer */
    float scale;        /* kind-specific float */
    float offset;       /* kind-specific float */
} FattosDevice;

typedef struct {
    const char *name;
    const FattosDevice *devices; /* usually a static const array */
    size_t count;
} FattosBoard;
```

Kind slot meanings: MOTOR — `pin`=PWM channel, `param`=timeout ms.
ODOMETRY — `param`=ticks/rev, `scale`=wheel radius m, `offset`=wheel base
m. IMU — `pin`=bus, `pin2`=address/aux. RC — `pin`=uart. ESTOP — GPIO
input.

```c
void         fattos_board_init(FattosBoard *board, const char *name,
                               const FattosDevice *devices, size_t count);
const FattosDevice *fattos_board_find(const FattosBoard *board, const char *name);
const FattosDevice *fattos_board_find_kind(const FattosBoard *board,
                                           FattosDeviceKind kind);
FattosStatus fattos_board_setup_safety(const FattosBoard *board, FattosSafety *safety);
FattosStatus fattos_board_setup_odometry(const FattosBoard *board, FattosOdometry *odom);
FattosStatus fattos_board_setup_vehicle(const FattosBoard *board,
                                        FattosVehicle *vehicle,
                                        FattosSafety *safety,
                                        uint32_t arm_hold_ms,
                                        uint32_t rc_timeout_ms);
FattosStatus fattos_board_setup_rc(const FattosBoard *board, FattosRCState *rc);
FattosStatus fattos_board_write_motor(const FattosBoard *board,
                                      FattosSafety *safety, const char *name,
                                      uint16_t duty_permille);
FattosStatus fattos_board_read_imu(const FattosBoard *board, float accel_g[3],
                                   float gyro_rads[3]);
```

Derived setup builds the OS's tables from the manifest: every MOTOR
becomes a safety channel (heartbeat + slew gate); the odometry device
supplies wheel geometry; `setup_vehicle` requires the RC device;
`write_motor` validates the device kind then routes through the safety
supervisor (fresh state check, clamp, slew, fail-safe); `read_imu` goes
through the platform HAL.

---

## 19. `fattos/fattos_telemetry.h` — framing

```c
#define FATTOS_TELEMETRY_SOF 0xA5U
#define FATTOS_TELEMETRY_MAX_PAYLOAD 64U
#define FATTOS_TELEMETRY_TYPE_STATUS 0x10U

typedef struct {
    uint8_t type, seq, len;
    const uint8_t *payload;  /* valid only during the FATTOS_OK delivery call */
} FattosTelemetryFrame;

size_t       fattos_telemetry_encode(uint8_t type, uint8_t seq,
                                     const void *payload, uint8_t len,
                                     uint8_t *out, size_t out_cap);
size_t       fattos_telemetry_encode_status(uint8_t seq, uint8_t *out,
                                            size_t out_cap);
void         fattos_telemetry_decoder_init(FattosTelemetryDecoder *decoder);
FattosStatus fattos_telemetry_decoder_push(FattosTelemetryDecoder *decoder,
                                           uint8_t byte,
                                           FattosTelemetryFrame *out_frame);
```

Frame: `[0xA5][type][seq][len][payload…][crc16-lo][crc16-hi]`; CRC-16/CCITT
covers type+seq+len+payload. `encode` returns bytes written, 0 on error.
Status frame packs per-sensor kind/health/errors plus quarantined-task
count (see `fattos_sensor_get_info`, `fattos_task_quarantined_count`).

Decoder is byte-at-a-time, self-resyncing: `FATTOS_OK` on a complete valid
frame; `FATTOS_ERR_EMPTY` mid-frame (normal) or after a CRC reject
(`frames_rejected++`); `FATTOS_ERR_INVALID_ARG` on NULL.
`FattosTelemetryDecoder` fields: `state`, `type`, `seq`, `len`, `index`,
`crc_lo`, `payload[64]`, `frames_ok`, `frames_rejected`. States:
`FATTOS_TELEMETRY_DEC_IDLE/TYPE/SEQ/LEN/PAYLOAD/CRC_LO/CRC_HI`.

---

## 20. `fattos/fattos_command.h` — command registry

```c
#define FATTOS_CMD_REQUEST 0x20U
#define FATTOS_CMD_REPLY 0x21U
#define FATTOS_CMD_STATUS_OK 0x00U
#define FATTOS_CMD_STATUS_UNKNOWN 0x01U
#define FATTOS_CMD_STATUS_BAD_ARGS 0x02U
#define FATTOS_CMD_STATUS_FAILED 0x03U

typedef uint8_t (*FattosCmdHandler)(void *context, const uint8_t *args,
                                    uint8_t args_len, uint8_t *reply,
                                    uint8_t *reply_len, uint8_t reply_cap);

FattosStatus fattos_cmd_registry_init(FattosCommandRegistry *registry);
FattosStatus fattos_cmd_register(FattosCommandRegistry *registry, uint8_t id,
                                 FattosCmdHandler handler, void *context);
FattosStatus fattos_cmd_dispatch(FattosCommandRegistry *registry,
                                 const FattosTelemetryFrame *frame,
                                 uint8_t seq, uint8_t *out, size_t out_cap,
                                 size_t *out_len);
```

Request payload `[cmd_id][args…]`; reply `[cmd_id][status][reply…]`.
`dispatch` returns `FATTOS_OK` when the frame was a command request
(handled/unknown/malformed — see `*out_len` for the encoded reply, 0 when
not a command frame) and `FATTOS_ERR_EMPTY` for non-command frames.
Registry counters: `handled`, `unknown`, `malformed`. Cap
`FATTOS_MAX_COMMANDS` (16). Handler runs in the dispatching task's
context — keep it bounded and non-blocking.

---

## 21. `fattos/fattos_rc.h` — SBUS decoder

```c
#define FATTOS_RC_MAX_CHANNELS 16U
#define FATTOS_RC_FRAME_LEN 25U
#define FATTOS_RC_SOF 0x0FU

void         fattos_rc_init(FattosRCState *rc);
FattosStatus fattos_rc_push(FattosRCState *rc, uint8_t byte);
bool         fattos_rc_signal_ok(const FattosRCState *rc, uint32_t now_ms,
                                 uint32_t timeout_ms);
uint16_t     fattos_rc_channel_raw(const FattosRCState *rc, uint8_t channel);
float        fattos_rc_channel_norm(const FattosRCState *rc, uint8_t channel);
float        fattos_rc_channel_throttle(const FattosRCState *rc, uint8_t channel);
```

Feed raw serial bytes one per call; `FATTOS_OK` exactly when a frame
completed and validated. Channel range 172..1811, center 992 (SBUS
native). `raw` returns 0 out of range; `norm` −1..+1 around center;
`throttle` 0..1. Struct counters: `frames_ok`, `frames_bad`,
`last_frame_ms`, flags `frame_lost`/`failsafe`/`valid`.

---

## 22. `fattos/fattos_log.h` — ring logger

```c
typedef enum { FATTOS_LOG_DEBUG = 0, FATTOS_LOG_INFO = 1,
               FATTOS_LOG_WARN = 2, FATTOS_LOG_ERROR = 3 } FattosLogLevel;

void        fattos_log_init(FattosLogger *logger, FattosLogLevel min_level);
void        fattos_log(FattosLogger *logger, FattosLogLevel level,
                       const char *fmt, ...);
void        fattos_log_v(FattosLogger *logger, FattosLogLevel level,
                         const char *fmt, va_list args);
void        fattos_log_str(FattosLogger *logger, FattosLogLevel level,
                           const char *text);
const FattosLogEntry *fattos_log_at(const FattosLogger *logger, size_t index);
const char *fattos_log_level_name(FattosLogLevel level);
```

Never allocates, never blocks; overflow drops oldest (counted in
`dropped`). `log_at` index 0 = newest; NULL out of range. Entry:
`{level, timestamp_ms, text[48]}`. Counters: `total`, `dropped`,
`filtered`. From ISRs prefer `fattos_log_str` and keep messages short.

---

## 23. `fattos/fattos_crc.h` — CRC primitives

```c
uint16_t fattos_crc16(const void *data, size_t length);  /* CCITT-FALSE,
   poly 0x1021, init 0xFFFF, check("123456789") = 0x29B1 */
uint32_t fattos_crc32(const void *data, size_t length);  /* reflected
   0xEDB88320, init 0xFFFFFFFF, final xor, check = 0xCBF43926 */
```

---

## 24. `fattos/fattos_bt.h` — behavior trees

```c
typedef enum { FATTOS_BT_SUCCESS = 0, FATTOS_BT_FAILURE = 1,
               FATTOS_BT_RUNNING = 2 } FattosBTStatus;
typedef FattosBTStatus (*FattosBTTickFn)(void *blackboard);
typedef enum { FATTOS_BT_SEQUENCE = 0, FATTOS_BT_SELECTOR = 1,
               FATTOS_BT_INVERTER = 2, FATTOS_BT_ACTION = 3,
               FATTOS_BT_CONDITION = 4 } FattosBTNodeType;

FattosStatus  fattos_bt_init(FattosBT *bt, FattosBTNode *nodes, size_t count,
                             void *blackboard);
FattosBTStatus fattos_bt_tick(FattosBT *bt);
```

Static tree, ticked from a periodic task, operating on a caller-owned
blackboard. Node: `{type, first_child, child_count, resume_child, tick,
context}`. Build parent-before-child, root at index 0, composite children
contiguous from `first_child`. SEQUENCE ticks children in order (first
FAILURE fails); SELECTOR (first SUCCESS succeeds); RUNNING children are
resumed next tick (memory). INVERTER flips SUCCESS/FAILURE, passes
RUNNING. ACTION/CONDITION leaves call `tick(blackboard)`.
`bt_init` validates shape; a broken tree ticks `FATTOS_BT_FAILURE` — never
asserts. Depth cap `FATTOS_BT_MAX_DEPTH` (16).

---

## 25. `fattos/fattos_nn.h` — INT8 NN runtime

```c
typedef enum { FATTOS_NN_OP_FULLY_CONNECTED = 0, FATTOS_NN_OP_RELU = 1 } FattosNNOp;

FattosStatus fattos_nn_bind(const FattosNNModel *model);
bool         fattos_nn_is_bound(void);
FattosStatus fattos_nn_run(const int8_t *input, size_t input_len,
                           int8_t *output, size_t output_len);
size_t       fattos_nn_output_len(void);
size_t       fattos_nn_input_len(void);
int          fattos_nn_argmax(const int8_t *values, size_t count);
FattosStatus fattos_nn_make_scale(float scale, int32_t *multiplier, int32_t *shift);
int8_t       fattos_nn_quantize_value(float value, float scale);
```

Fully-connected INT8 interpreter executing inside the static tensor arena
(`FATTOS_TENSOR_ARENA_SIZE`, default 16 KB). No malloc, no bytecode. Model
format: FC layers `int8 weights[out][in]`, `int32 bias[out]`, per-layer
requant `(multiplier, shift)`; ReLU = symmetric-INT8 clamp [0,127]; zero
point always 0. `nn_bind` validates the shape chain and fits scratch at
startup; `nn_run` is blocking, bounded, caller-context;
`input_len`/`output_len` must match the model ends.
`nn_make_scale` converts any float scale to a positive-int32
(multiplier, shift) pair; `nn_quantize_value` float→int8 with scale,
clamped. For CNNs/large models, integrate TFLM/CMSIS-NN behind
`FattosAIIO` (see `fattos_ai.h`).

---

## 26. `fattos/fattos_features.h` — feature pipeline

```c
FattosStatus fattos_features_init(FattosFeatureWindow *window, float *storage,
                                  size_t window_length, size_t axes);
FattosStatus fattos_features_push(FattosFeatureWindow *window, const float *sample);
size_t       fattos_features_count(const FattosFeatureWindow *window);
FattosStatus fattos_features_mean(const FattosFeatureWindow *window, float *out);
FattosStatus fattos_features_rms(const FattosFeatureWindow *window, float *out);
size_t       fattos_features_vector_len(const FattosFeatureWindow *window);
FattosStatus fattos_features_vector(const FattosFeatureWindow *window, float *out);
FattosStatus fattos_features_quantize(const float *vector, size_t length,
                                      float scale, int8_t *out);
```

Rolling window of multi-axis samples with mean/RMS extraction and INT8
quantization, feeding the NN runtime. Caller-provided storage
(`window_length × axes` floats), nothing allocates.
`vector` flattens `[mean(axes), rms(axes)]`, length `2×axes`;
`vector_len` gives that length; `quantize` is symmetric int8, clamped.

---

## 27. `fattos/fattos_ai.h` — backend seam

```c
typedef struct {
    const int8_t *input;  size_t input_size;
    int8_t *output;       size_t output_size;
} FattosAIIO;

FattosStatus fattos_ai_init(void);
FattosStatus fattos_ai_infer(const FattosAIIO *io);
size_t       fattos_ai_arena_size(void);
void        *fattos_ai_arena_data(void);
```

The seam a heavier runtime (TFLM, CMSIS-NN) binds to. The static arena is
reserved by FattOS; a backend must bind/use it when integrated.
`arena_size`/`arena_data` expose the reserved region.

---

## 28. `platform/host/fattos_host.h` — host test helpers

```c
void fattos_host_tick(void);              /* advance simulated MCU time 1 tick */
void fattos_host_advance(uint32_t ticks); /* advance N ticks */
```

Only in the host build; drive time deterministically in tests instead of
sleeping.

---

## See also

- [getting-started.md](getting-started.md) — build and first program
- [kernel.md](kernel.md) — scheduler internals, quarantine, hardening
- [sensors.md](sensors.md) — sensor manifest, driver catalog, fusion
- [safety.md](safety.md) — supervisor/vehicle/prearm chain
- [control.md](control.md) — PID, motion, odometry, nav, heading fusion
- [communications.md](communications.md) — telemetry, commands, RC, logging
- [writing-drivers.md](writing-drivers.md) — add chips, buses, device kinds
- [edge-ai.md](edge-ai.md) — NN runtime and feature pipeline in depth
- [autonomy.md](autonomy.md) — behavior trees and go-to-goal
- [architecture.md](architecture.md) — how the layers fit together
- [porting-stm32.md](porting-stm32.md) — MCU port checklist
