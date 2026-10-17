# Sensors & peripheral drivers

> **Declare the part. The OS knows the part.**

A FattOS board manifest says *what* chip sits *where*, at *what rate*.
The sensor subsystem does everything else: binds the right driver, probes
WHO_AM_I, programs the chip from plain parameters (data rate, full-scale
range), paces reads into rate groups, converts raw counts into SI units,
runs auto-calibration, checks physical plausibility, tracks health, and
gates vehicle arming. Application code never touches a chip register.

Headers: `fattos/fattos_sensor.h`, `fattos/fattos_bus.h`
Sources: `robot/fattos_sensor.c`, `robot/fattos_bus.c`, `robot/fattos_fusion.c`
Host simulator: `platform/host/fattos_sim_bus.{h,c}`

Contents:
1. [The manifest](#1-the-manifest)
2. [The bus HAL](#2-the-bus-hal)
3. [Setup: what happens at boot](#3-setup-what-happens-at-boot)
4. [Reading sensors](#4-reading-sensors)
5. [Rate groups: `fattos_sensor_poll`](#5-rate-groups-fattos_sensor_poll)
6. [Health model](#6-health-model)
7. [Auto-calibration](#7-auto-calibration)
8. [Driver catalog](#8-driver-catalog)
9. [Deriving everything: heading, altitude, climb rate](#9-deriving-everything-heading-altitude-climb-rate)
10. [Arming gate](#10-arming-gate)
11. [Diagnostics & telemetry](#11-diagnostics--telemetry)
12. [Testing on the host simulator](#12-testing-on-the-host-simulator)
13. [Adding a new chip](#13-adding-a-new-chip)

---

## 1. The manifest

```c
#include "fattos/fattos_sensor.h"

/* name, kind, bus index, 7-bit addr, ODR Hz, accel range g, gyro range dps */
static const FattosSensorEntry sensors[] = {
    { "imu",  FATTOS_SENSOR_MPU6050,  0, 0x68, 200, 4.0f,  500.0f },
    { "mag",  FATTOS_SENSOR_QMC5883,  0, 0x0D,  50, 0.0f,    0.0f },
    { "baro", FATTOS_SENSOR_BMP280,   0, 0x76,  25, 0.0f,    0.0f },
    { "tof",  FATTOS_SENSOR_VL53L0X,  0, 0x29,  20, 0.0f,    0.0f },
};
```

Rules:

- `name` — unique string; it is the handle used by
  `fattos_sensor_read("imu", ...)`, calibration, and telemetry lookups.
- `kind` — one of the `FattosSensorKind` values (the driver catalog below).
  The subsystem maps kind → driver automatically; an unknown kind simply
  stays `MISSING`.
- `bus` — index into the bus table handed to `fattos_sensor_setup`.
- `addr7` — 7-bit I2C address (write it the way the datasheet prints it:
  0x68, not 0xD0).
- `rate_hz` — desired output data rate. The driver picks the nearest legal
  chip setting ≤ this rate, and the subsystem uses `1000/rate` ms as the
  read period.
- `range_g` / `range_dps` — IMU full-scale selection (`0` = driver
  default: ±2 g / ±250 dps). Bigger range → bigger range before
  saturation, coarser quantization. Choose the smallest range your
  dynamics need.

Up to `FATTOS_MAX_SENSORS` (default 8) entries. The manifest is plain
data — grep-able, diff-able, and exactly what an AI agent needs to know a
robot's sensor payloads without reading schematics.

---

## 2. The bus HAL

Drivers never call the platform HAL directly. They talk to a `FattosBus` —
a name plus a vtable. One bus instance per physical bus; a board may mix
several I2C buses, an SPI bus, a UART, and a CAN bus without globals.

```c
typedef struct {
    /* Combined I2C transaction: write wr_len, repeated-start read rd_len.
       This is the register-access pattern every sensor uses. */
    FattosStatus (*i2c_write_read)(void *ctx, uint8_t addr7,
                                   const uint8_t *wr, size_t wr_len,
                                   uint8_t *rd, size_t rd_len);
    FattosStatus (*spi_transfer)(void *ctx, uint16_t cs_pin,
                                 const uint8_t *tx, uint8_t *rx, size_t len);
    FattosStatus (*uart_write)(void *ctx, uint8_t port,
                               const uint8_t *buf, size_t len);
    int  (*uart_read)(void *ctx, uint8_t port, uint8_t *buf, size_t cap);
    FattosStatus (*can_send)(void *ctx, uint32_t id,
                             const uint8_t *data, size_t len);
    int  (*can_recv)(void *ctx, uint32_t *id, uint8_t *data, size_t cap);
    void *ctx;   /* bus handle passed to every op */
} FattosBusOps;

typedef struct {
    const char *name;        /* "i2c0" */
    const FattosBusOps *ops; /* NULL = bus not present at all */
} FattosBus;
```

Helper wrappers (`fattos_bus.h`), all NULL-safe:

```c
fattos_i2c_write(bus, addr7, data, len);
fattos_i2c_read(bus, addr7, reg, buf, len);
fattos_i2c_write_u8(bus, addr7, reg, value);
fattos_i2c_read_u8(bus, addr7, reg, &value);
fattos_spi_transfer(bus, cs_pin, tx, rx, len);
fattos_uart_write(bus, port, buf, len);
int n = fattos_uart_read(bus, port, buf, cap);
fattos_can_send(bus, id, data, len);     /* len ≤ 8 */
int m = fattos_can_recv(bus, &id, buf, cap);
```

Contract for port authors:

- Return `FATTOS_OK` or an error code — never partial silent success.
- `i2c_write_read` must be a genuine combined transaction (repeated
  start), because some chips refuse separate write/read sequences.
- A missing bus class or absent bus returns `FATTOS_ERR_NOT_IMPLEMENTED`
  (helpers) or `FATTOS_ERR_PLATFORM` (NACK) — a misconfigured manifest
  fails loudly at the first read instead of returning phantom data.
- Bus ops may be called from a task context only (the sensor subsystem is
  not ISR-safe by design — reads happen in the polling task).

### Porting a real I2C peripheral

```c
static FattosStatus board_i2c_write_read(void *ctx, uint8_t addr7,
        const uint8_t *wr, size_t wr_len, uint8_t *rd, size_t rd_len)
{
    I2C_Handle *i2c = ctx;
    /* 1. START, addr<<1|W, write wr_len bytes
       2. repeated START, addr<<1|R, read rd_len bytes
       3. map NACK/timeouts -> FATTOS_ERR_PLATFORM / FATTOS_ERR_TIMEOUT */
}

static const FattosBusOps board_i2c0_ops = {
    .i2c_write_read = board_i2c_write_read,
    .ctx = &hi2c1,
};
static const FattosBus board_buses[] = {
    { "i2c0", &board_i2c0_ops },
};
```

---

## 3. Setup: what happens at boot

```c
size_t ok = fattos_sensor_setup(sensors, 4, board_buses, 1);
```

Per entry, the subsystem:

1. Validates kind/bus/name limits.
2. Reads the chip's **WHO_AM_I** register through the driver.
3. If and only if the ID matches the expected silicon, runs the driver's
   **configuration**: wake/reset, choose nearest-legal ODR and full-scale,
   read factory calibration (BMP280), start continuous measurement.
4. Sets the sensor STALE (present, awaiting its first sample) and arms its
   rate-group deadline.

Failure handling is deliberately non-fatal per entry:

| Outcome at setup | Health | Effect |
|---|---|---|
| Probe matches + config OK | `STALE` → `OK` at first sample | counted in the return value |
| No ACK / wrong WHO_AM_I | `MISSING` | prearm will refuse to arm |
| Unknown kind, bad bus index | `MISSING` | reported, others continue |

**One bad part must not brick the robot** — prearm policy decides whether
it may fly, but a missing compass chip still lets you boot, log, and
diagnose. The return value is the number of sensors that came up OK.

Call `fattos_sensor_setup` once at boot, after `fattos_init()` and after
the buses are initialized. Calling it again resets all sensor state
(it performs `fattos_sensor_reset()` first).

---

## 4. Reading sensors

```c
FattosSensorSample s;
FattosStatus st = fattos_sensor_read("imu", &s);
if (st == FATTOS_OK) {
    /* fresh, plausible data */
} else if (st == FATTOS_ERR_TIMEOUT) {
    /* stale or faulted: s still holds the LAST GOOD sample */
} else { /* FATTOS_ERR_INVALID_ARG: unknown name (or NULL out) */ }
```

`FattosSensorSample` (SI units, ready to use):

```c
typedef struct {
    uint32_t timestamp_ms;   /* when the sample was taken */
    uint8_t  valid;           /* bitmask of populated fields:
                                 1=accel 2=gyro 4=mag 8=pressure 16=temp 32=range */
    float accel_g[3];         /* accelerometer, g          */
    float gyro_rads[3];       /* gyroscope, rad/s          */
    float mag_ut[3];          /* magnetometer, microtesla  */
    float pressure_pa;        /* barometric pressure, Pa   */
    float temp_c;             /* die temperature, °C       */
    float range_m;            /* rangefinder distance, m   */
} FattosSensorSample;
```

Always check `s.valid` before consuming a field — a barometer never
populates `accel_g`, and the bitmask exists so generic code can handle
any payload.

The `timestamp_ms` lets your own logic detect staleness independently of
the built-in health model (e.g. fusing with `dt = now - s.timestamp`).

---

## 5. Rate groups: `fattos_sensor_poll`

Reading a sensor does not care what time it is. *Polling* does:

```c
static void sensor_task(void *ctx)
{
    (void)ctx;
    size_t n = fattos_sensor_poll(fattos_millis()); /* how many were due */
    (void)n;
}
```

- Each sensor only performs bus I/O when `now >= next_due`, where its
  period is `1000/rate_hz` ms (min 1 ms). Sensors with different rates
  form natural **rate groups**: at 100 Hz the IMU gets read every poll,
  the barometer once every 4th poll at 25 Hz — no code, just math.
- The return value is the number of sensors actually read this call —
  useful as a scheduling sanity check.
- Pacing is based on the `now_ms` you pass in; use `fattos_millis()`, or a
  simulated time in host tests (`fattos_host_advance`).
- After a failed read the sensor is rescheduled normally (errors do not
  busy-retry; they follow the rate anyway).

Failure inside a poll: the error counter increments; after
`5` consecutive bus errors the health becomes `ERROR`; a single glitch
between healthy reads just marks the sensor temporarily `STALE` until the
next good read.

---

## 6. Health model

```c
typedef enum {
    FATTOS_SENSOR_OK = 0,      /* present, fresh, plausible */
    FATTOS_SENSOR_MISSING,     /* not in manifest / probe failed */
    FATTOS_SENSOR_ERROR,       /* 5+ consecutive bus errors */
    FATTOS_SENSOR_STALE,       /* no good sample within 3x its period */
    FATTOS_SENSOR_PLAUSIBLE    /* sample outside physical bounds */
} FattosSensorHealth;
```

`fattos_sensor_health_name()` maps these to strings for logs.

What each state means operationally:

- **MISSING** — WHO_AM_I failed at boot (wrong chip, wiring fault, wrong
  address, bus not initialized). Persists until re-setup/replug.
- **ERROR** — the chip was there but the bus keeps failing now. If it
  recovers, the very next good read returns health to `OK`.
- **STALE** — the chip is alive but the data is too old. Also the state
  between setup and the first sample, and the transient state while
  errors accumulate.
- **PLAUSIBLE** (naming: "failed plausibility") — a sample arrived that
  violates physics; it is **discarded** (the last good sample stays), and
  health flips until a sane sample appears.

Plausibility envelopes (deliberately generous; they catch dead/wrong-
protocol parts, not vigorous maneuvers):

| Signal | Accepted range | Rationale |
|---|---|---|
| accel magnitude | ≥ 0.02 g, ≤ `range_g × 1.02` | a constant 0 output or rail-high output is a wiring/protocol failure |
| pressure | 30 000 … 120 000 Pa | covers altitudes −500 m … +9 000 m |
| temperature | −40 … +125 °C | silicon ratings |
| range | > 0 … 4 m | ToF sensor physics |

Staleness sweep: any `OK` sensor with no good sample for **3× its period**
is marked `STALE` by the next poll even without an error.

---

## 7. Auto-calibration

The one calibration every IMU needs — gyro bias at rest — is built in:

```c
/* at boot, vehicle still on the ground: */
fattos_sensor_calibrate("imu", FATTOS_CAL_GYRO_BIAS, 200);
```

- The driver averages `samples` gyro reads while you keep polling; the
  mean becomes a per-sensor bias subtracted from every subsequent
  `gyro_rads` output. State machine: `FATTOS_CAL_IDLE →
  FATTOS_CAL_GYRO_BIAS → FATTOS_CAL_DONE` (or `FATTOS_CAL_FAILED` with an
  unknown name / non-IMU sensor / NULL).
- Poll `fattos_sensor_cal_state("imu")` from a boot/wait task; do not arm
  until `FATTOS_CAL_DONE`.
- The bias survives into normal samples transparently — consumers keep
  reading `gyro_rads` and it is already corrected.
- Calibration stops automatically when its sample count completes; it
  never runs unbounded. Reset with `FATTOS_CAL_IDLE`.

```c
if (fattos_sensor_calibrate("imu", FATTOS_CAL_GYRO_BIAS, 400) != FATTOS_OK) {
    /* unknown name, or this sensor kind does not support it */
}
```

---

## 8. Driver catalog

Every driver implements: WHO_AM_I probe → configuration from manifest
parameters → SI-unit conversion → the health/plausibility rules above.

### `FATTOS_SENSOR_MPU6050` — 6-axis IMU (I2C, accel + gyro + temp)

- Probe: register `0x75` must read `0x68`.
- Config: wakes from sleep (internal 8 MHz PLL, `PWR_MGMT_1 = 0x01`),
  `SMPLRT_DIV = 1000/rate − 1`, nearest-legal ACCEL_CONFIG/GYRO_CONFIG
  full-scale selectors from `range_g` / `range_dps`.
- Output: `accel_g[3]`, `gyro_rads[3]`, `temp_c` (datasheet formula
  `raw/340 + 36.53`), `valid |= 0x01|0x02|0x10`.
- Scaling: `fs_g/32768` g per LSB at selected range (2/4/8/16 g),
  `fs_dps·π/180/32768` rad/s per LSB (250/500/1000/2000 dps).
- Supports auto-calibration of gyro bias.

### `FATTOS_SENSOR_QMC5883` — 3-axis magnetometer (I2C)

- Probe: register `0x0D` reads `0xFF` (5883L) or `0x01` (5883).
- Config: soft reset (`0x0A ← 0x80`), set/reset period enabled
  (`0x0B ← 0x01`), ODR selected from `rate_hz` (10/50/100/200 Hz),
  ±2 gauss full scale with maximum oversampling (OSR 512).
- Output: `mag_ut[3]` in microtesla at 12 LSB/µT, `valid |= 0x04`.
- Typical use: feed `mag_ut` into `fattos_imu_update_mag()` for absolute
  heading (see [docs/control.md](control.md)).

### `FATTOS_SENSOR_BMP280` — barometer + temperature (I2C)

- Probe: register `0xD0` reads `0x58`.
- Config: soft reset (`0xE0 ← 0xB6`), reads the 24-byte factory
  calibration block at `0x88`, chooses the IIR filter coefficient from
  `rate_hz` (faster ODR → less filtering), starts normal mode with ×16
  pressure / ×2 temperature oversampling.
- Output: `pressure_pa`, `temp_c` via the **datasheet integer
  compensation** exactly as published (also shared with the host
  emulator through `fattos_sensor_internal.h` so tests encode raw counts
  that mean a real pressure), `valid |= 0x08|0x10`.

### `FATTOS_SENSOR_VL53L0X` — time-of-flight rangefinder (I2C)

- Probe: model ID register `0xC0` reads `0xEE`.
- Config: boots the ranging state machine (sequence of single-register
  writes matching the ST basic init) and sets a ~33 ms timing budget.
- Output: `range_m` from the 16-bit millimeter reading at `0x1E`,
  `valid |= 0x20`.
- Typical use: tilt-corrected height into `FattosFusionInput.range_m`
  with `cos_tilt` from your attitude estimate.

### Not-yet-ported parts

A kind not in the enum is simply reported `MISSING`. The contract for
adding one is mechanical — see
[docs/writing-drivers.md](writing-drivers.md); it is: one enum value,
three functions (probe/configure/read), one row in the driver tables.

---

## 9. Deriving everything: heading, altitude, climb rate

The subsystem turns raw payload bytes into the quantities an autonomous
vehicle actually needs.

### Absolute heading (mag + accel → yaw)

```c
FattosSensorSample imu, mag;
if (fattos_sensor_read("imu", &imu) == FATTOS_OK &&
    fatostrue_read("mag", &mag) == FATTOS_OK) {
    fatostrue_update_mag(&state, imu.gyro_rads, imu.accel_g,
                        mag.mag_ut, dt);
}
```

(Exact call: `fattos_imu_update_mag(&imu_state, gyro, accel, mag, dt)` —
tilt-compensated magnetic heading corrects yaw at `kp_mag` and removes
gyro-integration drift.) See [docs/control.md](control.md).

### Altitude & climb rate (baro + ToF + accel → meters, m/s)

```c
#include "fattos/fattos_fusion.h"

static FattosFusion fusion;
fattos_fusion_init(&fusion);                       /* neutral gains     */
fattos_fusion_ground(&fusion, ground_pressure_pa); /* lock AGL origin
                                                      at boot           */

for (each control cycle) {
    FattosFusionInput in = {0};
    FattosSensorSample s;
    if (fattos_sensor_read("baro", &s) == FATTOS_OK) {
        in.have_baro = true; in.pressure_pa = s.pressure_pa;
    }
    if (fattos_sensor_read("tof", &s) == FATTOS_OK) {
        in.have_tof = true; in.range_m = s.range_m;
        in.cos_tilt = cos_tilt_from_attitude;      /* 1.0 if unknown */
    }
    if (fattos_sensor_read("imu", &s) == FATTOS_OK) {
        in.have_accel = true;
        in.accel_z = world_frame_az_ms2;           /* gravity removed */
    }
    in.dt_s = dt_seconds;
    float alt = fattos_fusion_step(&fusion, &in);
    float climb = fusion.climb_rate_mps;
}
```

The estimator is a two-state complementary observer (no matrix math,
bounded work per step):

- **ToF** is trusted most *near the ground* (< 3 m, trust 0.7–0.9), zero
  trust beyond its 3 m physical envelope.
- **Barometric altitude** (ISA conversion `44330·(1−(p/p₀)^0.190263)` with
  `p₀` the boot ground pressure) always contributes at trust 0.5 — it
  works at any altitude, drifts slowly.
- **Vertical acceleration** integrates the climb rate between updates.
- Missing sensors degrade gracefully: no baro → ToF + accel; no ToF →
  baro + accel; only baro → baro only. Absent fields are simply ignored.
- The first near-ground ToF reading auto-learns the mount offset
  (`tof_bias_m`); override with `fattos_fusion_set_tof_bias`.
- Guards: `|dt|` clamped to 0.5 s, altitude clamped to sane physical
  bounds, rate clamped ±100 m/s, non-finite inputs ignored.

Diagnostics for a ground station: `fusion.baro_altitude_m`,
`fusion.tof_height_m`, `baro_updates`, `tof_updates`, `accel_updates`.

---

## 10. Arming gate

```c
bool fatostrue(void);   /* exact name: fattos_sensors_prearm_ok() */
```

`fattos_sensors_prearm_ok()` returns true only when **every sensor in the
manifest** is `OK` (present, fresh, plausible). An empty manifest returns
false (nothing to trust).

The vehicle state manager consults this **automatically**: if a manifest
exists (`fattos_sensor_count() > 0`), its health is ANDed into the arming
condition and into the ARMED→FAILSAFE drop check — no wiring required:

```c
fattos_vehicle_update(&veh);
/* ARMED requires: RC ok + arm switch + prearm_ok + sensors healthy
   + supervisor healthy, held for arm_hold_ms */
```

Consequences worth knowing:

- A chip that dies mid-flight drops the vehicle to FAILSAFE through the
  same gate (via `fattos_sensors_prearm_ok()` evaluated each cycle).
- A board with **no** manifest keeps the old semantics: only the app's
  `fattos_vehicle_set_prearm()` bit gates arming.
- Boot order matters: call `fattos_sensor_setup` before expecting the
  gate to pass; a manifest that was never set up counts all-MISSING
  (nothing arms until it is configured — the safe default).

---

## 11. Diagnostics & telemetry

```c
size_t n = fattos_sensor_count();
for (size_t i = 0; i < n; i++) {
    const char *name; uint8_t kind, health; uint32_t errors;
    fattos_sensor_get_info(i, &name, &kind, &health, &errors);
}
```

Names for logs: `fattos_sensor_kind_name(kind)` ("MPU6050", …),
`fattos_sensor_health_name(health)` ("OK", "MISSING", "ERROR", "STALE",
"BOGUS").

Ground-station transport is one call:

```c
uint8_t frame[128];
size_t len = fattos_telemetry_encode_status(seq, frame, sizeof(frame));
/* frame payload: [0] sensor count, then 4 bytes per sensor
   (kind, health, errors u16 LE), last byte = quarantined-task count */
```

See [docs/communications.md](communications.md) for the framing layer.

---

## 12. Testing on the host simulator

`platform/host/fattos_sim_bus.{h,c}` provides an emulated I2C bus carrying
register-faithful MPU6050, QMC5883, BMP280 and VL53L0X chips. The **exact
driver code** the MCU runs executes on the host — only the bus vtable is
swapped.

```c
#include "fattos_sim_bus.h"

fattos_sim_reset();
fattos_sim_install(FATTOS_SENSOR_MPU6050, 0x68);
fattos_sim_install(FATTOS_SENSOR_BMP280,  0x76);

float accel[3] = {0, 0, 1.0f}, gyro[3] = {10, -5, 1}; /* g, dps */
fattos_sim_imu_values(accel, gyro);
fattos_sim_baro_values(101325.0f, 25.0f);
fattos_sim_tof_value(0.42f);

const FattosBus *buses = fattos_sim_bus();
fattos_sensor_setup(manifest, 4, buses, 1);
```

- Setpoints are *physical* (g, µT, Pa, m): the emulator converts them to
  raw counts using the full-scale ranges **the driver actually
  programmed**, so a wrong configuration register shows up as wrong data.
- The BMP280 emulator solves for the raw ADC counts that produce the
  requested pressure/temperature under the same shared compensation the
  driver uses — round-trip exact to counting resolution.
- `fattos_sim_set_present(kind, false)` unplugs a chip mid-test (address
  NACKs) to exercise MISSING/ERROR health paths and prearm refusals.

Build wiring for a host test:

```cmake
add_executable(my_tests tests/my_tests.c platform/host/fattos_sim_bus.c)
target_link_libraries(my_tests PRIVATE fattos)
target_include_directories(my_tests PRIVATE platform/host robot)
```

---

## 13. Adding a new chip

The registry is designed so a new part touches *no existing call sites*:

1. Add `FATTOS_SENSOR_<CHIP>` to `FattosSensorKind` (before `COUNT`).
2. Write three static functions in `robot/fattos_sensor.c`:
   `chip_probe(slot, &who)`, `chip_configure(slot)`,
   `chip_read(slot, &sample)` — see
   [docs/writing-drivers.md](writing-drivers.md) for full skeletons.
3. Add the three pointers to `g_probe_tab[]`, `g_configure_tab[]`, and the
   `driver_read` switch — one row each, same index.
4. Add a `who_is_valid` case and a `fattos_sensor_kind_name` case.
5. Optionally: emulate it in `fattos_sim_bus.c` to keep host tests.

Everything else — manifest parsing, pacing, health, plausibility,
calibration, prearm, telemetry — is generic and starts working for the
new chip automatically.