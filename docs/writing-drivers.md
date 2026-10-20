# Writing drivers and extending FattOS

This guide is for the person (or AI agent) who needs to make FattOS know a
chip or board that it has never heard of. The contract is short:

> **A driver converts a part's raw register bits into SI units and proves
> the data is sane. Everything else — pacing, health, prearm, telemetry —
> is already done for you by the OS.**

Read this once and you can add an I2C accelerometer, an SPI CAN
transceiver, or a whole new sensor class, without touching the kernel.

## Contents

1. [The extension surfaces](#1-the-extension-surfaces)
2. [Write a bus port (MCU bring-up)](#2-write-a-bus-port-mcu-bring-up)
3. [Write a sensor driver](#3-write-a-sensor-driver)
4. [Register the driver in the registry](#4-register-the-driver-in-the-registry)
5. [Write a simulated chip for tests](#5-write-a-simulated-chip-for-tests)
6. [Add a board device kind](#6-add-a-board-device-kind)
7. [Checklist](#7-checklist)

---

## 1. The extension surfaces

FattOS has exactly three places new hardware enters the system, and they
are deliberately few:

| You want to add... | You write... | Layer |
|---|---|---|
| A peripheral implementation (I2C on STM32, SPI bitbang, UART) | a `FattosBusOps` vtable | bus port |
| A chip the manifest knows by name (MPU6050-class part) | probe + configure + read functions, one registry row | sensor driver |
| A new actuator category (servo, ESC, pump) | a `FattosDeviceKind` and a board helper | board device |

Everything above those layers — rate grouping, staleness, plausibility,
health, prearm gating, telemetry frames — is generic and already written.
The rule of thumb: **if your code would contain a delay loop, a health
counter, or a timeout, stop: that belongs to the OS, not the driver.**

---

## 2. Write a bus port (MCU bring-up)

Drivers never touch vendor HALs. They call `FattosBus` helpers, which call
a vtable you fill once per bus instance at board bring-up. This is the
*only* place STM32/HAL/ESP-IDF/Zephyr-RTOS-style calls may appear — and
note that FattOS replaces all of those: a port is a few dozen lines, not
a middleware port.

```c
#include "fattos/fattos_bus.h"

/* One context per bus: whatever your vendor API needs. */
static FattosStatus my_i2c_write_read(void *ctx, uint8_t addr7,
                                      const uint8_t *wr, size_t wr_len,
                                      uint8_t *rd, size_t rd_len)
{
    I2C_HandleTypeDef *hi2c = ctx;
    /* Write, repeated start, read — the register access pattern. */
    if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)(addr7 << 1),
                                (uint8_t *)wr, wr_len, 10) != HAL_OK) {
        return FATTOS_ERR_PLATFORM;
    }
    if (rd_len > 0 &&
        HAL_I2C_Master_Receive(hi2c, (uint16_t)((addr7 << 1) | 1),
                               rd, rd_len, 10) != HAL_OK) {
        return FATTOS_ERR_PLATFORM;
    }
    return FATTOS_OK;
}

extern I2C_HandleTypeDef hi2c1;
static const FattosBusOps i2c0_ops = {
    .i2c_write_read = my_i2c_write_read,
    .ctx = &hi2c1,
    /* .spi_transfer / .uart_write / .uart_read / .can_send / .can_recv
       left NULL: this bus offers I2C only. Helpers return
       FATTOS_ERR_NOT_IMPLEMENTED for the others — never a crash. */
};
static const FattosBus i2c0 = { "i2c0", &i2c0_ops };
```

Rules a port must obey:

- **Return `FATTOS_ERR_PLATFORM` on a transport failure** (NACK, bus
  busy, framing error). Drivers count consecutive failures into sensor
  health; a silent wrong-value return defeats the whole health system.
- **`i2c_write_read` is a write followed by a repeated-start read** on one
  transaction. That is what every sensor's register read wants; do not
  insert a stop bit between the halves unless the chip demands it.
- **SPI: the port asserts/deasserts CS around the transfer.** Drivers pass
  only the `cs_pin` hint. `tx` or `rx` may be NULL (clock dummy / discard).
- **UART reads are non-blocking**: return the number of bytes read (0 when
  nothing is pending, negative on error). RC and telemetry decoders are
  fed from your RX path byte by byte.
- **CAN recv** likewise returns bytes stored, 0 when the queue is empty.
- Mixing classes on one bus object is legal: a vtable may offer I2C *and*
  CAN, and helpers route by function.

To unit-test a driver without hardware, see §5 — the same vtable accepts a
simulated chip, and FattOS's own test suite runs every built-in driver
that way. Code proved on the host is the code that runs on silicon.

---

## 3. Write a sensor driver

A sensor driver is three functions plus private state, all driven from the
manifest entry the user wrote. Look at the MPU6050 driver in
`robot/fattos_sensor.c` as the reference implementation — it is ~100 lines
for a full 6-axis IMU with WHO_AM_I, range programming, and SI scaling.

### The three functions

```c
/* 1. PROVE the chip is really there. Read WHO_AM_I (or equivalent) and
      return FATTOS_OK only when it matches a known-good value. */
static FattosStatus my5883_probe(SensorSlot *s, uint8_t *who)
{
    return fattos_i2c_read_u8(s->bus, s->entry.addr7, MY_REG_WHO, who);
}

/* 2. CONFIGURE the chip FROM THE MANIFEST's plain parameters. The user
      asked for "200 Hz, ±4 g"; translate that into register writes. Snap
      to the nearest legal setting and store the resulting conversion
      scales on the slot. */
static FattosStatus my5883_configure(SensorSlot *s)
{
    const FattosSensorEntry *e = &s->entry;
    uint16_t rate = e->rate_hz > 0 ? e->rate_hz : 100;
    /* ... map rate/range to register fields, write them ... */
    s->accel_scale = 4.0f / 32768.0f;      /* LSB -> g, stored for read */
    return FATTOS_OK;
}

/* 3. READ: fetch raw counts, convert to SI units, set the valid bits.
      Return an error (never a silent zero) when the bus read fails. */
static FattosStatus my5883_read(SensorSlot *s, FattosSensorSample *out)
{
    uint8_t raw[6];
    FattosStatus st = fattos_i2c_read(s->bus, s->entry.addr7,
                                      MY_REG_DATA, raw, sizeof raw);
    if (st != FATTOS_OK) { return st; }

    for (int i = 0; i < 3; i++) {
        int16_t v = (int16_t)((uint16_t)raw[i * 2 + 1] << 8 | raw[i * 2]);
        out->accel_g[i] = (float)v * s->accel_scale;
    }
    out->valid = S_VALID_ACCEL;   /* only what you actually filled */
    return FATTOS_OK;
}
```

### What the OS does for you once these exist

You do **not** write any of the following — they are generic and apply to
your driver automatically:

- **Rate pacing.** `fattos_sensor_poll(now_ms)` calls your read only when
  the manifest's `rate_hz` is due. No delays, no timers in the driver.
- **Staleness.** A sensor that exists but stops delivering goes `STALE`
  within 3× its period, automatically.
- **Bus-error health.** Consecutive read failures escalate to `ERROR`
  after `BUS_ERROR_LIMIT` (5); one glitch just increments a counter.
- **Plausibility.** Each sample is checked against physical envelopes
  (accel magnitude vs. full-scale, pressure 30–120 kPa, temp −40..125 °C,
  range 0–4 m). Impossible data never reaches the application; it marks
  the sensor `BOGUS` (see `FATTOS_SENSOR_PLAUSIBLE`).
- **Auto-calibration.** `fattos_sensor_calibrate("imu", FATTOS_CAL_GYRO_BIAS, n)`
  accumulates your driver's gyro samples at rest and subtracts the bias in
  the read path. Your driver gets this by simply publishing `gyro_rads`.
- **Prearm.** `fattos_sensors_prearm_ok()` is true only when every
  declared sensor is present, fresh and plausible — wired to the vehicle
  arming gate.
- **Telemetry.** `fattos_telemetry_encode_status()` ships your sensor's
  kind, health and error count in one packed frame. The ground station
  sees your new chip without knowing it exists.

### Conventions that matter

- **SI units only** in `FattosSensorSample`: g, rad/s, µT, Pa, °C, meters.
  Never degrees, never hPa, never raw LSB counts.
- **Set `valid` bits strictly** for fields you filled. Consumers check
  them; a garbage field marked valid is worse than an empty one.
- **Never `malloc`, never block, never delay.** Bounded register
  transactions only. The longest acceptable driver read is a single burst
  read of the chip's data registers.
- **Store conversion state on `SensorSlot`**, not in driver-global
  statics — two MPU6050s on different buses with different ranges must
  coexist.
- **Probe failures are not fatal.** `fattos_sensor_setup` marks a failed
  part MISSING and continues with the rest; policy (whether a missing
  sensor blocks arming) is prearm's job, not the driver's.

---

## 4. Register the driver in the registry

The registry binds kind → driver through four small edits in
`robot/fattos_sensor.c`:

1. **Add the enum value** to `FattosSensorKind` in
   `include/fattos/fattos_sensor.h`, before `FATTOS_SENSOR_COUNT`:

   ```c
   FATTOS_SENSOR_QMC5883,  /* magnetometer, I2C */
   FATTOS_SENSOR_MY5883,   /* my 3-axis accelerometer, I2C */   /* <- new */
   FATTOS_SENSOR_COUNT
   ```

2. **Add your three functions** to `robot/fattos_sensor.c`.

3. **Add a row to the two dispatch tables** (positions parallel the enum —
   `NULL` for `FATTOS_SENSOR_NONE`):

   ```c
   static const probe_fn g_probe_tab[] = {
       NULL, mpu6050_probe, qmc5883_probe, bmp280_probe, vl53l0x_probe,
       my5883_probe                                   /* <- new */
   };
   static const configure_fn g_configure_tab[] = {
       NULL, mpu6050_configure, qmc5883_configure, bmp280_configure,
       vl53l0x_configure,
       my5883_configure                               /* <- new */
   };
   ```
   and extend `driver_read()`'s switch and `who_is_valid()`:
   ```c
   case FATTOS_SENSOR_MY5883: return my5883_read(s, out);
   ...
   case FATTOS_SENSOR_MY5883: return who == 0x5AU;
   ```

4. **Add the name** to `fattos_sensor_kind_name()` so telemetry and logs
   print `MY5883` instead of `unknown`.

That's the entire integration. **No call-site changes anywhere**: the
manifest author can now write

```c
{ "accel", FATTOS_SENSOR_MY5883, 0 /*bus*/, 0x1A /*addr*/, 400 /*Hz*/, 8.0f, 0.0f },
```

and read SI-unit samples through the same `fattos_sensor_read("accel", &s)`
path as every other part.

For chips that need private calibration bytes (the BMP280's dig_T/P
words), put a fixed-size field on `SensorSlot` (`bmp_calib` is the
precedent) — slots are static, so "private state" costs compile-time RAM,
never heap.

---

## 5. Write a simulated chip for tests

Every built-in driver is tested against a host-side chip emulator behind
the same `FattosBusOps` vtable an MCU port uses. Write yours the same way
and you can test the driver — and any application logic built on it —
without hardware:

```c
/* A fake register file with the chip's real behavior. */
static uint8_t fake_regs[256];

static FattosStatus sim_i2c_write_read(void *ctx, uint8_t addr7,
                                       const uint8_t *wr, size_t wr_len,
                                       uint8_t *rd, size_t rd_len)
{
    (void)ctx; (void)addr7;
    if (wr_len == 0) { return FATTOS_ERR_PLATFORM; }
    uint8_t reg = wr[0];
    if (wr_len > 1) {                      /* write transaction */
        for (size_t i = 1; i < wr_len; i++) { fake_regs[reg + i - 1] = wr[i]; }
        return FATTOS_OK;
    }
    for (size_t i = 0; i < rd_len; i++) { rd[i] = fake_regs[reg + i]; }
    return FATTOS_OK;
}

static const FattosBusOps sim_ops = { .i2c_write_read = sim_i2c_write_read };
static const FattosBus sim_bus = { "sim", &sim_ops };
```

Test pattern (see `tests/test_robotics.c` for the full harness):

1. `fattos_sensor_reset()` — clears all slots and driver state.
2. Program the emulator's registers to encode known raw counts.
3. `fattos_sensor_setup(entries, n, &sim_bus, 1)` — probe must pass.
4. `fattos_sensor_poll(now)` — drive time forward yourself.
5. `fattos_sensor_read("name", &s)` — assert SI values within tolerance.

For the BMP280, the datasheet compensation lives in one shared function
(`fattos_bmp280_compensate`, declared in `fattos_sensor_internal.h`) used
by both the driver and the emulator, so tests can encode ADC counts that
*mean* a known pressure. Reuse that pattern when your chip has firmware
compensation: extract it, share it, test both sides against it.

---

## 6. Add a board device kind

Sensors are the deep path (probe, configure, SI conversion, health).
Non-sensor hardware — motors, odometry, RC receivers, e-stops — uses the
lighter board manifest in `fattos/fattos_board.h`. Adding a category:

1. Add an `FattosDeviceKind` value (`FATTOS_DEV_MOTOR = 1`, etc.).
2. Decide which of the three per-kind slots (`pin`, `pin2`, `param`,
   `scale`, `offset`) carry its configuration, and document it in the
   enum comment.
3. Add a `fattos_board_setup_*` helper that scans the manifest for the new
   kind and fills the corresponding subsystem — the existing helpers show
   the shape: motors become safety channels, the odometry device feeds
   wheel geometry, the RC device wires the SBUS UART.
4. If the device routes actuator writes, go **through the safety
   supervisor** (`fattos_safety_command`), never the raw platform HAL.
   `fattos_board_write_motor` is the reference.

The manifest stays plain data — that property is what lets an agent (or a
telemetry tool, or a code generator) know the robot's wiring by reading
one array.

---

## 7. Checklist

Before submitting a new driver or port:

- [ ] Probe validates WHO_AM_I (or equivalent); rejects a wrong chip.
- [ ] Configure maps manifest `rate_hz` / `range_*` to the nearest legal
      register settings and stores the resulting conversion scales on the
      slot.
- [ ] Read converts to SI units and sets `valid` bits strictly.
- [ ] Bus failures return `FATTOS_ERR_PLATFORM` (never a silent bad
      sample).
- [ ] No allocation, no delays, no blocking anywhere in the driver.
- [ ] Registry edits: enum, probe/configure tables, `driver_read` switch,
      `who_is_valid`, `kind_name`.
- [ ] A simulated-chip test that drives setup → poll → read and asserts
      real SI values.
- [ ] Documented in the driver catalog table in
      [sensors.md](sensors.md) (or the enum comment for board kinds).

Next: the full function index is in
[api-reference.md](api-reference.md).
