# Getting started with FattOS

FattOS is a freestanding C11 OS foundation for robotics and autonomous MCU
applications. Everything is static — no `malloc`, no heap, no hidden global
state in your application. If you can write a C struct and a function, you
can write FattOS code.

This guide takes you from zero to a complete, running control program.

## Contents

1. [Build the library (host)](#1-build-the-library-host)
2. [Your first program](#2-your-first-program)
3. [Add a sensor manifest](#3-add-a-sensor-manifest)
4. [Add control and safety](#4-add-control-and-safety)
5. [The run loop](#5-the-run-loop)
6. [Next reading](#6-next-reading)

---

## 1. Build the library (host)

Requirements: CMake 3.16+, a C11 compiler (GCC, Clang, or MSVC).

```sh
cmake -S . -B build -DFATTOS_BUILD_TESTS=ON -DFATTOS_BUILD_DEMO=ON
cmake --build build
ctest --test-dir build --output-on-failure     # add "-C Debug" with the Visual Studio generator
./build/fattos_demo                            # build\Debug\fattos_demo.exe on MSVC
```

Link your application against the `fattos` static target:

```cmake
add_executable(my_robot app/main.c)
target_link_libraries(my_robot PRIVATE fattos)
```

All public headers live under `include/fattos/` and are included as:

```c
#include "fattos/fattos.h"         /* kernel */
#include "fattos/fattos_sensor.h"  /* sensor subsystem */
```

## 2. Your first program

A FattOS application is: initialize, register periodic tasks, then drain
due tasks in a bounded loop. There is no `main` framework — you own `main`.

```c
#include "fattos/fattos.h"
#include <stdio.h>

static void control_task(void *ctx)
{
    (void)ctx;
    /* Runs every 10 ms. Bounded, non-blocking, returns promptly. */
    uint32_t now = fattos_millis();
    printf("tick %u\n", (unsigned)now);
}

int main(void)
{
    if (fattos_init() != FATTOS_OK) {
        return 1;
    }

    /* The overrun fault hook: a late task becomes a stop, not a surprise. */
    fattos_set_fault_hook(NULL); /* on a robot: call fattos_vehicle_estop() */

    uint8_t control_id;
    fattos_task_create("control", control_task, NULL,
                       10 /* ms */, 5 /* prio */, &control_id);

    /* Bounded drain: never spins, never freezes. */
    for (int i = 0; i < 100; i++) {
        uint32_t dispatched = 0;
        fattos_run_pending(8, &dispatched);
    }
    return 0;
}
```

### Rules that matter

| Rule | Why |
|---|---|
| Callbacks are bounded and non-blocking | The scheduler is cooperative: one long callback delays every other task |
| Durations are **milliseconds** | `FATTOS_TICK_HZ` is yours to choose; `fattos_ms_to_ticks()` converts |
| Everything fallible returns `FattosStatus`; `FATTOS_OK == 0` | Check every return |
| No dynamic allocation — ever | All storage is static or caller-provided |
| Never call `fattos_run_once` from inside a callback | Re-entrancy is forbidden |

On real hardware your main loop becomes:

```c
_Noreturn void app_run(void)
{
    for (;;) {
        bool ran = false;
        fattos_run_once(&ran);
        if (!ran) {
            fattos_platform_idle();   /* sleep until the next tick */
        }
    }
}
```

…with a hardware timer ISR calling `fattos_tick_isr()` exactly
`FATTOS_TICK_HZ` times per second.

## 3. Add a sensor manifest

Declare **what is connected**, and the OS configures the chip, converts
raw counts to SI units, paces reads at the requested rate, tracks health,
and gates arming — no register pokes in application code.

```c
#include "fattos/fattos_sensor.h"
#include "fattos/fattos_bus.h"

static const FattosSensorEntry sensors[] = {
    /* name    kind                    bus addr  rate  rangeG rangeDPS */
    { "imu",   FATTOS_SENSOR_MPU6050,  0,  0x68, 200,  4.0f,  500.0f },
    { "mag",   FATTOS_SENSOR_QMC5883,  0,  0x0D,  50,  0.0f,    0.0f },
    { "baro",  FATTOS_SENSOR_BMP280,   0,  0x76,  25,  0.0f,    0.0f },
    { "tof",   FATTOS_SENSOR_VL53L0X,  0,  0x29,  20,  0.0f,    0.0f },
};
```

Bind the manifest to a bus (on host, the simulated chip bus; on an MCU, your
I2C port — same vtable):

```c
const FattosBus *buses = fattos_sim_bus();      /* host/test */
size_t ok = fattos_sensor_setup(sensors, 4, buses, 1);
/* ok == number of chips that probed and configured successfully.
   A missing chip does not abort the others — prearm handles the policy. */
```

Read converted samples by name:

```c
FattosSensorSample s;
if (fattos_sensor_read("imu", &s) == FATTOS_OK) {
    /* s.accel_g[3] in g, s.gyro_rads[3] in rad/s, s.temp_c in C */
}
```

Inside a periodic task, poll everything at once (each sensor only actually
reads when its own rate is due):

```c
static void sensor_task(void *ctx)
{
    (void)ctx;
    fattos_sensor_poll(fattos_millis());
}
```

Calibrate the gyro at rest (before arming):

```c
fattos_sensor_calibrate("imu", FATTOS_CAL_GYRO_BIAS, 200);
/* ... poll until ... */
if (fattos_sensor_cal_state("imu") == FATTOS_CAL_DONE) { /* bias applied */ }
```

Full details, health semantics, and the driver catalog: **[docs/sensors.md](sensors.md)**.

## 4. Add control and safety

Three lines of policy make motion safe by default:

```c
/* 1. Actuator supervisor: every motor channel gets a heartbeat timeout. */
FattosSafety safety;
fattos_safety_init(&safety);
uint8_t left;
fattos_safety_register_channel(&safety, "left_motor", 0, 60 /* ms */, &left);

/* 2. Vehicle gate: deliberate arming, failsafe on RC loss, latched e-stop. */
FattosVehicle veh;
fattos_vehicle_init(&veh, &safety, 100 /* arm hold ms */, 100 /* RC timeout */);

/* 3. Prearm joins sensor health automatically: if a manifest exists, every
      sensor must be present, fresh and plausible to arm. */
fattos_vehicle_set_rc(&veh, true, arm_switch);

for (;;) {
    fattos_vehicle_update(&veh);
    if (fattos_vehicle_can_actuate(&veh)) {
        fattos_safety_heartbeat(&safety, left);
        fattos_safety_command(&safety, left, 400 /* permille */);
    }
}
```

**Never** call `fattos_platform_pwm_write()` from application code. Every
motor command goes through `fattos_safety_command()` (or
`fattos_board_write_motor()`), which re-checks health freshly, clamps to
[0, 1000], applies slew limiting, and forces zero duty with
`FATTOS_ERR_FAULT` whenever the supervisor is unhealthy.

More: **[docs/safety.md](safety.md)**.

## 5. The run loop

The canonical task layout (from the reference application):

| task | rate | priority | role |
|---|---|---|---|
| sensors | 5 ms | 4 | `fattos_sensor_poll` |
| imu fusion | 5 ms | 3 | `fattos_imu_update_mag` |
| control | 10 ms | 5 | vehicle → nav → BT → PID → mixer → heartbeats → gated writes |
| diagnostics | 100 ms | 1 | `fattos_safety_poll`, `fattos_wdt_poll`, hardware watchdog |
| telemetry | 100 ms | 1 | apply received commands, send status frames |

Priorities: **higher number wins** among due tasks. Keep the control loop
strictly highest; give I/O and telemetry the lowest.

## 6. Next reading

| You want to… | Read |
|---|---|
| Understand the kernel, timers, queues, watchdog | [docs/kernel.md](kernel.md) |
| Wire sensors and add new chips | [docs/sensors.md](sensors.md), [docs/writing-drivers.md](writing-drivers.md) |
| Build safe actuation | [docs/safety.md](safety.md) |
| Write control loops and navigation | [docs/control.md](control.md) |
| Talk to a ground station | [docs/communications.md](communications.md) |
| Port to an MCU | [docs/porting-stm32.md](porting-stm32.md) |
| Look up any function signature | [docs/api-reference.md](api-reference.md) |
| Give an AI agent full context | [AGENT.md](../AGENT.md) |