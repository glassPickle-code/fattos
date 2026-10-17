# FattOS — the robotics & autonomy OS for MCUs

FattOS is a freestanding-friendly C11 OS foundation focused specifically on
robotics and autonomous applications — unlike general-purpose RTOSes
(FreeRTOS, Zephyr) it ships the layers a robot actually needs: a supervised
scheduler, IPC and instrumentation, a native INT8 neural-net runtime in a
static arena, and a complete autonomy stack (vehicle arming/failsafe state
machine, behavior trees, motion profiles, RC input, ground-station
commands, safety-gated actuators). No dynamic allocation anywhere; every
buffer is static or caller-provided.

## Kernel
- Fixed-capacity task table, cooperative fixed-priority scheduling, 32-bit
  millisecond clock with rollover-safe due checks. No malloc, ever.
- Any `FATTOS_TICK_HZ`: public durations are milliseconds and converted with
  `fattos_ms_to_ticks()` (rounds up); no 1 kHz lock-in.
- Fair dispatch: equal-priority due tasks rotate through the scan, so no
  task is systematically executed last every cycle.
- Bounded drain: `fattos_run_pending(budget, &n)` runs at most `budget` due
  callbacks — a drain loop can never spin forever and freeze the caller.
- Per-task execution profiling at tick resolution: `exec_last_ticks`,
  `exec_max_ticks`, `missed_releases`, `overrun_count`, and
  `jitter_max_ticks` (worst lateness between due time and dispatch — your
  scheduling-latency WCET on target).
- Overrun fault hook (`fattos_set_fault_hook`): called in task context right
  after a callback runs longer than its period — latch an e-stop there and
  a late task becomes a safe stop, not a surprise.
- Message queues: copy-in/copy-out, fixed capacity, caller storage,
  protected by the platform irq lock (usable as ISR-to-task channel).
- Software timers: one-shot and auto-reload, driven by a service task;
  missed periods are skipped instead of bursting.
- 32-bit event flag groups (cooperative poll semantics: any/all matching).
- Fixed-block memory pool: O(1) alloc/free, no fragmentation, free list
  kept inside free blocks.
- Deadman watchdog: per-item timeouts; `fattos_wdt_kick_hardware()` services
  the hardware watchdog only while every item is healthy, so a stalled task
  causes a reset on MCU ports.

## Edge AI
- Static, aligned tensor arena (16 KiB default). The `FattosAIIO` seam is
  the single inference entry point: it routes to the native INT8 runtime
  when a model is bound (with exact shape checking) and refuses honestly
  with `FATTOS_ERR_NOT_IMPLEMENTED` when no backend exists.
- **Native INT8 neural-net runtime** (`fattos_nn`): fully-connected layers
  with int32 accumulation and per-layer requantization, plus ReLU — small
  dense models (anomaly detection, gesture/class classifiers) execute
  entirely in the arena with no external interpreter.
- Toolchain-friendly quantization: `fattos_nn_make_scale()` converts a float
  scale into the (multiplier, shift) requant pair; `fattos_nn_quantize_value()`
  and `fattos_features_quantize()` produce int8 inputs.
- Sensor feature pipeline (`fattos_features`): rolling multi-axis window,
  mean/RMS extraction, flattened feature vector, int8 quantization.
- For CNNs or large models, integrate TensorFlow Lite Micro / CMSIS-NN
  behind the `FattosAIIO` seam instead — the built-in runtime is deliberately
  tiny.

## Robotics & autonomy
- **Board manifest** (`fattos_board`): declare ONCE in your app what is
  connected where — motors, encoder geometry, IMU, RC link, e-stop GPIO —
  and the OS derives its own tables: safety channels, odometry, vehicle
  gate, RC input. Actuator writes go by NAME through the supervisor
  (`fattos_board_write_motor`). Your code never touches raw pins for I/O
  the OS manages; `AGENT.md` is the AI-agent-facing version of this.
- **Go-to-goal navigation** (`fattos_nav`): distance-proportional speed
  with heading-error fade (turn before you charge), arrival radius and
  goal counters — feeds the yaw PID's heading target.
- **Vehicle state manager** (`fattos_vehicle`): flight-controller arming
  semantics — DISARMED → ARMED only after the RC arm switch is held for a
  configured time with everything healthy; RC loss or supervisor fault →
  FAILSAFE; latched e-stop → ESTOP with explicit recovery. One question
  answers everything: `fattos_vehicle_can_actuate()`.
- **Behavior trees** (`fattos_bt`): static, allocation-free trees of
  sequence/selector/inverter nodes with leaf actions and conditions over a
  blackboard — autonomous behavior becomes a data structure, not control
  flow scattered across callbacks.
- **Motion profiles** (`fattos_motion`): accel-limited trapezoidal
  point-to-point reference generator with mid-profile retargeting and
  no-overshoot clamping; feed the velocity reference as feedforward.
- **RC input** (`fattos_rc`): SBUS decoder (16 channels, failsafe/frame-lost
  flags, signal-loss timeout) feeding the vehicle arming gate.
- **Ground-station commands** (`fattos_command`): typed command registry
  over the telemetry link with explicit ack/nack — unknown ids and bad
  payloads get answers, never silence.
- **Leveled ring logger** (`fattos_log`): non-blocking, allocation-free,
  overflow counted; a full log buffer can never stall a control loop.
- PID controller: derivative-on-measurement, first-order D low-pass,
  conditional anti-windup, **feedforward** (`fattos_pid_step_ff`) and
  **bumpless transfer** (`fattos_pid_seed`).
- 6-axis IMU attitude fusion (Mahony-style, gyro-bias integral feedback)
  with **magnetometer heading correction** (`fattos_imu_update_mag`) that
  removes yaw drift, plus direct yaw observation correction for GPS
  heading.
- Differential-drive odometry (exact arc integration) plus the inverse
  velocity mixer with peak clamping.
- Actuator safety supervisor: per-channel heartbeats with timeouts,
  latched e-stop, fail-safe zero-duty on any fault, explicit recovery,
  and a per-channel **slew-rate limiter** so a corrupted command cannot
  slam a motor from 0 to full in one cycle (faults bypass slew and go
  straight to zero — safety beats smoothness).
  `fattos_safety_command()` is the safety-first actuator path: it re-checks
  state freshly on every write, clamps duty to [0,1000] and forces zero
  duty with `FATTOS_ERR_FAULT` whenever the system is not healthy — route
  every motor command through it, never the raw HAL.
- CRC-16/CRC-32 and a framed telemetry protocol (SOF, type, seq, len,
  CRC) with a resyncing decoder state machine for host links.
- HAL contract: GPIO, PWM, IMU sample, timer, idle, **irq lock/unlock**,
  **hardware watchdog kick**.

## Important limitations
- The scheduler is **cooperative**, not preemptive. A task that blocks or
  runs too long delays every other task. Overruns are measured
  (`overrun_count`, `jitter_max_ticks`) and escalate through the fault
  hook, but they cannot be preempted away. Do not use this version for
  safety-critical motor control without the supervision layers and an
  independently tested hardware e-stop.
- No peripheral drivers yet: board ports must supply CAN, SPI/I²C sensor,
  and encoder drivers behind the HAL (see `docs/porting-stm32.md`).
- The host simulator is for logic tests, not MCU timing validation.
- Execution profiling resolves in whole ticks; sub-millisecond WCET needs
  a cycle-counter port.
- The built-in NN runtime handles small dense INT8 models only; it is not a
  drop-in TensorFlow Lite Micro replacement.
- IMU fusion needs `libm` trig (`sinf`/`cosf`/`atan2f`/`asinf`) — fine on
  FPU-class MCUs with newlib, swap for approximations when freestanding.
- IMU yaw is gyro-integrated (no magnetometer) and drifts; the demo shows
  this honestly (fused vs true yaw). Fuse an external heading source when
  absolute yaw matters.
- Watchdog/safety timeouts are evaluated when the supervising tasks run; a
  hard hang of the whole scheduler is only caught by the hardware watchdog.
- STM32 startup, linker script, clock tree, and real peripheral drivers
  must be implemented for a selected board before flashing.

## Build and test (host)
Requirements: CMake 3.16+, C11 compiler (GCC/Clang/MSVC).

```sh
cmake -S . -B build -DFATTOS_BUILD_TESTS=ON -DFATTOS_BUILD_DEMO=ON
cmake --build build
ctest --test-dir build --output-on-failure          # add -C Debug with Visual Studio
./build/fattos_demo                                  # .\build\Debug\fattos_demo.exe on MSVC
```

Five test suites cover the scheduler (priority, fairness, bounded drain,
profiling, jitter, fault hook), kernel services (queue, timers, events,
pool, watchdog), edge AI (NN exactness, features, AI seam), robotics (PID
feedforward/seed, IMU fusion with magnetometer, odometry, safety gating,
CRC, telemetry) and autonomy (behavior trees, motion profiles, vehicle
arming/failsafe/estop, SBUS RC, command registry, logger). The demo runs a
complete simulated robot: SBUS arming, a behavior-tree-driven trapezoidal
move, yaw-hold PID retargeted mid-run by a ground-station command,
magnetometer-corrected fusion, odometry, safety heartbeats, gated actuator
commands, deadman watchdog, NN inferences from a live feature window,
framed telemetry, and event logging — 2 s of deterministic simulated time,
ending in explicit PASS/FAIL acceptance checks.

## Project layout
- `include/fattos/` public API and configuration
- `kernel/` clock, scheduler, queue, timers, events, pool, watchdog
- `robot/` PID, IMU fusion, odometry, safety supervisor, CRC, telemetry,
  vehicle state manager, behavior trees, motion profiles, SBUS RC,
  ground-station commands, logger
- `ai/` tensor arena, INT8 NN runtime, feature pipeline
- `platform/host/` host tick simulation
- `platform/common/` default safe HAL implementation
- `app/` integrated robot example
- `tests/` deterministic suites (no hardware needed)
- `docs/` architecture, autonomy guide, edge-AI model format, STM32
  porting checklist
- `AGENT.md` — authoritative context file for AI coding agents

## Porting to an MCU
1. Select the exact MCU/board and toolchain.
2. Implement `FattosPlatform` functions: timer at `FATTOS_TICK_HZ`, GPIO,
   PWM, IMU sample, irq lock/unlock (save/restore PRIMASK), watchdog kick.
3. Call `fattos_tick_isr()` from a hardware timer interrupt at 1 kHz.
4. Ensure startup code initializes `.data`/`.bss`, stack, clocks, and the
   interrupt controller before `fattos_init()`.
5. Keep ISR work bounded. Do not call task functions from an ISR.
6. Verify actual WCET and deadlines on target and compare with the
   scheduler's `exec_max_ticks`/`missed_releases` counters.
7. Implement `fattos_platform_watchdog_kick()` and drive it through
   `fattos_wdt_kick_hardware()`; add an independent hardware motor-disable
   path.
8. Export your model to INT8 and see `docs/edge-ai.md`; only integrate
   TensorFlow Lite Micro if the model does not fit the built-in runtime.

## API contract
All public functions return a `FattosStatus` where failure is possible. Task
callbacks and timer callbacks must be short, non-blocking, and return
promptly. The scheduler executes at most one callback at a time.

## License
MIT. See `LICENSE`.
