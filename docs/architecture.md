# Architecture notes

## Execution model
The scheduler is fixed-capacity and cooperative. Each callback is a periodic
job. When several jobs are due, the highest numeric priority is selected
first; equal priorities rotate through the scan so no task is
systematically executed last every cycle. After one callback returns, the
caller invokes `fattos_run_once()` again, or drains a bounded budget with
`fattos_run_pending()` — the bound guarantees a drain loop can never spin
forever and freeze its caller.

Timers do not run on their own context: `fattos_timer_service()` executes
expired timer callbacks in the calling task's context. Register it once via
`fattos_timers_register()` and keep timer callbacks as short as task
callbacks.

## Timing assumptions
The public API expresses durations in milliseconds and converts them to
ticks with `fattos_ms_to_ticks()` (rounds up, never yields 0 for a non-zero
duration). Any `FATTOS_TICK_HZ` works: at 1 kHz, 1 ms is exactly one tick;
at slower rates all due checks and timeouts simply resolve at tick
granularity. The due test uses signed modular subtraction and is safe for
intervals less than 2^31 ticks. Keep task periods far below that limit.

## Interrupt rules
- `fattos_tick_isr()` must only be called by the configured periodic timer ISR.
- Do not invoke task callbacks from interrupts.
- Do not call blocking drivers from high-priority callbacks.
- On platforms where 32-bit reads are not atomic, protect tick reads.
- Register tasks before enabling interrupts that dispatch application code.
- The irq lock (`fattos_platform_irq_lock/unlock`) is what makes queues,
  the pool, timer/watchdog registries and safety heartbeats safe across the
  ISR/task boundary; ports must save and restore real interrupt state.

## Instrumentation
Every dispatch measures its callback at tick resolution:
`exec_last_ticks`, `exec_max_ticks`, `overrun_count` (ran longer than its
period), `missed_releases` (releases skipped because the task was late)
and `jitter_max_ticks` (worst lateness between due time and dispatch —
the scheduling-latency WCET to validate on target).
An overrun additionally invokes the registered fault hook
(`fattos_set_fault_hook`) in task context — use it to latch an e-stop so a
task that misses its deadline becomes a safe stop instead of a silent
stall.

## Supervision layers
Three complementary fail-safe mechanisms, stacked:
- **Vehicle state manager** (`fattos_vehicle`): the arming gate. Nothing
  actuates unless `fattos_vehicle_can_actuate()` — ARMED with a healthy
  supervisor. Arming requires the RC arm switch held for `arm_hold_ms`
  with RC fresh, prearm true and the supervisor OK. RC loss or supervisor
  fault drops to FAILSAFE (full re-arm required to recover); a latched
  e-stop drops to ESTOP with explicit clear only.
- **Task deadman** (`fattos_watchdog`): tasks feed within their timeout;
  `fattos_wdt_kick_hardware()` forwards to the platform watchdog only when
  all items are healthy, so a stalled task starves the HW watchdog.
- **Actuator safety** (`fattos_safety`): each PWM channel must be
  heartbeated; any timeout or a latched e-stop immediately writes zero duty
  to every registered channel. Recovery is explicit
  (`fattos_safety_estop_clear()`) and refused while channels are faulted.
  Actuator commands go through `fattos_safety_command()`, which re-evaluates
  state freshly on every write, clamps duty to [0, 1000], and forces zero
  duty with `FATTOS_ERR_FAULT` while unhealthy — there is no way to write a
  motor while the supervisor is faulted.

## Autonomy layer
Above supervision, the autonomy stack turns sensor input into deliberate
behavior — all static, allocation-free, deterministic per tick:
- **Behavior trees** (`fattos_bt`): the control task ticks a static tree;
  composites (sequence/selector with resume memory, inverter) orchestrate
  leaf actions and conditions over a caller-owned blackboard. Behaviors
  are data, not scattered callback state.
- **Motion profiles** (`fattos_motion`): trapezoidal accel-limited
  references for point-to-point moves; retargetable mid-profile, never
  overshoots, and the velocity reference feeds the PID feedforward term.
- **RC input** (`fattos_rc`): SBUS stream -> 16 channels + failsafe flags,
  feeding the vehicle arming gate.
- **Ground-station commands** (`fattos_command`): typed request/reply over
  the telemetry framing with explicit ack/nack counters.
- **Logger** (`fattos_log`): leveled ring buffer, never blocks, overflow
  counted; drain it to telemetry from a low-priority task.

## IPC
Queues are copy-based with fixed capacity and caller-provided storage;
events are 32-bit flag groups polled cooperatively; the pool is a
fixed-block allocator whose free list lives inside free blocks. All three
are O(1) and allocation-free.

## AI integration boundary
The arena is static SRAM storage. Two backends exist:
1. `fattos_nn`: the built-in INT8 dense runtime (see `docs/edge-ai.md`).
   It executes inside the arena and reports real results.
2. `FattosAIIO` seam: the single inference entry point. When a model is
   bound it routes to the native runtime with exact shape checking; when
   nothing is bound it returns `FATTOS_ERR_NOT_IMPLEMENTED` — an external
   runtime (TensorFlow Lite Micro, CMSIS-NN) can be connected here later.

Do not treat raw INT8 output as a motor command without model-specific
quantization, calibration, bounds checks, and safety validation — the
safety supervisor must gate any learned policy's outputs.

## Next architectural milestones
1. MCU startup and board port (see `docs/porting-stm32.md`).
2. Timer, GPIO, PWM, DMA, and sensor drivers.
3. Cycle-counter WCET instrumentation below tick resolution.
4. Optional preemptive scheduler with architecture-specific context switch.
5. TensorFlow Lite Micro / CMSIS-NN integration behind the `FattosAIIO` seam
   for models beyond the built-in runtime's scope.
6. Multi-node transport (CAN/UART framing) on top of the telemetry codec.
