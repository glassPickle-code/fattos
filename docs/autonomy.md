# Building a robot application on FattOS

This is the recommended structure for an autonomous robot app — the same
one the host demo (`app/robot_main.c`) runs end-to-end. The layering is
strict: **sensors → fusion → vehicle gate → behavior → control → safety →
actuators**. Each arrow is a hard boundary; nothing skips a layer.

## 0. The board manifest — declare wiring once

```c
static const FattosDevice devices[] = {
    {"left_motor",  FATTOS_DEV_MOTOR,    .pin = 0, .param = 60},
    {"right_motor", FATTOS_DEV_MOTOR,    .pin = 1, .param = 60},
    {"drive", FATTOS_DEV_ODOMETRY, .param = 1000,
     .scale = 0.03f, .offset = 0.15f},
    {"imu", FATTOS_DEV_IMU, .pin = 1},
    {"rc",  FATTOS_DEV_RC,  .pin = 3},
};
static const FattosBoard board = {"my-bot", devices, 5};

fattos_board_setup_safety(&board, &safety);  /* motors -> supervisor */
fattos_board_setup_odometry(&board, &odom);  /* geometry -> odom */
fattos_board_setup_rc(&board, &rc);
fattos_board_setup_vehicle(&board, &vehicle, &safety, 100, 100);
```

Everything downstream uses names, not pins:
`fattos_board_write_motor(&board, &safety, "left_motor", 400)` validates
the device kind, then routes through the supervisor's fresh health check,
clamp, slew limiter and fail-safe. A manifest with no actuators is
rejected at setup — wiring mistakes fail at startup, not on the road.

## 1. The task layout

| Task        | Rate  | Priority | Does                                              |
|-------------|-------|----------|---------------------------------------------------|
| rc_inject   | 20 ms | 4        | SBUS bytes -> `FattosRCState`                     |
| imu_fusion  | 5 ms  | 3        | `fattos_imu_update_mag` + feature window push     |
| control     | 10 ms | 5        | vehicle update, behavior tree tick, PID, profile, odometry, gated actuator writes, heartbeats |
| nn_infer    | 50 ms | 2        | features -> quantize -> `fattos_ai_infer`         |
| telemetry   | 100 ms| 1        | framed pose downlink, command registry dispatch   |
| diagnostics | 100 ms| 1        | `fattos_safety_poll`, `fattos_wdt_poll`, HW kick  |
| timers      | 10 ms | 1        | `fattos_timers_register` service task             |

The control task owns the loop; everything else feeds it or observes it.

## 2. The vehicle gate — decide "may we move?" once

```c
FattosSafety safety;  fattos_safety_init(&safety);
FattosVehicle vehicle; fattos_vehicle_init(&vehicle, &safety, 100, 100);

/* every control cycle: */
bool rc_ok = fattos_rc_signal_ok(&rc, fattos_millis(), 100);
fattos_vehicle_set_rc(&vehicle, rc_ok, rc.channels[5] > 1300);
fattos_vehicle_set_prearm(&vehicle, imu_converged && sensors_sane);
fattos_vehicle_update(&vehicle);
if (!fattos_vehicle_can_actuate(&vehicle)) { /* coast to zero, seed PID */ }
```

Arming needs the switch held for `arm_hold_ms` with everything healthy.
RC loss or a supervisor fault means FAILSAFE; recovery is always a full
re-arm — never a silent jump back to ARMED.

## 3. Behavior tree — what to do, as data

```c
FattosBTNode nodes[5] = {
    {FATTOS_BT_SEQUENCE, 1, 1, 0, NULL, NULL},        /* root: cond then act */
    {FATTOS_BT_INVERTER, 2, 1, 0, NULL, NULL},        /* obstacle? */
    {FATTOS_BT_CONDITION, 3, 0, 0, cond_clear, NULL},
    {FATTOS_BT_ACTION, -1, 0, 0, act_drive, NULL},
    {FATTOS_BT_ACTION, -1, 0, 0, act_stop, NULL},     /* fallback */
};
FattosBT bt; fattos_bt_init(&bt, nodes, 5, &brain);   /* root = index 0 */
fattos_bt_tick(&bt);                                  /* from control task */
```

Composites keep resume memory: a multi-tick ACTION returns
`FATTOS_BT_RUNNING` and is resumed exactly where it left off next tick.

## 3b. Go-to-goal navigation

```c
FattosNav nav; fattos_nav_init(&nav, 1.0f /*k*/, 0.4f /*max*/, 0.05f);
/* each control cycle: */
bool arrived = fattos_nav_step(&nav, odom.x, odom.y, odom.theta,
                               goal_x, goal_y, &linear, &heading_target);
float angular = fattos_pid_step_ff(&yaw_pid, heading_target, fused_yaw, dt, 0);
```

Speed scales with remaining distance (smooth stop) and fades with heading
error (turn before you charge). Arrival inside the radius zeroes speed and
increments `goals_reached`.

## 4. Motion profile — go somewhere without jerking

```c
FattosMotionProfile motion;
fattos_motion_init(&motion, 0.4f /* m/s */, 0.5f /* m/s^2 */);
fattos_motion_set_target(&motion, 0.35f);   /* legal mid-move too */
/* each control cycle while driving: */
fattos_motion_step(&motion, dt);
linear = motion.velocity;                    /* also your PID feedforward */
```

## 5. Control with feedforward and bumpless enable

```c
if (fattos_vehicle_can_actuate(&vehicle)) {
    angular = fattos_pid_step_ff(&yaw_pid, target, fused_yaw, dt, 0.0f);
} else {
    /* Seed against the CURRENT measurement (zero effort), not the future
       target — otherwise the integral parks at the anti-windup floor. */
    fattos_pid_seed(&yaw_pid, fused_yaw, fused_yaw, 0.0f);
}
fattos_safety_command(&safety, left_id, duty_l);   /* never raw HAL */
```

## 6. Ground-station commands

Register typed handlers once; the dispatcher turns a telemetry frame into
an ack/nack reply. Unknown ids and malformed payloads always get an
explicit answer with counters (`registry.handled/unknown/malformed`).

## 7. Logging

`fattos_log(&log, FATTOS_LOG_INFO, "vehicle %s", ...)` — never blocks,
never allocates; overflow drops oldest and counts it. Drain the newest
entries to telemetry from the diagnostics task.

## Rules that make this safe

1. Only the control task writes actuators, and only via
   `fattos_safety_command()`.
2. Every actuator channel is heartbeated every control cycle; the
   watchdog items mirror the tasks that must keep running.
3. Register the overrun fault hook to `fattos_vehicle_estop()` — a task
   that misses its deadline becomes a stop, not a surprise.
4. Keep callbacks bounded and non-blocking; use `fattos_run_pending()`
   with a budget in the main loop so nothing can spin forever.
