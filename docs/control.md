# Control, estimation, and navigation

How FattOS closes loops: PID control with feedforward, motion profiling,
odometry, go-to-goal navigation, and attitude/altitude estimation from
the sensor manifest.

Headers: `fattos/fattos_pid.h`, `fattos/fattos_motion.h`,
`fattos/fattos_odometry.h`, `fattos/fattos_nav.h`, `fattos/fattos_imu.h`,
`fattos/fattos_fusion.h`.

Contents:
1. [PID](#1-pid)
2. [Motion profiles](#2-motion-profiles)
3. [Odometry & mixing](#3-odometry--mixing)
4. [Go-to-goal navigation](#4-go-to-goal-navigation)
5. [Attitude estimation (IMU)](#5-attitude-estimation-imu)
6. [Altitude & climb rate (fusion)](#6-altitude--climb-rate-fusion)
7. [Putting it together: one control cycle](#7-putting-it-together-one-control-cycle)

---

## 1. PID

A discrete PID tuned for actuator loops:

```c
FattosPID yaw_pid;
fattos_pid_init(&yaw_pid,
                2.0f,   /* kp */
                0.05f,  /* ki */
                0.1f,   /* kd */
                -500.0f, 500.0f,  /* output clamp */
                0.3f);  /* d_alpha: low-pass weight on raw derivative */

float w = fattos_pid_step(&yaw_pid, heading_target, heading_now, dt);
```

Properties (all deliberate):

- **Derivative acts on measurement, not error** — no setpoint-kick when
  the target jumps (retargeting mid-flight is normal in FattOS).
- **First-order D low-pass** via `d_alpha`: 0 = derivative fully
  filtered (later/lazier), 1 = raw. ~0.2–0.4 suits most loops.
- **Conditional-integration anti-windup**: while the output is clamped,
  the integral stops integrating in the saturating direction — recovery
  from saturation is immediate, no overshoot windup.
- **Caller-owned state**: one struct per axis, statically allocated.

### Feedforward

```c
float effort = fattos_pid_step_ff(&vel_pid, v_ref, v_meas, dt,
                                  ff /* mass*accel + friction */);
```

Feedforward carries the known plant effort; the PID only trims model
error. Anti-windup accounts for the feedforward contribution before
clamping. Feed the velocity reference from a [motion profile](#2-motion-profiles).

### Bumpless transfer

```c
/* before enabling a loop (e.g. when the vehicle gate opens): */
fattos_pid_seed(&pid, setpoint_now, measurement_now, 0.0f /* or last output */);
fattos_pid_reset(&pid);   /* wipe integral + derivative state entirely */
```

`seed` arranges the internal state so the very first `step` outputs the
given value with zero integral windup. This is what you call on
`!fattos_vehicle_can_actuate()` transitions against the **current**
measurement (never the future target).

---

## 2. Motion profiles

Trapezoidal, accel-limited reference generator — an S-curve-free,
overshoot-free point-to-point move:

```c
FattosMotionProfile profile;
fattos_motion_init(&profile, 0.5f /* max speed */, 0.8f /* accel */);
fattos_motion_set_target(&profile, 2.0f);   /* retarget anytime */

/* in the control task: */
float x_ref  = fattos_motion_step(&profile, dt);
float v_ref  = profile.velocity;
float effort = fattos_pid_step_ff(&pid, x_ref, x_meas, dt, v_ref);
if (fattos_motion_done(&profile)) { /* target reached and stopped */ }
```

- Handles **retargeting mid-profile**: `set_target` is legal from any
  state and computes a deceleration-consistent path (never overshoots).
- `fattos_motion_set_state(&profile, pos, vel)` syncs external state
  (hand back the measured pose when the profile should re-plan from
  reality rather than its own integration).
- `done` becomes true only when target is reached **and** the reference
  velocity has stopped — safe to chain the next move from `done`.

---

## 3. Odometry & mixing

Differential-drive exact arc integration from encoder tick deltas:

```c
FattosOdometry odom;
fattos_odometry_init(&odom,
                     0.03f  /* wheel radius m */,
                     0.15f  /* wheel base m (contact patch to patch) */,
                     1000.0f /* ticks per revolution */);

/* per control period: */
int32_t dl = encoder_left_delta(), dr = encoder_right_delta();
fattos_odometry_update(&odom, dl, dr);
/* odom.x, odom.y meters; odom.theta radians wrapped to [-pi, pi] */
fattos_odometry_set_pose(&odom, x, y, theta);   /* e.g. GPS re-localization */
```

From the board manifest the geometry comes free:

```c
fattos_board_setup_odometry(&board, &odom);
/* FATTOS_DEV_ODOMETRY device: param = ticks/rev, scale = radius, offset = base */
```

### Inverse mixer

```c
float wl, wr;
/* linear: m/s forward; angular: omega * base/2 (differential speed) */
if (fattos_diff_mix(v, w, 0.6f /* max wheel speed m/s */, &wl, &wr)
        != FATTOS_OK) {
    /* demand beyond physics: outputs are clamped to +/- max */
}
```

---

## 4. Go-to-goal navigation

```c
FattosNav nav;
fattos_nav_init(&nav,
                1.0f,  /* k_linear: m/s per meter of remaining distance */
                0.4f,  /* max linear speed m/s */
                0.05f);/* arrival radius m */

float v, heading;
bool arrived = fattos_nav_step(&nav,
                               odom.x, odom.y, odom.theta,
                               goal_x, goal_y,
                               &v, &heading);
if (!arrived) {
    float w = fattos_pid_step(&yaw_pid, heading, fused_yaw, dt);
    fattos_diff_mix(v, w, max_wheel, &wl, &wr);
}
/* nav.goals_reached counts arrivals (telemetry/diagnostics) */
```

Design details worth relying on:

- **Speed scales with remaining distance** — natural smooth stop at the
  goal (at 5 cm the demand is already ~0.05·k m/s).
- **Heading-error fade**: as the bearing diverges from current yaw, the
  commanded linear speed fades toward zero — the robot *pivots toward the
  goal before charging it*, instead of arcing wide.
- Outputs are always written (even on arrival: `v = 0`).
- `fattos_wrap_pi(angle)` wraps any angle to (−π, π]; use it on every
  angle difference you subtract.

---

## 5. Attitude estimation (IMU)

Mahony-style complementary filter with gyro-bias integral feedback,
operating on SI samples (from `fattos_sensor_read("imu", …)` or the
platform IMU):

```c
static FattosIMUState imu_state;
fattos_imu_init(&imu_state);   /* unit quaternion, kp=2, ki=0.05-ish */

/* primary call — gyro + accel + magnetometer: */
fattos_imu_update_mag(&imu_state,
                      gyro_rads,      /* rad/s        */
                      accel_g,        /* any consistent unit; normalized
                                         internally, treated as gravity
                                         direction while quasi-static */
                      mag_ut,         /* body-frame mag, any unit */
                      dt);

/* accel-only variant (no magnetometer fitted): */
fattos_imu_update(&imu_state, gyro_rads, accel_g, dt);

/* absolute heading observation from another source (GPS heading fix): */
fattos_imu_yaw_correct(&imu_state, yaw_meas, dt);

float roll, pitch, yaw;
fattos_imu_euler(&imu_state, &roll, &pitch, &yaw);   /* radians */
```

Gains live in the state (set them after `init`):

- `kp` — weight of the accelerometer's gravity direction in correcting
  tilt. Higher = trusts accel more (uses it only while quasi-static).
- `ki` — integral gain: *learns gyro bias* continuously from the
  accelerometer reference (the static bias learned by
  `fattos_sensor_calibrate` and this dynamic term complement each other).
- `kp_mag` — weight of the **tilt-compensated magnetic heading** on yaw.
  This is what removes gyro-integration yaw drift: with a magnetometer
  driver in the manifest (`mag` → `mag_ut`), yaw holds absolutely.

Counters for telemetry: `updates`, `mag_updates`.

Demanding-yaw-precision rule: feed `update_mag` at your IMU rate (5–10 ms),
not at telemetry rate — a heading correction computed with a stale dt is
worse than none.

---

## 6. Altitude & climb rate (fusion)

Full API and semantics live in
[sensors §9](sensors.md#deriving-everything-heading-altitude-climb-rate);
the short version:

```c
fattos_fusion_init(&fus);
fattos_fusion_ground(&fus, ground_pressure_pa);   /* boot, on the ground */

/* each cycle: fill what you have */
FattosFusionInput in = {0};
in.have_baro  = read_ok_baro;  in.pressure_pa = pa;
in.have_tof   = read_ok_tof;   in.range_m = range_m; in.cos_tilt = c;
in.have_accel = read_ok_imu;   in.accel_z = az_world;   /* gravity removed */
in.dt_s = dt;

float altitude_m = fattos_fusion_step(&fus, &in);
float climb_mps  = fus.climb_rate_mps;
```

Exports: `fattos_fusion_baro_altitude(pressure_pa, sea_level_pa)` (pure
ISA conversion), `fattos_fusion_is_valid`, `fattos_fusion_set_tof_bias`.
Graceful degradation with any subset of inputs, as documented in
[sensors](sensors.md#9-deriving-everything-heading-altitude-climb-rate).

---

## 7. Putting it together: one control cycle

The canonical 10 ms control task, everything already introduced:

```c
static void control_task(void *ctx)
{
    App *app = ctx;
    float dt = 0.010f;

    /* 0. Policy first: inputs, never outputs */
    fattos_vehicle_set_rc(&app->veh, app->rc_ok, app->arm_switch);
    FattosVehicleState st = fattos_vehicle_update(&app->veh);
    bool may_move = fattos_vehicle_can_actuate(&app->veh);

    /* 1. Read the world (manifest-fed, converted, health-checked) */
    FattosSensorSample imu;
    bool have_imu = fattos_sensor_read("imu", &imu) == FATTOS_OK;

    /* 2. Estimate */
    if (have_imu) {
        FattosSensorSample mag;
        if (fattos_sensor_read("mag", &mag) == FATTOS_OK) {
            fattos_imu_update_mag(&app->att, imu.gyro_rads, imu.accel_g,
                                  mag.mag_ut, dt);
        } else {
            fattos_imu_update(&app->att, imu.gyro_rads, imu.accel_g, dt);
        }
    }

    /* 3. Decide (behavior tree retargets nav goal / PID setpoints) */
    fattos_bt_tick(&app->tree);

    /* 4. Close the loop */
    float v, heading;
    if (!fattos_nav_step(&app->nav, app->odom.x, app->odom.y,
                         app->odom.theta, app->goal_x, app->goal_y,
                         &v, &heading)) {
        if (may_move) {
            float yaw, roll, pitch;
            fattos_imu_euler(&app->att, &roll, &pitch, &yaw);
            float w = fattos_pid_step(&app->yaw, heading, yaw, dt);
            if (fattos_diff_mix(v, w, 0.6f, &app->wl, &app->wr)
                    != FATTOS_OK) { /* clamped */ }
        } else {
            app->wl = app->wr = 0.0f;
            fattos_pid_seed(&app->yaw, heading, heading, 0.0f);
        }
    }

    /* 5. Actuate ONLY through the supervisor */
    uint16_t duty_l = (uint16_t)(fabsf(app->wl) * 1000.0f / 0.6f);
    uint16_t duty_r = (uint16_t)(fabsf(app->wr) * 1000.0f / 0.6f);
    fattos_safety_heartbeat(&app->safety, app->left);
    fattos_safety_heartbeat(&app->safety, app->right);
    if (may_move) {
        fattos_safety_command(&app->safety, app->left,  duty_l);
        fattos_safety_command(&app->safety, app->right, duty_r);
    } else {
        fattos_safety_command(&app->safety, app->left,  0);
        fattos_safety_command(&app->safety, app->right, 0);
    }

    /* 6. Integrate motion from encoders */
    fattos_odometry_update(&app->odom, app->enc_left(), app->enc_right());

    /* 7. Feed my deadman item */
    fattos_wdt_feed(app->wd_control);
}
```

(Appearance notes: read the exact identifiers from the headers —
`fattos_vehicle_can_actuate`, `fattos_odometry_update`,
`fattos_pid_step`; all fallible calls return `FattosStatus`.)

The layering rule that keeps this honest: **step 0 decides whether motion
is allowed; step 5 is the only place motion happens; and step 5 re-checks
health itself even if 0–4 were wrong.**