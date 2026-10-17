#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "fattos/fattos.h"
#include "fattos/fattos_crc.h"
#include "fattos/fattos_imu.h"
#include "fattos/fattos_odometry.h"
#include "fattos/fattos_pid.h"
#include "fattos/fattos_safety.h"
#include "fattos/fattos_telemetry.h"
#include "fattos_host.h"
#include <stdlib.h>

/* Terminating check: the Windows debug-CRT assert can hang CI on a dialog. */
#undef assert
#define assert(cond)                                                            \
    do {                                                                        \
        if (!(cond)) {                                                          \
            fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__,    \
                    #cond);                                                     \
            exit(1);                                                            \
        }                                                                       \
    } while (0)

#define TEST_PI 3.14159265358979f

/* ---- CRC ---------------------------------------------------------------- */
static void test_crc(void)
{
    const char *check = "123456789";
    assert(fattos_crc16(check, 9U) == 0x29B1U);   /* CRC-16/CCITT-FALSE */
    assert(fattos_crc32(check, 9U) == 0xCBF43926U); /* CRC-32 */
}

/* ---- Telemetry ---------------------------------------------------------- */
static void test_telemetry(void)
{
    uint8_t payload[3] = {10U, 20U, 30U};
    uint8_t frame[32];
    size_t len = fattos_telemetry_encode(0x0AU, 7U, payload, 3U, frame,
                                        sizeof(frame));
    assert(len == 9U); /* SOF type seq len + 3 payload + 2 crc */
    assert(frame[0] == FATTOS_TELEMETRY_SOF);
    assert(fattos_telemetry_encode(1U, 1U, payload, 65U, frame, sizeof(frame)) ==
           0U); /* payload too large */
    assert(fattos_telemetry_encode(1U, 1U, payload, 3U, frame, 4U) == 0U);

    FattosTelemetryDecoder decoder;
    fattos_telemetry_decoder_init(&decoder);
    FattosTelemetryFrame decoded = {0U, 0U, 0U, NULL};
    int delivered = 0;
    for (size_t i = 0U; i < len; ++i) {
        if (fattos_telemetry_decoder_push(&decoder, frame[i], &decoded) == FATTOS_OK) {
            delivered++;
        }
    }
    assert(delivered == 1);
    assert(decoded.type == 0x0AU && decoded.seq == 7U && decoded.len == 3U);
    assert(decoded.payload[0] == 10U && decoded.payload[2] == 30U);
    assert(decoder.frames_ok == 1U);

    /* Corrupted payload byte: rejected, decoder recovers on next frame. */
    uint8_t corrupt[32];
    len = fattos_telemetry_encode(0x0BU, 8U, payload, 3U, corrupt, sizeof(corrupt));
    corrupt[5] ^= 0xFFU;
    delivered = 0;
    for (size_t i = 0U; i < len; ++i) {
        if (fattos_telemetry_decoder_push(&decoder, corrupt[i], &decoded) == FATTOS_OK) {
            delivered++;
        }
    }
    assert(delivered == 0);
    assert(decoder.frames_rejected == 1U);

    /* Garbage bytes before a good frame must not wedge the stream: a false
       SOF followed by an out-of-range length returns the decoder to idle. */
    uint8_t garbage[4] = {0xA5U, 0x00U, 0x00U, 0xFFU};
    for (size_t i = 0U; i < sizeof(garbage); ++i) {
        (void)fattos_telemetry_decoder_push(&decoder, garbage[i], &decoded);
    }
    len = fattos_telemetry_encode(0x0CU, 9U, payload, 3U, frame, sizeof(frame));
    delivered = 0;
    for (size_t i = 0U; i < len; ++i) {
        if (fattos_telemetry_decoder_push(&decoder, frame[i], &decoded) == FATTOS_OK) {
            delivered++;
        }
    }
    assert(delivered == 1);
    assert(decoded.type == 0x0CU);
}

/* ---- PID ---------------------------------------------------------------- */
static void test_pid(void)
{
    FattosPID pid;
    assert(fattos_pid_init(&pid, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f, 1.0f) ==
           FATTOS_OK);
    assert(fattos_pid_step(&pid, 10.0f, 0.0f, 0.01f) == 20.0f); /* P only */

    /* Output clamping. */
    FattosPID clamped;
    assert(fattos_pid_init(&clamped, 10.0f, 0.0f, 0.0f, -0.5f, 0.5f, 1.0f) ==
           FATTOS_OK);
    assert(fattos_pid_step(&clamped, 50.0f, 0.0f, 0.01f) == 0.5f);

    /* Pure integral accumulates error * dt. */
    FattosPID integral;
    assert(fattos_pid_init(&integral, 0.0f, 1.0f, 0.0f, -10.0f, 10.0f, 1.0f) ==
           FATTOS_OK);
    float out = 0.0f;
    for (int i = 0; i < 100; ++i) {
        out = fattos_pid_step(&integral, 1.0f, 0.0f, 0.01f);
    }
    assert(fabsf(out - 1.0f) < 1e-3f); /* 1.0 * 1.0s */

    /* Anti-windup: integral must NOT run past saturation. */
    FattosPID windup;
    assert(fattos_pid_init(&windup, 0.0f, 1.0f, 0.0f, -0.5f, 0.5f, 1.0f) ==
           FATTOS_OK);
    for (int i = 0; i < 500; ++i) {
        out = fattos_pid_step(&windup, 1.0f, 0.0f, 0.01f);
    }
    /* Held at saturation instead of integrating toward 5.0. */
    assert(out <= 0.5f && out > 0.45f);
    assert(windup.integral <= 0.55f);

    /* Derivative on measurement: consecutive rising samples give a constant
       negative D term (and prev_measurement must advance each step). */
    FattosPID deriv;
    assert(fattos_pid_init(&deriv, 0.0f, 0.0f, 1.0f, -100.0f, 100.0f, 1.0f) ==
           FATTOS_OK);
    out = fattos_pid_step(&deriv, 0.0f, 0.0f, 0.1f);   /* first: no D kick */
    assert(out == 0.0f);
    out = fattos_pid_step(&deriv, 0.0f, 1.0f, 0.1f);   /* rise 1.0 in 0.1 s */
    assert(out == -10.0f);
    out = fattos_pid_step(&deriv, 0.0f, 2.0f, 0.1f);   /* same slope again */
    assert(out == -10.0f); /* would be -20 if prev_measurement never advanced */

    /* Feedforward rides with the feedback terms, before clamping. */
    FattosPID ff;
    assert(fattos_pid_init(&ff, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f, 1.0f) ==
           FATTOS_OK);
    assert(fattos_pid_step_ff(&ff, 10.0f, 0.0f, 0.01f, 5.0f) == 25.0f);

    /* Bumpless transfer: the first step after seeding reproduces the
       seeded output exactly — no kick when the loop is enabled. */
    FattosPID bump;
    assert(fattos_pid_init(&bump, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f, 1.0f) ==
           FATTOS_OK);
    fattos_pid_seed(&bump, 10.0f, 0.0f, 5.0f);
    assert(fattos_pid_step(&bump, 10.0f, 0.0f, 0.01f) == 5.0f);
    fattos_pid_seed(NULL, 0.0f, 0.0f, 0.0f); /* null-safe */

    /* Reset clears state. */
    fattos_pid_reset(&integral);
    assert(integral.integral == 0.0f);
}

/* ---- Odometry ----------------------------------------------------------- */
static void test_odometry(void)
{
    FattosOdometry odom;
    assert(fattos_odometry_init(&odom, 0.03f, 0.15f, 1000.0f) == FATTOS_OK);

    float meters_per_tick = 2.0f * TEST_PI * 0.03f / 1000.0f;

    /* Straight line: equal ticks move forward only. */
    fattos_odometry_update(&odom, 1000, 1000);
    assert(fabsf(odom.x - 1000.0f * meters_per_tick) < 1e-3f);
    assert(fabsf(odom.y) < 1e-6f);
    assert(fabsf(odom.theta) < 1e-6f);

    /* In-place turn: opposite ticks rotate without translating much. */
    fattos_odometry_set_pose(&odom, 0.0f, 0.0f, 0.0f);
    fattos_odometry_update(&odom, -500, 500);
    float expected_theta = (2.0f * 500.0f * meters_per_tick) / 0.15f;
    assert(fabsf(odom.theta - expected_theta) < 1e-4f);
    assert(fabsf(odom.x) < 1e-3f && fabsf(odom.y) < 1e-3f);

    /* Mixer clamps and splits correctly. */
    float left;
    float right;
    assert(fattos_diff_mix(1.0f, 0.0f, 0.5f, &left, &right) == FATTOS_OK);
    assert(fabsf(left - 0.5f) < 1e-6f && fabsf(right - 0.5f) < 1e-6f);
    assert(fattos_diff_mix(1.0f, 1.0f, 0.5f, &left, &right) == FATTOS_OK);
    assert(fabsf(left - 0.0f) < 1e-6f && fabsf(right - 0.5f) < 1e-6f);
    assert(fattos_diff_mix(1.0f, 0.0f, 0.0f, &left, &right) == FATTOS_ERR_INVALID_ARG);
}

/* ---- IMU fusion --------------------------------------------------------- */
static float yaw_g;
static float yaw_m;
static void test_imu(void)
{
    /* Level and static: attitude stays level. */
    FattosIMUState imu;
    fattos_imu_init(&imu);
    float gyro_zero[3] = {0.0f, 0.0f, 0.0f};
    float accel_level[3] = {0.0f, 0.0f, 1.0f};
    for (int i = 0; i < 2000; ++i) {
        fattos_imu_update(&imu, gyro_zero, accel_level, 0.01f);
    }
    float roll;
    float pitch;
    float yaw;
    assert(fattos_imu_euler(&imu, &roll, &pitch, &yaw) == FATTOS_OK);
    assert(fabsf(roll) < 0.01f && fabsf(pitch) < 0.01f && fabsf(yaw) < 0.01f);
    assert(imu.updates == 2000U);

    /* Static tilt: filter converges onto the accelerometer direction.
       The Mahony PI loop's slow pole is ~-ki/kp, so allow 40 s of sim. */
    fattos_imu_init(&imu);
    const float tilt = 0.2f; /* ~11.5 degrees */
    float accel_tilt[3] = {0.0f, sinf(tilt), cosf(tilt)};
    for (int i = 0; i < 4000; ++i) {
        fattos_imu_update(&imu, gyro_zero, accel_tilt, 0.01f);
    }
    (void)fattos_imu_euler(&imu, &roll, &pitch, &yaw);
    assert(fabsf(fabsf(roll) - tilt) < 0.02f);

    /* Gyro-only yaw integration (no magnetometer correction). */
    fattos_imu_init(&imu);
    float gyro_z[3] = {0.0f, 0.0f, 1.0f};
    for (int i = 0; i < 100; ++i) {
        fattos_imu_update(&imu, gyro_z, accel_level, 0.01f); /* 1 second */
    }
    (void)fattos_imu_euler(&imu, &roll, &pitch, &yaw);
    assert(fabsf(yaw - 1.0f) < 0.05f);

    assert(fattos_imu_euler(NULL, &roll, &pitch, &yaw) == FATTOS_ERR_INVALID_ARG);

    /* Magnetometer correction pins absolute heading: a 0.02 rad/s gyro
       bias that drifts gyro-only yaw by 0.2 rad over 10 s is held to
       bias/kp_mag ≈ 0.04 rad with mag fusion. */
    FattosIMUState drifted;
    FattosIMUState corrected;
    fattos_imu_init(&drifted);
    fattos_imu_init(&corrected);
    const float gyro_bias = 0.02f;
    for (int i = 0; i < 1000; ++i) { /* 10 s at 100 Hz */
        float t = (float)(i + 1) * 0.01f;
        float true_yaw = 1.0f * t;                 /* true rate 1 rad/s */
        float gyro[3] = {0.0f, 0.0f, 1.0f + gyro_bias}; /* biased sensor */
        float accel[3] = {0.0f, 0.0f, 1.0f};
        float mag[3] = {cosf(true_yaw), -sinf(true_yaw), 0.0f}; /* north */
        fattos_imu_update(&drifted, gyro, accel, 0.01f);
        fattos_imu_update_mag(&corrected, gyro, accel, mag, 0.01f);
    }
    (void)fattos_imu_euler(&drifted, &roll, &pitch, &yaw_g);
    (void)fattos_imu_euler(&corrected, &roll, &pitch, &yaw_m);
    /* Euler yaw is wrapped to [-pi, pi]: compare against wrapped truth. */
    float true_wrapped = 10.0f - 4.0f * TEST_PI; /* 10 rad folded to -pi..pi */
    assert(fabsf(yaw_m - true_wrapped) < 0.06f); /* mag held the heading */
    assert(fabsf(yaw_g - true_wrapped) > 0.15f); /* gyro-only drifted */
    assert(corrected.mag_updates == 1000U);
}

/* ---- Safety supervisor -------------------------------------------------- */
static void test_safety(void)
{
    FattosSafety safety;
    uint8_t left;
    uint8_t right;
    assert(fattos_safety_init(&safety) == FATTOS_OK);
    assert(fattos_safety_register_channel(&safety, "left", 0U, 50U, &left) ==
           FATTOS_OK);
    assert(fattos_safety_register_channel(&safety, "right", 1U, 50U, &right) ==
           FATTOS_OK);

    assert(fattos_safety_heartbeat(&safety, left) == FATTOS_OK);
    assert(fattos_safety_heartbeat(&safety, right) == FATTOS_OK);
    assert(fattos_safety_poll(&safety) == FATTOS_SAFETY_OK);

    /* Within timeout: still safe. */
    fattos_host_advance(40U);
    assert(fattos_safety_poll(&safety) == FATTOS_SAFETY_OK);

    /* Past timeout without heartbeat: fault + fail-safe. */
    fattos_host_advance(20U);
    assert(fattos_safety_poll(&safety) == FATTOS_SAFETY_HEARTBEAT_TIMEOUT);
    assert(fattos_safety_fault_channel(&safety) == left);
    assert(!fattos_safety_is_safe(&safety));

    /* E-stop clear is refused while a channel is still faulted. */
    (void)fattos_safety_estop(&safety);
    assert(fattos_safety_estop_clear(&safety) == FATTOS_ERR_TIMEOUT);

    /* Heartbeat recovers channels; then the latch can clear. */
    assert(fattos_safety_heartbeat(&safety, left) == FATTOS_OK);
    assert(fattos_safety_heartbeat(&safety, right) == FATTOS_OK);
    assert(fattos_safety_estop_clear(&safety) == FATTOS_OK);
    assert(fattos_safety_poll(&safety) == FATTOS_SAFETY_OK);
    assert(fattos_safety_is_safe(&safety));

    /* E-stop latches and holds until explicitly cleared. */
    assert(fattos_safety_estop(&safety) == FATTOS_OK);
    assert(fattos_safety_poll(&safety) == FATTOS_SAFETY_ESTOP_LATCHED);
    assert(fattos_safety_estop_clear(&safety) == FATTOS_OK);
    assert(fattos_safety_poll(&safety) == FATTOS_SAFETY_OK);

    /* Gated actuator path: commands only land while healthy, requests
       above full duty are clamped (never rejected into an unknown state). */
    assert(fattos_safety_command(&safety, left, 600U) == FATTOS_OK);
    assert(fattos_safety_command(&safety, left, 2000U) == FATTOS_OK);
    assert(fattos_safety_command(&safety, 99U, 100U) == FATTOS_ERR_INVALID_ARG);

    /* After the heartbeat timeout the gate refuses and forces zero duty,
       even without an intervening poll. */
    fattos_host_advance(60U);
    assert(fattos_safety_command(&safety, left, 600U) == FATTOS_ERR_FAULT);
    assert(fattos_safety_poll(&safety) == FATTOS_SAFETY_HEARTBEAT_TIMEOUT);

    /* Healthy heartbeats restore the gate. */
    assert(fattos_safety_heartbeat(&safety, left) == FATTOS_OK);
    assert(fattos_safety_heartbeat(&safety, right) == FATTOS_OK);
    assert(fattos_safety_poll(&safety) == FATTOS_SAFETY_OK);
    assert(fattos_safety_command(&safety, right, 500U) == FATTOS_OK);
}

int main(void)
{
    assert(fattos_init() == FATTOS_OK);

    test_crc();
    test_telemetry();
    test_pid();
    test_odometry();
    test_imu();
    test_safety();

    puts("FattOS robotics tests passed");
    return 0;
}
