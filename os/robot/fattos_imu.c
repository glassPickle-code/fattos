#include "fattos/fattos_imu.h"
#include <math.h>

#define FATTOS_IMU_KP 1.0f
#define FATTOS_IMU_KI 0.1f
#define FATTOS_IMU_KP_MAG 0.5f
#define FATTOS_PI_F 3.14159265358979f

static float wrap_pi(float angle)
{
    while (angle > FATTOS_PI_F) {
        angle -= 2.0f * FATTOS_PI_F;
    }
    while (angle < -FATTOS_PI_F) {
        angle += 2.0f * FATTOS_PI_F;
    }
    return angle;
}

static float inv_sqrt(float value)
{
    if (!(value > 0.0f)) {
        return 0.0f;
    }
    float guess = value;
    /* Newton-Raphson for 1/sqrt(value); avoids libm for freestanding builds. */
    for (int i = 0; i < 6; ++i) {
        guess = 0.5f * guess * (3.0f - value * guess * guess);
    }
    return guess;
}

static void normalize_quaternion(FattosIMUState *state)
{
    float norm = inv_sqrt(state->q0 * state->q0 + state->q1 * state->q1 +
                          state->q2 * state->q2 + state->q3 * state->q3);
    if (!(norm > 0.0f)) {
        state->q0 = 1.0f;
        state->q1 = 0.0f;
        state->q2 = 0.0f;
        state->q3 = 0.0f;
        return;
    }
    state->q0 *= norm;
    state->q1 *= norm;
    state->q2 *= norm;
    state->q3 *= norm;
}

void fattos_imu_init(FattosIMUState *state)
{
    if (state == NULL) {
        return;
    }
    state->q0 = 1.0f;
    state->q1 = 0.0f;
    state->q2 = 0.0f;
    state->q3 = 0.0f;
    state->integral_fb[0] = 0.0f;
    state->integral_fb[1] = 0.0f;
    state->integral_fb[2] = 0.0f;
    state->kp = FATTOS_IMU_KP;
    state->ki = FATTOS_IMU_KI;
    state->kp_mag = FATTOS_IMU_KP_MAG;
    state->updates = 0U;
    state->mag_updates = 0U;
}

void fattos_imu_update(FattosIMUState *state, const float gyro[3],
                      const float accel[3], float dt_seconds)
{
    if (state == NULL || gyro == NULL || accel == NULL || !(dt_seconds > 0.0f)) {
        return;
    }

    float gx = gyro[0];
    float gy = gyro[1];
    float gz = gyro[2];

    float accel_norm = accel[0] * accel[0] + accel[1] * accel[1] +
                       accel[2] * accel[2];
    if (accel_norm > 1e-6f) {
        float inv = inv_sqrt(accel_norm);
        /* Estimated gravity direction in body frame for current attitude. */
        float halfvx = state->q1 * state->q3 - state->q0 * state->q2;
        float halfvy = state->q0 * state->q1 + state->q2 * state->q3;
        float halfvz = state->q0 * state->q0 - 0.5f + state->q3 * state->q3;

        float halfax = 0.5f * accel[0] * inv;
        float halfay = 0.5f * accel[1] * inv;
        float halfaz = 0.5f * accel[2] * inv;

        /* Error = measured x estimated gravity direction. */
        float ex = halfay * halfvz - halfaz * halfvy;
        float ey = halfaz * halfvx - halfax * halfvz;
        float ez = halfax * halfvy - halfay * halfvx;

        if (state->ki > 0.0f) {
            state->integral_fb[0] += state->ki * ex * dt_seconds;
            state->integral_fb[1] += state->ki * ey * dt_seconds;
            state->integral_fb[2] += state->ki * ez * dt_seconds;
            gx += state->integral_fb[0];
            gy += state->integral_fb[1];
            gz += state->integral_fb[2];
        }
        gx += state->kp * ex;
        gy += state->kp * ey;
        gz += state->kp * ez;
    }

    /* qDot = 0.5 * q (x) [0, gyro]. */
    float q0 = state->q0;
    float q1 = state->q1;
    float q2 = state->q2;
    float q3 = state->q3;
    state->q0 += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt_seconds;
    state->q1 += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt_seconds;
    state->q2 += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt_seconds;
    state->q3 += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt_seconds;
    normalize_quaternion(state);
    state->updates++;
}

/* Rotate the attitude about world Z (changes yaw only, keeps roll/pitch). */
static void rotate_world_z(FattosIMUState *state, float angle)
{
    float half = 0.5f * angle;
    float w1 = cosf(half);
    float z1 = sinf(half);
    float w = state->q0;
    float x = state->q1;
    float y = state->q2;
    float z = state->q3;
    state->q0 = w1 * w - z1 * z;
    state->q1 = w1 * x - z1 * y;
    state->q2 = w1 * y + z1 * x;
    state->q3 = w1 * z + z1 * w;
    normalize_quaternion(state);
}

void fattos_imu_yaw_correct(FattosIMUState *state, float yaw_meas,
                            float dt_seconds)
{
    if (state == NULL || !(dt_seconds > 0.0f)) {
        return;
    }
    float roll;
    float pitch;
    float yaw;
    if (fattos_imu_euler(state, &roll, &pitch, &yaw) != FATTOS_OK) {
        return;
    }
    (void)roll;
    (void)pitch;
    float error = wrap_pi(yaw_meas - yaw);
    rotate_world_z(state, state->kp_mag * error * dt_seconds);
}

void fattos_imu_update_mag(FattosIMUState *state, const float gyro[3],
                           const float accel[3], const float mag[3],
                           float dt_seconds)
{
    if (state == NULL || mag == NULL) {
        return;
    }
    fattos_imu_update(state, gyro, accel, dt_seconds);

    float mag_norm2 = mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2];
    if (!(mag_norm2 > 1e-6f)) {
        return; /* no field: nothing to correct against */
    }
    float inv = inv_sqrt(mag_norm2);
    float mx = mag[0] * inv;
    float my = mag[1] * inv;
    float mz = mag[2] * inv;

    /* Tilt-compensate the field into the horizontal plane using the
       current roll/pitch, then take its heading as the yaw observation. */
    float roll;
    float pitch;
    float yaw;
    if (fattos_imu_euler(state, &roll, &pitch, &yaw) != FATTOS_OK) {
        return;
    }
    (void)yaw;
    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    float sin_pitch = sinf(pitch);
    float cos_pitch = cosf(pitch);
    float xh = mx * cos_pitch + my * sin_roll * sin_pitch +
               mz * cos_roll * sin_pitch;
    float yh = my * cos_roll - mz * sin_roll;
    float yaw_mag = atan2f(-yh, xh);

    float error = wrap_pi(yaw_mag - yaw);
    rotate_world_z(state, state->kp_mag * error * dt_seconds);
    state->mag_updates++;
}

FattosStatus fattos_imu_euler(const FattosIMUState *state, float *roll,
                            float *pitch, float *yaw)
{
    if (state == NULL || roll == NULL || pitch == NULL || yaw == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    float q0 = state->q0;
    float q1 = state->q1;
    float q2 = state->q2;
    float q3 = state->q3;

    /* atan2/asin via libm are acceptable on FPU-class MCUs; kept in one
       place so freestanding ports can swap in a table version. */
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }
    *pitch = asinf(sinp);
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    return FATTOS_OK;
}
