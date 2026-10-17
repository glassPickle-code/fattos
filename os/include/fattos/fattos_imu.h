#ifndef FATTOS_IMU_H
#define FATTOS_IMU_H

#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 6-axis attitude estimation (gyro + accelerometer), Mahony-style
 * complementary filter with gyro-bias integral feedback. Float math
 * (Cortex-M4F/M7F class parts). No magnetometer: yaw is gyro-integrated
 * only and will drift; fuse an external heading source if absolute yaw
 * matters.
 */
typedef struct {
    float q0, q1, q2, q3;  /* unit quaternion, body -> world */
    float integral_fb[3];  /* gyro bias estimate, rad/s */
    float kp;              /* proportional gain on accel error */
    float ki;              /* integral (bias) gain */
    float kp_mag;          /* yaw correction gain from magnetometer */
    uint32_t updates;
    uint32_t mag_updates;  /* magnetometer corrections applied */
} FattosIMUState;

void fattos_imu_init(FattosIMUState *state);/* gyro: rad/s. accel: any consistent unit (normalized internally; treated
   as gravity direction while quasi-static). dt: seconds. */
void fattos_imu_update(FattosIMUState *state, const float gyro[3],
                       const float accel[3], float dt_seconds);

/* Same as fattos_imu_update plus magnetometer yaw correction: the tilt-
   compensated magnetic heading is fused at kp_mag, which removes the
   gyro-integration yaw drift. mag: body-frame field vector (any unit,
   normalized internally); when quasi-static accel is unavailable the
   previous correction still holds. */
void fattos_imu_update_mag(FattosIMUState *state, const float gyro[3],
                           const float accel[3], const float mag[3],
                           float dt_seconds);

/* Direct heading observation correction (GPS heading, landmark fix, ...):
   rotates the attitude yaw toward yaw_meas at kp_mag. */
void fattos_imu_yaw_correct(FattosIMUState *state, float yaw_meas,
                            float dt_seconds);

/* Euler angles in radians: roll (x), pitch (y), yaw (z). */
FattosStatus fattos_imu_euler(const FattosIMUState *state, float *roll,
                            float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif
