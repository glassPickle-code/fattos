#ifndef FATTOS_ODOMETRY_H
#define FATTOS_ODOMETRY_H

#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Differential-drive odometry: integrates wheel encoder tick deltas into a
 * planar pose (x, y, yaw), plus the inverse velocity mixer that maps
 * (linear, angular) body velocity commands to wheel speeds.
 */
typedef struct {
    float x;             /* meters */
    float y;             /* meters */
    float theta;         /* radians, wrapped to [-pi, pi] */
    float wheel_radius;  /* meters */
    float wheel_base;    /* meters, contact patch to contact patch */
    float ticks_per_rev; /* encoder ticks per wheel revolution */
} FattosOdometry;

FattosStatus fattos_odometry_init(FattosOdometry *odom, float wheel_radius_m,
                                float wheel_base_m, float ticks_per_rev);
void fattos_odometry_set_pose(FattosOdometry *odom, float x, float y, float theta);

/* Apply one control period of encoder deltas. */
void fattos_odometry_update(FattosOdometry *odom, int32_t left_delta,
                           int32_t right_delta);

/* linear: m/s forward, angular: m/s of differential speed (omega * base/2).
   Output wheel speeds clamped to +/- max_wheel_speed. */
FattosStatus fattos_diff_mix(float linear, float angular, float max_wheel_speed,
                           float *left_speed, float *right_speed);

#ifdef __cplusplus
}
#endif

#endif
