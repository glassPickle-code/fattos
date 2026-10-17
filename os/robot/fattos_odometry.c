#include "fattos/fattos_odometry.h"
#include <math.h>

#define FATTOS_PI 3.14159265358979f

FattosStatus fattos_odometry_init(FattosOdometry *odom, float wheel_radius_m,
                                float wheel_base_m, float ticks_per_rev)
{
    if (odom == NULL || !(wheel_radius_m > 0.0f) || !(wheel_base_m > 0.0f) ||
        !(ticks_per_rev > 0.0f)) {
        return FATTOS_ERR_INVALID_ARG;
    }
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
    odom->wheel_radius = wheel_radius_m;
    odom->wheel_base = wheel_base_m;
    odom->ticks_per_rev = ticks_per_rev;
    return FATTOS_OK;
}

void fattos_odometry_set_pose(FattosOdometry *odom, float x, float y, float theta)
{
    if (odom == NULL) {
        return;
    }
    odom->x = x;
    odom->y = y;
    odom->theta = theta;
}

void fattos_odometry_update(FattosOdometry *odom, int32_t left_delta,
                           int32_t right_delta)
{
    if (odom == NULL) {
        return;
    }
    float meters_per_tick =
        2.0f * FATTOS_PI * odom->wheel_radius / odom->ticks_per_rev;
    float dl = (float)left_delta * meters_per_tick;
    float dr = (float)right_delta * meters_per_tick;
    float distance = 0.5f * (dl + dr);
    float dtheta = (dr - dl) / odom->wheel_base;

    if (fabsf(dtheta) > 1e-6f) {
        /* Exact arc integration about the instantaneous center of rotation. */
        float radius = distance / dtheta;
        odom->x += radius * (sinf(odom->theta + dtheta) - sinf(odom->theta));
        odom->y += radius * (cosf(odom->theta) - cosf(odom->theta + dtheta));
    } else {
        odom->x += distance * cosf(odom->theta);
        odom->y += distance * sinf(odom->theta);
    }

    odom->theta += dtheta;
    /* Wrap to [-pi, pi]. */
    while (odom->theta > FATTOS_PI) {
        odom->theta -= 2.0f * FATTOS_PI;
    }
    while (odom->theta < -FATTOS_PI) {
        odom->theta += 2.0f * FATTOS_PI;
    }
}

FattosStatus fattos_diff_mix(float linear, float angular, float max_wheel_speed,
                           float *left_speed, float *right_speed)
{
    if (left_speed == NULL || right_speed == NULL || !(max_wheel_speed > 0.0f)) {
        return FATTOS_ERR_INVALID_ARG;
    }
    float left = linear - angular;
    float right = linear + angular;

    float peak = fabsf(left) > fabsf(right) ? fabsf(left) : fabsf(right);
    if (peak > max_wheel_speed) {
        float scale = max_wheel_speed / peak;
        left *= scale;
        right *= scale;
    }
    *left_speed = left;
    *right_speed = right;
    return FATTOS_OK;
}
