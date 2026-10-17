#include "fattos/fattos_nav.h"
#include <math.h>

#define FATTOS_NAV_PI 3.14159265358979f

float fattos_wrap_pi(float angle)
{
    while (angle > FATTOS_NAV_PI) {
        angle -= 2.0f * FATTOS_NAV_PI;
    }
    while (angle <= -FATTOS_NAV_PI) {
        angle += 2.0f * FATTOS_NAV_PI;
    }
    return angle;
}

FattosStatus fattos_nav_init(FattosNav *nav, float k_linear,
                             float max_linear, float arrive_radius)
{
    if (nav == NULL || !(k_linear > 0.0f) || !(max_linear > 0.0f) ||
        !(arrive_radius > 0.0f)) {
        return FATTOS_ERR_INVALID_ARG;
    }
    nav->k_linear = k_linear;
    nav->max_linear = max_linear;
    nav->arrive_radius = arrive_radius;
    nav->arrived = false;
    nav->goals_reached = 0U;
    return FATTOS_OK;
}

bool fattos_nav_step(FattosNav *nav, float pose_x, float pose_y,
                     float pose_theta, float goal_x, float goal_y,
                     float *linear_out, float *heading_out)
{
    float dx = goal_x - pose_x;
    float dy = goal_y - pose_y;
    float distance = sqrtf(dx * dx + dy * dy);
    float heading = atan2f(dy, dx);
    if (heading_out != NULL) {
        *heading_out = heading;
    }

    if (nav == NULL) {
        if (linear_out != NULL) {
            *linear_out = 0.0f;
        }
        return false;
    }

    if (nav->arrived && distance > nav->arrive_radius * 1.5f) {
        nav->arrived = false; /* goal moved away: resume the chase */
    }
    if (!nav->arrived && distance <= nav->arrive_radius) {
        nav->arrived = true;
        nav->goals_reached++;
    }

    float linear = 0.0f;
    if (!nav->arrived) {
        /* Speed proportional to remaining distance: never overshoot. */
        linear = nav->k_linear * distance;
        if (linear > nav->max_linear) {
            linear = nav->max_linear;
        }
        /* Fade speed with heading error so the robot turns before it
           commits: head-on full speed, 90 degrees off half, ~2 pi pivots. */
        float heading_error = fabsf(fattos_wrap_pi(heading - pose_theta));
        float turn_scale = 1.0f - 0.5f * heading_error;
        if (turn_scale < 0.0f) {
            turn_scale = 0.0f;
        }
        linear *= turn_scale;
    }
    if (linear_out != NULL) {
        *linear_out = linear;
    }
    return nav->arrived;
}
