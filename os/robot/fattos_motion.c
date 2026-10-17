#include "fattos/fattos_motion.h"
#include <math.h>

#define FATTOS_MOTION_POS_EPS 1e-5f
#define FATTOS_MOTION_VEL_EPS 1e-3f

FattosStatus fattos_motion_init(FattosMotionProfile *profile,
                                float max_speed, float accel)
{
    if (profile == NULL || !(max_speed > 0.0f) || !(accel > 0.0f)) {
        return FATTOS_ERR_INVALID_ARG;
    }
    profile->position = 0.0f;
    profile->velocity = 0.0f;
    profile->target = 0.0f;
    profile->max_speed = max_speed;
    profile->accel = accel;
    profile->done = true; /* already at the (zero) target */
    return FATTOS_OK;
}

void fattos_motion_set_target(FattosMotionProfile *profile, float target)
{
    if (profile == NULL) {
        return;
    }
    profile->target = target;
    profile->done = false;
}

void fattos_motion_set_state(FattosMotionProfile *profile, float position,
                             float velocity)
{
    if (profile == NULL) {
        return;
    }
    profile->position = position;
    profile->velocity = velocity;
}

float fattos_motion_step(FattosMotionProfile *profile, float dt_seconds)
{
    if (profile == NULL) {
        return 0.0f;
    }
    if (profile->done || !(dt_seconds > 0.0f)) {
        return profile->position;
    }

    float remaining = profile->target - profile->position;
    float dir = remaining >= 0.0f ? 1.0f : -1.0f;

    /* Arrived and stopped. */
    if (fabsf(remaining) < FATTOS_MOTION_POS_EPS &&
        fabsf(profile->velocity) < FATTOS_MOTION_VEL_EPS) {
        profile->position = profile->target;
        profile->velocity = 0.0f;
        profile->done = true;
        return profile->position;
    }

    bool moving_toward = profile->velocity * dir >= 0.0f;
    float accel_cmd;
    if (!moving_toward) {
        /* Retargeted opposite to current motion: brake first. */
        accel_cmd = profile->velocity >= 0.0f ? -profile->accel
                                              : profile->accel;
    } else {
        /* Distance needed to brake from current speed. */
        float braking =
            profile->velocity * profile->velocity / (2.0f * profile->accel);
        if (braking >= fabsf(remaining)) {
            accel_cmd = -dir * profile->accel; /* start braking now */
        } else if (fabsf(profile->velocity) < profile->max_speed) {
            accel_cmd = dir * profile->accel; /* build toward vmax */
        } else {
            accel_cmd = 0.0f; /* cruise */
        }
    }

    float v_new = profile->velocity + accel_cmd * dt_seconds;
    if (v_new > profile->max_speed) {
        v_new = profile->max_speed;
    } else if (v_new < -profile->max_speed) {
        v_new = -profile->max_speed;
    }
    if (!moving_toward && v_new * profile->velocity < 0.0f) {
        v_new = 0.0f; /* finished braking; forward accel starts next step */
    }

    float new_pos = profile->position + v_new * dt_seconds;
    if ((new_pos - profile->target) * dir > 0.0f) {
        /* Integration would cross the target: clamp exactly on it. */
        new_pos = profile->target;
        v_new = 0.0f;
        profile->done = true;
    }

    profile->position = new_pos;
    profile->velocity = v_new;

    if (fabsf(profile->target - profile->position) < FATTOS_MOTION_POS_EPS &&
        fabsf(profile->velocity) < FATTOS_MOTION_VEL_EPS) {
        profile->position = profile->target;
        profile->velocity = 0.0f;
        profile->done = true;
    }
    return profile->position;
}

bool fattos_motion_done(const FattosMotionProfile *profile)
{
    return profile == NULL || profile->done;
}