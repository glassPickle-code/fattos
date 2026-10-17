#ifndef FATTOS_MOTION_H
#define FATTOS_MOTION_H

#include <stdbool.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Trapezoidal motion profile: an accel-limited reference generator for
 * point-to-point moves. Step it at your control rate and feed the
 * resulting position/velocity references to your loop (velocity as
 * feedforward through fattos_pid_step_ff). Handles retargeting
 * mid-profile and never overshoots the target.
 */
typedef struct {
    float position;   /* current reference */
    float velocity;   /* current reference rate */
    float target;     /* where the profile is going */
    float max_speed;  /* |velocity| limit, > 0 */
    float accel;      /* |acceleration| limit, > 0 */
    bool done;        /* true once target reached and stopped */
} FattosMotionProfile;

FattosStatus fattos_motion_init(FattosMotionProfile *profile,
                                float max_speed, float accel);

/* (Re)target from wherever the profile currently is — legal mid-move. */
void fattos_motion_set_target(FattosMotionProfile *profile, float target);

/* External state sync (e.g. measured pose handed back in). */
void fattos_motion_set_state(FattosMotionProfile *profile, float position,
                             float velocity);

/* Advance the reference by dt seconds; returns the position reference. */
float fattos_motion_step(FattosMotionProfile *profile, float dt_seconds);

bool fattos_motion_done(const FattosMotionProfile *profile);

#ifdef __cplusplus
}
#endif

#endif