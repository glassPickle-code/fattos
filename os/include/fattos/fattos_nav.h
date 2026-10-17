#ifndef FATTOS_NAV_H
#define FATTOS_NAV_H

#include <stdbool.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Go-to-goal navigation for differential drives: given the current pose
 * and a goal, produce a linear speed command and the heading target your
 * yaw loop (fattos_pid) should track. Speed scales with remaining
 * distance (smooth stop at the goal) and fades out as heading error
 * grows, so the robot pivots toward the goal before charging it.
 *
 *   float v, heading;
 *   if (!fattos_nav_step(&nav, odom.x, odom.y, odom.theta, gx, gy, &v, &h)) {
 *       float w = fattos_pid_step(&yaw, h, fused_yaw, dt);  // never mind
 *       diff_mix(v, w, ...);
 *   }
 */
typedef struct {
    float k_linear;      /* m/s per meter of remaining distance */
    float max_linear;    /* speed cap, m/s */
    float arrive_radius; /* considered arrived within this many meters */
    bool arrived;
    uint32_t goals_reached; /* arrivals so far (diagnostics/telemetry) */
} FattosNav;

/* Sensible defaults: k=1.0 m/s per m capped at 0.4, arrive within 5 cm. */
FattosStatus fattos_nav_init(FattosNav *nav, float k_linear,
                             float max_linear, float arrive_radius);

/* One control step. Returns true once inside the arrival radius.
   heading_out is the wrapped bearing to the goal (rad, -pi..pi);
   linear_out is 0 when arrived. Outputs are always written. */
bool fattos_nav_step(FattosNav *nav, float pose_x, float pose_y,
                     float pose_theta, float goal_x, float goal_y,
                     float *linear_out, float *heading_out);

/* Wrap any angle into (-pi, pi]. */
float fattos_wrap_pi(float angle);

#ifdef __cplusplus
}
#endif

#endif