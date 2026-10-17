#ifndef FATTOS_PID_H
#define FATTOS_PID_H

#include <stdbool.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Discrete PID controller for motor/servo loops.
 * - Derivative acts on measurement (no setpoint-kick).
 * - Optional first-order low-pass on the derivative term.
 * - Conditional integration anti-windup against output clamping.
 * All state is caller-owned; safe for one instance per control axis.
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_measurement;
    float d_state;   /* filtered derivative */
    float d_alpha;   /* 0..1 low-pass weight on raw derivative */
    float out_min;
    float out_max;
    bool initialized;
} FattosPID;

FattosStatus fattos_pid_init(FattosPID *pid, float kp, float ki, float kd,
                           float out_min, float out_max, float d_alpha);
void fattos_pid_reset(FattosPID *pid);/* One control step. dt_seconds must be > 0 and consistent. Returns the
   clamped actuator command. */
float fattos_pid_step(FattosPID *pid, float setpoint, float measurement,
                      float dt_seconds);

/* Same as fattos_pid_step plus a feedforward term added before clamping
   (anti-windup accounts for it). Use feedforward for known plant effort:
   ff = mass*accel + friction, the PID only trims the model error. */
float fattos_pid_step_ff(FattosPID *pid, float setpoint, float measurement,
                         float dt_seconds, float feedforward);

/* Bumpless transfer: seed the controller so the first step after enabling
   outputs `output` at (setpoint, measurement) with no integral windup. */
void fattos_pid_seed(FattosPID *pid, float setpoint, float measurement,
                     float output);

#ifdef __cplusplus
}
#endif

#endif
