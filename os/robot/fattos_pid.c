#include "fattos/fattos_pid.h"

static float clampf(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

FattosStatus fattos_pid_init(FattosPID *pid, float kp, float ki, float kd,
                           float out_min, float out_max, float d_alpha)
{
    if (pid == NULL || out_min > out_max) {
        return FATTOS_ERR_INVALID_ARG;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->d_alpha = clampf(d_alpha, 0.0f, 1.0f);
    fattos_pid_reset(pid);
    return FATTOS_OK;
}

void fattos_pid_reset(FattosPID *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_state = 0.0f;
    pid->initialized = false;
}

float fattos_pid_step(FattosPID *pid, float setpoint, float measurement,
                     float dt_seconds)
{
    return fattos_pid_step_ff(pid, setpoint, measurement, dt_seconds, 0.0f);
}

float fattos_pid_step_ff(FattosPID *pid, float setpoint, float measurement,
                         float dt_seconds, float feedforward)
{
    if (pid == NULL || !(dt_seconds > 0.0f)) {
        return 0.0f;
    }

    float error = setpoint - measurement;

    /* Derivative on measurement (negative sign), first-order filtered. */
    if (pid->initialized) {
        float raw_d = -(measurement - pid->prev_measurement) / dt_seconds;
        pid->d_state += pid->d_alpha * (raw_d - pid->d_state);
        pid->prev_measurement = measurement;
    } else {
        pid->d_state = 0.0f;
        pid->prev_measurement = measurement;
        pid->initialized = true;
    }

    float p_term = pid->kp * error;
    float d_term = pid->kd * pid->d_state;

    /* Conditional anti-windup: only integrate when the error does not push
       an already saturated output further into saturation. Feedforward is
       part of the output, so it is inside this decision too. */
    float candidate = pid->integral + pid->ki * error * dt_seconds;
    float unsaturated = p_term + candidate + d_term + feedforward;
    if ((unsaturated > pid->out_max && error > 0.0f) ||
        (unsaturated < pid->out_min && error < 0.0f)) {
        candidate = pid->integral; /* hold integral */
    }
    pid->integral = candidate;

    return clampf(p_term + pid->integral + d_term + feedforward,
                  pid->out_min, pid->out_max);
}

void fattos_pid_seed(FattosPID *pid, float setpoint, float measurement,
                     float output)
{
    if (pid == NULL) {
        return;
    }
    /* Fresh derivative state, then choose the integral that makes the first
       step reproduce `output` exactly: no bump when the loop is enabled. */
    pid->prev_measurement = measurement;
    pid->d_state = 0.0f;
    pid->initialized = true;
    pid->integral = clampf(output - pid->kp * (setpoint - measurement),
                           pid->out_min, pid->out_max);
}
