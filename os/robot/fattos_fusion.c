#include "fattos/fattos_fusion.h"

#include <math.h>
#include <string.h>

#define FUSION_MAX_DT 0.5f      /* clamp absurd dt (task hiccups) */
#define TOF_MAX_RANGE_M 3.0f    /* beyond this the rangefinder is untrustworthy */

float fattos_fusion_baro_altitude(float pressure_pa, float sea_level_pa)
{
    if (pressure_pa <= 0.0f || sea_level_pa <= 0.0f) {
        return 0.0f;
    }
    /* International Standard Atmosphere, troposphere:
       h = T0/L * (1 - (p/p0)^(R*L/g/M)); exponent 0.190263. */
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.190263f));
}

FattosStatus fattos_fusion_init(FattosFusion *fusion)
{
    if (fusion == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    memset(fusion, 0, sizeof(*fusion));
    fusion->sea_level_pa = 101325.0f;
    /* Ground-relative gains: pull hard enough to track a climbing vehicle,
       soft enough that baro noise (~1 m pp) does not chattering-rate. */
    fusion->fusion_gain = 2.5f;
    fusion->rate_gain = 1.2f;
    return FATTOS_OK;
}

FattosStatus fattos_fusion_ground(FattosFusion *fusion,
                                 float ground_pressure_pa)
{
    if (fusion == NULL || ground_pressure_pa <= 0.0f) {
        return FATTOS_ERR_INVALID_ARG;
    }
    /* Store as sea-level-equivalent reference so baro_altitude ~= 0 here
       and forecasts stay continuous through the flight. */
    fusion->sea_level_pa = ground_pressure_pa;
    fusion->altitude_m = 0.0f;
    fusion->climb_rate_mps = 0.0f;
    fusion->baro_altitude_m = 0.0f;
    fusion->initialized = true;
    fusion->tof_calibrated = false;
    return FATTOS_OK;
}

static bool finite_f(float v)
{
    return isfinite((double)v);
}

float fattos_fusion_step(FattosFusion *fusion, const FattosFusionInput *in)
{
    if (fusion == NULL || in == NULL || !fusion->initialized) {
        return 0.0f;
    }

    float dt = in->dt_s;
    if (!finite_f(dt) || dt <= 0.0f) { dt = 0.0f; }
    if (dt > FUSION_MAX_DT) { dt = FUSION_MAX_DT; }

    /* --- measurement candidates (each optional) --- */
    bool have_meas = false;
    float meas_alt = 0.0f;
    float meas_trust = 0.0f; /* 0..1: how much of the innovation to apply */

    if (in->have_baro && finite_f(in->pressure_pa) &&
        in->pressure_pa > 0.0f) {
        /* sea_level_pa holds the ground pressure locked at boot, so the
           ISA conversion yields ground-relative, positive-up altitude. */
        float baro_alt = fattos_fusion_baro_altitude(in->pressure_pa,
                                                     fusion->sea_level_pa);
        fusion->baro_altitude_m = baro_alt;
        fusion->baro_updates++;
        meas_alt = baro_alt;
        meas_trust = 0.5f;
        have_meas = true;
    }

    if (in->have_tof && finite_f(in->range_m) && in->range_m > 0.0f &&
        in->range_m <= TOF_MAX_RANGE_M) {
        float cos_tilt = finite_f(in->cos_tilt) && in->cos_tilt > 0.0f
                             ? in->cos_tilt
                             : 1.0f;
        float h = in->range_m * cos_tilt;

        /* Auto-learn the mount offset from the first solid close reading:
           the vehicle is presumably still on the ground at boot. */
        if (!fusion->tof_calibrated && h < 1.0f) {
            fusion->tof_bias_m = h;
            fusion->tof_calibrated = true;
        }
        h -= fusion->tof_bias_m;
        if (h < 0.0f) { h = 0.0f; }

        fusion->tof_height_m = h;
        fusion->tof_updates++;
        /* Range is exact near the ground: trust it over baro there. */
        meas_alt = h;
        meas_trust = h < 1.0f ? 0.9f : 0.7f;
        have_meas = true;
    }

    /* --- predict --- */
    if (in->have_accel && finite_f(in->accel_z)) {
        fusion->climb_rate_mps += in->accel_z * dt;
        fusion->accel_updates++;
    }
    fusion->altitude_m += fusion->climb_rate_mps * dt;

    /* --- correct (two-state complementary observer) --- */
    if (have_meas) {
        float innov = meas_alt - fusion->altitude_m;
        float k = fusion->fusion_gain * meas_trust * dt;
        if (k > 1.0f) { k = 1.0f; }
        fusion->altitude_m += k * innov;

        /* rate correction: pull rate toward the innovation-damped trend so
           a sustained offset decays instead of integrating error forever */
        float rk = fusion->rate_gain * meas_trust * dt;
        if (rk > 1.0f) { rk = 1.0f; }
        fusion->climb_rate_mps += rk * innov;

        /* absolute fall-through guards: the ground is at or below zero,
           a sane atmosphere is 300-1200 hPa of altitude */
        if (fusion->altitude_m < -2.0f) {
            fusion->altitude_m = -2.0f;
            if (fusion->climb_rate_mps < 0.0f) {
                fusion->climb_rate_mps = 0.0f;
            }
        }
        if (fusion->altitude_m > 100000.0f) {
            fusion->altitude_m = 100000.0f;
        }
    }

    if (fusion->climb_rate_mps > 100.0f) { fusion->climb_rate_mps = 100.0f; }
    if (fusion->climb_rate_mps < -100.0f) { fusion->climb_rate_mps = -100.0f; }

    return fusion->altitude_m;
}

bool fattos_fusion_is_valid(const FattosFusion *fusion)
{
    return fusion != NULL && fusion->initialized &&
           finite_f(fusion->altitude_m) &&
           finite_f(fusion->climb_rate_mps);
}

void fattos_fusion_set_tof_bias(FattosFusion *fusion, float bias_m)
{
    if (fusion == NULL || !finite_f(bias_m)) {
        return;
    }
    fusion->tof_bias_m = bias_m;
    fusion->tof_calibrated = true;
}