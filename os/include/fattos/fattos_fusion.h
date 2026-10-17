#ifndef FATTOS_FUSION_H
#define FATTOS_FUSION_H

#include <stdbool.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Vertical state estimator: fuses barometric altitude, a downward
 * time-of-flight rangefinder and vertical acceleration into one altitude
 * and climb rate. This is the "do the needed calculations for me" layer —
 * the app feeds raw sensor bytes from the manifest, the estimator answers
 * with meters and m/s.
 *
 *   FattosFusion fu;
 *   fattos_fusion_init(&fu);
 *   fattos_fusion_ground(&fu, baro_pressure_pa);   // lock AGL origin at boot
 *   for each control cycle:
 *       FattosFusionInput in = { 0 };
 *       in.have_baro = (fattos_sensor_read("baro", &s) == FATTOS_OK);
 *       in.pressure_pa = s.pressure_pa;
 *       ...
 *       float alt = fattos_fusion_step(&fu, &in);
 *       float vs = fu.climb_rate_mps;
 *
 * Trust: ToF when in range and tilted little (fast, exact near ground),
 * baro always (slow drift, works to any altitude), accel for rate between
 * updates. A 2-state complementary observer: altitude tracks measurements
 * at fusion_gain, rate corrects on altitude innovation at rate_gain — no
 * matrix math, bounded work per step (MCU-friendly by construction).
 */

typedef struct {
    bool have_baro;    /* pressure_pa + temp sampled this step */
    bool have_tof;     /* range_m sampled this step */
    bool have_accel;   /* vertical accel (world frame, gravity removed) */
    float pressure_pa; /* barometric pressure */
    float range_m;     /* down-ranger distance (slant range before tilt) */
    float cos_tilt;    /* cos(tilt) from attitude: range*cos_tilt = height
                          (1.0 when unknown/level) */
    float accel_z;     /* m/s^2, upward positive, gravity already removed */
    float dt_s;        /* elapsed since previous step, seconds */
} FattosFusionInput;

typedef struct {
    float altitude_m;        /* fused height above ground origin, positive up */
    float climb_rate_mps;    /* fused vertical rate, positive up */
    float baro_altitude_m;   /* last raw barometric altitude (diagnostics) */
    float tof_height_m;      /* last tilt-corrected ToF height (diagnostics) */
    float sea_level_pa;      /* reference pressure for the ISA conversion */
    float tof_bias_m;        /* range offset removed by first-return learning */
    bool initialized;        /* ground reference locked */
    bool tof_calibrated;     /* first valid near-ground range seen */
    uint32_t baro_updates;
    uint32_t tof_updates;
    uint32_t accel_updates;
    /* tuning (sensible defaults from init) */
    float fusion_gain;   /* per-step pull toward measurements (1/s scale) */
    float rate_gain;     /* rate correction on altitude innovation */
} FattosFusion;

/* ISA barometric altitude in meters above sea level, 0 Pa-safe. */
float fattos_fusion_baro_altitude(float pressure_pa, float sea_level_pa);

/* Neutral estimator: sea level 101325 Pa, gains set, uninitialized. */
FattosStatus fattos_fusion_init(FattosFusion *fusion);

/* Lock the AGL origin: altitude 0, rate 0 at the given ground pressure.
   Call at boot on the ground (same place you would run gyro calibration). */
FattosStatus fattos_fusion_ground(FattosFusion *fusion,
                                 float ground_pressure_pa);

/* One estimation step. Input fields are honored only when have_* is set;
   missing sensors simply contribute nothing (graceful degradation: no
   baro -> ToF + accel; no ToF -> baro + accel; only baro -> baro).
   Returns the fused altitude; NaN/infinite inputs are ignored. */
float fattos_fusion_step(FattosFusion *fusion, const FattosFusionInput *in);

bool fattos_fusion_is_valid(const FattosFusion *fusion);

/* Offset a rangefinder that reads a few mm off (mounting, cover glass):
   the first near-ground reading also auto-learns tof_bias_m. */
void fattos_fusion_set_tof_bias(FattosFusion *fusion, float bias_m);

#ifdef __cplusplus
}
#endif

#endif