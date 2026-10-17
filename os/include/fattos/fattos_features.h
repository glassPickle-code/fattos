#ifndef FATTOS_FEATURES_H
#define FATTOS_FEATURES_H

#include <stddef.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Fixed-length sensor feature pipeline: a rolling window of multi-axis
 * samples with mean/RMS extraction and INT8 quantization, feeding the NN
 * runtime. All storage is caller-provided; nothing allocates.
 *
 * Typical use at 100 Hz on a 3-axis accelerometer:
 *   float storage[32 * 3];
 *   FattosFeatureWindow win;
 *   fattos_features_init(&win, storage, 32, 3);
 *   ... fattos_features_push(&win, accel_xyz) each sample ...
 *   float vec[6]; fattos_features_vector(&win, vec);   // mean+rms per axis
 */
typedef struct {
    float *samples; /* capacity * axes floats, row-major */
    size_t capacity;
    size_t axes;
    size_t head;  /* next write slot */
    size_t count; /* samples currently held (<= capacity) */
} FattosFeatureWindow;

FattosStatus fattos_features_init(FattosFeatureWindow *window, float *storage,
                                size_t window_length, size_t axes);
FattosStatus fattos_features_push(FattosFeatureWindow *window, const float *sample);
size_t fattos_features_count(const FattosFeatureWindow *window);

/* Per-axis mean and RMS over the window (out has `axes` entries). */
FattosStatus fattos_features_mean(const FattosFeatureWindow *window, float *out);
FattosStatus fattos_features_rms(const FattosFeatureWindow *window, float *out);

/* Flattened vector [mean(axes), rms(axes)], length 2*axes. */
size_t fattos_features_vector_len(const FattosFeatureWindow *window);
FattosStatus fattos_features_vector(const FattosFeatureWindow *window, float *out);

/* Quantize a float vector to symmetric int8 with a fixed scale. */
FattosStatus fattos_features_quantize(const float *vector, size_t length,
                                    float scale, int8_t *out);

#ifdef __cplusplus
}
#endif

#endif
