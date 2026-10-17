#include "fattos/fattos_features.h"
#include "fattos/fattos_nn.h"
#include <string.h>

FattosStatus fattos_features_init(FattosFeatureWindow *window, float *storage,
                                size_t window_length, size_t axes)
{
    if (window == NULL || storage == NULL || window_length == 0U || axes == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }
    window->samples = storage;
    window->capacity = window_length;
    window->axes = axes;
    window->head = 0U;
    window->count = 0U;
    memset(storage, 0, window_length * axes * sizeof(float));
    return FATTOS_OK;
}

FattosStatus fattos_features_push(FattosFeatureWindow *window, const float *sample)
{
    if (window == NULL || sample == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    float *slot = &window->samples[window->head * window->axes];
    memcpy(slot, sample, window->axes * sizeof(float));
    window->head = (window->head + 1U) % window->capacity;
    if (window->count < window->capacity) {
        window->count++;
    }
    return FATTOS_OK;
}

size_t fattos_features_count(const FattosFeatureWindow *window)
{
    return window != NULL ? window->count : 0U;
}

/* Oldest sample index is head-count when full, else 0. */
static size_t oldest_index(const FattosFeatureWindow *window)
{
    if (window->count < window->capacity) {
        return 0U;
    }
    return window->head;
}

FattosStatus fattos_features_mean(const FattosFeatureWindow *window, float *out)
{
    if (window == NULL || out == NULL || window->count == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }
    for (size_t a = 0U; a < window->axes; ++a) {
        out[a] = 0.0f;
    }
    size_t index = oldest_index(window);
    for (size_t n = 0U; n < window->count; ++n) {
        const float *row = &window->samples[index * window->axes];
        for (size_t a = 0U; a < window->axes; ++a) {
            out[a] += row[a];
        }
        index = (index + 1U) % window->capacity;
    }
    float divisor = (float)window->count;
    for (size_t a = 0U; a < window->axes; ++a) {
        out[a] /= divisor;
    }
    return FATTOS_OK;
}

FattosStatus fattos_features_rms(const FattosFeatureWindow *window, float *out)
{
    if (window == NULL || out == NULL || window->count == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }
    for (size_t a = 0U; a < window->axes; ++a) {
        out[a] = 0.0f;
    }
    size_t index = oldest_index(window);
    for (size_t n = 0U; n < window->count; ++n) {
        const float *row = &window->samples[index * window->axes];
        for (size_t a = 0U; a < window->axes; ++a) {
            out[a] += row[a] * row[a];
        }
        index = (index + 1U) % window->capacity;
    }
    float divisor = (float)window->count;
    for (size_t a = 0U; a < window->axes; ++a) {
        float mean_square = out[a] / divisor;
        /* Square root without libm: Newton iterations, seeded well enough
           for feature use. Converges in <6 iterations for normal ranges. */
        if (mean_square <= 0.0f) {
            out[a] = 0.0f;
            continue;
        }
        float guess = mean_square;
        for (int i = 0; i < 6; ++i) {
            guess = 0.5f * (guess + mean_square / guess);
        }
        out[a] = guess;
    }
    return FATTOS_OK;
}

size_t fattos_features_vector_len(const FattosFeatureWindow *window)
{
    return window != NULL ? window->axes * 2U : 0U;
}

FattosStatus fattos_features_vector(const FattosFeatureWindow *window, float *out)
{
    if (window == NULL || out == NULL) {
        return FATTOS_ERR_INVALID_ARG;
    }
    FattosStatus status = fattos_features_mean(window, out);
    if (status != FATTOS_OK) {
        return status;
    }
    return fattos_features_rms(window, &out[window->axes]);
}

FattosStatus fattos_features_quantize(const float *vector, size_t length,
                                    float scale, int8_t *out)
{
    if (vector == NULL || out == NULL || length == 0U || !(scale > 0.0f)) {
        return FATTOS_ERR_INVALID_ARG;
    }
    for (size_t i = 0U; i < length; ++i) {
        out[i] = fattos_nn_quantize_value(vector[i], scale);
    }
    return FATTOS_OK;
}
