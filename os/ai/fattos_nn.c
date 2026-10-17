#include "fattos/fattos_nn.h"
#include "fattos/fattos_ai.h"
#include "fattos/fattos_config.h"

#if defined(__GNUC__) || defined(__clang__)
#define FATTOS_NN_ALIGN_ATTR __attribute__((aligned(4)))
#else
#define FATTOS_NN_ALIGN_ATTR
#endif

static const FattosNNModel *g_model;
static int8_t *g_ping;
static int8_t *g_pong;
static int32_t *g_acc;
static size_t g_scratch_features;
static size_t g_input_len;
static size_t g_output_len;

static int8_t requantize(int32_t accumulator, int32_t multiplier, int32_t shift)
{
    /* Arithmetic right shift of a negative int64 is implementation-defined
       pre-C20 but arithmetic on every supported toolchain (GCC/Clang/MSVC). */
    int64_t product = (int64_t)accumulator * (int64_t)multiplier;
    int32_t result = (int32_t)(product >> shift);
    if (result > 127) {
        result = 127;
    } else if (result < -128) {
        result = -128;
    }
    return (int8_t)result;
}

static void relu_inplace(int8_t *data, size_t count)
{
    for (size_t i = 0U; i < count; ++i) {
        if (data[i] < 0) {
            data[i] = 0;
        }
    }
}

FattosStatus fattos_nn_bind(const FattosNNModel *model)
{
    if (model == NULL || model->layers == NULL || model->layer_count == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }

    size_t max_features = 0U;
    for (size_t l = 0U; l < model->layer_count; ++l) {
        const FattosNNLayer *layer = &model->layers[l];
        if (layer->input_features == 0U || layer->output_features == 0U) {
            return FATTOS_ERR_INVALID_ARG;
        }
        if (l > 0U &&
            layer->input_features != model->layers[l - 1U].output_features) {
            return FATTOS_ERR_INVALID_ARG; /* Broken shape chain. */
        }
        if (layer->op == FATTOS_NN_OP_FULLY_CONNECTED) {
            if (layer->weights == NULL || layer->bias == NULL ||
                layer->multiplier <= 0 || layer->shift < 0 || layer->shift > 62) {
                return FATTOS_ERR_INVALID_ARG;
            }
        } else if (layer->op != FATTOS_NN_OP_RELU) {
            return FATTOS_ERR_INVALID_ARG;
        }
        size_t bigger = layer->input_features > layer->output_features
                            ? layer->input_features
                            : layer->output_features;
        if (bigger > max_features) {
            max_features = bigger;
        }
    }

    /* Scratch: two int8 ping/pong buffers + one int32 accumulator row. */
    size_t needed = 2U * max_features * sizeof(int8_t) + max_features * sizeof(int32_t);
    if (needed > fattos_ai_arena_size()) {
        return FATTOS_ERR_FULL;
    }

    uint8_t *cursor = (uint8_t *)fattos_ai_arena_data();
    /* Align cursor to 4 bytes for the accumulator. */
    uintptr_t raw = (uintptr_t)cursor;
    cursor += (4U - (raw & 3U)) & 3U;

    g_ping = (int8_t *)cursor;
    cursor += max_features;
    g_pong = (int8_t *)cursor;
    cursor += max_features;
    g_acc = (int32_t *)(void *)cursor;

    g_model = model;
    g_scratch_features = max_features;
    g_input_len = model->layers[0].input_features;
    g_output_len = model->layers[model->layer_count - 1U].output_features;
    return FATTOS_OK;
}

bool fattos_nn_is_bound(void)
{
    return g_model != NULL;
}

FattosStatus fattos_nn_run(const int8_t *input, size_t input_len,
                         int8_t *output, size_t output_len)
{
    if (g_model == NULL) {
        return FATTOS_ERR_NOT_INITIALIZED;
    }
    if (input == NULL || output == NULL || input_len != g_input_len ||
        output_len != g_output_len) {
        return FATTOS_ERR_INVALID_ARG;
    }

    int8_t *src = g_ping;
    int8_t *dst = g_pong;
    for (size_t i = 0U; i < input_len; ++i) {
        src[i] = input[i];
    }
    size_t active = input_len;

    for (size_t l = 0U; l < g_model->layer_count; ++l) {
        const FattosNNLayer *layer = &g_model->layers[l];
        if (layer->op == FATTOS_NN_OP_FULLY_CONNECTED) {
            /* Write the accumulator through the shared int32 row, requantize
               into dst. */
            size_t in = layer->input_features;
            size_t out = layer->output_features;
            for (size_t o = 0U; o < out; ++o) {
                int32_t acc = layer->bias[o];
                const int8_t *row = &layer->weights[o * in];
                for (size_t i = 0U; i < in; ++i) {
                    acc += (int32_t)row[i] * (int32_t)src[i];
                }
                g_acc[o] = acc;
            }
            for (size_t o = 0U; o < out; ++o) {
                dst[o] = requantize(g_acc[o], layer->multiplier, layer->shift);
            }
            active = out;
        } else {
            /* ReLU is in-place: keep both pointers — the next layer must
               read the transformed buffer, not the stale one. */
            relu_inplace(src, active);
            continue;
        }
        int8_t *tmp = src;
        src = dst;
        dst = tmp;
    }

    for (size_t i = 0U; i < g_output_len; ++i) {
        output[i] = src[i];
    }
    return FATTOS_OK;
}

size_t fattos_nn_output_len(void)
{
    return g_output_len;
}

size_t fattos_nn_input_len(void)
{
    return g_model != NULL ? g_input_len : 0U;
}

int fattos_nn_argmax(const int8_t *values, size_t count)
{
    if (values == NULL || count == 0U) {
        return -1;
    }
    int best = 0;
    for (size_t i = 1U; i < count; ++i) {
        if (values[i] > values[best]) {
            best = (int)i;
        }
    }
    return best;
}

FattosStatus fattos_nn_make_scale(float scale, int32_t *multiplier, int32_t *shift)
{
    if (multiplier == NULL || shift == NULL || !(scale > 0.0f)) {
        return FATTOS_ERR_INVALID_ARG;
    }
    /* result = (acc * multiplier) >> shift ~= acc * scale.
       Prefer shift 31 (max precision for small scales); step down when the
       product overflows int32 (scale >= ~1.0), up when it underflows
       (tiny scales), keeping the multiplier in [1, 2^31-1]. */
    int32_t s = 31;
    float scaled = scale * (float)(1ULL << s);
    while (scaled >= 2147483647.0f && s > 0) {
        s--;
        scaled = scale * (float)(1ULL << s);
    }
    while (scaled < 1.0f && s < 62) {
        s++;
        scaled = scale * (float)(1ULL << s);
    }
    if (scaled < 1.0f || scaled >= 2147483647.0f) {
        return FATTOS_ERR_INVALID_ARG;
    }
    *multiplier = (int32_t)(scaled + 0.5f);
    *shift = s;
    return *multiplier > 0 ? FATTOS_OK : FATTOS_ERR_INVALID_ARG;
}

int8_t fattos_nn_quantize_value(float value, float scale)
{
    if (!(scale > 0.0f)) {
        return 0;
    }
    float scaled = value / scale;
    float rounded = scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f;
    int32_t q = (int32_t)rounded;
    if (q > 127) {
        q = 127;
    } else if (q < -128) {
        q = -128;
    }
    return (int8_t)q;
}
