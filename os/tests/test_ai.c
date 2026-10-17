#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "fattos/fattos_ai.h"
#include "fattos/fattos_features.h"
#include "fattos/fattos_nn.h"
#include <stdlib.h>

/* Terminating check: the Windows debug-CRT assert can hang CI on a dialog. */
#undef assert
#define assert(cond)                                                            \
    do {                                                                        \
        if (!(cond)) {                                                          \
            fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__,    \
                    #cond);                                                     \
            exit(1);                                                            \
        }                                                                       \
    } while (0)

/* ---- INT8 neural net: hand-computed expectations ------------------------ */
static const int8_t w0[4] = {1, 0, 0, 1};   /* 2x2 identity, row-major */
static const int32_t b0[2] = {0, 0};
static const int8_t w2[4] = {1, 1, 1, 0};   /* [[1,1],[1,0]] */
static const int32_t b2[2] = {0, 0};

static void test_nn_forward(void)
{
    FattosNNLayer layers[3];
    layers[0] = (FattosNNLayer){FATTOS_NN_OP_FULLY_CONNECTED, w0, b0, 2U, 2U, 1, 0};
    layers[1] = (FattosNNLayer){FATTOS_NN_OP_RELU, NULL, NULL, 2U, 2U, 0, 0};
    layers[2] = (FattosNNLayer){FATTOS_NN_OP_FULLY_CONNECTED, w2, b2, 2U, 2U, 1, 0};
    FattosNNModel model = {layers, 3U};

    assert(fattos_ai_init() == FATTOS_OK);
    assert(!fattos_nn_is_bound());
    assert(fattos_nn_bind(&model) == FATTOS_OK);
    assert(fattos_nn_is_bound());
    assert(fattos_nn_output_len() == 2U);

    /* {10,-20} -> identity -> ReLU {10,0} -> [[1,1],[1,0]] -> {10,10} */
    int8_t input[2] = {10, -20};
    int8_t output[2] = {0, 0};
    assert(fattos_nn_run(input, 2U, output, 2U) == FATTOS_OK);
    assert(output[0] == 10 && output[1] == 10);
    assert(fattos_nn_argmax(output, 2U) == 0);

    /* Negative path exercises ReLU clamping. */
    int8_t input_neg[2] = {-5, -7};
    assert(fattos_nn_run(input_neg, 2U, output, 2U) == FATTOS_OK);
    assert(output[0] == 0 && output[1] == 0); /* both clamped by ReLU */

    /* Shape mismatches are rejected, not guessed. */
    assert(fattos_nn_run(input, 1U, output, 2U) == FATTOS_ERR_INVALID_ARG);
    assert(fattos_nn_run(input, 2U, output, 1U) == FATTOS_ERR_INVALID_ARG);
    assert(fattos_nn_run(NULL, 2U, output, 2U) == FATTOS_ERR_INVALID_ARG);
}

static void test_nn_requant_scale(void)
{
    int32_t multiplier = 0;
    int32_t shift = 0;
    assert(fattos_nn_make_scale(0.25f, &multiplier, &shift) == FATTOS_OK);
    /* acc * (0.25 * 2^31) >> 31 == acc * 0.25 */
    assert(shift == 31);
    assert(multiplier == (int32_t)(1U << 29));

    /* weight 3, bias 1, input 10 -> acc 31 -> 31 * 0.25 = 7 (truncating). */
    static const int8_t w[1] = {3};
    static const int32_t b[1] = {1};
    FattosNNLayer layer = {FATTOS_NN_OP_FULLY_CONNECTED, w, b, 1U, 1U,
                          multiplier, shift};
    FattosNNModel model = {&layer, 1U};
    assert(fattos_nn_bind(&model) == FATTOS_OK);

    int8_t input[1] = {10};
    int8_t output[1] = {-1};
    assert(fattos_nn_run(input, 1U, output, 1U) == FATTOS_OK);
    assert(output[0] == 7);

    /* Value quantization helpers clamp symmetric int8. */
    assert(fattos_nn_quantize_value(1.0f, 0.25f) == 4);
    assert(fattos_nn_quantize_value(-1.0f, 0.25f) == -4);
    assert(fattos_nn_quantize_value(1000.0f, 0.25f) == 127);
    assert(fattos_nn_quantize_value(-1000.0f, 0.25f) == -128);
    assert(fattos_nn_make_scale(0.0f, &multiplier, &shift) == FATTOS_ERR_INVALID_ARG);

    /* Regression: scale >= 1.0 used to loop to failure (shift only ever
       increased). scale 1.0 must requantize as an exact identity. */
    assert(fattos_nn_make_scale(1.0f, &multiplier, &shift) == FATTOS_OK);
    assert(shift == 30 && multiplier == (int32_t)(1U << 30));
    FattosNNLayer identity = {FATTOS_NN_OP_FULLY_CONNECTED, w, b, 1U, 1U,
                             multiplier, shift};
    FattosNNModel one = {&identity, 1U};
    assert(fattos_nn_bind(&one) == FATTOS_OK);
    int8_t one_out[1] = {-1};
    assert(fattos_nn_run(input, 1U, one_out, 1U) == FATTOS_OK);
    assert(one_out[0] == 31); /* 3*10 + 1, unscaled */

    /* Broken shape chain rejected at bind. */
    FattosNNLayer broken[2];
    broken[0] = layer;
    broken[1] = layer; /* input_features 1 != previous output 1? equal, ok */
    broken[1].input_features = 5U; /* now mismatched */
    FattosNNModel bad = {broken, 2U};
    assert(fattos_nn_bind(&bad) == FATTOS_ERR_INVALID_ARG);
    assert(fattos_nn_bind(NULL) == FATTOS_ERR_INVALID_ARG);
}

/* ---- Feature pipeline --------------------------------------------------- */
static void test_features(void)
{
    float storage[4U * 2U];
    FattosFeatureWindow window;
    assert(fattos_features_init(&window, storage, 4U, 2U) == FATTOS_OK);
    assert(fattos_features_count(&window) == 0U);
    assert(fattos_features_vector_len(&window) == 4U);

    const float samples[4][2] = {{1.0f, 2.0f}, {3.0f, 4.0f},
                                 {5.0f, 6.0f}, {7.0f, 8.0f}};
    for (size_t i = 0U; i < 4U; ++i) {
        assert(fattos_features_push(&window, samples[i]) == FATTOS_OK);
    }
    assert(fattos_features_count(&window) == 4U);

    float mean[2];
    assert(fattos_features_mean(&window, mean) == FATTOS_OK);
    assert(fabsf(mean[0] - 4.0f) < 1e-4f); /* (1+3+5+7)/4 */
    assert(fabsf(mean[1] - 5.0f) < 1e-4f); /* (2+4+6+8)/4 */

    float rms[2];
    assert(fattos_features_rms(&window, rms) == FATTOS_OK);
    assert(fabsf(rms[0] - sqrtf(21.0f)) < 1e-2f); /* (1+9+25+49)/4 */
    assert(fabsf(rms[1] - sqrtf(30.0f)) < 1e-2f); /* (4+16+36+64)/4 */

    /* Window wraps: oldest sample falls out. */
    const float fifth[2] = {9.0f, 10.0f};
    assert(fattos_features_push(&window, fifth) == FATTOS_OK);
    assert(fattos_features_count(&window) == 4U);
    assert(fattos_features_mean(&window, mean) == FATTOS_OK);
    assert(fabsf(mean[0] - 6.0f) < 1e-4f); /* (3+5+7+9)/4 */
    assert(fabsf(mean[1] - 7.0f) < 1e-4f); /* (4+6+8+10)/4 */

    float vector[4];
    assert(fattos_features_vector(&window, vector) == FATTOS_OK);
    assert(fabsf(vector[0] - 6.0f) < 1e-4f && fabsf(vector[1] - 7.0f) < 1e-4f);

    int8_t quantized[4];
    assert(fattos_features_quantize(vector, 4U, 0.5f, quantized) == FATTOS_OK);
    assert(quantized[0] == 12); /* 6.0 / 0.5 */
    assert(quantized[1] == 14); /* 7.0 / 0.5 */
    assert(fattos_features_quantize(vector, 4U, 0.0f, quantized) ==
           FATTOS_ERR_INVALID_ARG);
}

/* ---- AI seam ----------------------------------------------------------- */
static void test_ai_seam(void)
{
    int8_t in[2] = {10, -20};
    int8_t out[2] = {0, 0};
    FattosAIIO io = {in, 2U, out, 2U};

    /* Bind a known model: identity FC + ReLU. */
    FattosNNLayer layers[2] = {
        {FATTOS_NN_OP_FULLY_CONNECTED, w0, b0, 2U, 2U, 1, 0},
        {FATTOS_NN_OP_RELU, NULL, NULL, 2U, 2U, 0, 0},
    };
    FattosNNModel model = {layers, 2U};
    assert(fattos_nn_bind(&model) == FATTOS_OK);

    /* Seam routes to the native runtime with exact shape checking. */
    assert(fattos_ai_infer(&io) == FATTOS_OK);
    assert(out[0] == 10 && out[1] == 0); /* identity then ReLU */

    FattosAIIO bad_in = {in, 1U, out, 2U};
    assert(fattos_ai_infer(&bad_in) == FATTOS_ERR_INVALID_ARG);
    FattosAIIO bad_out = {in, 2U, out, 1U};
    assert(fattos_ai_infer(&bad_out) == FATTOS_ERR_INVALID_ARG);
    FattosAIIO nil_in = {NULL, 2U, out, 2U};
    assert(fattos_ai_infer(&nil_in) == FATTOS_ERR_INVALID_ARG);
    assert(fattos_ai_infer(NULL) == FATTOS_ERR_INVALID_ARG);
}

int main(void)
{
    test_features();   /* no scheduler dependency */

    /* Unbound seam refuses honestly — checked before any model is bound. */
    {
        int8_t in[2] = {10, -20};
        int8_t out[2] = {0, 0};
        FattosAIIO io = {in, 2U, out, 2U};
        assert(fattos_ai_infer(&io) == FATTOS_ERR_NOT_IMPLEMENTED);
    }

    test_nn_forward();
    test_nn_requant_scale();
    test_ai_seam();

    puts("FattOS AI tests passed");
    return 0;
}
