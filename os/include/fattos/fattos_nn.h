#ifndef FATTOS_NN_H
#define FATTOS_NN_H

#include <stddef.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Tiny native edge-AI runtime: a fully-connected INT8 neural-network
 * interpreter that executes entirely inside the static FattOS tensor arena.
 * No malloc, no interpreter bytecode, no external runtime dependency.
 *
 * Model format (produced offline, e.g. from a quantized Keras/ONNX MLP):
 *   - FC layers: int8 weights [out_features][input_features], int32 bias,
 *     per-layer requantization (int32 multiplier, right shift).
 *   - ReLU layers: symmetric-INT8 clamp to [0, 127].
 *   - Symmetric quantization, zero point is always 0.
 *
 * This is intended for small dense models (anomaly detection, gesture
 * classifiers, learned residuals). For CNNs or large models integrate
 * TensorFlow Lite Micro / CMSIS-NN behind the FattosAIIO seam instead.
 */
typedef enum {
    FATTOS_NN_OP_FULLY_CONNECTED = 0,
    FATTOS_NN_OP_RELU = 1
} FattosNNOp;

typedef struct {
    uint8_t op;                     /* FattosNNOp */
    const int8_t *weights;          /* FC: [out][in]. Ignored for ReLU. */
    const int32_t *bias;            /* FC: [out]. Ignored for ReLU. */
    size_t input_features;
    size_t output_features;
    int32_t multiplier;             /* Requant multiplier, > 0 */
    int32_t shift;                  /* Requant right shift, 0..62 */
} FattosNNLayer;

typedef struct {
    const FattosNNLayer *layers;
    size_t layer_count;
} FattosNNModel;

/* Bind model to the static tensor arena (validates shape chain and fits
   scratch). Call once at startup. */
FattosStatus fattos_nn_bind(const FattosNNModel *model);
bool fattos_nn_is_bound(void);

/* input_len must equal first layer input_features, output_len the last
   layer output_features. Blocking, bounded, runs in the caller's context. */
FattosStatus fattos_nn_run(const int8_t *input, size_t input_len,
                         int8_t *output, size_t output_len);
size_t fattos_nn_output_len(void);
size_t fattos_nn_input_len(void); /* 0 while no model is bound */

/* argmax of an int8 class-score vector (classification decision). */
int fattos_nn_argmax(const int8_t *values, size_t count);

/* Convert a float scale to the (multiplier, shift) requant pair so
   toolchains can emit runtime parameters from float quantization.
   Works for any scale > 0 (including scale >= 1.0): the shift is chosen
   so the multiplier always fits a positive int32. */
FattosStatus fattos_nn_make_scale(float scale, int32_t *multiplier, int32_t *shift);

/* float value -> symmetric int8 with the given scale, clamped. */
int8_t fattos_nn_quantize_value(float value, float scale);

#ifdef __cplusplus
}
#endif

#endif
