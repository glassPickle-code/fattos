# Native edge AI on FattOS

## What runs on the MCU
`fattos_nn` is a small symmetric-INT8 neural-network interpreter that runs
inside the static FattOS tensor arena (16 KiB default, `FATTOS_TENSOR_ARENA_SIZE`):

- Ops: `FULLY_CONNECTED` (int8 weights, int32 bias, int32 accumulate) and
  `RELU` (clamps to [0, 127] in symmetric quantization).
- Per-layer requantization: `result = clamp8((acc * multiplier) >> shift)`.
- Scratch: two int8 ping/pong buffers + one int32 accumulator row,
  carved out of the arena at `fattos_nn_bind()`; the shape chain is
  validated there (first bind failure wins, nothing is allocated).
- Blocking and bounded: `fattos_nn_run()` executes in the calling task's
  context with a fixed step count — no bytecode dispatch, no recursion.

## Producing a model
Train anywhere (Keras, PyTorch, ONNX), then quantize a small MLP to
symmetric INT8 (zero point 0) and export per layer:

1. `weights[out][in]` as `int8`, `bias[out]` as `int32` (bias scaled by
   input_scale * weight_scale before the requant step).
2. For each layer compute the float requant scale
   `scale = (input_scale * weight_scale) / output_scale`.
3. Convert to runtime parameters with
   `fattos_nn_make_scale(scale, &multiplier, &shift)`, which solves
   `scale ≈ multiplier / 2^shift`.
4. Fill a `FattosNNLayer` per layer (op, pointers, feature counts,
   multiplier, shift), then `FattosNNModel` and call `fattos_nn_bind()`.

At runtime quantize inputs with `fattos_nn_quantize_value()` (or
`fattos_features_quantize()` for a feature vector) using the input scale
your toolchain reported, call `fattos_nn_run()`, and read class scores from
the output buffer; `fattos_nn_argmax()` gives the class decision.

## Feature pipeline
`fattos_features` turns raw sensor streams into those inputs: push samples
into a fixed rolling window (axes × length), then
`fattos_features_vector()` produces `[mean(axes), rms(axes)]` ready for
quantization. Typical setup: 32-sample × 3-axis window at 100 Hz →
6-feature vector → quantize → `fattos_nn_run()`.

## When to use something bigger
The runtime is intentionally dense-only. Choose TensorFlow Lite Micro or
CMSIS-NN behind the `FattosAIIO` seam when you need convolutions, depthwise
ops, LSTM, or models whose scratch exceeds the arena. Keep the same rule
either way: validate outputs (bounds, rate-of-change, fallback policy)
before they can influence actuator commands, and let the safety supervisor
gate the final outputs.
