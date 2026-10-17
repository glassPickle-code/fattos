#include "fattos/fattos_ai.h"
#include "fattos/fattos_config.h"
#include "fattos/fattos_nn.h"

#if defined(__GNUC__) || defined(__clang__)
#define FATTOS_ALIGN_ATTR __attribute__((aligned(FATTOS_TENSOR_ARENA_ALIGNMENT)))
#else
#define FATTOS_ALIGN_ATTR
#endif

static uint8_t g_tensor_arena[FATTOS_TENSOR_ARENA_SIZE] FATTOS_ALIGN_ATTR;

FattosStatus fattos_ai_init(void)
{
    /* A model may be bound now (fattos_nn_bind) or an external backend may
       take over the seam later; both are valid after init returns OK. */
    return FATTOS_OK;
}

FattosStatus fattos_ai_infer(const FattosAIIO *io)
{
    if (io == NULL || io->input == NULL || io->output == NULL ||
        io->input_size == 0U || io->output_size == 0U) {
        return FATTOS_ERR_INVALID_ARG;
    }

    /* Native INT8 runtime is the default backend: when a model is bound to
       the arena, route the seam to it with exact shape checking. */
    if (fattos_nn_is_bound()) {
        if (io->input_size != fattos_nn_input_len() ||
            io->output_size != fattos_nn_output_len()) {
            return FATTOS_ERR_INVALID_ARG;
        }
        return fattos_nn_run(io->input, io->input_size, io->output,
                            io->output_size);
    }

    /* No backend bound: never pretend inference succeeded. Integrate an
       external runtime (TFLM/CMSIS-NN) behind this seam. */
    return FATTOS_ERR_NOT_IMPLEMENTED;
}

size_t fattos_ai_arena_size(void)
{
    return sizeof(g_tensor_arena);
}

void *fattos_ai_arena_data(void)
{
    return (void *)g_tensor_arena;
}