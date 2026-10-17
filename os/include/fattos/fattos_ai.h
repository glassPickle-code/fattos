#ifndef FATTOS_AI_H
#define FATTOS_AI_H

#include <stddef.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const int8_t *input;
    size_t input_size;
    int8_t *output;
    size_t output_size;
} FattosAIIO;

/* Static arena is reserved by FattOS; backend must bind/use it when integrated. */
FattosStatus fattos_ai_init(void);
FattosStatus fattos_ai_infer(const FattosAIIO *io);
size_t fattos_ai_arena_size(void);
void *fattos_ai_arena_data(void);

#ifdef __cplusplus
}
#endif

#endif
