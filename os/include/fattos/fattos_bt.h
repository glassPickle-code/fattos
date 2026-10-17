#ifndef FATTOS_BT_H
#define FATTOS_BT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fattos.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Behavior-tree executor for autonomous behaviors: a static tree of
 * composites and leaf nodes, ticked from a periodic task, operating on a
 * caller-owned blackboard. No allocation, no recursion beyond the tree
 * depth (capped), deterministic per tick — the whole point is that an
 * agent- or human-written behavior is a DATA STRUCTURE, not control flow
 * scattered across callbacks.
 *
 * Node types:
 *   SEQUENCE:   tick children in order; first FAILURE fails the whole
 *               node, a RUNNING child is resumed on the next tick (memory).
 *   SELECTOR:   tick children in order; first SUCCESS succeeds the node,
 *               a RUNNING child is resumed on the next tick (memory).
 *   INVERTER:   flips SUCCESS/FAILURE of its single child, passes RUNNING.
 *   ACTION:     leaf; calls its tick function with the blackboard.
 *   CONDITION:  leaf; same call convention, semantic difference only.
 *
 * Build the node array parent-before-child with the root at index 0;
 * children of a composite are contiguous starting at first_child.
 */
typedef enum {
    FATTOS_BT_SUCCESS = 0,
    FATTOS_BT_FAILURE = 1,
    FATTOS_BT_RUNNING = 2
} FattosBTStatus;

typedef FattosBTStatus (*FattosBTTickFn)(void *blackboard);

typedef enum {
    FATTOS_BT_SEQUENCE = 0,
    FATTOS_BT_SELECTOR = 1,
    FATTOS_BT_INVERTER = 2,
    FATTOS_BT_ACTION = 3,
    FATTOS_BT_CONDITION = 4
} FattosBTNodeType;

typedef struct {
    uint8_t type;         /* FattosBTNodeType */
    int8_t first_child;   /* index of first child; -1 for leaves */
    int8_t child_count;   /* 0 for leaves, 1 for INVERTER */
    int8_t resume_child;  /* RUNNING child to resume (runtime state) */
    FattosBTTickFn tick;  /* leaf callback, receives the blackboard */
    void *context;        /* per-node scratch, optional */
} FattosBTNode;

typedef struct {
    FattosBTNode *nodes;  /* caller-owned, mutable (resume state lives here) */
    size_t count;
    void *blackboard;     /* opaque app state handed to every leaf */
    uint32_t ticks;       /* root ticks so far */
} FattosBT;

#ifndef FATTOS_BT_MAX_DEPTH
#define FATTOS_BT_MAX_DEPTH 16
#endif

/* Validate the tree shape (indices in range, leaves have callbacks). */
FattosStatus fattos_bt_init(FattosBT *bt, FattosBTNode *nodes, size_t count,
                            void *blackboard);

/* Tick the root once. Returns the root status; FATTOS_BT_FAILURE with an
   invalid tree, never asserts — a broken tree is a failed tick. */
FattosBTStatus fattos_bt_tick(FattosBT *bt);

#ifdef __cplusplus
}
#endif

#endif