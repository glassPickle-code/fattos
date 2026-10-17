#include "fattos/fattos_bt.h"

static bool valid_node(const FattosBT *bt, int index)
{
    return index >= 0 && (size_t)index < bt->count;
}

FattosStatus fattos_bt_init(FattosBT *bt, FattosBTNode *nodes, size_t count,
                            void *blackboard)
{
    if (bt == NULL || nodes == NULL || count == 0U ||
        count > (size_t)FATTOS_BT_MAX_DEPTH * 8U) {
        return FATTOS_ERR_INVALID_ARG;
    }
    for (size_t i = 0U; i < count; ++i) {
        FattosBTNode *node = &nodes[i];
        node->resume_child = 0;
        switch ((FattosBTNodeType)node->type) {
        case FATTOS_BT_SEQUENCE:
        case FATTOS_BT_SELECTOR:
            if (node->child_count <= 0 || node->first_child < 0 ||
                (size_t)node->first_child + (size_t)node->child_count > count) {
                return FATTOS_ERR_INVALID_ARG; /* children out of range */
            }
            break;
        case FATTOS_BT_INVERTER:
            if (node->child_count != 1 || node->first_child < 0 ||
                (size_t)node->first_child >= count) {
                return FATTOS_ERR_INVALID_ARG;
            }
            break;
        case FATTOS_BT_ACTION:
        case FATTOS_BT_CONDITION:
            if (node->tick == NULL) {
                return FATTOS_ERR_INVALID_ARG; /* leaf without callback */
            }
            break;
        default:
            return FATTOS_ERR_INVALID_ARG;
        }
    }
    bt->nodes = nodes;
    bt->count = count;
    bt->blackboard = blackboard;
    bt->ticks = 0U;
    return FATTOS_OK;
}

static FattosBTStatus tick_node(FattosBT *bt, int index, int depth)
{
    if (depth > FATTOS_BT_MAX_DEPTH || !valid_node(bt, index)) {
        return FATTOS_BT_FAILURE; /* degenerate tree: fail safe, don't hang */
    }
    FattosBTNode *node = &bt->nodes[index];

    if (node->type == FATTOS_BT_ACTION ||
        node->type == FATTOS_BT_CONDITION) {
        return node->tick(bt->blackboard);
    }
    if (node->type == FATTOS_BT_INVERTER) {
        FattosBTStatus status = tick_node(bt, node->first_child, depth + 1);
        if (status == FATTOS_BT_SUCCESS) {
            return FATTOS_BT_FAILURE;
        }
        if (status == FATTOS_BT_FAILURE) {
            return FATTOS_BT_SUCCESS;
        }
        return FATTOS_BT_RUNNING;
    }

    /* SEQUENCE / SELECTOR with resume memory. */
    bool is_sequence = (node->type == FATTOS_BT_SEQUENCE);
    int start = node->resume_child;
    for (int c = start; c < node->child_count; ++c) {
        FattosBTStatus status = tick_node(bt, node->first_child + c, depth + 1);
        if (status == FATTOS_BT_RUNNING) {
            node->resume_child = (int8_t)c; /* resume here next tick */
            return FATTOS_BT_RUNNING;
        }
        bool done = is_sequence ? (status == FATTOS_BT_FAILURE)
                                : (status == FATTOS_BT_SUCCESS);
        if (done) {
            node->resume_child = 0;
            return is_sequence ? FATTOS_BT_FAILURE : FATTOS_BT_SUCCESS;
        }
        /* this child settled the other way: advance */
    }
    node->resume_child = 0;
    return is_sequence ? FATTOS_BT_SUCCESS : FATTOS_BT_FAILURE;
}

FattosBTStatus fattos_bt_tick(FattosBT *bt)
{
    if (bt == NULL || bt->nodes == NULL || bt->count == 0U) {
        return FATTOS_BT_FAILURE;
    }
    bt->ticks++;
    return tick_node(bt, 0, 0);
}