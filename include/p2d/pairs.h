/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

/*
    This is a smaller subsystem that provides an O(1) lookup for collision pairs.

    Because I don't want to resize the grid dynamically (for now), the best way to
    avoid recomputing the same collision pairs across tiles is to do a fast lookup
*/

#ifndef P2D_PAIRS_H
#define P2D_PAIRS_H

#include <stdbool.h>

#include "p2d/export.h"
#include "p2d/core.h"

#ifndef P2D_PAIR_BUCKET_COUNT
    #define P2D_PAIR_BUCKET_COUNT 256
#endif

struct p2d_pair_node {
    struct p2d_object *a;
    struct p2d_object *b;

    struct p2d_pair_node *next;
};

struct p2d_pair_table {
    struct p2d_pair_node *buckets[P2D_PAIR_BUCKET_COUNT];
};

P2D_API void p2d_pairs_init(void);

P2D_API bool p2d_collision_pair_exists(struct p2d_object *a, struct p2d_object *b);

P2D_API bool p2d_add_collision_pair(struct p2d_object *a, struct p2d_object *b);

P2D_API bool p2d_reset_collision_pairs(void);

#endif // P2D_PAIRS_H