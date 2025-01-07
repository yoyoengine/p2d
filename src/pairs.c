/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <stdlib.h>

#include "p2d/pairs.h"

static struct p2d_pair_table pair_table;

static size_t hash_pair(struct p2d_object *a, struct p2d_object *b) {
    return (((size_t)a ^ (size_t)b) & (P2D_PAIR_BUCKET_COUNT - 1));
}

void p2d_pairs_init(void) {
    for(int i = 0; i < P2D_PAIR_BUCKET_COUNT; i++) {
        pair_table.buckets[i] = NULL;
    }
}

bool p2d_collision_pair_exists(struct p2d_object *a, struct p2d_object *b) {
    size_t bucket = hash_pair(a, b);
    struct p2d_pair_node *current = pair_table.buckets[bucket];
    
    while(current) {
        if((current->a == a && current->b == b) || 
            (current->a == b && current->b == a)) {
            return true;
        }
        current = current->next;
    }
    return false;
}

bool p2d_add_collision_pair(struct p2d_object *a, struct p2d_object *b) {
    if(p2d_collision_pair_exists(a, b)) {
        return false;
    }

    size_t bucket = hash_pair(a, b);
    struct p2d_pair_node *node = malloc(sizeof(struct p2d_pair_node));
    
    node->a = a;
    node->b = b;
    node->next = pair_table.buckets[bucket];
    pair_table.buckets[bucket] = node;
    
    p2d_state.p2d_collision_pairs++;

    return true;
}

bool p2d_reset_collision_pairs(void) {
    for(int i = 0; i < P2D_PAIR_BUCKET_COUNT; i++) {
        struct p2d_pair_node *current = pair_table.buckets[i];
        while(current) {
            struct p2d_pair_node *next = current->next;
            free(current);
            current = next;
        }
        pair_table.buckets[i] = NULL;
    }
    p2d_state.p2d_collision_pairs = 0;
    return true;
}
