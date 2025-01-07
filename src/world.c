/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <stdlib.h>

#include "p2d/core.h"
#include "p2d/world.h"

// collection of world tiles
struct p2d_object * p2d_objects[P2D_MAX_OBJECTS] = {NULL};
struct p2d_world_node *p2d_world[P2D_MAX_OBJECTS] = {NULL};

int p2d_world_hash(int tile_x, int tile_y) {
    int hash_x = tile_x * 73856093;
    int hash_y = tile_y * 19349663;
    int hash = (hash_x + hash_y) % P2D_MAX_OBJECTS;
    if(hash < 0)
        hash += P2D_MAX_OBJECTS;

    return hash;
}

void p2d_world_insert(int world_hash, struct p2d_object *object) {
    if(object == NULL) {
        p2d_logf(P2D_LOG_ERROR, "p2d_world_insert: object is NULL.\n");
        return;
    }

    int index = world_hash;

    struct p2d_world_node *node = malloc(sizeof(struct p2d_world_node));
    node->object = object;
    node->next = p2d_world[index];
    if(p2d_world[index] == NULL) // track new buckets
        p2d_state.p2d_world_node_count++;
    p2d_world[index] = node;
}

void p2d_world_remove(int world_hash, struct p2d_object *object) {
    if(object == NULL) {
        p2d_logf(P2D_LOG_ERROR, "p2d_world_remove: object is NULL.\n");
        return;
    }

    int index = world_hash;
    struct p2d_world_node *node = p2d_world[index];
    struct p2d_world_node *prev = NULL;
    bool was_first = true;

    while(node != NULL) {
        if(node->object == object) {
            if(prev == NULL) {
                p2d_world[index] = node->next;
            } else {
                prev->next = node->next;
            }

            free(node);
            
            // Only decrement if we removed the last node in this bucket
            if(p2d_world[index] == NULL && was_first) {
                p2d_state.p2d_world_node_count--;
            }
            return;
        }
        was_first = false;
        prev = node;
        node = node->next;
    }
}

void p2d_world_remove_all(void) {
    for(int i = 0; i < P2D_MAX_OBJECTS; i++) {
        struct p2d_world_node *node = p2d_world[i];
        while(node != NULL) {
            struct p2d_world_node *next = node->next;
            free(node);
            node = next;
        }
        p2d_world[i] = NULL;
    }
    p2d_state.p2d_world_node_count = 0;
    // p2d_state.p2d_object_count = 0;
}

/*
    TODO: further optimize, static objs never move

    see TODO in README.md, we can just not bother to create the world until ready
*/
void p2d_rebuild_world(void) {
    p2d_world_remove_all();

    for(int i = 0; i < P2D_MAX_OBJECTS; i++) {
        struct p2d_object *object = p2d_objects[i];
        if(object != NULL) {
            p2d_for_each_intersecting_tile(object, _register_intersecting_tiles);
        }
    }
}
