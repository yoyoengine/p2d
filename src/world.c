/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <stdlib.h>

#include "p2d/core.h"
#include "p2d/world.h"

// collection of world tiles
struct p2d_world_node *p2d_world[P2D_MAX_OBJECTS] = {NULL};

int p2d_world_hash(float x, float y) {
    return (int)((x * 73856093) + (y * 19349663)) % P2D_MAX_OBJECTS;
}

void p2d_world_insert(struct p2d_object *object) {
    if(object == NULL) {
        return;
    }

    int index = p2d_world_hash(object->x, object->y);

    struct p2d_world_node *node = malloc(sizeof(struct p2d_world_node));
    node->object = object;
    node->next = p2d_world[index];
    p2d_world[index] = node;
}

void p2d_world_remove(struct p2d_object *object) {
    if(object == NULL) {
        return;
    }

    int index = p2d_world_hash(object->x, object->y);

    struct p2d_world_node *node = p2d_world[index];
    struct p2d_world_node *prev = NULL;

    while(node != NULL) {
        if(node->object == object) {
            if(prev == NULL) {
                p2d_world[index] = node->next;
            } else {
                prev->next = node->next;
            }

            free(node);
            break;
        }

        prev = node;
        node = node->next;
    }
}