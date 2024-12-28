/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_WORLD_H
#define P2D_WORLD_H

#include "p2d/core.h"

struct p2d_world_node {
    struct p2d_object *object;
    struct p2d_world_node *next;
};

/*
    Explanation of the world representation:
    The hash table contains world tiles, that each contain lists of objects in their tiles.
*/
extern struct p2d_world_node *p2d_world[P2D_MAX_OBJECTS];

/*
    Converts an object's position to a hash bucket tile index
*/
P2D_API int p2d_world_hash(float x, float y);

/*
    Inserts an object into the world

    Uses the hash table under the hood to place
    the object in it's world tile bucket
*/
P2D_API void p2d_world_insert(struct p2d_object *object);

P2D_API void p2d_world_remove(struct p2d_object *object);

#endif // P2D_WORLD_H