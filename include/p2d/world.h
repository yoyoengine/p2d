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
    Also keep a reference to all objects in the world, that doesnt require accessing
    spatially
*/
extern struct p2d_object * p2d_objects[P2D_MAX_OBJECTS];

/*
    Converts an object's position to a hash bucket tile index
*/
P2D_API int p2d_world_hash(int tile_x, int tile_y);

/*
    Inserts an object into the world

    Uses the hash table under the hood to place
    the object in it's world tile bucket
*/
P2D_API void p2d_world_insert(int world_hash, struct p2d_object *object);

/*
    Removes an object from the world

    Uses the hash table under the hood to remove
    the object from it's world tile bucket
*/
P2D_API void p2d_world_remove(int world_hash, struct p2d_object *object);

/*
    Unmap every object from the hash table
*/
P2D_API void p2d_world_remove_all();

/*
    Rebuild the world state for broad phase collision detection
*/
P2D_API void p2d_rebuild_world();

#endif // P2D_WORLD_H