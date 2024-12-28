/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_CORE_H
#define P2D_CORE_H

#include "p2d/export.h"

#include <stdbool.h>

// compile time parameters
#ifndef P2D_MAX_OBJECTS
    #define P2D_MAX_OBJECTS 2000
#endif

#ifndef P2D_BUCKETS
    #define P2D_BUCKETS 1000
#endif

// TODO
// #ifdef P2D_HAS_YOYOENGINE
//     #include "yoyoengine.h"
//     #define p2d_log(...) ye_logf(YE_LL_DEBUG, __VA_ARGS__)
// #else
//     #include <stdio.h>
//     #define p2d_log(...) printf(__VA_ARGS__)
// #endif

/*
    I really like runtime state tracking in my libraries,
    particularly for visualization and debug overlays.
*/
struct p2d_state {
    int p2d_object_count;
    int p2d_world_node_count;
};
extern struct p2d_state p2d_state;

enum p2d_object_type {
    P2D_OBJECT_RECTANGLE,
    P2D_OBJECT_CIRCLE
};

struct p2d_object {
    // defining information
    enum p2d_object_type type;
    bool is_static;

    // location
    float x;
    float y;

    // forces
    float vx;
    float vy;
    
    // physical properties
    float rotation;
    union {
        struct {
            float width;
            float height;
        } rectangle;

        struct {
            float radius;
        } circle;
    };
};

/*
    Initialize the p2d simulation
    cell_size: the size of each cell in the world (used for broad phase collision detection)
*/
P2D_API bool p2d_init(int cell_size);

/*
    Register a p2d object to be simulated
*/
// P2D_API bool p2d_create_object(struct p2d_object *object);

/*
    Remove a p2d object from the simulation
*/
// P2D_API void p2d_remove_object(struct p2d_object *object);

/*
    Remove all objects from the simulation
*/
// P2D_API void p2d_remove_all_objects();

#endif // P2D_CORE_H