/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_CORE_H
#define P2D_CORE_H

#include "p2d/export.h"

#include <stdbool.h>
#include <stdarg.h>

// compile time parameters
#ifndef P2D_MAX_OBJECTS
    #define P2D_MAX_OBJECTS 2000
#endif

#ifndef P2D_BUCKETS
    #define P2D_BUCKETS 1000
#endif

#ifdef P2D_HAS_YOYOENGINE
    #include <yoyoengine.h>
    #define p2d_logf(...) ye_logf(__VA_ARGS__)
    #define P2D_LOG_DEBUG YE_LL_DEBUG
    #define P2D_LOG_INFO YE_LL_INFO
    #define P2D_LOG_WARN YE_LL_WARNING
    #define P2D_LOG_ERROR YE_LLL_ERROR
#else
    enum p2d_log_level {
        P2D_LOG_DEBUG,
        P2D_LOG_INFO,
        P2D_LOG_WARN,
        P2D_LOG_ERROR
    };

    void p2d_logf(int level, const char *fmt, ...);
#endif


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
    Shutdown the p2d simulation and free its memory
*/
P2D_API bool p2d_shutdown();

/*
    Register a p2d object to be simulated
*/
P2D_API bool p2d_create_object(struct p2d_object *object);

/*
    Remove a p2d object from the simulation
*/
P2D_API bool p2d_remove_object(struct p2d_object *object);

/*
    Remove all objects from the simulation
*/
P2D_API bool p2d_remove_all_objects();

/*
    Called externally to run one simulation step
*/
P2D_API void p2d_step(float delta_time);

#endif // P2D_CORE_H