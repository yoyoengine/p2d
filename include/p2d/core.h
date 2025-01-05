/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_CORE_H
#define P2D_CORE_H

#include "p2d/export.h"
#include "p2d/queue.h"
#include "p2d/types.h"

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
    How callbacks and resolutions work:

    Engine will call p2d_step() every frame.
    During p2d_step(), p2d will use the on_collision and on_trigger callbacks to notify the engine of collisions and triggers as they occur.

    Once p2d_step() resolves, it will return a built list of simulation changes for the engine to consume and update
    in it's own ECS or other entity management system.
*/

struct p2d_cb_data {
    struct p2d_object *a;
    struct p2d_object *b;
};

/*
    I really like runtime state tracking in my libraries,
    particularly for visualization and debug overlays.
*/
struct p2d_state {
    // config
    // passed on init
    int _cell_size;

    // init
    struct p2d_vec2 gravity;

    // callbacks
    void (*on_collision)(struct p2d_cb_data *data);
    void (*on_trigger)(struct p2d_cb_data *data);

    // tracking / debug
    int p2d_object_count;
    int p2d_world_node_count;
    int p2d_contact_checks;
    int p2d_contacts_found;
    int p2d_collision_pairs;
};
extern struct p2d_state p2d_state;

// TODO: split rects to OBB and AABB?
enum p2d_object_type {
    P2D_OBJECT_RECTANGLE,
    P2D_OBJECT_CIRCLE
};

// TODO: allow frozen axes?
struct p2d_object {
    // defining information
    enum p2d_object_type type;
    bool is_static;     // if true, this object is immovable
    bool is_trigger;    // if true, this object does not collide with other objects, but instead emits events when collided with

    // location
    float x;
    float y;

    // forces
    float vx;
    float vy;
    float vr;
    // float force_x;
    // float force_y;
    // float rotation_force;
    
    // shape properties
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

    // physical properties
    float mass;
    float restitution;

    /*
        User data, useful for things like identifying this object in an
        engine's ECS to operate on its impulses when collisions occur.
    */
    void *user_data;
};

struct p2d_collision_manifold {
    struct p2d_object *a;
    struct p2d_object *b;
    struct p2d_vec2 normal;
    float penetration;

    struct p2d_vec2 contact_points[2];
    int contact_count;
};

/*
    Initialize the p2d simulation
    cell_size: the size of each cell in the world (used for broad phase collision detection)
*/
P2D_API bool p2d_init(int cell_size, void (*on_collision)(struct p2d_cb_data *data), void (*on_trigger)(struct p2d_cb_data *data));

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
P2D_API struct p2d_contact_list * p2d_step(float delta_time);

/*
    Helper-ish (poorly organized) functions
*/
void p2d_for_each_intersecting_tile(struct p2d_object *object, void (*callback)(struct p2d_object *object, int tile_hash));

void _register_intersecting_tiles(struct p2d_object *object, int hash);

void _unregister_intersecting_tiles(struct p2d_object *object, int hash);

#endif // P2D_CORE_H