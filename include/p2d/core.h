/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_CORE_H
#define P2D_CORE_H

#include <Lilith.h>

#include "p2d/export.h"
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

/*
    Densities: g/cm^3
*/
#ifndef P2D_MIN_DENSITY
    #define P2D_MIN_DENSITY 0.25f
#endif

#ifndef P2D_MAX_DENSITY
    #define P2D_MAX_DENSITY 20.0f
#endif

#ifndef P2D_DEFAULT_MASS_SCALE
    #define P2D_DEFAULT_MASS_SCALE 0.00015f
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

    /*
        Runtime Parameters
    */
    int     p2d_cell_size;
    int     p2d_substeps;
    vec2_t  p2d_gravity;
    float   p2d_mass_scaling;

    // callbacks
    void (*on_collision)(struct p2d_cb_data *data);
    void (*on_trigger)(struct p2d_cb_data *data);
    void (*log)(int level, const char *fmt, ...);

    // tracking / debug
    int p2d_object_count;
    int p2d_world_node_count;
    int p2d_contact_checks;
    int p2d_contacts_found;
    int p2d_collision_pairs;

    // optional
    struct p2d_contact_list *out_contacts; // will be populated and cleared assuming user has filled this. user must free it themselves
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
    
    // shape properties
    float rotation;
    union {
        struct {
            float width;
            float height;

            // COMPUTED //
            float width_meters;
            float height_meters;
        } rectangle;

        struct {
            float radius;

            // COMPUTED //
            float radius_meters;
        } circle;
    };

    // physical properties
    float density;

    float mass; // TODO: auto calc mass from density?
    float restitution;

    // computed physical properties
    
    // 1m = 100 pixels

    float inertia; // automatically calculated?

    // real computed
    float inv_mass;
    float inv_inertia;
    float area;

    /*
        User data, useful for things like identifying this object in an
        engine's ECS to operate on its impulses when collisions occur.
    */
    void *user_data;

    /*
        in/out reference values
    */
    bool * in_active;
    float * out_x;
    float * out_y;
    float * out_rotation;

    /*
        Debug Optionals
    */
    
};

struct p2d_collision_manifold {
    struct p2d_object *a;
    struct p2d_object *b;
    vec2_t normal;
    float penetration;

    vec2_t contact_points[2];
    int contact_count;
};

/*
    Initialize the p2d simulation
    cell_size: the size of each cell in the world (used for broad phase collision detection)
*/
P2D_API bool p2d_init(
    int cell_size,
    int substeps,
    void (*on_collision)(struct p2d_cb_data *data), 
    void (*on_trigger)(struct p2d_cb_data *data),
    void (*log_fn)(int level, const char *fmt, ...)
);

/*
    Shutdown the p2d simulation and free its memory
*/
P2D_API bool p2d_shutdown(void);

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
P2D_API bool p2d_remove_all_objects(void);

/*
    Called externally to run one simulation step
*/
// P2D_API struct p2d_contact_list * p2d_step(float delta_time);
P2D_API void p2d_step(float delta_time);

/*
    Helper-ish (poorly organized) functions
*/
void p2d_for_each_intersecting_tile(struct p2d_object *object, void (*callback)(struct p2d_object *object, int tile_hash));

void _register_intersecting_tiles(struct p2d_object *object, int hash);

void _unregister_intersecting_tiles(struct p2d_object *object, int hash);

#endif // P2D_CORE_H
