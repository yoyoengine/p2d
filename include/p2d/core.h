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

#include <stdint.h>

// compile time parameters
#ifndef P2D_MAX_OBJECTS
    #define P2D_MAX_OBJECTS 2000
#endif

#ifndef P2D_MAX_JOINTS
    #define P2D_MAX_JOINTS 500
#endif

#ifndef P2D_BUCKETS
    #define P2D_BUCKETS 1000
#endif

/*
    Densities: g/cm^3
*/
#ifndef P2D_MIN_DENSITY
    #define P2D_MIN_DENSITY 0.5f
#endif

#ifndef P2D_MAX_DENSITY
    #define P2D_MAX_DENSITY 20.0f
#endif

#ifndef P2D_DEFAULT_MASS_SCALE
    #define P2D_DEFAULT_MASS_SCALE 0.00015f
#endif

#ifndef P2D_DEFAULT_AIR_DENSITY
    #define P2D_DEFAULT_AIR_DENSITY 0.00001f
#endif

#ifndef P2D_DEFAULT_SUBSTEPS
    #define P2D_DEFAULT_SUBSTEPS 10
#endif

#ifndef P2D_DEFAULT_JOINT_SUBSTEPS
    #define P2D_DEFAULT_JOINT_SUBSTEPS 5
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
    int     p2d_joint_iterations;
    vec2_t  p2d_gravity;
    float   p2d_mass_scaling;
    float   p2d_air_density;

    // frustum sleeping
    bool   p2d_frustum_sleeping;
    struct p2d_obb p2d_frustum;

    // callbacks
    void (*on_collision)(struct p2d_cb_data *data);
    void (*on_trigger)(struct p2d_cb_data *data);
    void (*log)(int level, const char *fmt, ...);

    // tracking / debug
    int p2d_object_count;
    int p2d_sleeping_count;
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

// just a helper for user to use
enum p2d_collision_layer {
    P2D_LAYER_1 = 1 << 0,
    P2D_LAYER_2 = 1 << 1,
    P2D_LAYER_3 = 1 << 2,
    P2D_LAYER_4 = 1 << 3,
    P2D_LAYER_5 = 1 << 4,
    P2D_LAYER_6 = 1 << 5,
    P2D_LAYER_7 = 1 << 6,
    P2D_LAYER_8 = 1 << 7,
    P2D_LAYER_9 = 1 << 8,
    P2D_LAYER_10 = 1 << 9,
    P2D_LAYER_11 = 1 << 10,
    P2D_LAYER_12 = 1 << 11,
    P2D_LAYER_13 = 1 << 12,
    P2D_LAYER_14 = 1 << 13,
    P2D_LAYER_15 = 1 << 14,
    P2D_LAYER_16 = 1 << 15,
    P2D_LAYER_ALL = 0xFFFF
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
    float restitution;

    float static_friction;
    float dynamic_friction;


    // computed physical properties
    float area;
    float mass,     inv_mass;
    float inertia,  inv_inertia;

    /*
        User data, useful for things like identifying this object in an
        engine's ECS to operate on its impulses when collisions occur.
    */
    void *user_data;

    /*
        States
    */
    bool sleeping;

    uint16_t mask;

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

// TODO: damping
enum p2d_joint_type { // distance is a spring with coef 1
    P2D_JOINT_SPRING,
    P2D_JOINT_HINGE,
    // P2D_JOINT_WELD,
};

struct p2d_joint {
    struct p2d_object *a;

    bool anchored_to_world;
    union {
        struct p2d_object *b;   // anchored to another object
        vec2_t world_anchor_b;  // anchored to world
    };

    enum p2d_joint_type type;

    vec2_t local_anchor_a;
    vec2_t local_anchor_b;

    float bias_factor;

    bool disable_collisions;

    union {
        struct {
            float rest_length;
            float spring_constant;
        } spring_joint;
        // struct {
        // } hinge_joint;
    };
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
    Computes whether or not two objects are eligible collision canidates,
    from their bitmasks and other considerations (like constraints)

    TODO: this could use an optimized lookup cache or some other O(1) solution
    that doesnt involve querying joints
*/
P2D_API bool p2d_should_collide(struct p2d_object *a, struct p2d_object *b);

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
