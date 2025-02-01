/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "p2d/log.h"
#include "p2d/core.h"
#include "p2d/world.h"
#include "p2d/types.h"
#include "p2d/joint.h"
#include "p2d/pairs.h"
#include "p2d/collide.h"
#include "p2d/helpers.h"
#include "p2d/contacts.h"
#include "p2d/detection.h"
#include "p2d/resolution.h"

struct p2d_state p2d_state = {0};

//
// INIT
//

// gravity needs initialized by game, mass scaling also can be overridden

bool p2d_init(
    int cell_size,
    void (*on_collision)(struct p2d_cb_data *data),
    void (*on_trigger)(struct p2d_cb_data *data),
    void (*log_fn)(int level, const char *fmt, ...)
){
    // initialize external logger (if provided)
    p2d_state.log = log_fn;

    if(cell_size <= 0) {
        p2d_logf(P2D_LOG_ERROR, "p2d_init: cell_size must be greater than 0.\n");
        return false;
    }
    p2d_state.p2d_cell_size = cell_size;

    p2d_state.p2d_substeps = P2D_DEFAULT_SUBSTEPS;
    p2d_state.p2d_joint_iterations = P2D_DEFAULT_JOINT_SUBSTEPS;

    p2d_state.p2d_frustum_sleeping = false;

    p2d_state.p2d_mass_scaling = P2D_DEFAULT_MASS_SCALE;
    p2d_state.p2d_air_density = P2D_DEFAULT_AIR_DENSITY;

    if(!on_collision) {
        p2d_logf(P2D_LOG_WARN, "p2d_init: on_collision is NULL.\n");
    }
    p2d_state.on_collision = on_collision;

    if(!on_trigger) {
        p2d_logf(P2D_LOG_WARN, "p2d_init: on_trigger is NULL.\n");
    }
    p2d_state.on_trigger = on_trigger;

    p2d_state.p2d_object_count = 0;
    p2d_state.p2d_world_node_count = 0;

    p2d_pairs_init();

    p2d_logf(P2D_LOG_INFO, "p2d initialized with cell size: %d.\n", cell_size);

    return true;
}



//
// TILE INTERSECTION DETECTION/RESOLUTION
//

bool _object_intersects_tile(struct p2d_object *object, struct p2d_aabb tile) {
    if (object->type == P2D_OBJECT_RECTANGLE) {
        struct p2d_obb obb = p2d_get_obb(object);
        
        // TODO: replace when we can check AABB against OBB
        struct p2d_obb tile_obb = {
            .x = tile.x,
            .y = tile.y,
            .w = tile.w,
            .h = tile.h,
            .r = 0
        };

        return p2d_obb_intersects_obb(tile_obb, obb);
    }
    else { // P2D_OBJECT_CIRCLE
        struct p2d_circle circle = {
            .x = object->x,
            .y = object->y,
            .radius = object->circle.radius
        };
        return p2d_circle_intersects_aabb(circle, tile);
    }
}

// runs callback for each tile the object intersects with
void p2d_for_each_intersecting_tile(struct p2d_object *object, void (*callback)(struct p2d_object *object, int tile_hash)) {
    if (!object) {
        p2d_logf(P2D_LOG_ERROR, "p2d_for_each_intersecting_tile: object is NULL.\n");
        return;
    }

    struct p2d_aabb aabb = p2d_get_aabb(object);
    
    int start_tile_x = (int)(aabb.x / p2d_state.p2d_cell_size);
    int start_tile_y = (int)(aabb.y / p2d_state.p2d_cell_size);
    int end_tile_x = (int)((aabb.x + aabb.w) / p2d_state.p2d_cell_size);
    int end_tile_y = (int)((aabb.y + aabb.h) / p2d_state.p2d_cell_size);

    for (int tile_x = start_tile_x; tile_x <= end_tile_x; tile_x++) {
        for (int tile_y = start_tile_y; tile_y <= end_tile_y; tile_y++) {
            struct p2d_aabb tile = {
                .x = (float)(tile_x * p2d_state.p2d_cell_size),
                .y = (float)(tile_y * p2d_state.p2d_cell_size),
                .w = (float)p2d_state.p2d_cell_size,
                .h = (float)p2d_state.p2d_cell_size
            };

            if (_object_intersects_tile(object, tile)) {
                int hash = p2d_world_hash(tile_x, tile_y);
                callback(object, hash);
            }
        }
    }
}

void _register_intersecting_tiles(struct p2d_object *object, int hash) {
    p2d_world_insert(hash, object);
}

void _unregister_intersecting_tiles(struct p2d_object *object, int hash) {
    p2d_world_remove(hash, object);
}



//
// OBJECT MANAGEMENT
//

bool p2d_create_object(struct p2d_object *object) {
    if(!object) {
        p2d_logf(P2D_LOG_ERROR, "p2d_create_object: object is NULL.\n");
        return false;
    }

    // Detect all world tiles the object intersects with, and add it to each
    // p2d_for_each_intersecting_tile(object, _register_intersecting_tiles);
    // ^^^ NO! this happens implicitely each frame

    // insert into track array
    for(int i = 0; i < P2D_MAX_OBJECTS; i++) {
        if(p2d_objects[i] == NULL) {
            p2d_objects[i] = object;
            break;
        }
    }

    object->mass = 0.0f;
    object->inertia = 0.0f;

    if(object->density < P2D_MIN_DENSITY || object->density > P2D_MAX_DENSITY) {
        p2d_logf(P2D_LOG_WARN, "p2d_create_object: object density is out of range.\n");
    }

    /*
        UNUSED (for now?)
    */
    #ifndef P2D_PIXELS_PER_METER
        #define P2D_PIXELS_PER_METER 1.0f
    #endif

    switch(object->type) {
        case P2D_OBJECT_RECTANGLE:
            object->rectangle.width_meters = object->rectangle.width / P2D_PIXELS_PER_METER;
            object->rectangle.height_meters = object->rectangle.height / P2D_PIXELS_PER_METER;

            object->area = object->rectangle.width_meters * object->rectangle.height_meters;
            break;
        case P2D_OBJECT_CIRCLE:
            object->circle.radius_meters = object->circle.radius / P2D_PIXELS_PER_METER;

            object->area = M_PI * object->circle.radius_meters * object->circle.radius_meters;
            break;
    }

    float mass_scaling = p2d_state.p2d_mass_scaling; // TODO: this breaks when changing mid sim- kinda on the user atp

    if(!object->is_static) {
        // compute mass and inertia from density and size
        if(object->type == P2D_OBJECT_RECTANGLE) {
            object->mass = object->density * object->area;
            object->mass *= mass_scaling;
            object->inertia = (1.0f / 12.0f) * object->mass * ((object->rectangle.width_meters * object->rectangle.width_meters) + (object->rectangle.height_meters * object->rectangle.height_meters));
        }
        else if(object->type == P2D_OBJECT_CIRCLE) {
            object->mass = object->density * object->area;
            object->mass *= mass_scaling;
            object->inertia = (1.0f / 2.0f) * object->mass * object->circle.radius_meters * object->circle.radius_meters;
        }
    }

    object->inv_mass = (object->mass > 0.0f) ? 1.0f / object->mass : 0.0f;
    object->inv_inertia = (object->inertia > 0.0f) ? 1.0f / object->inertia : 0.0f;

    p2d_state.p2d_object_count++;
    return true;
}

bool p2d_remove_object(struct p2d_object *object) {
    if(!object) {
        p2d_logf(P2D_LOG_ERROR, "p2d_remove_object: object is NULL.\n");
        return false;
    }

    // Detect all world tiles the object intersects with, and remove it from each
    // p2d_for_each_intersecting_tile(object, _unregister_intersecting_tiles);
    // ^^^ NO! this happens implicitely each frame

    // remove from track array
    for(int i = 0; i < P2D_MAX_OBJECTS; i++) {
        if(p2d_objects[i] == object) {
            p2d_objects[i] = NULL;
            break;
        }
    }

    p2d_state.p2d_object_count--;
    return true;
}

bool p2d_remove_all_objects(void) {
    p2d_world_remove_all();
    return true;
}

bool p2d_shutdown(void) {
    p2d_remove_all_objects();
    p2d_reset_collision_pairs();
    p2d_logf(P2D_LOG_INFO, "p2d shutdown.\n");
    return true;
}



/*
    In 2D, we can only have two possible contacts.

    This struct is really just for ease of use (and avoiding traversing a list
    in the resolution for contacts)
*/
struct p2d_collision_manifold p2d_generate_manifold(struct p2d_object *a, struct p2d_object *b, vec2_t normal, float penetration, struct p2d_contact_list *contacts) {
    struct p2d_collision_manifold manifold = {0};
    manifold.a = a;
    manifold.b = b;

    manifold.normal = normal;
    manifold.penetration = penetration;

    manifold.contact_points[0] = contacts->contacts[0].contact_point;
    manifold.contact_points[1] = contacts->contacts[1].contact_point;
    manifold.contact_count = (int)contacts->count;

    return manifold;
}

void p2d_separate_bodies(struct p2d_object *a, struct p2d_object *b, vec2_t normal, float depth) {
    vec2_t mtv = {.x = normal.x * depth, .y = normal.y * depth};

    if(a->is_static) {
        b->x += mtv.x;
        b->y += mtv.y;

        // out
        if(b->out_x)
            *b->out_x += mtv.x;
        if(b->out_y)
            *b->out_y += mtv.y;
    }
    else if(b->is_static) {
        a->x += -mtv.x;
        a->y += -mtv.y;

        // out
        if(a->out_x)
            *a->out_x += -mtv.x;
        if(a->out_y)
            *a->out_y += -mtv.y;
    }
    else {
        a->x += (-mtv.x / 2.0f);
        a->y += (-mtv.y / 2.0f);
        b->x += (mtv.x / 2.0f);
        b->y += (mtv.y / 2.0f);

        // out
        if(a->out_x)
            *a->out_x += (-mtv.x / 2.0f);
        if(a->out_y)
            *a->out_y += (-mtv.y / 2.0f);
        if(b->out_x)
            *b->out_x += (mtv.x / 2.0f);
        if(b->out_y)
            *b->out_y += (mtv.y / 2.0f);
    }
}

bool p2d_should_collide(struct p2d_object *a, struct p2d_object *b) {
    if(a->is_trigger && b->is_trigger) {
        return false;
    }

    if(a->is_static && b->is_static) {
        return false;
    }

    // TODO: layer masks

    /*
        Find any joints between these two objects, to check if they have their
        collisions disabled. We also take the liberty to force this for hinges
        
        TODO: early out by counting joints seen vs registered
    */
    for(int i = 0; i < P2D_MAX_JOINTS; i++) {
        struct p2d_joint *joint = p2d_joints[i];
        if(joint == NULL) {
            continue;
        }

        if(joint->anchored_to_world) {
            continue;
        }

        if((joint->a == a && joint->b == b) || (joint->a == b && joint->b == a)) {
            if(joint->disable_collisions) {
                return false;
            }

            if(joint->type == P2D_JOINT_HINGE) {
                return false;
            }
        }
    }

    return true;
}

// struct p2d_contact_list * p2d_step(float delta_time) {
void p2d_step(float delta_time) {
    // struct p2d_contact_list *every_contact = p2d_contact_list_create(25);

    if(delta_time <= 0.0f) {
        p2d_logf(P2D_LOG_ERROR, "p2d_step: delta_time must be greater than 0.\n");
        return;
    }

    if(p2d_state.out_contacts) {
        p2d_contact_list_clear(p2d_state.out_contacts);
    }

    // substepping
    for(int it_track = 0; it_track < p2d_state.p2d_substeps; it_track++) {

    /*
        Step each object in the world
    */
    for(int i = 0; i < P2D_MAX_OBJECTS; i++) {
        struct p2d_object *object = p2d_objects[i];
        if(object == NULL) {
            continue;
        }

        if(!object->sleeping)
            p2d_object_step(object, delta_time, p2d_state.p2d_substeps);
    }

    /*
        Reset and re-register world state for broad phase collision detection
    */
    p2d_rebuild_world();

    // reset collision pairs
    p2d_reset_collision_pairs();

    /*
        For each bucket containing objects, generate contacts
        with all other objects in the bucket (excluding self)
    */
    p2d_state.p2d_contact_checks = 0;
    p2d_state.p2d_contacts_found = 0;
    for(int i = 0; i < P2D_MAX_OBJECTS; i++) {
        struct p2d_world_node *head_node = p2d_world[i];
        if(head_node == NULL) {
            continue;
        }

        struct p2d_world_node *node_a = head_node;

        while(node_a) {
            struct p2d_world_node *node_b = node_a->next;

            while(node_b) {
                p2d_state.p2d_contact_checks++;
                if(node_a != node_b) {
                    struct p2d_object *a = node_a->object;
                    struct p2d_object *b = node_b->object;

                    /*
                        last check - might be expensive (profile)
                        we want to see if they are even eligible to collide,
                        taking a collision mask and some other meta like if they revolute each other into account
                    */
                    if(!p2d_should_collide(a, b)) {
                        node_b = node_b->next;
                        continue;
                    }

                    // if already collided, skip (multi grid node collision)
                    if(p2d_collision_pair_exists(a, b)) {
                        node_b = node_b->next;
                        continue;
                    }

                    struct p2d_collision_info d = {0};
                    if(p2d_collide(a, b, &d)) {
                        p2d_add_collision_pair(a, b);

                        /*
                            If one is a trigger, no need to seperate or solve

                            TODO: could also include normal and depth, or collider collidee info
                        */
                        if(a->is_trigger || b->is_trigger) {
                            if(p2d_state.on_trigger) {
                                struct p2d_cb_data data = {
                                    .a = a,
                                    .b = b
                                };
                                p2d_state.on_trigger(&data);
                                break;
                            }
                        }

                        // get all contacts
                        struct p2d_contact_list *contacts = p2d_generate_contacts(a, b);

                        // seperate after contacts - i think 2bit had some weird deferred movement
                        p2d_separate_bodies(a, b, d.normal, d.depth);

                        // early out
                        if(!contacts || contacts->count <= 0) {
                            node_b = node_b->next;
                            continue;
                        }
                        p2d_state.p2d_contacts_found += (int)contacts->count;

                        // debug: add all contacts to the global list
                        if(p2d_state.out_contacts) {
                            for(size_t z = 0; z < contacts->count; z++) {
                                p2d_contact_list_add(p2d_state.out_contacts, contacts->contacts[z]);
                            }
                        }

                        // create contact manifold for resolution
                        struct p2d_collision_manifold manifold =
                            p2d_generate_manifold(a, b, d.normal, d.depth, contacts);

                        // now, resolve their collision
                        p2d_resolve_collision(&manifold);

                        // cleanup
                        p2d_contact_list_destroy(contacts);

                        // inform the subscriber of the collision
                        if(p2d_state.on_collision) {
                            struct p2d_cb_data data = {
                                .a = a,
                                .b = b
                            };
                            p2d_state.on_collision(&data);
                        }
                    }
                }

                node_b = node_b->next;
            }

            node_a = node_a->next;
        }
    }

    } // substepping

    // resolve joints
    for(int i = 0; i < p2d_state.p2d_joint_iterations; i++)
        p2d_resolve_joints(delta_time, p2d_state.p2d_substeps);

}
