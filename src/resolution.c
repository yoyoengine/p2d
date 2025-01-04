/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include "p2d/resolution.h"
#include "p2d/contacts.h"
#include "p2d/helpers.h"

void p2d_object_step(struct p2d_object *object, float delta_time) {
    if(!object) {
        p2d_logf(P2D_LOG_ERROR, "p2d_object_step: object is NULL.\n");
        return;
    }

    if(object->is_static) {
        return;
    }

    // apply gravity
    object->vx += p2d_state.gravity.x * delta_time;
    object->vy += p2d_state.gravity.y * delta_time;

    // apply existing forces
    object->x += object->vx * delta_time;
    object->y += object->vy * delta_time;
    object->rotation += object->vr * delta_time;
    
}

void _p2d_resolve_collision_basic(struct p2d_collision_manifold *manifold) {
    // value is the same for both contact points, if they exist
    struct p2d_vec2 normal = manifold->normal;
    float depth = manifold->penetration;

    struct p2d_object *a = manifold->a;
    struct p2d_object *b = manifold->b;

    vec2_t relative_velocity = lla_vec2_sub((vec2_t){.data={a->vx, a->vy}}, (vec2_t){.data={b->vx, b->vy}});

    if(lla_vec2_dot(relative_velocity, p2d_struct_to_vec(normal)) > 0) {
        return;
    }

    float e = fminf(a->restitution, b->restitution);

    float j = -(1.0f + e) * lla_vec2_dot(relative_velocity, p2d_struct_to_vec(normal));
    j /= (1.0f/a->mass) + (1.0f/b->mass);

    a->vx += j * normal.x * (1.0f/a->mass);
    a->vy += j * normal.y * (1.0f/a->mass);

    b->vx -= j * normal.x * (1.0f/b->mass);
    b->vy -= j * normal.y * (1.0f/b->mass);
}

void p2d_resolve_collision(struct p2d_collision_manifold *manifold) {
    struct p2d_object *a = manifold->a;
    struct p2d_object *b = manifold->b;

    vec2_t relative_velocity = lla_vec2_sub((vec2_t){.data={b->vx, b->vy}}, (vec2_t){.data={a->vx, a->vy}});

    if(lla_vec2_dot(relative_velocity, p2d_struct_to_vec(manifold->normal)) > 0) {
        return;
    }

    float e = fminf(a->restitution, b->restitution);

    float j = -(1.0f + e) * lla_vec2_dot(relative_velocity, p2d_struct_to_vec(manifold->normal));
    j /= (1.0f/a->mass) + (1.0f/b->mass);

    a->vx -= j * manifold->normal.x * (1.0f/a->mass);
    a->vy -= j * manifold->normal.y * (1.0f/a->mass);

    b->vx += j * manifold->normal.x * (1.0f/b->mass);
    b->vy += j * manifold->normal.y * (1.0f/b->mass);
}