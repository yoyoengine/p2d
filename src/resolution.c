/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include "p2d/log.h"
#include "p2d/helpers.h"
#include "p2d/contacts.h"
#include "p2d/resolution.h"

void p2d_object_step(struct p2d_object *object, float delta_time, int iterations) {
    if(!object) {
        p2d_logf(P2D_LOG_ERROR, "p2d_object_step: object is NULL.\n");
        return;
    }

    if(object->is_static) {
        object->vx = 0;
        object->vy = 0;
        return;
    }

    // account for substepping
    delta_time /= (float)iterations;

    // apply gravity
    object->vx += p2d_state.gravity.x * delta_time;
    object->vy += p2d_state.gravity.y * delta_time;

    // linear velocity
    // object->vx += object->force_x * delta_time;
    // object->vy += object->force_y * delta_time;
    // object->vr += object->rotation_force * delta_time;

    // update position
    object->x += object->vx * delta_time;
    object->y += object->vy * delta_time;
    object->rotation += object->vr * delta_time;

    // delta updates (engine syncing)
    if(object->out_x)
        *object->out_x += object->vx * delta_time;
    if(object->out_y)
        *object->out_y += object->vy * delta_time;
    if(object->out_rotation)
        *object->out_rotation += object->vr * delta_time;

    // debug - print objects new velocities
    // printf("object %p: vx: %f, vy: %f, vr: %f\n", (void *)object, object->vx, object->vy, object->vr);

    // reset forces
    // object->force_x = 0;
    // object->force_y = 0;
    // object->rotation_force = 0;
}

void _p2d_basic_resolution(struct p2d_collision_manifold *manifold) {
    struct p2d_object *a = manifold->a;
    struct p2d_object *b = manifold->b;

    vec2_t relative_velocity = lla_vec2_sub((vec2_t){.data={b->vx, b->vy}}, (vec2_t){.data={a->vx, a->vy}});

    if(lla_vec2_dot(relative_velocity, manifold->normal) > 0) {
        return;
    }

    float e = fminf(a->restitution, b->restitution);

    float j = -(1.0f + e) * lla_vec2_dot(relative_velocity, manifold->normal);
    j /= (p2d_inv_mass(a)) + (p2d_inv_mass(b));

    a->vx -= (j * manifold->normal.x) * (p2d_inv_mass(a));
    a->vy -= (j * manifold->normal.y) * (p2d_inv_mass(a));

    b->vx += (j * manifold->normal.x) * (p2d_inv_mass(b));
    b->vy += (j * manifold->normal.y) * (p2d_inv_mass(b));

    // a->vx -= j / a->mass * manifold->normal.x;
    // a->vy -= j / a->mass * manifold->normal.y;

    // b->vx += j / b->mass * manifold->normal.x;
    // b->vy += j / b->mass * manifold->normal.y;
}

void _p2d_rotational_resolution(struct p2d_collision_manifold *manifold) {
    struct p2d_object *obj_a = manifold->a;
    struct p2d_object *obj_b = manifold->b;
    vec2_t normal = manifold->normal;
    vec2_t contact1 = manifold->contact_points[0];
    vec2_t contact2 = manifold->contact_points[1];
    int contact_count = manifold->contact_count;

    float e = fminf(obj_a->restitution, obj_b->restitution);

    vec2_t contact_list[2] = {contact1, contact2};
    vec2_t impulse_list[2] = {{{0,0}},{{0,0}}};
    vec2_t ra_list[2] = {{{0,0}},{{0,0}}};
    vec2_t rb_list[2] = {{{0,0}},{{0,0}}};

    for(int i = 0; i < contact_count; i++) {
        vec2_t cent_a = p2d_object_center(obj_a);
        vec2_t cent_b = p2d_object_center(obj_b);
        vec2_t ra = lla_vec2_sub(contact_list[i], cent_a);
        vec2_t rb = lla_vec2_sub(contact_list[i], cent_b);

        ra_list[i] = ra;
        rb_list[i] = rb;

        vec2_t ra_perp = (vec2_t){.data={-ra.y, ra.x}};
        vec2_t rb_perp = (vec2_t){.data={-rb.y, rb.x}};

        vec2_t angular_linear_velocity_a = (vec2_t){.data={obj_a->vr * ra_perp.x, obj_a->vr * ra_perp.y}};
        vec2_t angular_linear_velocity_b = (vec2_t){.data={obj_b->vr * rb_perp.x, obj_b->vr * rb_perp.y}};

        vec2_t relative_velocity = lla_vec2_sub(
            lla_vec2_add((vec2_t){.data={obj_b->vx, obj_b->vy}}, angular_linear_velocity_b),
            lla_vec2_add((vec2_t){.data={obj_a->vx, obj_a->vy}}, angular_linear_velocity_a)
        );

        float contact_velocity_mag = lla_vec2_dot(relative_velocity, normal);

        if(contact_velocity_mag > 0.0f) {
            continue;
        }

        float ra_perp_dot_n = lla_vec2_dot(ra_perp, normal);
        float rb_perp_dot_n = lla_vec2_dot(rb_perp, normal);

        float denom = p2d_inv_mass(obj_a) + p2d_inv_mass(obj_b) +
        (ra_perp_dot_n * ra_perp_dot_n) * p2d_inv_inertia(obj_a) +
        (rb_perp_dot_n * rb_perp_dot_n) * p2d_inv_inertia(obj_b);

        float j = -(1.0f + e) * contact_velocity_mag;
        j /= denom;
        j /= (float)contact_count;

        vec2_t impulse = lla_vec2_scale(normal, j);
        impulse_list[i] = impulse;
    }

    for(int i = 0; i < contact_count; i++) {
        vec2_t impulse = impulse_list[i];
        vec2_t ra = ra_list[i];
        vec2_t rb = rb_list[i];


        vec2_t a_lin_vel_delta = lla_vec2_scale(lla_vec2_scale(impulse, -1.0f), p2d_inv_mass(obj_a));
        // printf("impules : %f, %f\n", impulse.x, impulse.y);
        // printf("a_lin_vel_delta: %f, %f\n", a_lin_vel_delta.x, a_lin_vel_delta.y);
        obj_a->vx += a_lin_vel_delta.x;
        obj_a->vy += a_lin_vel_delta.y;

        float a_ang_vel_delta = -1.0f * lla_vec2_cross(ra, impulse) * p2d_inv_inertia(obj_a);
        obj_a->vr += a_ang_vel_delta;

        vec2_t b_lin_vel_delta = lla_vec2_scale(impulse, p2d_inv_mass(obj_b));
        obj_b->vx += b_lin_vel_delta.x;
        obj_b->vy += b_lin_vel_delta.y;

        float b_ang_vel_delta = lla_vec2_cross(rb, impulse) * p2d_inv_inertia(obj_b);
        obj_b->vr += b_ang_vel_delta;
    }

    // printf("impulse: %f, %f\n", impulse_list[0].x, impulse_list[0].y);
    // printf("impulse: %f, %f\n", impulse_list[1].x, impulse_list[1].y);
    // printf("ra: %f, %f\n", ra_list[0].x, ra_list[0].y);
    // printf("ra: %f, %f\n", ra_list[1].x, ra_list[1].y);
    // printf("rb: %f, %f\n", rb_list[0].x, rb_list[0].y);
    // printf("rb: %f, %f\n", rb_list[1].x, rb_list[1].y);
    // printf("normal: %f, %f\n", normal.x, normal.y);
    // printf("contact1: %f, %f\n", contact1.x, contact1.y);
    // printf("contact2: %f, %f\n", contact2.x, contact2.y);
    // printf("contact_count: %d\n", contact_count);
}

void p2d_resolve_collision(struct p2d_collision_manifold *manifold) {
    // _p2d_basic_resolution(manifold);
    _p2d_rotational_resolution(manifold);
}
