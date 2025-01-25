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
    object->vx += p2d_state.p2d_gravity.x * delta_time;
    object->vy += p2d_state.p2d_gravity.y * delta_time;

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
    j /= (a->inv_mass) + (b->inv_mass);

    a->vx -= (j * manifold->normal.x) * (a->inv_mass);
    a->vy -= (j * manifold->normal.y) * (a->inv_mass);

    b->vx += (j * manifold->normal.x) * (b->inv_mass);
    b->vy += (j * manifold->normal.y) * (b->inv_mass);

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

    // print a load of details from this resolution
    printf("normal: %f, %f\n", normal.x, normal.y);
    printf("contact count: %d\n", contact_count);
    printf("mass a: %f inv: %f\n", obj_a->mass, obj_a->inv_mass);
    printf("mass b: %f inv: %f\n", obj_b->mass, obj_b->inv_mass);
    printf("inertia a: %f inv: %f\n", obj_a->inertia, obj_a->inv_inertia);
    printf("inertia b: %f inv: %f\n", obj_b->inertia, obj_b->inv_inertia);

    for(int i = 0; i < contact_count; i++) {
        vec2_t cent_a = p2d_object_center(obj_a);
        vec2_t cent_b = p2d_object_center(obj_b);
        vec2_t ra = lla_vec2_sub(contact_list[i], cent_a);
        vec2_t rb = lla_vec2_sub(contact_list[i], cent_b);

        ra_list[i] = ra;
        rb_list[i] = rb;

        vec2_t ra_perp = (vec2_t){.data={-ra.y, ra.x}};
        vec2_t rb_perp = (vec2_t){.data={-rb.y, rb.x}};

        float c = M_PI / 180.0f;

        // radians -> degrees
        float a_ang_radians = obj_a->vr * c;
        float b_ang_radians = obj_b->vr * c;

        vec2_t angular_linear_velocity_a = (vec2_t){.data={a_ang_radians * ra_perp.x, a_ang_radians * ra_perp.y}};
        vec2_t angular_linear_velocity_b = (vec2_t){.data={b_ang_radians * rb_perp.x, b_ang_radians * rb_perp.y}};

        vec2_t relative_velocity = lla_vec2_sub(
            lla_vec2_add((vec2_t){.data={obj_b->vx, obj_b->vy}}, angular_linear_velocity_b),
            lla_vec2_add((vec2_t){.data={obj_a->vx, obj_a->vy}}, angular_linear_velocity_a)
        );

        float contact_velocity_mag = lla_vec2_dot(relative_velocity, normal);

        if(contact_velocity_mag > 0.0f) {
            continue;
        }

        float ra_perp_dot_n = lla_vec2_dot(ra_perp, normal);
        float ra_perp_dot_n_squared = ra_perp_dot_n * ra_perp_dot_n;
        float rb_perp_dot_n = lla_vec2_dot(rb_perp, normal);
        float rb_perp_dot_n_squared = rb_perp_dot_n * rb_perp_dot_n;

        float rapdn_squared_times_iv = ra_perp_dot_n_squared * obj_a->inv_inertia;
        float rbpdn_squared_times_iv = rb_perp_dot_n_squared * obj_b->inv_inertia;

        printf("rapdn_squared_times_iv: %f\n", rapdn_squared_times_iv);
        printf("rbpdn_squared_times_iv: %f\n", rbpdn_squared_times_iv);

        float denom_right = rapdn_squared_times_iv + rbpdn_squared_times_iv;
        
        float added_inv_masses = obj_a->inv_mass + obj_b->inv_mass;
        vec2_t norm_scaled_mass = lla_vec2_scale(normal, added_inv_masses);
        float denom_left = lla_vec2_dot(normal, norm_scaled_mass);

        float denom = denom_left + denom_right;

        float numerator = -(1.0f + e) * contact_velocity_mag;

        printf("denom left: %f\n", denom_left);
        printf("denom right: %f\n", denom_right);
        printf("denom: %f\n", denom);

        printf("numerator: %f\n", numerator);

        float j = numerator / denom;

        float imp = j * obj_a->inv_mass;
        j /= (float)contact_count;

        vec2_t impulse = lla_vec2_scale(normal, imp);

        impulse_list[i] = impulse;

        // no angular, yet

        printf("contact %d:\n", i);
        printf("   ra: %f, %f\n", ra.x, ra.y);
        printf("   rb: %f, %f\n", rb.x, rb.y);
        printf("   ra_perp: %f, %f\n", ra_perp.x, ra_perp.y);
        printf("   rb_perp: %f, %f\n", rb_perp.x, rb_perp.y);
        printf("   angular_linear_velocity_a: %f, %f\n", angular_linear_velocity_a.x, angular_linear_velocity_a.y);
        printf("   angular_linear_velocity_b: %f, %f\n", angular_linear_velocity_b.x, angular_linear_velocity_b.y);
        printf("   relative_velocity: %f, %f\n", relative_velocity.x, relative_velocity.y);
        printf("   contact_velocity_mag: %f\n", contact_velocity_mag);
        printf("   ra_perp_dot_n: %f\n", ra_perp_dot_n);
        printf("   rb_perp_dot_n: %f\n", rb_perp_dot_n);
        printf("   denom: %f\n", denom);
        printf("   j: %f\n", j);
        printf("   impulse: %f, %f\n", impulse.x, impulse.y);
    }

    for(int i = 0; i < contact_count; i++) {
        vec2_t impulse = impulse_list[i];
        vec2_t ra = ra_list[i];
        vec2_t rb = rb_list[i];

        printf("applying impulse %d\n", i);

        vec2_t a_lin_vel_delta = lla_vec2_scale(impulse, -1.0f);
        obj_a->vx += a_lin_vel_delta.x;
        obj_a->vy += a_lin_vel_delta.y;

        /*
            One way of handling the screen to cartesian in the dot/cross

            The 2bit way is object_a_rot - impl, object_b_rot - impl
        */
        // vec2_t tmp_ra = (vec2_t){{ra.x, -ra.y}};
        // vec2_t tmp_rb = (vec2_t){{rb.x, -rb.y}};
        // vec2_t tmp_impulse = (vec2_t){{impulse.x, -impulse.y}};

        float a_ang_vel_delta = lla_vec2_cross(ra, impulse);
        printf("a_ang_vel_delta cross: %f\n", a_ang_vel_delta);
        a_ang_vel_delta *= obj_a->inv_inertia;

        // compiler will optimize
        float c = 180.0f / M_PI;

        // radians -> degrees
        a_ang_vel_delta *= c;

        obj_a->vr -= a_ang_vel_delta;

        printf("a_ang_vel_delta scaled by ii: %f\n", a_ang_vel_delta);

        vec2_t b_lin_vel_delta = lla_vec2_scale(impulse, obj_b->inv_mass);
        obj_b->vx += b_lin_vel_delta.x;
        obj_b->vy += b_lin_vel_delta.y;

        float b_ang_vel_delta = lla_vec2_cross(rb, impulse);
        printf("b_ang_vel_delta cross: %f\n", b_ang_vel_delta);
        b_ang_vel_delta *= obj_b->inv_inertia;

        // radians -> degrees
        b_ang_vel_delta *= c;

        obj_b->vr += b_ang_vel_delta;

        printf("b_ang_vel_delta scaled by ii: %f\n", b_ang_vel_delta);
    }
}

void p2d_resolve_collision(struct p2d_collision_manifold *manifold) {
    printf("velocity before resolution: %f, %f, %f\n", manifold->a->vx, manifold->a->vy, manifold->a->vr);

    // _p2d_basic_resolution(manifold);
    _p2d_rotational_resolution(manifold);

    printf("resolved velocity: %f, %f, %f\n", manifold->a->vx, manifold->a->vy, manifold->a->vr);
}
