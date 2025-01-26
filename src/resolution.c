/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include "p2d/log.h"
#include "p2d/helpers.h"
#include "p2d/contacts.h"
#include "p2d/resolution.h"

void _p2d_apply_air_resistance(struct p2d_object *object, float delta_time) {
    if(!object) {
        p2d_logf(P2D_LOG_ERROR, "_p2d_apply_air_resistance: object is NULL.\n");
        return;
    }
    
    vec2_t v_sq = (vec2_t){{object->vx * object->vx, object->vy * object->vy}};

    float drag_coefficient;
    float xc_area;
    switch(object->type) {
        case P2D_OBJECT_RECTANGLE:
            drag_coefficient = 2.05f;
            xc_area = object->rectangle.width_meters * object->rectangle.height_meters;
            break;
        case P2D_OBJECT_CIRCLE:
            drag_coefficient = 1.17f;
            xc_area = M_PI * object->circle.radius_meters * object->circle.radius_meters;
            break;
        default:
            p2d_logf(P2D_LOG_WARN, "_p2d_apply_air_resistance: object type not recognized.\n");
            return;
    }

    // TODO: take in
    float mass_scaling = p2d_state.p2d_mass_scaling;
    xc_area *= mass_scaling;

    float rx = 0.5f * p2d_state.p2d_air_density * drag_coefficient * xc_area * v_sq.x;
    float ry = 0.5f * p2d_state.p2d_air_density * drag_coefficient * xc_area * v_sq.y;

    // TODO: clever epsilon
    object->vx -= (rx * delta_time);
    object->vy -= (ry * delta_time);
}

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

    // apply air resistance
    _p2d_apply_air_resistance(object, delta_time);

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

/*
    TODO: optimize computation
*/
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

        float c = M_PI / 180.0f;

        // degrees -> radians
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

        float denom_right = rapdn_squared_times_iv + rbpdn_squared_times_iv;
        
        float added_inv_masses = obj_a->inv_mass + obj_b->inv_mass;
        vec2_t norm_scaled_mass = lla_vec2_scale(normal, added_inv_masses);
        float denom_left = lla_vec2_dot(normal, norm_scaled_mass);

        float denom = denom_left + denom_right;

        float numerator = -(1.0f + e) * contact_velocity_mag;

        float j = numerator / denom;
        j /= (float)contact_count;

        vec2_t impulse = lla_vec2_scale(normal, j);

        impulse_list[i] = impulse;
    }

    for(int i = 0; i < contact_count; i++) {
        vec2_t impulse = impulse_list[i];
        vec2_t ra = ra_list[i];
        vec2_t rb = rb_list[i];

        vec2_t a_lin_vel_delta = lla_vec2_scale(impulse, -1.0f);
        a_lin_vel_delta = lla_vec2_scale(a_lin_vel_delta, obj_a->inv_mass);
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
        a_ang_vel_delta *= obj_a->inv_inertia;

        // compiler will optimize
        float c = 180.0f / M_PI;

        // radians -> degrees
        a_ang_vel_delta *= c;
        obj_a->vr -= a_ang_vel_delta;

        vec2_t b_lin_vel_delta = lla_vec2_scale(impulse, obj_b->inv_mass);
        obj_b->vx += b_lin_vel_delta.x;
        obj_b->vy += b_lin_vel_delta.y;

        float b_ang_vel_delta = lla_vec2_cross(rb, impulse);
        b_ang_vel_delta *= obj_b->inv_inertia;

        // radians -> degrees
        b_ang_vel_delta *= c;
        obj_b->vr += b_ang_vel_delta;
    }
}

void _p2d_rotational_friction_resolution(struct p2d_collision_manifold *manifold) {
    struct p2d_object *obj_a = manifold->a;
    struct p2d_object *obj_b = manifold->b;
    vec2_t normal = manifold->normal;
    vec2_t contact1 = manifold->contact_points[0];
    vec2_t contact2 = manifold->contact_points[1];
    int contact_count = manifold->contact_count;

    float sf = (obj_a->static_friction + obj_b->static_friction) * 0.5f;
    float df = (obj_a->dynamic_friction + obj_b->dynamic_friction) * 0.5f;

    float e = fminf(obj_a->restitution, obj_b->restitution);

    vec2_t contact_list[2] = {contact1, contact2};
    vec2_t impulse_list[2] = {{{0,0}},{{0,0}}};
    float j_list[2] = {0,0};
    vec2_t friction_impulse_list[2] = {{{0,0}},{{0,0}}};
    vec2_t ra_list[2] = {{{0,0}},{{0,0}}};
    vec2_t rb_list[2] = {{{0,0}},{{0,0}}};


    /*
        TORQUE - Impulse Calc
    */
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

        // degrees -> radians
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

        float denom_right = rapdn_squared_times_iv + rbpdn_squared_times_iv;
        
        float added_inv_masses = obj_a->inv_mass + obj_b->inv_mass;
        vec2_t norm_scaled_mass = lla_vec2_scale(normal, added_inv_masses);
        float denom_left = lla_vec2_dot(normal, norm_scaled_mass);

        float denom = denom_left + denom_right;

        float numerator = -(1.0f + e) * contact_velocity_mag;

        float j = numerator / denom;
        j /= (float)contact_count;

        j_list[i] = j;

        vec2_t impulse = lla_vec2_scale(normal, j);

        impulse_list[i] = impulse;
    }

    /*
        TORQUE - Impulse Apply
    */
    for(int i = 0; i < contact_count; i++) {
        vec2_t impulse = impulse_list[i];
        vec2_t ra = ra_list[i];
        vec2_t rb = rb_list[i];

        vec2_t a_lin_vel_delta = lla_vec2_scale(impulse, -1.0f);
        a_lin_vel_delta = lla_vec2_scale(a_lin_vel_delta, obj_a->inv_mass);
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
        a_ang_vel_delta *= obj_a->inv_inertia;

        // compiler will optimize
        float c = 180.0f / M_PI;

        // radians -> degrees
        a_ang_vel_delta *= c;
        obj_a->vr -= a_ang_vel_delta;

        vec2_t b_lin_vel_delta = lla_vec2_scale(impulse, obj_b->inv_mass);
        obj_b->vx += b_lin_vel_delta.x;
        obj_b->vy += b_lin_vel_delta.y;

        float b_ang_vel_delta = lla_vec2_cross(rb, impulse);
        b_ang_vel_delta *= obj_b->inv_inertia;

        // radians -> degrees
        b_ang_vel_delta *= c;
        obj_b->vr += b_ang_vel_delta;
    }




    /*
        Friction Calc
    */
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

        // degrees -> radians
        float a_ang_radians = obj_a->vr * c;
        float b_ang_radians = obj_b->vr * c;

        vec2_t angular_linear_velocity_a = (vec2_t){.data={a_ang_radians * ra_perp.x, a_ang_radians * ra_perp.y}};
        vec2_t angular_linear_velocity_b = (vec2_t){.data={b_ang_radians * rb_perp.x, b_ang_radians * rb_perp.y}};

        vec2_t relative_velocity = lla_vec2_sub(
            lla_vec2_add((vec2_t){.data={obj_b->vx, obj_b->vy}}, angular_linear_velocity_b),
            lla_vec2_add((vec2_t){.data={obj_a->vx, obj_a->vy}}, angular_linear_velocity_a)
        );

        vec2_t tangent = lla_vec2_sub(relative_velocity, lla_vec2_scale(normal, lla_vec2_dot(relative_velocity, normal)));

        float j = j_list[i];

        if(p2d_vec2_nearly_equal(tangent, (vec2_t){{0,0}})) {
            continue;
        }
        else {
            tangent = lla_vec2_normalize(tangent);
        }

        float ra_perp_dot_t = lla_vec2_dot(ra_perp, tangent);
        float ra_perp_dot_t_squared = ra_perp_dot_t * ra_perp_dot_t;
        float rb_perp_dot_t = lla_vec2_dot(rb_perp, tangent);
        float rb_perp_dot_t_squared = rb_perp_dot_t * rb_perp_dot_t;

        float rapdt_squared_times_iv = ra_perp_dot_t_squared * obj_a->inv_inertia;
        float rbpdt_squared_times_iv = rb_perp_dot_t_squared * obj_b->inv_inertia;

        float denom_right = rapdt_squared_times_iv + rbpdt_squared_times_iv;
        
        float added_inv_masses = obj_a->inv_mass + obj_b->inv_mass;
        vec2_t norm_scaled_mass = lla_vec2_scale(tangent, added_inv_masses);
        float denom_left = lla_vec2_dot(tangent, norm_scaled_mass);

        float denom = denom_left + denom_right;

        float numerator = -lla_vec2_dot(relative_velocity, tangent);

        float jt = numerator / denom;

        // 2 bit difference: mult by im here
        float imp = jt * obj_a->inv_mass;
        jt /= (float)contact_count;

        vec2_t friction_impulse;

        if(fabs(jt) <= j * sf) {
            friction_impulse = lla_vec2_scale(tangent, imp);
        }
        else {
            friction_impulse = lla_vec2_scale(tangent, -j * df);
        }

        friction_impulse_list[i] = friction_impulse;
    }

    /*
        Friction Apply
    */
    for(int i = 0; i < contact_count; i++) {
        vec2_t friction_impulse = friction_impulse_list[i];
        vec2_t ra = ra_list[i];
        vec2_t rb = rb_list[i];

        vec2_t a_lin_vel_delta = lla_vec2_scale(friction_impulse, -1.0f);
        a_lin_vel_delta = lla_vec2_scale(a_lin_vel_delta, obj_a->inv_mass);
        obj_a->vx += a_lin_vel_delta.x;
        obj_a->vy += a_lin_vel_delta.y;

        /*
            One way of handling the screen to cartesian in the dot/cross

            The 2bit way is object_a_rot - impl, object_b_rot - impl
        */
        // vec2_t tmp_ra = (vec2_t){{ra.x, -ra.y}};
        // vec2_t tmp_rb = (vec2_t){{rb.x, -rb.y}};
        // vec2_t tmp_impulse = (vec2_t){{impulse.x, -impulse.y}};

        float a_ang_vel_delta = lla_vec2_cross(ra, friction_impulse);
        a_ang_vel_delta *= obj_a->inv_inertia;

        // compiler will optimize
        float c = 180.0f / M_PI;

        // radians -> degrees
        a_ang_vel_delta *= c;
        obj_a->vr -= a_ang_vel_delta;

        vec2_t b_lin_vel_delta = lla_vec2_scale(friction_impulse, obj_b->inv_mass);
        obj_b->vx += b_lin_vel_delta.x;
        obj_b->vy += b_lin_vel_delta.y;

        float b_ang_vel_delta = lla_vec2_cross(rb, friction_impulse);
        b_ang_vel_delta *= obj_b->inv_inertia;

        // radians -> degrees
        b_ang_vel_delta *= c;
        obj_b->vr += b_ang_vel_delta;
    }
}

void p2d_resolve_collision(struct p2d_collision_manifold *manifold) {
    // printf("velocity before resolution: %f, %f, %f\n", manifold->a->vx, manifold->a->vy, manifold->a->vr);

    // _p2d_basic_resolution(manifold);
    // _p2d_rotational_resolution(manifold);
    _p2d_rotational_friction_resolution(manifold);

    // printf("resolved velocity: %f, %f, %f\n", manifold->a->vx, manifold->a->vy, manifold->a->vr);
}
