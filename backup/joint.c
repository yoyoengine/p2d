/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <math.h>
#include <string.h>

#include <Lilith.h>

#include "p2d/joint.h"

struct p2d_joint * p2d_joints[P2D_MAX_JOINTS] = {NULL};

void p2d_add_joint(struct p2d_joint *joint) {
    for(int i = 0; i < P2D_MAX_JOINTS; i++) {
        if(p2d_joints[i] == NULL) {
            p2d_joints[i] = joint;
            return;
        }
    }
}

void p2d_remove_joint(struct p2d_joint *joint) {
    for(int i = 0; i < P2D_MAX_JOINTS; i++) {
        if(p2d_joints[i] == joint) {
            p2d_joints[i] = NULL;
            return;
        }
    }
}

void p2d_remove_all_joints(void) {
    memset(p2d_joints, 0, sizeof(p2d_joints));
}

vec2_t p2d_get_joint_world_anchor(struct p2d_object *object, vec2_t local_anchor) {
    vec2_t world_anchor = local_anchor;

    // Create rotation matrix and apply rotation
    mat3_t rot_m = lla_mat3_identity();
    rot_m = lla_mat3_rotate(rot_m, object->rotation);
    world_anchor = lla_mat3_mult_vec2(rot_m, world_anchor);

    if(object->type == P2D_OBJECT_RECTANGLE) {
        world_anchor = lla_vec2_add(world_anchor, (vec2_t){{object->rectangle.width / 2, object->rectangle.height / 2}});
    }

    // Add object's position to get final world coordinates
    world_anchor = (vec2_t){{object->x + world_anchor.x, object->y + world_anchor.y}};

    return world_anchor;
}

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

vec2_t _scalar_cross(float s, vec2_t v) {
	return (vec2_t){{-s * v.y, s * v.x}};
}

/*
    https://github.com/erincatto/box2d-lite/blob/master/src/Joint.cpp
*/
void _p2d_resolve_joint(struct p2d_joint *joint, float delta_time) {
    struct p2d_object *a = joint->a;
    struct p2d_object *b = joint->b;

    vec2_t world_anchor_a = p2d_get_joint_world_anchor(joint->a, joint->local_anchor_a);
    vec2_t world_anchor_b = p2d_get_joint_world_anchor(joint->b, joint->local_anchor_b);

    float bias_factor = joint->bias_factor;
    float softness = joint->softness;

    // vec2_t distance = lla_vec2_sub(world_anchor_b, world_anchor_a);
    // vec2_t relative_velocity = (vec2_t){{b.vx - a.vx, b.vy - a.vy}};
    // float C = sqrtf((distance.x * distance.x) + (distance.y * distance.y)) - joint->distance;

    // float ra_rads = a.rotation * DEG_TO_RAD;
    // float rb_rads = b.rotation * DEG_TO_RAD;

    // printf("a->rotation: %f\n", a->rotation);
    // printf("b->rotation: %f\n", b->rotation);

    mat3_t a_rot = lla_mat3_identity();
    a_rot = lla_mat3_rotate(a_rot, a->rotation); // degrees

    mat3_t b_rot = lla_mat3_identity();
    b_rot = lla_mat3_rotate(b_rot, b->rotation); // degrees

    // // visualize rotation
    // const char * ac = lla_mat3_string(a_rot);
    // printf("a rot: %s\n", ac);
    // free((void*)ac);

    // const char * bc = lla_mat3_string(b_rot);
    // printf("b rot: %s\n", bc);
    // free((void*)bc);

    // // print out anchors
    // const char * aac = lla_vec2_string(joint->local_anchor_a);
    // printf("a anchor: %s\n", aac);
    // free((void*)aac);

    // const char * bac = lla_vec2_string(joint->local_anchor_b);
    // printf("b anchor: %s\n", bac);
    // free((void*)bac);

    vec2_t r1 = lla_mat3_mult_vec2(a_rot, joint->local_anchor_a);
    vec2_t r2 = lla_mat3_mult_vec2(b_rot, joint->local_anchor_b);

    // const char * r1c = lla_vec2_string(r1);
    // printf("r1: %s\n", r1c);
    // free((void*)r1c);

    // const char * r2c = lla_vec2_string(r2);
    // printf("r2: %s\n", r2c);
    // free((void*)r2c);

    // deltaV = deltaV0 + K * impulse
	// invM = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]

    // mat3_t test = lla_mat3_identity();
    // test.data[0][0] = 10;
    // const char * testc = lla_mat3_string(test);
    // printf("test: %s\n", testc);
    // free((void*)testc);

    // printf("a inv mass: %f\n", a->inv_mass);
    // printf("b inv mass: %f\n", b->inv_mass);

    mat3_t k1 = lla_mat3_identity();
    k1.data[0][0] = a->inv_mass + b->inv_mass;
    k1.data[1][1] = a->inv_mass + b->inv_mass;

    // const char * k1c = lla_mat3_string(k1);
    // printf("k1: %s\n", k1c);
    // free((void*)k1c);

    mat3_t k2 = lla_mat3_identity();
    k2.data[0][0] = a->inv_inertia * r1.y * r1.y;    k2.data[0][1] = a->inv_inertia * -r1.x * r1.y;
    k2.data[1][0] = a->inv_inertia * -r1.x * r1.y;   k2.data[1][1] = a->inv_inertia * r1.x * r1.x;

    // const char * k2c = lla_mat3_string(k2);
    // printf("k2: %s\n", k2c);
    // free((void*)k2c);

    mat3_t k3 = lla_mat3_identity();
    k3.data[0][0] = b->inv_inertia * r2.y * r2.y;    k3.data[0][1] = b->inv_inertia * -r2.x * r2.y;
    k3.data[1][0] = b->inv_inertia * -r2.x * r2.y;    k3.data[1][1] = b->inv_inertia * r2.x * r2.x;

    // const char * k3c = lla_mat3_string(k3);
    // printf("k3: %s\n", k3c);
    // free((void*)k3c);

    mat3_t k = lla_mat3_add(lla_mat3_add(k1, k2), k3);
    k.data[0][0] += softness;
    k.data[1][1] += softness;

    // const char * kc = lla_mat3_string(k);
    // printf("k: %s\n", kc);
    // free((void*)kc);

    mat3_t M = lla_mat3_inverse(k);

    // const char * c = lla_mat3_string(M);
    // printf("M: %s\n", c);
    // free((void*)c);

    // Compute position error (bias)
    vec2_t dp = lla_vec2_sub(world_anchor_b, world_anchor_a);

    // -biasFactor * inv_dt * dp;
    vec2_t bias;
    if(p2d_state.p2d_position_correction) {
        bias = lla_vec2_scale(dp, -bias_factor * (1.0f / delta_time));
    }
    else {
        bias = (vec2_t){{0.0f, 0.0f}};
    }

    // TODO: radian or degree?
    vec2_t bvr_r2 = _scalar_cross(b->vr, r2);
    vec2_t avr_r1 = _scalar_cross(a->vr, r1);

    vec2_t dv = {{
        b->vx + bvr_r2.x - a->vx - avr_r1.x,
        b->vy + bvr_r2.y - a->vy - avr_r1.y
    }};
    // float dvx = b.vx + _scalar_cross(b.vr, r2).x - a.vx - _scalar_cross(a.vr, r1).x;
    // float dvy = b.vy + _scalar_cross(b.vr, r2).x - a.vy - _scalar_cross(a.vr, r1).y;

    // cartesian to screen?

    vec2_t impulse;

    vec2_t softness_vec = (vec2_t){{softness, softness}};

    impulse = lla_mat3_mult_vec2(M, lla_vec2_sub(lla_vec2_sub(bias, dv), softness_vec));

    // TODO: radian or degree?
    a->vx -= a->inv_mass * impulse.x;
    a->vy -= a->inv_mass * -impulse.y;
    a->vr -= a->inv_inertia * (r1.x * impulse.y - r1.y * impulse.x);

    // TODO: radian or degree?
    b->vx += b->inv_mass * impulse.x;
    b->vy += b->inv_mass * -impulse.y;
    b->vr += b->inv_inertia * (r2.x * impulse.y - r2.y * impulse.x);

    // old:

    // vec2_t b_vel = (vec2_t){{b.vx, b.vy}};
    // vec2_t a_vel = (vec2_t){{a.vx, a.vy}};

    // // Compute relative velocity (including angular velocity contribution)
    // vec2_t dv = lla_vec2_sub(
    //     lla_vec2_add(b_vel, (vec2_t){{(-b.vr *DEG_TO_RAD) * r2.y, (b.vr * DEG_TO_RAD) * r2.x}}), // Cross product inlined
    //     lla_vec2_add(a_vel, (vec2_t){{(-a.vr *DEG_TO_RAD) * r1.y, (a.vr * DEG_TO_RAD) * r1.x}})  // Cross product inlined
    // );

    // // Compute impulse
    // vec2_t impulse = lla_mat3_mult_vec2(M, lla_vec2_sub(bias, dv));

    // printf("impulse: %f, %f\n", impulse.x, impulse.y);

    // if(a.is_static) {
    //     b.vx += b.inv_mass * impulse.x;
    //     b.vy += b.inv_mass * impulse.y;
    //     b.vr += b.inv_inertia * (r2.x * impulse.y - r2.y * impulse.x); // Inlined cross product
    // }
    // else if(b.is_static) {
    //     a.vx -= a.inv_mass * impulse.x;
    //     a.vy -= a.inv_mass * impulse.y;
    //     a.vr -= a.inv_inertia * (r1.x * impulse.y - r1.y * impulse.x); // Inlined cross product
    // }
    // else {
    //     a.vx -= a.inv_mass * impulse.x;
    //     a.vy -= a.inv_mass * impulse.y;
    //     a.vr -= a.inv_inertia * (r1.x * impulse.y - r1.y * impulse.x); // Inlined cross product

    //     b.vx += b.inv_mass * impulse.x;
    //     b.vy += b.inv_mass * impulse.y;
    //     b.vr += b.inv_inertia * (r2.x * impulse.y - r2.y * impulse.x); // Inlined cross product
    // }

    // // Store updated velocities
    // joint->a->vx = a.vx;
    // joint->a->vy = a.vy;
    // joint->a->vr = a.vr * RAD_TO_DEG;

    // joint->b->vx = b.vx;
    // joint->b->vy = b.vy;
    // joint->b->vr = b.vr * RAD_TO_DEG;
}

void p2d_resolve_joints(float delta_time, int substeps) {

    float dt = delta_time / substeps;

    for(int i = 0; i < P2D_MAX_JOINTS; i++) {
        struct p2d_joint *joint = p2d_joints[i];
        if(joint != NULL) {
            _p2d_resolve_joint(joint, dt);
        }
    }
}
