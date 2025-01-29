/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <math.h>
#include <string.h>

#include <Lilith.h>

#include "p2d/log.h"
#include "p2d/joint.h"
#include "p2d/helpers.h"

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

/*
    https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials/8constraintsandsolvers/Physics%20-%20Constraints%20and%20Solvers.pdf
*/
void _p2d_resolve_distance_joint(struct p2d_joint *joint, float deltatime) {
    struct p2d_object *a = joint->a;
    struct p2d_object *b = joint->b;

    printf("=========== distance joint ===========\n");
    printf("original velocity a: %f %f\n", a->vx, a->vy);
    printf("original velocity b: %f %f\n", b->vx, b->vy);

    vec2_t world_anchor_a = p2d_get_joint_world_anchor(joint->a, joint->local_anchor_a);
    vec2_t world_anchor_b = p2d_get_joint_world_anchor(joint->b, joint->local_anchor_b);

    printf("world anchor a: %f %f\n", world_anchor_a.x, world_anchor_a.y);
    printf("world anchor b: %f %f\n", world_anchor_b.x, world_anchor_b.y);

    float bias_factor = joint->bias_factor;
    float length = joint->distance.length;

    printf("bias factor: %f\n", bias_factor);

    vec2_t relative_position = lla_vec2_sub(world_anchor_a, world_anchor_b);
    float distance = lla_vec2_magnitude(relative_position);

    printf("relative position: %f %f\n", relative_position.x, relative_position.y);
    printf("distance: %f\n", distance);
    float offset = distance - length; // should be 0 when at correct length

    printf("offset: %f\n", offset);

    if(fabs(offset) > 0) {
        vec2_t relative_velocity = (vec2_t){{a->vx - b->vx, a->vy - b->vy}};

        printf("relative velocity: %f %f\n", relative_velocity.x, relative_velocity.y);

        vec2_t rel_pos_norm = lla_vec2_normalize(relative_position);
        rel_pos_norm.y = -rel_pos_norm.y;
        rel_pos_norm.x = -rel_pos_norm.x;
        // ^ aka offset direction

        printf("rel pos norm: %f %f\n", rel_pos_norm.x, rel_pos_norm.y);

        float constraint_mass = a->inv_mass + b->inv_mass;

        printf("constraint mass: %f\n", constraint_mass);

        if(constraint_mass > 0) {
            float velocity_dot = lla_vec2_dot(relative_velocity, rel_pos_norm);
            
            printf("velocity dot: %f\n", velocity_dot);

            float bias = -( bias_factor / deltatime ) * offset;
            // bias = 0;
            printf("bias: %f\n", bias);

            float lambda = -( velocity_dot + bias ) / constraint_mass;

            printf("lambda: %f\n", lambda);

            vec2_t a_impulse = lla_vec2_scale(rel_pos_norm, lambda);
            vec2_t b_impulse = lla_vec2_scale(lla_vec2_scale(rel_pos_norm,-1), lambda);
        
            printf("a impulse: %f %f\n", a_impulse.x, a_impulse.y);
            printf("b impulse: %f %f\n", b_impulse.x, b_impulse.y);

            a->vx += a_impulse.x * a->inv_mass;
            a->vy += a_impulse.y * a->inv_mass;
            b->vx += b_impulse.x * b->inv_mass;
            b->vy += b_impulse.y * b->inv_mass;
        }
    }

    printf("resolved velocity a: %f %f\n", a->vx, a->vy);
    printf("resolved velocity b: %f %f\n", b->vx, b->vy);

    printf("======================================\n");

}

void _p2d_resolve_joint(struct p2d_joint *joint, float delta_time) {

    if(!joint) {
        p2d_logf(P2D_LOG_ERROR, "_p2d_resolve_joint: Joint is NULL\n");
        return;
    }

    switch(joint->type) {
        case P2D_JOINT_DISTANCE:
            _p2d_resolve_distance_joint(joint, delta_time);
            break;
        default:
            p2d_logf(P2D_LOG_ERROR, "_p2d_resolve_joint: Unknown joint type\n");
            break;
    }
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
