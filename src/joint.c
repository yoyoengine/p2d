/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

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

void _p2d_resolve_joint(struct p2d_joint *joint) {
    struct p2d_object a = *joint->a;
    struct p2d_object b = *joint->b;

    vec2_t world_anchor_a = p2d_get_joint_world_anchor(joint->a, joint->local_anchor_a);
    vec2_t world_anchor_b = p2d_get_joint_world_anchor(joint->b, joint->local_anchor_b);

    float bias_factor = joint->bias_factor;
    float softness = joint->softness;

    // TODO: impl
}

void p2d_resolve_joints(float delta_time, int substeps) {
    (void)delta_time;
    (void)substeps;
    for(int i = 0; i < P2D_MAX_JOINTS; i++) {
        struct p2d_joint *joint = p2d_joints[i];
        if(joint != NULL) {
            _p2d_resolve_joint(joint);
        }
    }
}
