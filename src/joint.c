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
    credit:
        https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials/8constraintsandsolvers/Physics%20-%20Constraints%20and%20Solvers.pdf
    and
        Chapter 6: Game Physics Engine Development (2nd edition) - Ian Millington
*/
void _p2d_resolve_spring_joint(struct p2d_joint *joint, float deltatime) {
    struct p2d_object *a = joint->a;
    struct p2d_object *b = NULL;

    if(!joint->anchored_to_world)
        b = joint->b;
    
    vec2_t world_anchor_a = p2d_get_joint_world_anchor(joint->a, joint->local_anchor_a);

    vec2_t world_anchor_b;
    if(joint->anchored_to_world)
        world_anchor_b = joint->world_anchor_b;
    else
        world_anchor_b = p2d_get_joint_world_anchor(joint->b, joint->local_anchor_b);

    float bias_factor = joint->bias_factor;
    float rest_length = joint->spring_joint.rest_length;
    float spring_constant = joint->spring_joint.spring_constant;
    
    vec2_t relative_position = lla_vec2_sub(world_anchor_a, world_anchor_b);
    float distance = lla_vec2_magnitude(relative_position);

    float offset = distance - rest_length; // should be 0 when at correct length

    if(fabs(offset) > 0) {

        vec2_t relative_velocity;
        if(joint->anchored_to_world) {
            relative_velocity = (vec2_t){{a->vx, a->vy}};
        } else {
            relative_velocity = (vec2_t){{a->vx - b->vx, a->vy - b->vy}};
        }

        vec2_t rel_pos_norm = lla_vec2_normalize(relative_position);
        rel_pos_norm.y = -rel_pos_norm.y;
        rel_pos_norm.x = -rel_pos_norm.x;
        // ^ aka offset direction

        float constraint_mass = a->inv_mass;
        if(!joint->anchored_to_world)
            constraint_mass += b->inv_mass;

        if(constraint_mass > 0) {
            float velocity_dot = lla_vec2_dot(relative_velocity, rel_pos_norm);
            float bias = -( bias_factor / deltatime ) * offset * spring_constant; // TODO: should the constant be included here in bias?
            float lambda = -( velocity_dot + bias ) / constraint_mass;

            vec2_t a_impulse = lla_vec2_scale(rel_pos_norm, lambda);
            vec2_t b_impulse = lla_vec2_scale(lla_vec2_scale(rel_pos_norm,-1), lambda);
        
            a->vx += a_impulse.x * a->inv_mass;
            a->vy += a_impulse.y * a->inv_mass;
            if(!joint->anchored_to_world && !b->is_static) {
                b->vx += b_impulse.x * b->inv_mass;
                b->vy += b_impulse.y * b->inv_mass;
            }
        }
    }
}

/*
    This is a stupid, completely made up hinge joint.

    It just pins the overlap points together, and relies on the other part of the solver
    to get angular impulse. The problem with that is the other solver has no concept of rotation about this hinge point,
    and instead calculates it about the center, meaning we have a problem for this.abs

    To be honest, I'm totally done with this engine right now, so I'm leaving this for future me to fix.abs
    If youre me reading this in the future: <https://tenor.com/view/xqc-middle-finger-gif-13191308>
*/
void _p2d_resolve_hinge_joint(struct p2d_joint *joint, float deltatime) {
    struct p2d_object *a = joint->a;
    struct p2d_object *b = NULL;
    bool anchored_to_world = joint->anchored_to_world;

    if(!anchored_to_world)
        b = joint->b;
    
    vec2_t world_anchor_a = p2d_get_joint_world_anchor(joint->a, joint->local_anchor_a);

    vec2_t world_anchor_b;
    if(joint->anchored_to_world)
        world_anchor_b = joint->world_anchor_b;
    else
        world_anchor_b = p2d_get_joint_world_anchor(joint->b, joint->local_anchor_b);
    
    vec2_t anchor_diff = lla_vec2_sub(world_anchor_a, world_anchor_b);
    if(a->is_static && !anchored_to_world) {
        b->x += anchor_diff.x;
        b->y += anchor_diff.y;
    } else if(anchored_to_world) {
        a->x -= anchor_diff.x;
        a->y -= anchor_diff.y;
    } else {
        a->x -= anchor_diff.x / 2;
        a->y -= anchor_diff.y / 2;
        if(!anchored_to_world) {
            b->x += anchor_diff.x / 2;
            b->y += anchor_diff.y / 2;
        }
    }

    a->vx = 0;
    a->vy = 0;
    if(b) {
        b->vx = 0;
        b->vy = 0;
    }

    (void)deltatime;
}

void _p2d_resolve_joint(struct p2d_joint *joint, float delta_time) {

    if(!joint) {
        p2d_logf(P2D_LOG_ERROR, "_p2d_resolve_joint: Joint is NULL\n");
        return;
    }

    switch(joint->type) {
        case P2D_JOINT_SPRING:
            _p2d_resolve_spring_joint(joint, delta_time);
            break;
        case P2D_JOINT_HINGE:
            _p2d_resolve_hinge_joint(joint, delta_time);
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
