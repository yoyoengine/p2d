/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_JOINT_H
#define P2D_JOINT_H

#include "p2d/core.h"
#include "p2d/export.h"

extern struct p2d_joint * p2d_joints[P2D_MAX_JOINTS];

/*
    yoyoengine stores joints inside the physics component in a linked list?
*/

P2D_API void p2d_add_joint(struct p2d_joint *joint);

P2D_API void p2d_remove_joint(struct p2d_joint *joint);

P2D_API void p2d_remove_all_joints(void);

P2D_API vec2_t p2d_get_joint_world_anchor(struct p2d_object *object, vec2_t local_anchor);

P2D_API void p2d_resolve_joints(float delta_time, int substeps);

#endif // P2D_JOINT_H
