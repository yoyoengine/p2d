/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

/*
    Seperate from contacts, collide step gives normal and depth,
    then we can move apart and see the contact points
*/

#ifndef P2D_COLLIDE_H
#define P2D_COLLIDE_H

#include <float.h>
#include <stdbool.h>

#include "p2d/export.h"
#include "p2d/core.h"

// shape collisions (norm+depth) //

struct p2d_collision_info {
    struct p2d_vec2 normal;
    float depth;
};

P2D_API bool p2d_collide_circle_circle(struct p2d_object *a, struct p2d_object *b, struct p2d_collision_info *info);
P2D_API bool p2d_collide_rect_rect(struct p2d_object *a, struct p2d_object *b, struct p2d_collision_info *info); 
P2D_API bool p2d_collide_rect_circle(struct p2d_object *a, struct p2d_object *b, struct p2d_collision_info *info);

// main collision //

P2D_API bool p2d_collide(struct p2d_object *a, struct p2d_object *b, struct p2d_collision_info *info);

#endif // P2D_COLLIDE_H