
/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include "p2d/core.h"
#include "p2d/helpers.h"
#include "p2d/types.h"

struct p2d_vec2 p2d_object_center(struct p2d_object *object) {
    struct p2d_vec2 center = {0};

    switch(object->type) {
        case P2D_OBJECT_RECTANGLE:
            center.x = object->x + object->rectangle.width / 2;
            center.y = object->y + object->rectangle.height / 2;
            break;
        case P2D_OBJECT_CIRCLE:
            center.x = object->x;
            center.y = object->y;
            break;
    }

    return center;
}

struct p2d_aabb p2d_get_aabb(struct p2d_object *object) {
    if(!object) {
        p2d_logf(P2D_LOG_ERROR, "p2d_get_aabb: object is NULL.\n");
        return (struct p2d_aabb){0};
    }

    if (object->type == P2D_OBJECT_RECTANGLE) {
        struct p2d_obb obb = {
            .x = object->x,
            .y = object->y,
            .w = object->rectangle.width,
            .h = object->rectangle.height,
            .r = object->rotation
        };
        return p2d_obb_to_aabb(obb);
    }
    else { // P2D_OBJECT_CIRCLE
        return (struct p2d_aabb){
            .x = object->x - object->circle.radius,
            .y = object->y - object->circle.radius,
            .w = object->circle.radius * 2,
            .h = object->circle.radius * 2
        };
    }
}

struct p2d_obb p2d_get_obb(struct p2d_object *object) {
    if(!object) {
        p2d_logf(P2D_LOG_ERROR, "p2d_get_obb: object is NULL.\n");
        return (struct p2d_obb){0};
    }

    if (object->type == P2D_OBJECT_RECTANGLE) {
        return (struct p2d_obb){
            .x = object->x,
            .y = object->y,
            .w = object->rectangle.width,
            .h = object->rectangle.height,
            .r = object->rotation
        };
    }
    else { // P2D_OBJECT_CIRCLE
        return (struct p2d_obb){
            .x = object->x,
            .y = object->y,
            .w = object->circle.radius * 2,
            .h = object->circle.radius * 2,
            .r = 0
        };
    }
}

vec2_t p2d_struct_to_vec(struct p2d_vec2 vec) {
    return (vec2_t){vec.x, vec.y};
}

struct p2d_vec2 p2d_vec_to_struct(vec2_t vec) {
    return (struct p2d_vec2){vec.data[0], vec.data[1]};
}