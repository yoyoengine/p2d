
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

void p2d_closest_point_on_segment_to_point(struct p2d_vec2 sega, struct p2d_vec2 segb, struct p2d_vec2 point, struct p2d_vec2 *outPoint, float *outDist) {
    struct p2d_vec2 ab = {segb.x - sega.x, segb.y - sega.y};
    struct p2d_vec2 ap = {point.x - sega.x, point.y - sega.y};

    float ab_dot_ap = ab.x * ap.x + ab.y * ap.y;
    float ab_len_sq = ab.x * ab.x + ab.y * ab.y;
    
    float t = ab_dot_ap / ab_len_sq;
    
    struct p2d_vec2 closest;
    if(t < 0) {
        closest = sega;
    }
    else if(t > 1) {
        closest = segb;
    }
    else {
        closest = (struct p2d_vec2){
            .x = sega.x + ab.x * t,
            .y = sega.y + ab.y * t
        };
    }

    if(outPoint) {
        *outPoint = closest;
    }

    if(outDist) {
        // Calculate actual distance to the closest point
        float dx = point.x - closest.x;
        float dy = point.y - closest.y;
        *outDist = sqrtf(dx * dx + dy * dy);
    }
}

bool p2d_nearly_equal(float a, float b) {
    return fabs(a - b) < EPSILON;
}

bool p2d_vec2_nearly_equal(struct p2d_vec2 a, struct p2d_vec2 b) {
    return p2d_nearly_equal(a.x, b.x) && p2d_nearly_equal(a.y, b.y);
}

// bool p2d_point_in_obb(struct p2d_vec2 point, struct p2d_obb obb) {
//     // Transform point to OBB local space
//     struct p2d_vec2 local = {
//         .x = point.x - obb.x,
//         .y = point.y - obb.y
//     };

//     // Rotate point back to align with OBB axes
//     float angle = -obb.r;
//     float s = sinf(angle);
//     float c = cosf(angle);

//     struct p2d_vec2 rotated = {
//         .x = c * local.x - s * local.y,
//         .y = s * local.x + c * local.y
//     };

//     // Check if point is within half-width/height from center
//     return rotated.x >= -obb.w/2 && rotated.x <= obb.w/2 && rotated.y >= -obb.h/2 && rotated.y <= obb.h/2;
// }

bool p2d_aabbs_intersect(struct p2d_aabb a, struct p2d_aabb b) {
    return a.x < b.x + b.w && a.x + a.w > b.x && a.y < b.y + b.h && a.y + a.h > b.y;
}

struct p2d_vec2 p2d_vec2_normalize(struct p2d_vec2 vec) {
    float mag = sqrtf(vec.x * vec.x + vec.y * vec.y);
    return (struct p2d_vec2){vec.x / mag, vec.y / mag};
}