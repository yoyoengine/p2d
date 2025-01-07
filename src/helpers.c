
/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <float.h>

#include "p2d/core.h"
#include "p2d/helpers.h"
#include "p2d/types.h"

vec2_t p2d_object_center(struct p2d_object *object) {
    vec2_t center = {0};

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

void p2d_closest_point_on_segment_to_point(vec2_t sega, vec2_t segb, vec2_t point, vec2_t *outPoint, float *outDist) {
    vec2_t ab = {segb.x - sega.x, segb.y - sega.y};
    vec2_t ap = {point.x - sega.x, point.y - sega.y};

    float ab_dot_ap = ab.x * ap.x + ab.y * ap.y;
    float ab_len_sq = ab.x * ab.x + ab.y * ab.y;
    
    float t = ab_dot_ap / ab_len_sq;
    
    vec2_t closest;
    if(t < 0) {
        closest = sega;
    }
    else if(t > 1) {
        closest = segb;
    }
    else {
        closest = (vec2_t){
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

bool p2d_vec2_nearly_equal(vec2_t a, vec2_t b) {
    return p2d_nearly_equal(a.x, b.x) && p2d_nearly_equal(a.y, b.y);
}

// bool p2d_point_in_obb(vec2_t point, struct p2d_obb obb) {
//     // Transform point to OBB local space
//     vec2_t local = {
//         .x = point.x - obb.x,
//         .y = point.y - obb.y
//     };

//     // Rotate point back to align with OBB axes
//     float angle = -obb.r;
//     float s = sinf(angle);
//     float c = cosf(angle);

//     vec2_t rotated = {
//         .x = c * local.x - s * local.y,
//         .y = s * local.x + c * local.y
//     };

//     // Check if point is within half-width/height from center
//     return rotated.x >= -obb.w/2 && rotated.x <= obb.w/2 && rotated.y >= -obb.h/2 && rotated.y <= obb.h/2;
// }

bool p2d_aabbs_intersect(struct p2d_aabb a, struct p2d_aabb b) {
    return a.x < b.x + b.w && a.x + a.w > b.x && a.y < b.y + b.h && a.y + a.h > b.y;
}

void p2d_project_obb_to_axis(struct p2d_obb_verts verts, vec2_t axis, float *min, float *max) {
    *min = FLT_MAX;
    *max = FLT_MIN;

    for(int i = 0; i < 4; i++) {
        vec2_t v = {verts.verts[i].x, verts.verts[i].y};
        float proj = lla_vec2_dot(v, axis);

        if(proj < *min) { *min = proj; }
        if(proj > *max) { *max = proj; }
    }
}

void p2d_project_circle_to_axis(vec2_t center, float radius, vec2_t axis, float *min, float *max) {
    vec2_t direction = lla_vec2_normalize(axis);
    vec2_t direction_and_radius = lla_vec2_scale(direction, radius);

    vec2_t p1 = lla_vec2_add(center, direction_and_radius);
    vec2_t p2 = lla_vec2_sub(center, direction_and_radius);

    *min = lla_vec2_dot(p1, axis);
    *max = lla_vec2_dot(p2, axis);

    if(*min > *max) {
        float temp = *min;
        *min = *max;
        *max = temp;
    }
}

int p2d_closest_circle_point_on_rect(vec2_t circle_center, struct p2d_obb_verts verts) {
    int result = -1;
    float min_dist = FLT_MAX;

    for(int i = 0; i < 4; i++) {
        vec2_t v = verts.verts[i];
        float distance = sqrtf((v.x - circle_center.x) * (v.x - circle_center.x) + (v.y - circle_center.y) * (v.y - circle_center.y));

        if(distance < min_dist) {
            min_dist = distance;
            result = i;
        }
    }

    return result;
}