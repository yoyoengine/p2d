/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_HELPERS_H
#define P2D_HELPERS_H

#include <stdbool.h>

#include <Lilith.h>

#include "p2d/export.h"
#include "p2d/core.h"
#include "p2d/types.h"

P2D_API vec2_t p2d_object_center(struct p2d_object *object);

P2D_API struct p2d_aabb p2d_get_aabb(struct p2d_object *object);

P2D_API struct p2d_obb p2d_get_obb(struct p2d_object *object);

P2D_API bool p2d_aabbs_intersect(struct p2d_aabb a, struct p2d_aabb b);

P2D_API void p2d_closest_point_on_segment_to_point(vec2_t sega, vec2_t segb, vec2_t point, vec2_t *outPoint, float *outDist);

#define EPSILON 0.0005 // 1/2mm

P2D_API bool p2d_nearly_equal(float a, float b);

P2D_API bool p2d_vec2_nearly_equal(vec2_t a, vec2_t b);

// P2D_API bool p2d_point_in_obb(vec2_t point, struct p2d_obb obb);

P2D_API void p2d_project_obb_to_axis(struct p2d_obb_verts verts, vec2_t axis, float *min, float *max);

P2D_API void p2d_project_circle_to_axis(vec2_t center, float radius, vec2_t axis, float *min, float *max);

// returns obb verts index of closest vert
P2D_API int p2d_closest_circle_point_on_rect(vec2_t circle_center, struct p2d_obb_verts verts);

#endif // P2D_HELPERS_H
