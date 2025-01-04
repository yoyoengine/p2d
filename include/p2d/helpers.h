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

P2D_API struct p2d_vec2 p2d_object_center(struct p2d_object *object);

P2D_API struct p2d_aabb p2d_get_aabb(struct p2d_object *object);

P2D_API struct p2d_obb p2d_get_obb(struct p2d_object *object);

P2D_API bool p2d_aabbs_intersect(struct p2d_aabb a, struct p2d_aabb b);

P2D_API vec2_t p2d_struct_to_vec(struct p2d_vec2 vec);

P2D_API struct p2d_vec2 p2d_vec_to_struct(vec2_t vec);

P2D_API void p2d_closest_point_on_segment_to_point(struct p2d_vec2 sega, struct p2d_vec2 segb, struct p2d_vec2 point, struct p2d_vec2 *outPoint, float *outDist);

#define EPSILON 0.0005 // 1/2mm

P2D_API bool p2d_nearly_equal(float a, float b);

P2D_API bool p2d_vec2_nearly_equal(struct p2d_vec2 a, struct p2d_vec2 b);

// note, just rewrite LLA while youre at it...

P2D_API struct p2d_vec2 p2d_vec2_normalize(struct p2d_vec2 vec);

// P2D_API bool p2d_point_in_obb(struct p2d_vec2 point, struct p2d_obb obb);

#endif // P2D_HELPERS_H