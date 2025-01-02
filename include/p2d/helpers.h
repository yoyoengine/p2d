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

P2D_API vec2_t p2d_struct_to_vec(struct p2d_vec2 vec);

P2D_API struct p2d_vec2 p2d_vec_to_struct(vec2_t vec);

#endif // P2D_HELPERS_H