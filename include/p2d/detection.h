/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_DETECTION_H
#define P2D_DETECTION_H

#include <stdbool.h>

#include "p2d/export.h"

P2D_API bool p2d_obb_intersects_obb(struct p2d_obb a, struct p2d_obb b);

P2D_API bool p2d_obb_verts_intersects_obb_verts(struct p2d_obb_verts rect1, struct p2d_obb_verts rect2);

P2D_API bool p2d_circle_intersects_aabb(struct p2d_circle circle, struct p2d_aabb aabb);

#endif // P2D_DETECTION_H
