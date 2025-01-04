/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_TYPES_H
#define P2D_TYPES_H

#include "p2d/export.h"

/*
    OBB with assumed center at w/2, h/2
*/
struct p2d_obb {
    float x;
    float y;
    float w;
    float h;
    float r;
};

struct p2d_circle {
    float x;
    float y;
    float radius;
};

/*
    TODO: just use Lilith vec2?
    I dont rlly like indexing .data[i] compared to .x, .y
*/
struct p2d_vec2 {
    float x;
    float y;
};

struct p2d_aabb {
    float x;
    float y;
    float w;
    float h;
};

struct p2d_obb_verts {
    struct p2d_vec2 verts[4];
};

P2D_API struct p2d_obb_verts p2d_obb_to_verts(struct p2d_obb obb);

P2D_API struct p2d_aabb p2d_obb_to_aabb(struct p2d_obb obb);

#endif // P2D_TYPES_H