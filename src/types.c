/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <Lilith.h>

#include "p2d/types.h"

struct p2d_obb_verts p2d_obb_to_verts(struct p2d_obb obb) {
    struct p2d_obb_verts out = {0};

    // top left
    out.verts[0].x = obb.x;
    out.verts[0].y = obb.y;

    // top right
    out.verts[1].x = obb.x + obb.w;
    out.verts[1].y = obb.y;

    // bottom right
    out.verts[2].x = obb.x + obb.w;
    out.verts[2].y = obb.y + obb.h;

    // bottom left
    out.verts[3].x = obb.x;
    out.verts[3].y = obb.y + obb.h;

    /*
        Rotate around center
    */
    vec2_t center = {obb.x + obb.w / 2, obb.y + obb.h / 2};
    mat3_t rot = lla_mat3_rotate_around(lla_mat3_identity(), center, obb.r);

    for(int i = 0; i < 4; i++) {
        vec2_t vert = {out.verts[i].x, out.verts[i].y};
        vert = lla_mat3_mult_vec2(rot, vert);
        out.verts[i].x = vert.data[0];
        out.verts[i].y = vert.data[1];
    }

    return out;
}

struct p2d_aabb p2d_obb_to_aabb(struct p2d_obb obb) {
    struct p2d_obb_verts verts = p2d_obb_to_verts(obb);
    struct p2d_aabb aabb = {0};

    // Initialize min/max with first vertex
    float min_x = verts.verts[0].x;
    float max_x = verts.verts[0].x;
    float min_y = verts.verts[0].y;
    float max_y = verts.verts[0].y;

    // Find min/max coordinates
    for(int i = 1; i < 4; i++) {
        if(verts.verts[i].x < min_x) min_x = verts.verts[i].x;
        if(verts.verts[i].x > max_x) max_x = verts.verts[i].x;
        if(verts.verts[i].y < min_y) min_y = verts.verts[i].y;
        if(verts.verts[i].y > max_y) max_y = verts.verts[i].y;
    }

    // Set AABB properties
    aabb.x = min_x;
    aabb.y = min_y;
    aabb.w = max_x - min_x;
    aabb.h = max_y - min_y;

    return aabb;
}