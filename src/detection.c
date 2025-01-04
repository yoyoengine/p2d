/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <float.h>

#include "p2d/types.h"
#include "p2d/detection.h"

bool p2d_obb_intersects_obb(struct p2d_obb a, struct p2d_obb b) {
    struct p2d_obb_verts rect1 = p2d_obb_to_verts(a);
    struct p2d_obb_verts rect2 = p2d_obb_to_verts(b);

    for(int polyi = 0; polyi < 2; polyi++) {
        struct p2d_obb_verts rect = polyi == 0 ? rect1 : rect2;

        for(int i1 = 0; i1 < 4; i1++) {
            int i2 = (i1 + 1) % 4;

            float normalx = -(rect.verts[i2].y - rect.verts[i1].y);
            float normaly = rect.verts[i2].x - rect.verts[i1].x;

            float mina = FLT_MAX;
            float maxa = -FLT_MAX;
            for(int ai = 0; ai < 4; ai++) {
                float projected = (normalx * rect1.verts[ai].x) + (normaly * rect1.verts[ai].y);
                if(projected < mina) mina = projected;
                if(projected > maxa) maxa = projected;
            }

            float minb = FLT_MAX;
            float maxb = -FLT_MAX;
            for(int bi = 0; bi < 4; bi++) {
                float projected = (normalx * rect2.verts[bi].x) + (normaly * rect2.verts[bi].y);
                if(projected < minb) minb = projected;
                if(projected > maxb) maxb = projected;
            }

            if(maxa < minb || maxb < mina)
                return false;
        }
    }
    return true;
}

// TODO: p2d_obb_intersects_aabb (should reduce computation for classifying into broad phase tiles)

bool p2d_circle_intersects_aabb(struct p2d_circle circle, struct p2d_aabb aabb) {
    float closest_x = circle.x;
    float closest_y = circle.y;

    if(circle.x < aabb.x) {
        closest_x = aabb.x;
    } else if(circle.x > aabb.x + aabb.w) {
        closest_x = aabb.x + aabb.w;
    }

    if(circle.y < aabb.y) {
        closest_y = aabb.y;
    } else if(circle.y > aabb.y + aabb.h) {
        closest_y = aabb.y + aabb.h;
    }

    float distance = sqrt((circle.x - closest_x) * (circle.x - closest_x) + (circle.y - closest_y) * (circle.y - closest_y));

    return distance < circle.radius;
}