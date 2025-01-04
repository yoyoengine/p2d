/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include "p2d/core.h"
#include "p2d/collide.h"
#include "p2d/detection.h"
#include "p2d/helpers.h"

/*
    Shape Collide Functions (not yet for contact generation)
*/

bool p2d_collide_circle_circle(struct p2d_object *a, struct p2d_object *b, struct p2d_collision_info *info) {
    if(!info || !a || !b) {
        p2d_logf(P2D_LOG_ERROR, "p2d_collide_circle_circle: invalid arguments.\n");
        return false;
    }
    info->depth = 0; info->normal = (struct p2d_vec2){0, 0};

    struct p2d_vec2 midline = { b->x - a->x, b->y - a->y };
    float mag = lla_vec2_magnitude(p2d_struct_to_vec(midline));

    if(mag <= 0 || mag >= a->circle.radius + b->circle.radius)
        return false;        

    info->normal = (struct p2d_vec2){ midline.x / mag, midline.y / mag };
    info->depth = a->circle.radius + b->circle.radius - mag;

    return true;
}

bool p2d_collide_rect_circle(struct p2d_object *rect, struct p2d_object *circle, struct p2d_collision_info *info) {
    if(!info || !rect || !circle) {
        p2d_logf(P2D_LOG_ERROR, "p2d_collide_rect_circle: invalid arguments.\n");
        return false;
    }
    info->depth = 0; info->normal = (struct p2d_vec2){0, 0};

    // might not be needed
    // https://github.com/twobitcoder101/FlatPhysics/blob/main/Collisions.cs#L355

    // struct p2d_vec2 axis = {0, 0};
    // float axis_depth = 0;
    // float min_a, max_a, min_b, max_b;

    // struct p2d_obb rect_obb = p2d_get_obb(rect);
    // struct p2d_obb_verts rect_verts = p2d_obb_to_verts(rect_obb);

    // for(int i = 0; i < 4; i++) {
    //     struct p2d_vec2 va = rect_verts.verts[i];
    //     struct p2d_vec2 vb = rect_verts.verts[(i + 1) % 4];

    //     struct p2d_vec2 edge = {vb.x - va.x, vb.y - va.y};
    //     axis = (struct p2d_vec2){-edge.y, edge.x};
    //     axis = p2d_vec2_normalize(axis);
    // }

    struct p2d_obb_verts verts = p2d_obb_to_verts(p2d_get_obb(rect));

    float min_dist = FLT_MAX;
    struct p2d_vec2 min_closest_point = {0};
    struct p2d_vec2 va = verts.verts[0];
    for(int i = 0; i < 4; i++) {
        struct p2d_vec2 vb = verts.verts[(i + 1) % 4];

        struct p2d_vec2 closest_point = {0};
        float dist = 0;
        p2d_closest_point_on_segment_to_point(va, vb, (struct p2d_vec2){circle->x, circle->y}, &closest_point, &dist);

        if(dist < min_dist) {
            min_dist = dist;
            min_closest_point = closest_point;
        }

        va = vb;
    }

    // early out
    if(min_dist > circle->circle.radius) {
        return false;
    }

    info->depth = circle->circle.radius - min_dist;

    struct p2d_vec2 delta = {circle->x - min_closest_point.x, circle->y - min_closest_point.y};
    float dist = lla_vec2_magnitude(p2d_struct_to_vec(delta));
    if(dist > 0) {
        info->normal = (struct p2d_vec2){delta.x / dist, delta.y / dist};
    }
    else {
        info->normal = (struct p2d_vec2){1, 0};
    }

    if(info->depth < 0) { return false; }

    return true;
}

bool p2d_collide_rect_rect(struct p2d_object *a, struct p2d_object *b, struct p2d_collision_info *info) {
    if(!info || !a || !b) {
        p2d_logf(P2D_LOG_ERROR, "p2d_collide_rect_rect: invalid arguments.\n");
        return false;
    }
    info->depth = FLT_MAX; info->normal = (struct p2d_vec2){0, 0};

    struct p2d_obb a_obb = p2d_get_obb(a);
    struct p2d_obb b_obb = p2d_get_obb(b);

    struct p2d_obb_verts rect1 = p2d_obb_to_verts(a_obb);
    struct p2d_obb_verts rect2 = p2d_obb_to_verts(b_obb);

    for(int polyi = 0; polyi < 2; polyi++) {
        struct p2d_obb_verts rect = polyi == 0 ? rect1 : rect2;

        for(int i1 = 0; i1 < 4; i1++) {
            int i2 = (i1 + 1) % 4;

            float normalx = -(rect.verts[i2].y - rect.verts[i1].y);
            float normaly = rect.verts[i2].x - rect.verts[i1].x;
            
            // Normalize the normal
            float len = sqrtf(normalx * normalx + normaly * normaly);
            normalx /= len;
            normaly /= len;

            float mina = FLT_MAX, maxa = -FLT_MAX;
            float minb = FLT_MAX, maxb = -FLT_MAX;

            // Project both shapes onto the normal
            for(int ai = 0; ai < 4; ai++) {
                float projected = (normalx * rect1.verts[ai].x) + (normaly * rect1.verts[ai].y);
                mina = fminf(mina, projected);
                maxa = fmaxf(maxa, projected);
            }

            for(int bi = 0; bi < 4; bi++) {
                float projected = (normalx * rect2.verts[bi].x) + (normaly * rect2.verts[bi].y);
                minb = fminf(minb, projected);
                maxb = fmaxf(maxb, projected);
            }

            if(maxa < minb || maxb < mina) {
                return false;
            }

            // Calculate penetration
            float overlap = fminf(maxa - minb, maxb - mina);
            
            if(overlap < info->depth) {
                info->depth = overlap;
                info->normal = (struct p2d_vec2){normalx, normaly};
                
                // Ensure normal points from A to B
                if((minb - mina) < 0) {
                    info->normal.x = -normalx;
                    info->normal.y = -normaly;
                }
            }
        }
    }

    return true;
}

/*
    Main Collision Detection Function
*/

bool p2d_collide(struct p2d_object *a, struct p2d_object *b, struct p2d_collision_info *info) {
    // safeguard
    if(!a || !b || !info) {
        p2d_logf(P2D_LOG_ERROR, "p2d_collide: invalid arguments.\n");
        return false;
    }
    info->depth = 0; info->normal = (struct p2d_vec2){0, 0};

    /*
        Circle Circle
    */
    if(a->type == P2D_OBJECT_CIRCLE && b->type == P2D_OBJECT_CIRCLE) {
        return p2d_collide_circle_circle(a, b, info);
    }

    /*
        Rect Circle
    */
    if(a->type == P2D_OBJECT_CIRCLE && b->type == P2D_OBJECT_RECTANGLE) {
        return p2d_collide_rect_circle(b, a, info);
    }
    if(a->type == P2D_OBJECT_RECTANGLE && b->type == P2D_OBJECT_CIRCLE) {
        return p2d_collide_rect_circle(a, b, info);
    }

    /*
        Rect Rect
    */
    if(a->type == P2D_OBJECT_RECTANGLE && b->type == P2D_OBJECT_RECTANGLE) {
        return p2d_collide_rect_rect(a, b, info);
    }

    return false;
}