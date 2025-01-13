/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include "p2d/log.h"
#include "p2d/core.h"
#include "p2d/collide.h"
#include "p2d/helpers.h"
#include "p2d/detection.h"

/*
    Shape Collide Functions (not yet for contact generation)
*/

bool p2d_collide_circle_circle(struct p2d_object *a, struct p2d_object *b, struct p2d_collision_info *info) {
    if(!info || !a || !b) {
        p2d_logf(P2D_LOG_ERROR, "p2d_collide_circle_circle: invalid arguments.\n");
        return false;
    }
    info->depth = 0; info->normal = (vec2_t){{0, 0}};

    vec2_t midline = {{ b->x - a->x, b->y - a->y }};
    float mag = lla_vec2_magnitude(midline);

    if(mag <= 0 || mag >= a->circle.radius + b->circle.radius)
        return false;        

    info->normal = (vec2_t){{ midline.x / mag, midline.y / mag }};
    info->depth = a->circle.radius + b->circle.radius - mag;

    return true;
}

bool p2d_collide_rect_circle(struct p2d_object *rect, struct p2d_object *circle, struct p2d_collision_info *info) {
    if(!info || !rect || !circle) {
        p2d_logf(P2D_LOG_ERROR, "p2d_collide_rect_circle: invalid arguments.\n");
        return false;
    }
    info->depth = FLT_MAX; info->normal = (vec2_t){{0, 0}};

    vec2_t axis = {{0, 0}};
    float axis_depth = 0;
    float min_a, max_a, min_b, max_b;

    struct p2d_obb rect_obb = p2d_get_obb(rect);
    struct p2d_obb_verts rect_verts = p2d_obb_to_verts(rect_obb);

    for(int i = 0; i < 4; i++) {
        vec2_t va = rect_verts.verts[i];
        vec2_t vb = rect_verts.verts[(i + 1) % 4];

        vec2_t edge = {{vb.x - va.x, vb.y - va.y}};
        axis = (vec2_t){{-edge.y, edge.x}};
        axis = lla_vec2_normalize(axis);

        p2d_project_obb_to_axis(rect_verts, axis, &min_a, &max_a);

        vec2_t circ_center = {{circle->x, circle->y}};
        p2d_project_circle_to_axis(circ_center, circle->circle.radius, axis, &min_b, &max_b);

        if(min_a >= max_b || max_a <= min_b) {
            return false;
        }

        axis_depth = fminf(max_a - min_b, max_b - min_a);

        if(axis_depth < info->depth) {
            info->depth = axis_depth;
            info->normal = axis;
        }
    }

    int cp_index = p2d_closest_circle_point_on_rect((vec2_t){{circle->x, circle->y}}, rect_verts);
    vec2_t cp = rect_verts.verts[cp_index];

    axis = (vec2_t){{cp.x - circle->x, cp.y - circle->y}};
    axis = lla_vec2_normalize(axis);

    p2d_project_obb_to_axis(rect_verts, axis, &min_a, &max_a);
    p2d_project_circle_to_axis((vec2_t){{circle->x, circle->y}}, circle->circle.radius, axis, &min_b, &max_b);

    if(min_a >= max_b || max_a <= min_b) {
        return false;
    }

    axis_depth = fminf(max_a - min_b, max_b - min_a);

    if(axis_depth < info->depth) {
        info->depth = axis_depth;
        info->normal = axis;
    }

    vec2_t rect_center = p2d_object_center(rect);

    vec2_t direction = {{rect_center.x - circle->x, rect_center.y - circle->y}};

    // WARNING WARNING: DIFFERENT FROM 2bit video!! has to be >0 (I have no clue why)
    if(lla_vec2_dot(direction, info->normal) > 0.0f) {
        info->normal = (vec2_t){{-info->normal.x, -info->normal.y}};
    }

    return true;
}

bool p2d_collide_rect_rect(struct p2d_object *a, struct p2d_object *b, struct p2d_collision_info *info) {
    if(!info || !a || !b) {
        p2d_logf(P2D_LOG_ERROR, "p2d_collide_rect_rect: invalid arguments.\n");
        return false;
    }
    info->depth = FLT_MAX; info->normal = (vec2_t){{0, 0}};

    float min_a, max_a, min_b, max_b;
    struct p2d_obb_verts a_verts = p2d_obb_to_verts(p2d_get_obb(a));
    struct p2d_obb_verts b_verts = p2d_obb_to_verts(p2d_get_obb(b));

    for(int i = 0; i < 4; i++) {
        vec2_t va = a_verts.verts[i];
        vec2_t vb = a_verts.verts[(i + 1) % 4];

        vec2_t edge = {{vb.x - va.x, vb.y - va.y}};
        vec2_t axis = {{-edge.y, edge.x}}; 
        axis = lla_vec2_normalize(axis);

        p2d_project_obb_to_axis(a_verts, axis, &min_a, &max_a);
        p2d_project_obb_to_axis(b_verts, axis, &min_b, &max_b);

        if(min_a >= max_b || max_a <= min_b) {
            return false;
        }

        float axis_depth = fminf(max_a - min_b, max_b - min_a);

        if(axis_depth < info->depth) {
            info->depth = axis_depth;
            info->normal = axis;
        }
    }

    for(int i = 0; i < 4; i++) {
        vec2_t va = b_verts.verts[i];
        vec2_t vb = b_verts.verts[(i + 1) % 4];

        vec2_t edge = {{vb.x - va.x, vb.y - va.y}};
        vec2_t axis = {{-edge.y, edge.x}}; 
        axis = lla_vec2_normalize(axis);

        p2d_project_obb_to_axis(a_verts, axis, &min_a, &max_a);
        p2d_project_obb_to_axis(b_verts, axis, &min_b, &max_b);

        if(min_a >= max_b || max_a <= min_b) {
            return false;
        }

        float axis_depth = fminf(max_a - min_b, max_b - min_a);

        if(axis_depth < info->depth) {
            info->depth = axis_depth;
            info->normal = axis;
        }
    }

    vec2_t a_center = p2d_object_center(a);
    vec2_t b_center = p2d_object_center(b);

    vec2_t direction = {{b_center.x - a_center.x, b_center.y - a_center.y}};

    if(lla_vec2_dot(direction, info->normal) < 0.0f) {
        info->normal = (vec2_t){{-info->normal.x, -info->normal.y}};
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
    info->depth = 0; info->normal = (vec2_t){{0, 0}};

    bool result, swapped = false;

    /*
        Circle Circle
    */
    if(a->type == P2D_OBJECT_CIRCLE && b->type == P2D_OBJECT_CIRCLE) {
        result = p2d_collide_circle_circle(a, b, info);
    }

    /*
        Rect Circle
    */
    else if(a->type == P2D_OBJECT_CIRCLE && b->type == P2D_OBJECT_RECTANGLE) {
        result = p2d_collide_rect_circle(b, a, info);
        swapped = true;
    }
    else if(a->type == P2D_OBJECT_RECTANGLE && b->type == P2D_OBJECT_CIRCLE) {
        result = p2d_collide_rect_circle(a, b, info);
    }

    /*
        Rect Rect
    */
    else if(a->type == P2D_OBJECT_RECTANGLE && b->type == P2D_OBJECT_RECTANGLE) {
        result = p2d_collide_rect_rect(a, b, info);
    }

    else {
        p2d_logf(P2D_LOG_ERROR, "p2d_collide: invalid object types.\n");
        return false;
    }

    if(result && swapped) {
        info->normal = (vec2_t){{-info->normal.x, -info->normal.y}};
    }

    return result;
}
