/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <stdarg.h>
#include <stdio.h>

#include "p2d/core.h"
#include "p2d/world.h"
#include "p2d/types.h"
#include "p2d/helpers.h"
#include "p2d/detection.h"

struct p2d_state p2d_state = {0};

//
// INIT
//

bool p2d_init(
    int cell_size,
    void (*on_collision)(struct p2d_cb_data *data),
    void (*on_trigger)(struct p2d_cb_data *data))
{
    if(cell_size <= 0) {
        return false;
    }
    p2d_state._cell_size = cell_size;

    if(!on_collision) {
        p2d_logf(P2D_LOG_WARN, "p2d_init: on_collision is NULL.\n");
    }
    p2d_state.on_collision = on_collision;

    if(!on_trigger) {
        p2d_logf(P2D_LOG_WARN, "p2d_init: on_trigger is NULL.\n");
    }
    p2d_state.on_trigger = on_trigger;

    p2d_state.p2d_object_count = 0;
    p2d_state.p2d_world_node_count = 0;

    p2d_logf(P2D_LOG_INFO, "p2d initialized with cell size: %d.\n", cell_size);
    
    return true;
}



//
// TILE INTERSECTION DETECTION/RESOLUTION
//

bool _object_intersects_tile(struct p2d_object *object, struct p2d_aabb tile) {
    if (object->type == P2D_OBJECT_RECTANGLE) {
        struct p2d_obb obb = p2d_get_obb(object);
        
        // TODO: replace when we can check AABB against OBB
        struct p2d_obb tile_obb = {
            .x = tile.x,
            .y = tile.y,
            .w = tile.w,
            .h = tile.h,
            .r = 0
        };

        return p2d_obb_intersects_obb(tile_obb, obb);
    }
    else { // P2D_OBJECT_CIRCLE
        struct p2d_circle circle = {
            .x = object->x,
            .y = object->y,
            .radius = object->circle.radius
        };
        return p2d_circle_intersects_aabb(circle, tile);
    }
}

// runs callback for each tile the object intersects with
void p2d_for_each_intersecting_tile(struct p2d_object *object, void (*callback)(struct p2d_object *object, int tile_hash)) {
    if (!object) {
        p2d_logf(P2D_LOG_ERROR, "p2d_for_each_intersecting_tile: object is NULL.\n");
        return;
    }

    struct p2d_aabb aabb = p2d_get_aabb(object);
    
    int start_tile_x = aabb.x / p2d_state._cell_size;
    int start_tile_y = aabb.y / p2d_state._cell_size;
    int end_tile_x = (aabb.x + aabb.w) / p2d_state._cell_size;
    int end_tile_y = (aabb.y + aabb.h) / p2d_state._cell_size;

    for (int tile_x = start_tile_x; tile_x <= end_tile_x; tile_x++) {
        for (int tile_y = start_tile_y; tile_y <= end_tile_y; tile_y++) {
            struct p2d_aabb tile = {
                .x = tile_x * p2d_state._cell_size,
                .y = tile_y * p2d_state._cell_size,
                .w = p2d_state._cell_size,
                .h = p2d_state._cell_size
            };

            if (_object_intersects_tile(object, tile)) {
                int hash = p2d_world_hash(tile_x, tile_y);
                callback(object, hash);
            }
        }
    }
}

void _register_intersecting_tiles(struct p2d_object *object, int hash) {
    p2d_world_insert(hash, object);
}

void _unregister_intersecting_tiles(struct p2d_object *object, int hash) {
    p2d_world_remove(hash, object);
}



//
// OBJECT MANAGEMENT
//

bool p2d_create_object(struct p2d_object *object) {
    if(!object) {
        p2d_logf(P2D_LOG_ERROR, "p2d_create_object: object is NULL.\n");
        return false;
    }

    // Detect all world tiles the object intersects with, and add it to each
    p2d_for_each_intersecting_tile(object, _register_intersecting_tiles);

    p2d_state.p2d_object_count++;
    return true;
}

bool p2d_remove_object(struct p2d_object *object) {
    if(!object) {
        p2d_logf(P2D_LOG_ERROR, "p2d_remove_object: object is NULL.\n");
        return false;
    }

    // Detect all world tiles the object intersects with, and remove it from each
    p2d_for_each_intersecting_tile(object, _unregister_intersecting_tiles);

    p2d_state.p2d_object_count--;
    return true;
}

bool p2d_remove_all_objects() {
    p2d_world_remove_all();
    return true;
}

bool p2d_shutdown() {
    p2d_remove_all_objects();
    p2d_logf(P2D_LOG_INFO, "p2d shutdown.\n");
    return true;
}



//
// MISC
//

#ifndef P2D_HAS_YOYOENGINE
void p2d_logf(int level, const char *fmt, ...) {
    switch(level){
        case P2D_LOG_DEBUG:
            printf("[DEBUG] ");
            break;
        case P2D_LOG_INFO:
            printf("[INFO] ");
            break;
        case P2D_LOG_WARN:
            printf("[WARN] ");
            break;
        case P2D_LOG_ERROR:
            printf("[ERROR] ");
            break;
    }

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}
#endif

struct p2d_queue_event * p2d_step(float delta_time) {
    
    // purge last queue, user should have consumed it by now
    p2d_purge_queue();

    // TODO SIMULATION
    
    return p2d_resolution_queue.head;
}