/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <stdarg.h>
#include <stdio.h>

#include "p2d/core.h"
#include "p2d/world.h"

struct p2d_state p2d_state = {0};

bool p2d_init(int cell_size) {
    if(cell_size <= 0) {
        return false;
    }

    p2d_state.p2d_object_count = 0;
    p2d_state.p2d_world_node_count = 0;

    p2d_logf(P2D_LOG_INFO, "p2d initialized with cell size: %d.\n", cell_size);
    
    return true;
}

bool p2d_create_object(struct p2d_object *object) {

    // TODO:
    // detect it's intersection within world tiles, and insert it into each tile to track

    p2d_state.p2d_object_count++;
    return true;
}

bool p2d_remove_object(struct p2d_object *object) {

    // TODO:
    // detect it's intersection within world tiles, and remove it from each tile

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

void p2d_step(float delta_time) {
    
    // TODO
    
    return;
}