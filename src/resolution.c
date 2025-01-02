/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include "p2d/resolution.h"

void p2d_object_step(struct p2d_object *object, float delta_time) {
    if(!object) {
        p2d_logf(P2D_LOG_ERROR, "p2d_object_step: object is NULL.\n");
        return;
    }

    // apply existing forces
    object->x += object->vx * delta_time;
    object->y += object->vy * delta_time;
    object->rotation += object->vr * delta_time;
    
}