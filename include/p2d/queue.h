/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_QUEUE_H
#define P2D_QUEUE_H

#include "p2d/export.h"

/*
    The p2d simulation builds a queue of deltas to be consumed by the parent program
    after each simulation step. This is critical for synchronization between the engine
    and simulation.
*/

struct p2d_queue_event {
    struct p2d_object *object;
    float delta_x;
    float delta_y;
    float delta_rotation;

    struct p2d_queue_event *next;
};

struct p2d_resolution_queue {
    struct p2d_queue_event *head;
    struct p2d_queue_event *tail;
};
extern struct p2d_resolution_queue p2d_resolution_queue;

P2D_API void p2d_purge_queue();

P2D_API void p2d_queue_push(struct p2d_object *object, float delta_x, float delta_y, float delta_rotation);

#endif // P2D_QUEUE_H