/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <stdlib.h>

#include "p2d/queue.h"

struct p2d_resolution_queue p2d_resolution_queue = {0};

void p2d_purge_queue() {
    struct p2d_queue_event *current = p2d_resolution_queue.head;
    while(current) {
        struct p2d_queue_event *next = current->next;
        free(current);
        current = next;
    }
    p2d_resolution_queue.head = NULL;
    p2d_resolution_queue.tail = NULL;
}

void p2d_queue_push(struct p2d_object *object, float delta_x, float delta_y, float delta_rotation) {
    struct p2d_queue_event *event = malloc(sizeof(struct p2d_queue_event));
    event->object = object;
    event->delta_x = delta_x;
    event->delta_y = delta_y;
    event->delta_rotation = delta_rotation;
    event->next = NULL;

    if(!p2d_resolution_queue.head) {
        p2d_resolution_queue.head = event;
        p2d_resolution_queue.tail = event;
    }
    else {
        p2d_resolution_queue.tail->next = event;
        p2d_resolution_queue.tail = event;
    }
}