/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include "p2d/core.h"

struct p2d_state p2d_state = {0};

bool p2d_init(int cell_size) {
    if(cell_size <= 0) {
        return false;
    }

    p2d_state.p2d_object_count = 0;
    p2d_state.p2d_world_node_count = 0;

    return true;
}