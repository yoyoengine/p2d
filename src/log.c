/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <stdarg.h>

#include "p2d/log.h"

void p2d_logf(enum p2d_log_level lvl, const char *fmt, ...) {
    // if we have an external logging function, use it
    if(p2d_state.log) {
        va_list args;
        va_start(args, fmt);
        p2d_state.log((int)lvl, fmt, args);
        va_end(args);
        return;
    }

    // fallback to internal stdout logging
    switch(lvl){
        case P2D_LOG_DEBUG:
            printf("[P2D] [DEBUG] ");
            break;
        case P2D_LOG_INFO:
            printf("[P2D] [INFO] ");
            break;
        case P2D_LOG_WARN:
            printf("[P2D] [WARN] ");
            break;
        case P2D_LOG_ERROR:
            printf("[P2D] [ERROR] ");
            break;
    }
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}
