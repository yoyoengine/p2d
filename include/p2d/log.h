/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_LOG_H
#define P2D_LOG_H

#include "p2d/core.h"

enum p2d_log_level {
    P2D_LOG_DEBUG,
    P2D_LOG_INFO,
    P2D_LOG_WARN,
    P2D_LOG_ERROR
};

#ifdef P2D_HAS_YOYOENGINE
    // forward declare the yoyoengine logging enum
    enum logLevel;
#endif

P2D_API void p2d_logf(enum p2d_log_level lvl, const char *fmt, ...);

#endif // P2D_LOG_H
