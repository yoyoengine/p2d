/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_EXPORT_H
#define P2D_EXPORT_H

#ifdef _WIN32
    #define P2D_API __declspec(dllexport)
#else
    #define P2D_API
#endif

#endif // P2D_EXPORT_H
