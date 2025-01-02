/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#ifndef P2D_CONTACTS_H
#define P2D_CONTACTS_H

#include <stdbool.h>

#include "p2d/export.h"
#include "p2d/types.h"
#include "p2d/core.h"

enum p2d_contact_type {
    P2D_CONTACT_NONE,
    P2D_CONTACT_FACE_FACE,
};

struct p2d_contact {
    enum p2d_contact_type type;

    struct p2d_vec2 contact_point;
    struct p2d_vec2 contact_normal;

    float penetration;
};

struct p2d_contact_list {
    struct p2d_contact* contacts;
    size_t count;
    size_t capacity;
};

P2D_API struct p2d_contact_list * p2d_contact_list_create(size_t initial_capacity);
P2D_API void p2d_contact_list_destroy(struct p2d_contact_list* list);
P2D_API void p2d_contact_list_add(struct p2d_contact_list* list, struct p2d_contact contact);

P2D_API struct p2d_contact_list * p2d_generate_contacts(struct p2d_object *a, struct p2d_object *b);

#endif // P2D_CONTACTS_H