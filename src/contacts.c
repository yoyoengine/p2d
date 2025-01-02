/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <stdlib.h>

#include "p2d/pairs.h"
#include "p2d/helpers.h"
#include "p2d/contacts.h"

/*
    CONTACT LIST
*/

struct p2d_contact_list* p2d_contact_list_create(size_t initial_capacity) {
    struct p2d_contact_list* list = malloc(sizeof(struct p2d_contact_list));
    list->contacts = malloc(sizeof(struct p2d_contact) * initial_capacity);
    list->count = 0;
    list->capacity = initial_capacity;
    return list;
}

void p2d_contact_list_destroy(struct p2d_contact_list* list) {
    free(list->contacts);
    free(list);
}

void p2d_contact_list_add(struct p2d_contact_list* list, struct p2d_contact contact) {
    if (list->count >= list->capacity) {
        list->capacity *= 2;
        list->contacts = realloc(list->contacts, sizeof(struct p2d_contact) * list->capacity);
    }
    list->contacts[list->count++] = contact;
}

/*
    IMPL
*/

struct p2d_contact_list * p2d_generate_contacts(struct p2d_object *a, struct p2d_object *b) {
    struct p2d_contact_list* data = p2d_contact_list_create(1); // assume 1 contact initially

    // to seperate logic later TODO
    if(a->type == P2D_OBJECT_CIRCLE && b->type == P2D_OBJECT_CIRCLE) {
        struct p2d_vec2 midline = { b->x - a->x, b->y - a->y };
        float mag = lla_vec2_magnitude(p2d_struct_to_vec(midline));
    
        if(mag <= 0 || mag >= a->circle.radius + b->circle.radius) {
            return data;
        }

        struct p2d_contact contact = {0};

        struct p2d_vec2 normal = { midline.x / mag, midline.y / mag };

        contact.type = P2D_CONTACT_FACE_FACE;
        contact.contact_point = (struct p2d_vec2){ a->x + normal.x * a->circle.radius, a->y + normal.y * a->circle.radius };
    
        contact.contact_normal = normal;
        contact.penetration = a->circle.radius + b->circle.radius - mag;

        p2d_contact_list_add(data, contact);
        p2d_add_collision_pair(a, b);

        return data;
    }

    return data;
}