/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <float.h>
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

void p2d_contact_list_clear(struct p2d_contact_list* list) {
    list->count = 0;
}

/*
    IMPL
*/

struct p2d_contact_list * p2d_generate_contacts(struct p2d_object *a, struct p2d_object *b) {
    struct p2d_contact_list* data = p2d_contact_list_create(1); // assume 1 contact initially

    /*
        Circle Circle
    */
    if(a->type == P2D_OBJECT_CIRCLE && b->type == P2D_OBJECT_CIRCLE) {
        p2d_generate_circle_circle_contacts(data, a, b);
        return data;
    }

    /*
        Rect Circle
    */
    if(a->type == P2D_OBJECT_CIRCLE && b->type == P2D_OBJECT_RECTANGLE) {
        p2d_generate_rect_circle_contacts(data, b, a);
        return data;
    }
    if(a->type == P2D_OBJECT_RECTANGLE && b->type == P2D_OBJECT_CIRCLE) {
        p2d_generate_rect_circle_contacts(data, a, b);
        return data;
    }

    /*
        Rect Rect
    */
    // if(a->type == P2D_OBJECT_RECTANGLE && b->type == P2D_OBJECT_RECTANGLE) {
    //     p2d_generate_rect_rect_contacts(data, a, b);
    //     return data;
    // }

    return data;
}

/*
    Circle Circle
*/

void p2d_generate_circle_circle_contacts(struct p2d_contatct_list *contacts, struct p2d_object *a, struct p2d_object *b) {
    struct p2d_vec2 midline = { b->x - a->x, b->y - a->y };
    float mag = lla_vec2_magnitude(p2d_struct_to_vec(midline));

    if(mag <= 0 || mag >= a->circle.radius + b->circle.radius) {
        return;
    }

    struct p2d_contact contact = {0};

    struct p2d_vec2 normal = { midline.x / mag, midline.y / mag };

    contact.type = P2D_CONTACT_FACE_FACE;
    contact.contact_point = (struct p2d_vec2){ a->x + normal.x * a->circle.radius, a->y + normal.y * a->circle.radius };

    contact.contact_normal = normal;
    contact.penetration = a->circle.radius + b->circle.radius - mag;

    p2d_contact_list_add(contacts, contact);
    p2d_add_collision_pair(a, b);
}

/*
    Rect Circle
*/

/*
    Shoutout to THE GOAT! <https://youtu.be/V2JI_P9bvik?si=Ld7m52tOIoRP94NE>

    We dont perform pure vert checks then edge checks, by the nature of
    checking closest point on each line segment of the vert, we implicitely
    check each exact vert in the case that its the closest on two edges
*/
void p2d_generate_rect_circle_contacts(struct p2d_contact_list *contacts, struct p2d_object *rect, struct p2d_object *circle) {
    struct p2d_contact contact = {0};

    struct p2d_obb_verts verts = p2d_obb_to_verts(p2d_get_obb(rect));

    float min_dist = FLT_MAX;
    struct p2d_vec2 min_closest_point = {0};
    struct p2d_vec2 va = verts.verts[0];
    for(int i = 0; i < 4; i++) {
        struct p2d_vec2 vb = verts.verts[(i + 1) % 4];

        struct p2d_vec2 closest_point = {0};
        float dist = 0;
        p2d_closest_point_on_segment_to_point(va, vb, (struct p2d_vec2){circle->x, circle->y}, &closest_point, &dist);

        if(dist < min_dist) {
            min_dist = dist;
            min_closest_point = closest_point;
        }

        va = vb;
    }

    contact.contact_point = min_closest_point;
    contact.penetration = circle->circle.radius - min_dist;

    struct p2d_vec2 delta = {circle->x - contact.contact_point.x, circle->y - contact.contact_point.y};
    float dist = lla_vec2_magnitude(p2d_struct_to_vec(delta));
    if(dist > 0) {
        contact.contact_normal = (struct p2d_vec2){delta.x / dist, delta.y / dist};
    }
    else {
        contact.contact_normal = (struct p2d_vec2){1, 0};
    }

    if(contact.penetration < 0) { return; }

    p2d_contact_list_add(contacts, contact);
    p2d_add_collision_pair(rect, circle);
}