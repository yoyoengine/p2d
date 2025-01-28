/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2025  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <stdio.h>

#include <SDL3/SDL.h>

#include <vector>
#include <memory>

// extern "C" {
#include <p2d/p2d.h>
// }

std::vector<std::shared_ptr<p2d_object>> objects;
std::vector<std::shared_ptr<p2d_joint>> joints;

void _draw_circle(SDL_Renderer *renderer, int center_x, int center_y, int radius)
{
    const int32_t diameter = (radius * 2);

    int32_t x = (radius - 1);
    int32_t y = 0;
    int32_t tx = 1;
    int32_t ty = 1;
    int32_t error = (tx - diameter);

    while (x >= y)
    {
        //  Each of the following renders an octant of the circle
        SDL_RenderPoint(renderer, center_x + x, center_y - y);
        SDL_RenderPoint(renderer, center_x + x, center_y + y);
        SDL_RenderPoint(renderer, center_x - x, center_y - y);
        SDL_RenderPoint(renderer, center_x - x, center_y + y);
        SDL_RenderPoint(renderer, center_x + y, center_y - x);
        SDL_RenderPoint(renderer, center_x + y, center_y + x);
        SDL_RenderPoint(renderer, center_x - y, center_y - x);
        SDL_RenderPoint(renderer, center_x - y, center_y + x);

        if (error <= 0)
        {
            ++y;
            error += ty;
            ty += 2;
        }

        if (error > 0)
        {
            --x;
            tx += 2;
            error += (tx - diameter);
        }
    }
}

void draw_object(SDL_Renderer* renderer, p2d_object* object) {
    
    struct p2d_aabb aabb = p2d_get_aabb(object);
    struct p2d_obb obb = p2d_get_obb(object);
    
    // // draw the aabb
    // SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    // SDL_FRect fr = {
    //     .x = aabb.x,
    //     .y = aabb.y,
    //     .w = aabb.w,
    //     .h = aabb.h,
    // };
    // SDL_RenderRect(renderer, &fr);

    if(object->type == P2D_OBJECT_RECTANGLE) {
        struct p2d_obb_verts verts = p2d_obb_to_verts(obb);

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        for(int i = 0; i < 4; i++) {
            int i2 = (i + 1) % 4;
            SDL_RenderLine(renderer, verts.verts[i].x, verts.verts[i].y, verts.verts[i2].x, verts.verts[i2].y);
        }
    }
    else if(object->type == P2D_OBJECT_CIRCLE) {
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        _draw_circle(renderer, object->x, object->y, object->circle.radius);

        float rot_rad = object->rotation * (M_PI / 180);

        float end_x = object->x + (object->circle.radius * cosf(rot_rad));
        float end_y = object->y + (object->circle.radius * sinf(rot_rad));
        SDL_RenderLine(renderer, object->x, object->y, end_x, end_y);
    }
}

// yoiiiiiiink
bool _object_intersects_tile(struct p2d_object *object, struct p2d_aabb tile) {
    if (object->type == P2D_OBJECT_RECTANGLE) {
        struct p2d_obb obb = p2d_get_obb(object);
        
        // TODO: replace when we can check AABB against OBB
        struct p2d_obb tile_obb = {
            .x = tile.x,
            .y = tile.y,
            .w = tile.w,
            .h = tile.h,
            .r = 0
        };

        return p2d_obb_intersects_obb(tile_obb, obb);
    }
    else { // P2D_OBJECT_CIRCLE
        struct p2d_circle circle = {
            .x = object->x,
            .y = object->y,
            .radius = object->circle.radius
        };
        return p2d_circle_intersects_aabb(circle, tile);
    }
}

bool paused = false;
bool single_setp = false;

void spawn_circle(int x, int y) {
    auto obj = std::make_shared<p2d_object>(p2d_object{
        .type = P2D_OBJECT_CIRCLE,
        .is_static = false,
        .x = (float)x,
        .y = (float)y,
        .circle = {
            .radius = 50,
        },
        // .density = 0.5f,
        .density = 2.0f,
        // .mass = 3,
        .restitution = 0.6,
        // .inertia = 0.5,
        .static_friction = 1.0,
        .dynamic_friction = 0.7,
    });
    objects.push_back(obj);

    p2d_create_object(obj.get());
}

void spawn_rect(int x, int y) {
    auto obj = std::make_shared<p2d_object>(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = false,
        .x = (float)x,
        .y = (float)y,
        // .rotation = 45,
        .rectangle = {
            .width = 100,
            .height = 100,
        },
        .density = 2.0f,
        // .density = 0.25f,
        // .density = 2.0f,
        // .mass = 5,
        .restitution = .6,
        // .inertia = 0.5,
        // .static_friction = 1.0,
        .static_friction = 1.0,
        // .dynamic_friction = 0.7,
        .dynamic_friction = 0.7,
    });
    objects.push_back(obj);

    p2d_create_object(obj.get());
}

void log_wrapper(int level, const char* fmt, ...) {
    printf("[WRAP] ");
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

void trigger_callback(p2d_cb_data* data) {
    printf("trigger detected\n");
}

void collision_callback(p2d_cb_data* data) {
    printf("collision detected\n");
}

int main(int argc, char** argv) {
    if(!SDL_Init(SDL_INIT_VIDEO)) {
        printf("SDL_Init failed: %s\n", SDL_GetError());
    }

    int tile_size = 100;
    // int iterations = 20;
    int iterations = 1;

    if(!p2d_init(tile_size, iterations, collision_callback, trigger_callback, log_wrapper)) {
        printf("p2d_init failed\n");
        return 1;
    }

    // objects.push_back(p2d_object{
    //     .type = P2D_OBJECT_CIRCLE,
    //     .is_static = false,
    //     .x = 300,
    //     .y = 600,
    //     // .vx = 10,
    //     // .vy = 500,
    //     // .vr = -20,
    //     // .rotation = 45,
    //     .circle = {
    //         .radius = 50,
    //     },
    //     .mass = 2,
    //     .restitution = 0.6,
    // });

    auto obj = std::make_shared<p2d_object>(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = true,
        .x = 60,
        .y = 1000,
        // .vx = 10,
        // .vy = 10,
        // .vr = -20,
        // .rotation = 45,
        .rectangle = {
            .width = 1800,
            .height = 40,
        },
        .density = 1,
        .restitution = .5,
        .static_friction = 1.0,
        .dynamic_friction = 0.7,
    });
    objects.push_back(obj);
    p2d_create_object(obj.get());

    auto b = std::make_shared<p2d_object>(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = true,
        .x = 1300,
        .y = 600,
        // .vx = 10,
        // .vy = 10,
        // .vr = -20,
        .vx = 0,
        .vy = 0,
        .vr = 0,
        // .rotation = 45,
        .rectangle = {
            .width = 200,
            .height = 200,
        },
        .density = 1,
        // .mass = 10,
        .restitution = .5,
        // .inertia = 0.5,
        .static_friction = 1.0,
        .dynamic_friction = 0.7,
    });
    objects.push_back(b);
    p2d_create_object(b.get());

    auto a = std::make_shared<p2d_object>(p2d_object{
        .type = P2D_OBJECT_CIRCLE,
        .is_static = true,
        .x = 1920/2,
        .y = 1080/2,
        // .vx = 10,
        // .vy = 10,
        // .vr = -20,
        // .rotation = 45,
        .circle= {
            .radius = 150,
        },
        .density = 1,
        .restitution = .5,
        .static_friction = 1.0,
        .dynamic_friction = 0.7,
    });
    objects.push_back(a);
    p2d_create_object(a.get());

    auto z = std::make_shared<p2d_object>(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = true,
        .x = 0,
        .y = 0,
        // .vx = 10,
        // .vy = 10,
        // .vr = -20,
        // .rotation = 45,
        .rectangle= {
            .width = 500,
            .height = 50,
        },
        .density = 1,
        .restitution = .5,
        .static_friction = 1.0,
        .dynamic_friction = 0.7,
    });
    objects.push_back(z);
    p2d_create_object(z.get());

    auto y = std::make_shared<p2d_object>(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = true,
        .x = 0,
        .y = 500,
        // .vx = 10,
        // .vy = 10,
        // .vr = -20,
        // .rotation = 45,
        .rectangle= {
            .width = 500,
            .height = 50,
        },
        .density = 1,
        .restitution = .5,
        .static_friction = 1.0,
        .dynamic_friction = 0.7,
    });
    objects.push_back(y);
    p2d_create_object(y.get());

    auto n = std::make_shared<p2d_object>(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = true,
        .x = 0,
        .y = 0,
        // .vx = 10,
        // .vy = 10,
        // .vr = -20,
        // .rotation = 45,
        .rectangle= {
            .width = 50,
            .height = 500,
        },
        .density = 1,
        .restitution = .5,
        .static_friction = 1.0,
        .dynamic_friction = 0.7,
    });
    objects.push_back(n);
    p2d_create_object(n.get());

    auto x = std::make_shared<p2d_object>(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = true,
        .x = 500,
        .y = 0,
        // .vx = 10,
        // .vy = 10,
        // .vr = -20,
        // .rotation = 45,
        .rectangle= {
            .width = 50,
            .height = 500,
        },
        .density = 1,
        .restitution = .5,
        .static_friction = 1.0,
        .dynamic_friction = 0.7,
    });
    objects.push_back(x);
    p2d_create_object(x.get());

    auto f = std::make_shared<p2d_object>(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = true,
        .x = 600,
        .y = 200,
        // .vx = 10,
        // .vy = 10,
        // .vr = -20,
        // .rotation = 45,
        .rectangle= {
            .width = 50,
            .height = 50,
        },
        .density = 1,
        .restitution = .5,
        .static_friction = 1.0,
        .dynamic_friction = 0.7,
    });
    objects.push_back(f);
    p2d_create_object(f.get());

    auto fc = std::make_shared<p2d_object>(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        // .type = P2D_OBJECT_CIRCLE,
        .is_static = false,
        .x = 600,
        .y = 400,
        // .vx = 10,
        // .vy = 10,
        // .vr = -20,
        // .rotation = 45,
        .rectangle= {
            .width = 50,
            .height = 50,
        },
        // .circle= {
        //     .radius = 50,
        // },
        .density = 1,
        .restitution = .5,
        .static_friction = 1.0,
        .dynamic_friction = 0.7,
    });
    objects.push_back(fc);
    p2d_create_object(fc.get());

    // add a joint
    auto joint = std::make_shared<p2d_joint>(p2d_joint{
        .a = f.get(),
        .b = fc.get(),
        .local_anchor_a = (vec2_t){.x = 0, .y = 0},
        .local_anchor_b = (vec2_t){.x = 0, .y = -25},
        .bias_factor = 0.1,
        .softness = 0.5,
    });
    joints.push_back(joint);
    p2d_add_joint(joint.get());

    // auto ball = std::make_shared<p2d_object>(p2d_object{
    //     .type = P2D_OBJECT_RECTANGLE,
    //     .is_static = false,
    //     .x = 600,
    //     .y = 0,
    //     .vx = 250,
    //     // .vy = 10,
    //     // .vr = -20,
    //     // .rotation = 45,
    //     .rectangle= {
    //         .width = 100,
    //         .height = 100,
    //     },
    //     .density = 2,
    //     .restitution = .5,
    //     .static_friction = 1.0,
    //     .dynamic_friction = 0.7,
    // });
    // objects.push_back(ball);
    // p2d_create_object(ball.get());

    // auto ball2 = std::make_shared<p2d_object>(p2d_object{
    //     .type = P2D_OBJECT_RECTANGLE,
    //     .is_static = false,
    //     .x = 800,
    //     .y = 0,
    //     .vx = 0,
    //     // .vy = 10,
    //     // .vr = -20,
    //     // .rotation = 45,
    //     .rectangle= {
    //         .width = 100,
    //         .height = 100,
    //     },
    //     .density = 2,
    //     .restitution = .5,
    //     .static_friction = 1.0,
    //     .dynamic_friction = 0.7,
    // });
    // objects.push_back(ball2);
    // p2d_create_object(ball2.get());

    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_CreateWindowAndRenderer("p2d demo", 1920, 1080, 0, &window, &renderer);
    SDL_SetRenderVSync(renderer, 1);
    int last_frame_time = SDL_GetTicks();

    struct p2d_contact_list *last_contacts = p2d_contact_list_create(25);
    p2d_state.out_contacts = last_contacts;

    p2d_state.p2d_gravity = (vec2_t){.x = 0, .y = 60.0f};
    // p2d_state.p2d_frustum_sleeping = true;
    // p2d_state.p2d_frustum = (struct p2d_obb){.x = (1920 - 1280) / 2, .y = (1080 - 720) / 2, .w = 1280, .h = 720};

    p2d_state.on_trigger = trigger_callback;
    p2d_state.on_collision = collision_callback;

    while(1) {
        int time = SDL_GetTicks();

        if(time - last_frame_time < 16) {
            SDL_Delay(16 - (time - last_frame_time));
        }

        float delta_time = (time - last_frame_time) / 1000.0f;

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        SDL_Event event;
        while(SDL_PollEvent(&event)) {
            if(event.type == SDL_EVENT_QUIT) {
                SDL_DestroyRenderer(renderer);
                SDL_DestroyWindow(window);
                SDL_Quit();
                p2d_shutdown();
                return 0;
            }

            // space - pause/unpause
            if(event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_SPACE) {
                paused = !paused;
            }

            // right arrow, single step
            single_setp = event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_RIGHT;

            // left click - spawn circle at mouse
            if(event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT) {
                float x, y;
                SDL_GetMouseState(&x, &y);
                spawn_circle(x, y);
                printf("spawned circle at %f, %f\n", x, y);
            }

            // right click - spawn rect at mouse
            if(event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_RIGHT) {
                float x, y;
                SDL_GetMouseState(&x, &y);
                spawn_rect(x, y);
                printf("spawned rect at %f, %f\n", x, y);
            }
        }

        // printf("delta_time: %f\n", delta_time);
        
        if(!paused || single_setp) {
            // p2d_contact_list_destroy(last_contacts);

            // last_contacts = p2d_step(delta_time);
            p2d_step(delta_time);
        }

        // TODO: better alternative for accessing debug like this is set flag in state and read from field that gets internally mem managed

        // display the contact poitns on screen
        for(int i = 0; i < last_contacts->count; i++) {
            struct p2d_contact contact = last_contacts->contacts[i];
            SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
            

            // larger rect to be more visible
            SDL_FRect fr = {
                .x = contact.contact_point.x - 5,
                .y = contact.contact_point.y - 5,
                .w = 10,
                .h = 10,
            };
            SDL_RenderRect(renderer, &fr);

            // draw the normal with proper magnitude given by penetration
            SDL_RenderLine(renderer, contact.contact_point.x + contact.contact_normal.x * contact.penetration, contact.contact_point.y + contact.contact_normal.y * contact.penetration, contact.contact_point.x, contact.contact_point.y);
        }

        // // draw lines to divide tiles by specified size
        // SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        // for(int x = 0; x < 1920; x += tile_size) {
        //     SDL_RenderLine(renderer, x, 0, x, 1080);
        // }
        // for(int y = 0; y < 1080; y += tile_size) {
        //     SDL_RenderLine(renderer, 0, y, 1920, y);
        // }

        // // next, draw pink tile rects for non-empty tiles
        // // we have to recompute (ok for demo) because we cant reverse hash
        // for(int i = 0; i < P2D_MAX_OBJECTS; i++) {
        //     if(p2d_objects[i] == NULL) {
        //         continue;
        //     }

        //     struct p2d_object *object = p2d_objects[i];

        //     struct p2d_aabb aabb = p2d_get_aabb(object);
    
        //     int start_tile_x = aabb.x / p2d_state.p2d_cell_size;
        //     int start_tile_y = aabb.y / p2d_state.p2d_cell_size;
        //     int end_tile_x = (aabb.x + aabb.w) / p2d_state.p2d_cell_size;
        //     int end_tile_y = (aabb.y + aabb.h) / p2d_state.p2d_cell_size;

        //     for (int tile_x = start_tile_x; tile_x <= end_tile_x; tile_x++) {
        //         for (int tile_y = start_tile_y; tile_y <= end_tile_y; tile_y++) {
        //             struct p2d_aabb tile = {
        //                 .x = tile_x * p2d_state.p2d_cell_size,
        //                 .y = tile_y * p2d_state.p2d_cell_size,
        //                 .w = p2d_state.p2d_cell_size,
        //                 .h = p2d_state.p2d_cell_size
        //             };
                    
        //             SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        //             SDL_FRect fr = {
        //                 .x = tile.x,
        //                 .y = tile.y,
        //                 .w = tile.w,
        //                 .h = tile.h,
        //             };
        //             SDL_RenderRect(renderer, &fr);

        //             if(_object_intersects_tile(object, tile)) {
        //                 SDL_SetRenderDrawColor(renderer, 255, 165, 0, 255);
        //                 SDL_FRect fr = {
        //                     .x = tile.x,
        //                     .y = tile.y,
        //                     .w = tile.w,
        //                     .h = tile.h,
        //                 };
        //                 SDL_RenderRect(renderer, &fr);
        //             }
        //         }
        //     }
        // }

        // draw objects
        for(auto& object : objects) {
            draw_object(renderer, object.get());
        }

        // draw joints
        for (auto& joint : joints) {
            struct p2d_joint j = *joint.get();

            p2d_object a = *j.a;
            p2d_object b = *j.b;

            vec2_t world_anchor_a = p2d_get_joint_world_anchor(j.a, j.local_anchor_a);
            vec2_t world_anchor_b = p2d_get_joint_world_anchor(j.b, j.local_anchor_b);

            SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
            SDL_RenderLine(renderer, world_anchor_a.x, world_anchor_a.y, world_anchor_b.x, world_anchor_b.y);
        }

        // draw frustum
        if(p2d_state.p2d_frustum_sleeping) {
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            struct p2d_obb frustum = p2d_state.p2d_frustum;
            struct p2d_obb_verts verts = p2d_obb_to_verts(frustum);
            for(int i = 0; i < 4; i++) {
                int i2 = (i + 1) % 4;
                SDL_RenderLine(renderer, verts.verts[i].x, verts.verts[i].y, verts.verts[i2].x, verts.verts[i2].y);
            }
        }

        SDL_RenderPresent(renderer);
    
        // // clear the terminal
        // printf("\033[2J\033[1;1H");

        if(!paused || single_setp) {
            printf("+---------------------+\n");
            printf("|        STATE        |\n");
            printf("+---------------------+\n");
            printf("objects: %d\n", p2d_state.p2d_object_count);
            printf("sleeping: %d\n", p2d_state.p2d_sleeping_count);
            printf("world nodes: %d\n", p2d_state.p2d_world_node_count);
            printf("contact checks: %d\n", p2d_state.p2d_contact_checks);
            printf("contacts found: %d\n", p2d_state.p2d_contacts_found);
            printf("collision pairs: %d\n", p2d_state.p2d_collision_pairs);
        }

        last_frame_time = time;
        single_setp = false;

    }
    p2d_contact_list_destroy(last_contacts);
    p2d_shutdown();
    return 0;
}
