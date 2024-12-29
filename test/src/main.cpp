/*
    This file is a part of yoyoengine. (https://github.com/yoyoengine)
    Copyright (C) 2023-2024  Ryan Zmuda

    Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <stdio.h>

#include <SDL3/SDL.h>

#include <vector>

// extern "C" {
#include <p2d/p2d.h>
// }

std::vector<p2d_object> objects;

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

void collision_callback(p2d_cb_data* data) {
    printf("collision detected\n");
}

void trigger_callback(p2d_cb_data* data) {
    printf("trigger detected\n");
}

void draw_object(SDL_Renderer* renderer, p2d_object* object) {
    
    struct p2d_aabb aabb = p2d_get_aabb(object);
    struct p2d_obb obb = p2d_get_obb(object);
    
    // draw the aabb
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    SDL_FRect fr = {
        .x = aabb.x,
        .y = aabb.y,
        .w = aabb.w,
        .h = aabb.h,
    };
    SDL_RenderRect(renderer, &fr);

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
    }
}

int main(int argc, char** argv) {
    if(!SDL_Init(SDL_INIT_VIDEO)) {
        printf("SDL_Init failed: %s\n", SDL_GetError());
    }

    objects.push_back(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = false,
        .x = 300,
        .y = 200,
        .vx = 0,
        .vy = 0,
        .rotation = 45,
        .rectangle = {
            .width = 500,
            .height = 20,
        },
    });
    // objects[0].user_data = &objects[0];

    objects.push_back(p2d_object{
        .type = P2D_OBJECT_CIRCLE,
        .is_static = false,
        .x = 250,
        .y = 250,
        .vx = 0,
        .vy = 0,
        .rotation = 0,
        .rectangle = {
            .width = 100,
            .height = 100,
        },
    });
    // objects[1].user_data = &objects[1];

    // objects.push_back(p2d_object{
    //     .type = P2D_OBJECT_RECTANGLE,
    //     .is_static = false,
    //     .x = 100,
    //     .y = 100,
    //     .vx = 0,
    //     .vy = 0,
    //     .rotation = 0,
    // });
    // objects.push_back(p2d_object{
    //     .type = P2D_OBJECT_RECTANGLE,
    //     .is_static = false,
    //     .x = 100,
    //     .y = 100,
    //     .vx = 0,
    //     .vy = 0,
    //     .rotation = 0,
    // });

    // objects.push_back(p2d_object{
    //     .type = P2D_OBJECT_RECTANGLE,
    //     .is_static = false,
    //     .x = 1000,
    //     .y = 1000,
    //     .vx = 0,
    //     .vy = 0,
    //     .rotation = 0,
    // });

    int tile_size = 100;

    if(!p2d_init(tile_size, collision_callback, trigger_callback)) {
        printf("p2d_init failed\n");
        return 1;
    }

    for(auto& object : objects) {
        // p2d_world_insert(&object);
        p2d_create_object(&object);
    }

    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_CreateWindowAndRenderer("p2d demo", 1920, 1080, 0, &window, &renderer);

    int last_frame_time = SDL_GetTicks();

    while(1) {
        int time = SDL_GetTicks();

        float delta_time = (time - last_frame_time) / 1000.0f;

        SDL_Event event;
        while(SDL_PollEvent(&event)) {
            if(event.type == SDL_EVENT_QUIT) {
                SDL_DestroyRenderer(renderer);
                SDL_DestroyWindow(window);
                SDL_Quit();
                p2d_shutdown();
                return 0;
            }
        }

        // printf("delta_time: %f\n", delta_time);
        struct p2d_queue_event *current = p2d_step(delta_time);
        while(current) {
            printf("QUEUE EVENT:\n");
            printf("object: %p\n", current->object);
            printf("delta_x: %f\n", current->delta_x);
            printf("delta_y: %f\n", current->delta_y);
            printf("delta_rotation: %f\n", current->delta_rotation);
            current = current->next;
        }

        // draw lines to divide tiles by specified size
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        for(int x = 0; x < 1920; x += tile_size) {
            SDL_RenderLine(renderer, x, 0, x, 1080);
        }
        for(int y = 0; y < 1080; y += tile_size) {
            SDL_RenderLine(renderer, 0, y, 1920, y);
        }

        // draw objects
        for(auto& object : objects) {
            draw_object(renderer, &object);
        }

        SDL_RenderPresent(renderer);
    
        // // clear the terminal
        // printf("\033[2J\033[1;1H");

        printf("+---------------------+\n");
        printf("|        STATE        |\n");
        printf("+---------------------+\n");
        printf("objects: %d\n", p2d_state.p2d_object_count);
        printf("world nodes: %d\n", p2d_state.p2d_world_node_count);

        last_frame_time = time;
    }
    p2d_shutdown();
    return 0;
}
