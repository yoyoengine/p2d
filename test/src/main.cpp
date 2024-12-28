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

int main(int argc, char** argv) {
    if(!SDL_Init(SDL_INIT_VIDEO)) {
        printf("SDL_Init failed: %s\n", SDL_GetError());
    }

    objects.push_back(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = false,
        .x = 100,
        .y = 100,
        .vx = 0,
        .vy = 0,
        .rotation = 0,
    });
    objects.push_back(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = false,
        .x = 100,
        .y = 100,
        .vx = 0,
        .vy = 0,
        .rotation = 0,
    });
    objects.push_back(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = false,
        .x = 100,
        .y = 100,
        .vx = 0,
        .vy = 0,
        .rotation = 0,
    });

    objects.push_back(p2d_object{
        .type = P2D_OBJECT_RECTANGLE,
        .is_static = false,
        .x = 1000,
        .y = 1000,
        .vx = 0,
        .vy = 0,
        .rotation = 0,
    });

    if(!p2d_init(100)) {
        printf("p2d_init failed\n");
        return 1;
    }

    for(auto& object : objects) {
        // p2d_world_insert(&object);
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
        p2d_step(delta_time);

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
