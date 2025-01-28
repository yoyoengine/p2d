<div align="center">
    <picture style="width: 100%; height: auto;">
        <source srcset=".github/media/p2dlogo.png"  media="(prefers-color-scheme: dark)">
        <img src=".github/media/p2dlogo.png">
    </picture>
</div>

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![GitHub repo size](https://img.shields.io/github/repo-size/yoyoengine/p2d)

---

A 2D rigidbody physics system written for yoyoengine.

## Features

- Broad phase collision detection, using a hashed spatial grid
- OOB and Circle collision detection and resolution
- Collision and trigger callbacks
- Easy synchronization with existing ECS
- Frustum-culled sleeping objects

> [!WARNING]  
> Internal rotations are stored in degrees (not radians!) so make sure you convert properly.

## Usage

```c
#include <p2d/p2d.h>

// example collision callback
void collision_callback(struct p2d_cb_data* data) {
  // ...
}

// example trigger callback
void trigger_callback(struct p2d_cb_data* data) {
  // ...
}

// at some point during init
p2d_init(TILE_SIZE, SUBSTEP_COUNT, collision_callback, trigger_callback);

// create and register objects
// ...
struct p2d_object obj = {0};
obj.type = P2D_OBJECT_RECTANGLE,
obj.x = 100;
obj.y = 200;
obj.vx = 100;
obj.out_x = &YOUR_ECS_X;
obj.out_y = &YOUR_ECS_Y;
obj.out_rotation = &YOUR_ECS_ROTATION;
p2d_register_object(&obj);
// ...

// in your engine update loop: (run this at the hz you want your physics to run at)
p2d_step(physics_delta_time);

// before quitting, shutdown
p2d_shutdown();
```

## Future work

| Item                | Description                                 | Priority | Progress        |
|---------------------|---------------------------------------------|----------|-----------------|
| Debug Resolution    | The current basic resolution has edge cases | High     | Done            |
| Rotation Resolution | Implement rotation and torque               | High     | Done            |
| Friction Resolution | Implement friction                          | High     | Done            |
| Joints              | Constraints between objects                 | High     | In Progress     |
| Collision Layers    | Specify what can collide with what          | Medium   | Planned         |
| Advanced Gravity    | Allow seperate spatial fields of gravity    | Low      | Maybe Later     |
| New Shapes          | Implement planes for more complex shapes    | Low      | Maybe Later     |
| Optimization        | Micro-optimize for performance              | Low      | Maybe Later     |
| Add "True" Sleeping | Standstill objects get optimized out        | Low      | Maybe Later     |

## Testing

`cmake -DBUILD_P2D_TESTS=ON ..`

## Resources

Here are some resources that helped me along the way, which can hopefully be useful to you too!

- [(Book) Game Physics Engine Development](https://www.amazon.com/Game-Physics-Engine-Development-Commercial-Grade/dp/0123819768)
- [(Book) Real Time Collision Detection (Book)](https://a.co/d/g9Rpjsk)
- [(Youtube) Two-Bit Coding Physics Engine Playlist](https://www.youtube.com/playlist?list=PLSlpr6o9vURwq3oxVZSimY8iC-cdd3kIs)
- [(Articles) Rigid Body Dyanmics - Chris Hecker](https://www.chrishecker.com/Rigid_Body_Dynamics)
- [(Reference Example) Box2D](https://github.com/erincatto/box2d-lite/tree/master/Box2D)
