# Physics2D

A 2 dimensional rigidbody physics system written for yoyoengine

## Features

- Broad phase collision detection, using a hashed spatial grid
- OOB and Circle collision detection and resolution
- exposes an iterator for synchronizing simulation to renderer

## Usage

```c
#include <p2d/p2d.h>

// example collision callback
void collision_callback(p2d_cb_data* data) {
  // ...
}

// example trigger callback
void trigger_callback(p2d_cb_data* data) {
  // ...
}

// at some point during init
p2d_init(TILE_SIZE, collision_callback, trigger_callback);

// create and register objects
// ...
struct p2d_object obj = {0};
obj.type = P2D_OBJECT_RECTANGLE,
obj.x = 100;
obj.y = 200;
obj.vx = 100;
p2d_register_object(&obj);
// ...



// typical update loop: (run this at the hz you want your physics to run at)
struct p2d_queue_event *current = p2d_step(delta_time);
while(current) {

  /*
    ...
    sync the given object with your engine using the provided deltas in the callback
    ...
  */
  current = current->next;
} // do not free the queue objects, this happens internally

// before quitting, shutdown
p2d_shutdown();
```

## thoughts

abandoning seperate colliders and rigidbodies

how can we do logging from p2d into yoyoengine?

## TODO:

- maybe add obb check against aabb for optimization? rn we convert aabb to obb
- determine parameters and callbacks this lib will perform. how do we handle trigger colliders?
- how is yoyoengine merging collider AND rigidbody together?? if at all.
- actual collision resolution

## considerations

in C, make sure we call a rigidbody_sync function to re-update the simulation with ecs changes

## Future Improvements

- Thread safe
  - Should be simple because we are already "asynchronously" returning a queue of deltas.
  - The complicated part is the "realtime" callbacks during simulation.
    - Solution: these could be compiled into the queue for later execution.

## TODO:

Actually, i dont think i like the queue idea, like at all.

- Wait until you finish the engine and then go back in to decide the sync. maybe its up to the engine to determine it based on its own delta tracking!
  - honestly might be ideal, beacuse then there is no interop other than step, and its pretty easy to embed last_x,y,r into the ECS, and then update based on current p2d object 

maybe fix seperation of world tracking, the insert into tile stuff being in core is really weird

- honestly if we use the tracking array we dont need to index spatially, we can create the temp spatial every tick, we dont need to add until the exact start of broad phase

refactor main.cpp stuff (yoinked obb thing)

add planes?

chgange P2D_MAX_OBJECTS to P2D_MAX_BUCKETS for clarity (only where applies) since both are mixed

TODO:
- (sink) stacking issues (velocity is continuously increasing, so penetration each frame is increasing)
- still intermittant bug with freak out collisions (WHY!??!?!?)
- continue the grind with the rotation and torque stuff
- collision and trigger callbacks!
