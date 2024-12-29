# Physics2D

A 2 dimensional rigidbody physics system written for yoyoengine

## Features

- Broad phase collision detection, using a hashed spatial grid
- OOB and Circle collision detection and resolution
- exposes an iterator for synchronizing simulation to renderer

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
