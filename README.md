# Physics2D

A 2 dimensional rigidbody physics system written for yoyoengine

## Features

- OOB and Circle collision detection and resolution
- builds a bsp tree and handles logic internally
- exposes an iterator for synchronizing simulation to renderer

## thoughts

abandoning seperate colliders and rigidbodies

how can we do logging from p2d into yoyoengine?

## TODO:

detect object spanning multiple cells and add it to all cells it spans

- this involves making decisions on how the linear algebra will be structured for the lib
