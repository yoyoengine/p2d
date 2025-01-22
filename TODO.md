# TODO

actually call callbacks

in new solver, cubes are very messed up, refuse to bounce at all
- my guess is that its something very stupid, like a bad value in a different place

jank densitys...

attune deltatime for debugger so you can do more than one frame

DO NOT USE LOGICAL SCALING UNLESS YOU APPLY IT GLOBALLY.

angular impulse is practically nonexistant for some reason.

my best guess is the inertia and mass values are just screwed. the hecker equations seem correct...

something else is very wrong, because mid air collisions arent respected either by anything

## Summary 1/22/24

- hecker equations
  - seem right?
  - basic resolution differs
- inertia formulas
  - i messed with scaling coefficients but couldnt find a working value
- box2d-lite
  - seems like theyre using logical units

conclusion:
- wtf
- start over? begin with inertia and fill in the rest later (no broad phase)
