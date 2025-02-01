# TODO

attune deltatime for debugger so you can do more than one frame

## notes/divergences

- noticed that 2bit does not scale inv masses along normal dotted with normal, he just adds them to J?
- i do NOT divide by contact count, like 2bit does

future:

- add collision layers

ontriggerenter ontriggerexit ontriggerstay
/ general callback improvement

<https://github.com/erincatto/box2d/blob/main/src/revolute_joint.c>

- add revolute/hinge
- add weld????
- maybe some way to approximate ACTUAL rigid rod (game physics engine book - approximating true stiff springs)

add mask that can set layer-to-layer collisions rather than just shared
