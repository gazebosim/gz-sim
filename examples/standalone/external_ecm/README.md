# External ECM

Example showing how to get a snapshot of all entities and components in a
running simulation from an external program using the state message.

## Build

From the root of the `gz-sim` repository, do the following to build the example:

~~~
cd gz-sim/examples/standalone/external_ecm
mkdir build
cd build
cmake ..
make
~~~

This will generate the `external_ecm` executable under `build`.

## Run

Start a simulation, for example:

    gz sim shapes.sdf

On another terminal, run the `external_ecm` executable, passing the name of the
running world you want to inspect:

  cd gz-sim/examples/standalone/external_ecm
  ./external_ecm shapes

You should see something like this:

```
$ ./external_ecm shapes

Requesting state for world [shapes] on service [/world/shapes/state]...

Entity [1]
  - Name: shapes
  - Parent:
Entity [4]
  - Name: ground_plane
  - Parent: shapes [1]
Entity [5]
  - Name: link
  - Parent: ground_plane [4]
Entity [6]
  - Name: visual
  - Parent: link [5]
Entity [7]
  - Name: collision
  - Parent: link [5]
Entity [8]
  - Name: box
  - Parent: shapes [1]
Entity [9]
  - Name: box_link
  - Parent: box [8]
Entity [10]
  - Name: box_visual
  - Parent: box_link [9]
Entity [11]
  - Name: box_collision
  - Parent: box_link [9]
Entity [12]
  - Name: cylinder
  - Parent: shapes [1]
Entity [13]
  - Name: cylinder_link
  - Parent: cylinder [12]
Entity [14]
  - Name: cylinder_visual
  - Parent: cylinder_link [13]
Entity [15]
  - Name: cylinder_collision
  - Parent: cylinder_link [13]
Entity [16]
  - Name: sphere
  - Parent: shapes [1]
Entity [17]
  - Name: sphere_link
  - Parent: sphere [16]
Entity [18]
  - Name: sphere_visual
  - Parent: sphere_link [17]
Entity [19]
  - Name: sphere_collision
  - Parent: sphere_link [17]
Entity [20]
  - Name: sun
  - Parent: shapes [1]
```
