# Multi-LRAUV Swimming Race Example

This example shows the usage of the Thruster plugin and rudder joint control on
multiple autonomous underwater vehicles (AUV) with buoyancy, lift drag, and
hydrodynamics plugins. The multiple vehicles are differentiated by namespaces.

## Build Instructions

From this directory, run the following to compile:

    mkdir build
    cd build
    cmake ..
    make

## Execute Instructions

From the `build` directory, run Gazebo Sim and the example controller:

    gz sim -r ../../../worlds/multi_lrauv_race.sdf
    ./multi_lrauv_race

The example controller will output pseudorandom propeller and rudder commands
to move the vehicles forward. The low speed is by design to model the actual
vehicle velocity.
