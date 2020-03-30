# Custom server

This example demonstrates how to run a simulation server headless
using the C++ API, instead of using Ignition Gazebo from the command line.

## Build Instructions

From this directory:

    cd ign-gazebo/examples/standalone/custom_server
    mkdir build
    cd build
    cmake ..
    make

## Execute Instructions

    ./custom_server

The server will run `shapes.sdf` for 100 iterations and exit. No GUI will
show up.
