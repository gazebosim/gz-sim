# Custom server

This example demonstrates how to run an `ignition::gazebo::Server` from
an executable, instead of using Ignition Gazebo from the command line.

## Build Instructions

From this directory:

    mkdir build
    cd build
    cmake ..
    make

## Execute Instructions

    ./custom_server

The server will run `shapes.sdf` for 100 iterations and exit.
