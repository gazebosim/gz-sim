# Multi-LRAUV Acoustic comms example

This example shows the usage of the Acoustic comms plugin on
multiple autonomous underwater vehicles (AUV) with buoyancy, lift drag, and
hydrodynamics plugins. The multiple vehicles are differentiated by namespaces.

It consists of 3 vehicles,
Triton, Tethys, and Daphne floating side by side. Triton sends
a move command using acoustic comms to the other 2 vehicles,
which start moving on receiving the command. The speed of sound
is purposely slowed down here to show that the middle vehicle (Tethys)
will receive the signal and start moving before Daphne.

## Build Instructions

From this directory, run the following to compile:

```bash
cmake -B build
cmake --build build
```

## Execute Instructions

Run Gazebo Sim and the example controller:

```bash
gz sim -r ../../worlds/acoustic_comms_demo.sdf
./build/acoustic_comms_demo
```

It can be seen visually that one of the vehicles (Triton) starts moving
immediately, then afer a while Tethys will start moving, and then finally Daphne.
