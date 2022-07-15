# Keyboard

## Build

From the root of the `gz-sim` repository, do the following to build the example:

~~~
cd examples/standalone/keyboard
mkdir build
cd build
cmake ..
make
~~~

This will generate the `keyboard` executable under `build`.

## Run

The executable expects an SDF file with configurations.
An example file, `keyboard.sdf`, is provided.

You can run the example as follows:

    ./keyboard ../keyboard.sdf

## Demo example

Gazebo ships with an example file which has a differential drive vehicle
that can be controlled using a keyboard. You can run it as follows:

1. In a terminal, run the keyboard executable as described above:

        cd examples/standalone/keyboard/build
        ./keyboard ../keyboard.sdf

1. On another terminal, run `gz sim` with the vehicle that will
   consume the twist messages:

        cd examples/worlds
        gz sim -v 4 diff_drive.sdf

1. Switch back to the first terminal. Use the arrow keys to control one vehicle,
   and ASDW to control the other.
