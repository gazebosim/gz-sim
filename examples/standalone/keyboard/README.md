# Keyboard

## Build

From the root of the `ign-gazebo` repository, do the following to build the example:

~~~
cd ign-gazebo/examples/standalone/keyboard
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


