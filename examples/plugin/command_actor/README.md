# Command actor

This example shows how to command actors in various ways from a plugin.

## Build

From the root of the `gz-sim` repository, do the following to build the example:

~~~
cd examples/plugin/command_actor
mkdir build
cd build
cmake ..
make
~~~

This will generate the `libCommandActor.so` library under `build`.

## Run

Add the library to the path:

~~~
cd examples/plugin/command_actor
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then run the demo world:

    gz sim -v 4 command_actor.sdf -r
