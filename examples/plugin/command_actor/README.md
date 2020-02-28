# Command actor

This example shows how to command actors in various ways from a plugin.

## Build

From the root of the `ign-gazebo` repository, do the following to build the example:

~~~
cd ign-gazebo/examples/plugins/command_actor
mkdir build
cd build
cmake ..
make
~~~

This will generate the `libCommandActor.so` library under `build`.

## Run

Add the library to the path:

~~~
cd ign-gazebo/examples/plugins/command_actor
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then run the demo world:

    ign gazebo -v 4 command_actor.sdf -r

