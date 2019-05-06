# Custom components

This example shows how to create a custom component from a system plugin.

See `CustomComponentPlugin.hh` for more information.

## Build

From the root of the `ign-gazebo` repository, do the following to build the example:

~~~
cd ign-gazebo/examples/plugins/custom_component
mkdir build
cd build
cmake ..
make
~~~

This will generate the `libCustomComponent.so` library under `build`.

## Run

Add the library to the path:

~~~
cd ign-gazebo/examples/plugins/custom_component
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then run a world that loads the plugin as follows:

    ign gazebo -s -v 4 custom_component.sdf

