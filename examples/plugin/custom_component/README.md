# Custom components

This example shows how to create a custom component from a system plugin.

See `CustomComponentPlugin.hh` for more information.

## Build

From the root of the `gz-sim` repository, do the following to build the example:

~~~
cd examples/plugin/custom_component
mkdir build
cd build
cmake ..
make
~~~

This will generate the `CustomComponent` library under `build`.

## Run

Add the library to the path:

~~~
cd examples/plugin/custom_component
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then run a world that loads the plugin as follows:

    gz sim -s -v 4 custom_component.sdf
