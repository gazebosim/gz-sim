# Custom sensor system

TODO

## Build

From the root of the `ign-gazebo` repository, do the following to build the example:

~~~
cd examples/plugins/custom_sensor_system
mkdir build
cd build
cmake ..
make
~~~

This will generate the `CustomSensorSystem` library under `build`.

## Run

The plugin must be attached to an entity to be loaded. This is demonstrated in
the `custom_sensor.sdf` file that's going to be loaded.

Before starting Gazebo, we must make sure it can find the plugin by doing:

~~~
cd examples/plugins/custom_sensor_system
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then load the example world:

    ign gazebo -v 3 custom_sensor.sdf


TODO
