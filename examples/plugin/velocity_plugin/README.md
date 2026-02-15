# Velocity World

This example shows how to set a velocity command on a link. 
This example will set the velocity command on a stand-alone link but the same approach can be applied for more complex robots.

In particular, in the world there is a box that is free to move on a ground plane. 
The plugin presented will apply a linear speed of 1 m/s along the x-axis.

## Build

From the root of the `gz-sim` repository, do the following to build the example:

~~~
cd gz-sim/examples/plugins/velocity_plugin
mkdir build
cd build
cmake ..
make
~~~

This will generate the `PluginLinearVelocity` library under `build`.

## Run

The plugin must be attached to an entity to be loaded.
This is demonstrated in the `velocity_world_plugin.sdf`.

Before starting Gazebo, we must make sure it can find the plugin by doing:

~~~
cd gz-sim/examples/plugins/velocity_plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then load the example world:

    gz sim -v 3 velocity_world_plugin.sdf

Once Gazebo is opened, toggle the `play/pause` buttons to see the box moving along a linear path.