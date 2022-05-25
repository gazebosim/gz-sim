# Hello world

This example contains the bare minimum that's necessary to create a Gazebo
system plugin.

## Build

From the root of the `ign-gazebo` repository, do the following to build the example:

~~~
cd ign-gazebo/examples/plugins/hello_world
mkdir build
cd build
cmake ..
make
~~~

This will generate the `HelloWorld` library under `build`.

## Run

The plugin must be attached to an entity to be loaded. This is demonstrated in
the `hello_world_plugin.sdf` file that's going to be loaded.

Before starting Gazebo, we must make sure it can find the plugin by doing:

~~~
cd ign-gazebo/examples/plugins/hello_world
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then load the example world:

    ign gazebo -v 3 hello_world_plugin.sdf

You should see green messages on the terminal like:

```
[Msg] Hello, world! Simulation is paused.
```

Toggle the play / pause buttons to see the message change.
