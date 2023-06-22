# System Reset API

This example uses the JointPositionRandomizer system to randomize the joint
positions of a robot arm at every reset.


## Build

~~~
cd examples/plugin/reset_plugin
mkdir build
cd build
cmake ..
make
~~~

This will generate the `JointPositionRandomizer` library under `build`.

## Run

Add the library to the path:

~~~
cd examples/plugin/reset_plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then run a world that loads the plugin as follows:

    gz sim -r -v 4 joint_position_randomizer.sdf

In another terminal, run the following to reset the world.

    gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'
