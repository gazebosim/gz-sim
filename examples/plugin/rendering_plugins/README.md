# Rendering plugins

Demo of 2 plugins that use Ignition Rendering, one for the server and one for the client.

## Build

From the root of the `ign-gazebo` repository, do the following to build the example:

~~~
cd examples/plugin/rendering_plugins
mkdir build
cd build
cmake ..
make
~~~

This will generate the `RenderingGuiPlugin` and `RenderingServerPlugin` libraries under `build`.

## Run

Add the libraries to the correct paths:

~~~
cd examples/plugin/rendering_plugins
export IGN_GUI_PLUGIN_PATH=`pwd`/build
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Run the example world

~~~
cd examples/plugin/rendering_plugins
ign gazebo -v 4 -r rendering_plugins.sdf
~~~

The ambient light on the server scene, visible from the camera sensor, will change every 2 seconds.

The ambient light on the client scene, visible from the GUI, will change every time the button is pressed.
