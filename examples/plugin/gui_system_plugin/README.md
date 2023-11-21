# GUI system plugin

This example shows how to create a GUI system plugin.

Gazebo supports any kind of GUI plugin
(`gz-gui` library: `gz::gui::Plugin`). However, GuiSystem plugins
(`gz-sim` library: `gz::sim::GuiSystem`) are a special type of GUI plugin,
which also have access to entity and component updates coming from the server.

See `GuiSystemPluginPlugin.hh` for more information.

## Build

From the root of the `gz-sim` repository, do the following to build the example:

~~~
cd examples/plugin/gui_system_plugin
mkdir build
cd build
cmake ..
make
~~~

This will generate the `GuiSystemPlugin` library under `build`.

## Run

Add the library to the path:

~~~
cd examples/plugin/gui_system_plugin
export GZ_GUI_PLUGIN_PATH=`pwd`/build
~~~

Then run a world, for example:

    gz sim -v 4 shapes.sdf

From the GUI plugin menu on the top-right, choose "Gui System Plugin".

You'll see your plugin, displaying the world name `shapes`.
