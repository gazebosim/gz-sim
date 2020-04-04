# GUI system plugin

This example shows how to create a GUI system plugin.

Ignition Gazebo supports any kind of Ignition GUI plugin
(`ignition::gui::Plugin`). Gazebo GUI plugins are a special type of Ignition
GUI plugin which also have access to entity and component updates coming from
the server.

See `GuiSystemPluginPlugin.hh` for more information.

## Build

From the root of the `ign-gazebo` repository, do the following to build the example:

~~~
cd ign-gazebo/examples/plugins/gui_system_plugin
mkdir build
cd build
cmake ..
make
~~~

This will generate the `libGuiSystemPlugin.so` library under `build`.

## Run

Add the library to the path:

~~~
cd ign-gazebo/examples/plugins/gui_system_plugin
export IGN_GUI_PLUGIN_PATH=`pwd`/build
~~~

Then run a world, for example:

    ign gazebo -v 4 shapes.sdf

From the GUI plugin menu on the top-right, choose "Gui System Plugin".

You'll see your plugin, displaying the world name `shapes`.
