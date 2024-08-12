\page rendering_plugins Rendering plugins

This tutorial will go over how to write Gazebo plugins that alter the
3D scene's visual appearance using Gazebo Rendering APIs.

This is not to be confused with integrating a new rendering engine. See
[How to write your own rendering engine plugin](https://gazebosim.org/api/rendering/8/renderingplugin.html)
for that.

This tutorial will go over a couple of example plugins that are located at
https://github.com/gazebosim/gz-sim/tree/main/examples/plugin/rendering_plugins.

## Scenes

During simulation, there are up to two 3D scenes being rendered by
Gazebo, one on the server process and one on the client process.

The server-side scene will only be created when using the
`gz::sim::systems::Sensors` system plugin on the server. This is the
scene that shows what the sensors see.

The client-side scene will only be created when using the
`gz::gui::plugins::MinimalScene` GUI system plugin on the client. This is the
scene that shows what the user sees.

For the user to see what the sensors see, they need to use other GUI plugins
that display sensor data, such as `gz::gui::plugins::ImageDisplay` for
camera images or `gz::sim::VisualizeLidar` for lidar point clouds.

Gazebo keeps these scenes in sync by sending periodic state messages
from the server to the client that contain entity and component data with
the `gz::sim::systems::SceneBroadcaster` plugin. Any
changes done to these scenes using Gazebo Rendering APIs directly, as
described in this tutorial, will only affect one of the scenes and will not be
synchronized. The examples below will show how to change the ambient light for
each scene separately.

## Plugin types

Depending on the scene that you want to affect, you'll need to write a
different plugin.

To interact with the server-side scene, you'll need to write an
`gz::sim::System`.
See [Create System Plugins](createsystemplugins.html).

To interact with the client-side scene, you'll need to write an
\ref gz::gui::Plugin, or a more specialized `gz::sim::GuiSystem`
if you need to access entities and components.
See the [GUI system plugin example](https://github.com/gazebosim/gz-sim/tree/main/examples/plugin/gui_system_plugin).

## Getting the scene

When writing either plugin type, the `gz::rendering::Scene` pointer can
be conveniently found using the rendering engine's singleton. Both example
plugins use the exact same logic to get the scene:

\snippet examples/plugin/rendering_plugins/RenderingServerPlugin.cc findScene

The function above works for most cases, but you're welcome to customize it
for your use case.

## Render thread

Rendering operations aren't thread-safe. To make sure there are no race
conditions, all rendering operations should happen in the same thread, the
"render thread". In order to access that thread from a custom plugin, it's
necessary to listen to events that the 3D scene is emitting. These are
different for each plugin type.

### Render events on the GUI

The GUI plugin will need to listen to \ref gz::gui::events::Render
events. Here's how to do it:

1. Include the GUI events header:

    \snippet examples/plugin/rendering_plugins/RenderingGuiPlugin.cc includeGuiEvents

2. The 3D scene sends render events periodically to the `gz::gui::MainWindow`,
   not directly to every plugin. Therefore, your plugin will need to install a filter
   so that it receives all events coming from the `MainWindow`. In your plugin's
   `LoadConfig` call, install the filter as follows:

    \snippet examples/plugin/rendering_plugins/RenderingGuiPlugin.cc connectToGuiEvent

3. The filter will direct all of `MainWindow`'s events to the `eventFilter` callback. Add
   that function to your plugin as follows. Be sure to check for the event that you want,
   and end the `eventFilter` function by forwarding the event to the base class.

    \snippet examples/plugin/rendering_plugins/RenderingGuiPlugin.cc eventFilter

4. All your rendering operations should happen right there where
   `PerformRenderingOperations` is located. In this example plugin, it checks if the
   `dirty` flag is set, and if it is, it changes the scene's ambient light color randomly.

    \snippet examples/plugin/rendering_plugins/RenderingGuiPlugin.cc performRenderingOperations

### Render events on the server

The server plugin will need to listen `gz::sim::events::PreRender`, `gz::sim::events::Render` or
`gz::sim::events::PostRender` events.

Here's how to do it:

1. Include the rendering events header:

    \snippet examples/plugin/rendering_plugins/RenderingServerPlugin.cc includeRenderingEvents

2. To receive `PreRender` events, connect to it as follows, and the
   `PerformRenderingOperations` function will be called periodically:

    \snippet examples/plugin/rendering_plugins/RenderingServerPlugin.cc connectToServerEvent

3. All your rendering operations should happen at `PerformRenderingOperations` is located.
   In this example plugin, it checks if enough time has elapsed since the last color update,
   and updates the color if it's time:

    \snippet examples/plugin/rendering_plugins/RenderingServerPlugin.cc performRenderingOperations

## Running examples

Follow the build instructions on the rendering plugins
[README](https://github.com/gazebosim/gz-sim/blob/main/examples/plugin/rendering_plugins)
and you'll generate both plugins:

* `RenderingGuiPlugin`: GUI plugin that updates the GUI scene's ambient light with a random color at each click.
* `RenderingServerPlugin`: Server plugin that updates the server scene's ambient light every 2 simulation seconds.

Run the example world that uses both plugins and observe how the scene seen by
the camera sensor, displayed on the top-left camera image, is different from
the one on the GUI. Try pausing simulation and pressing the
`RANDOM GUI COLOR` button to see which scene gets updated.

@image html files/rendering_plugins.gif
