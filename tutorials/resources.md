\page resources Finding resources

Setting up and running a simulation can involve loading various kinds of
resources, such as robot models and plugins, from different locations, which
can include a local filesystem and online servers. Gazebo offers
a few different mechanisms for locating required resources.

## Plugins

A plugin is a shared library that adheres to a specific API and is loaded at
runtime. Typically, plugins are scoped to perform a narrow set of features.
For example, the `diff_drive` plugin, provided by Gazebo, implements
a differential drive controller for mobile robots.

Gazebo relies on plugins for rendering, physics simulation, sensor data
generation, and many of the capabilities. The following sections describe
how Gazebo finds and loads different types of plugins.

### System Plugins

A system plugin is used by Gazebo, and provides an entry point for
simulation customization and control. Refer to the \subpage createsystemplugins
tutorial for information about creating your own system plugin.

System plugins may be loaded through:

* Tags in SDF files, where `filename` is the shared library and
  `name` is the class to be loaded. A system plugin can be attached to
  different entities.
    * Attached to the **world**: `<world><plugin>`
    * Attached to a **model**: `<model><plugin>`
    * Attached to a **sensor**: `<sensor><plugin>`
* Passing the shared library and class to be loaded through
  [PluginInfo](https://gazebosim.org/api/gazebo/4.6/classignition_1_1gazebo_1_1ServerConfig_1_1PluginInfo.html)
  (within [ServerConfig](https://gazebosim.org/api/gazebo/4.6/classignition_1_1gazebo_1_1ServerConfig.html))
  when instantiating the
  [Server](https://gazebosim.org/api/gazebo/4.6/classignition_1_1gazebo_1_1Server.html#a084ef7616f5af42061a7aeded5651ab0).

Gazebo will look for system plugins on the following paths, in order:

1. All paths on the `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` environment variable
2. `$HOME/.gz/sim/plugins`
3. [Systems that are installed with Gazebo](https://gazebosim.org/api/gazebo/4.6/namespace gz_1_1gazebo_1_1systems.html)

### Gazebo GUI plugins

Each [Gazebo GUI](https://gazebosim.org/libs/rendering) plugin
defines a widget.

GUI plugins may be loaded through:

* Tags in SDF world files, where `filename` is the shared library:
    * `<world><gui><plugin>`
* Tags in [GUI config files](https://gazebosim.org/api/gui/4.2/config.html),
  where `filename` is the shared library:
    * `<plugin>`
* The plugin menu on the top-right of the screen.

Gazebo will look for GUI plugins on the following paths, in order:

1. All paths set on the `IGN_GUI_PLUGIN_PATH` environment variable
2. [GUI plugins that are installed with Gazebo](https://github.com/gazebosim/gz-sim/tree/main/src/gui/plugins)
3. Other paths added by calling `gz::gui::App()->AddPluginPath`
4. `~/.gz/gui/plugins`
5. [Plugins which are installed with Gazebo GUI](https://gazebosim.org/api/gui/4.2/namespace gz_1_1gui_1_1plugins.html)

### Physics engines

[Gazebo Physics](https://gazebosim.org/libs/physics)
uses a plugin architecture and its physics engines are
built as plugins that are loaded at run time using
[Gazebo Plugin](https://gazebosim.org/libs/plugin).

See the [Physics engines](physics.html)
tutorial for more details.

### Rendering engines

[Gazebo Rendering](https://gazebosim.org/libs/rendering)
uses a plugin architecture and its render engines are
built as plugins that are loaded at run time using
[Gazebo Plugin](https://gazebosim.org/libs/plugin).

At the moment, Gazebo Rendering will only look for render engine plugin
shared libraries installed within its `<install_prefix>/lib` directory.
Likewise, the resources used by these engines are located in Gazebo
Rendering's `<install_prefix>/share` directory.

### Sensors

Each unique type of sensor in
[Gazebo Sensors](https://gazebosim.org/libs/sensors) is a plugin. When
a particular sensor type is requested, the relevant plugin is loaded by
[Gazebo Plugin](https://gazebosim.org/libs/plugin) and a
sensor object is instantiated from it.

At the moment, Gazebo Sensors will only look for sensor plugin
shared libraries installed within its `<install_prefix>/lib` directory.

## Models, lights, actors

Top-level entities such as models, lights and actors may be loaded through:

* Tags in SDF world files:
    * `<world><model>`
    * `<world><light>`
    * `<world><actor>`
    * `<include><uri>` (path / URL)
* The `/world/<world_name>/create` service:
    * SDF file as string (`<model>` / `<light>` / `<actor>` root)
    * Path / URL to SDF file
    * (TODO) `gz::msgs::Model`, `gz::msgs::Light`
* Within a system, using
  [SdfEntityCreator](https://gazebosim.org/api/gazebo/4.6/classignition_1_1gazebo_1_1SdfEntityCreator.html)
  or directly creating components and entities.

Gazebo will look for URIs (path / URL) in the following, in order:

1. All paths on the `IGN_GAZEBO_RESOURCE_PATH`\* environment variable (if
   path is URI, scheme is stripped)
2. Current running path / absolute path
3. [Gazebo Fuel](https://app.gazebosim.org/fuel/models)
    1. Cache (i.e. `$HOME/.ignition/fuel`)
    2. Web server

\* The `SDF_PATH` environment variable also works in some scenarios, but
  it's not recommended when using Gazebo.

## Meshes

Mesh files may be loaded through:

* Tags in SDF files:
    * `<geometry><mesh><uri>`
    * `<actor><skin><filename>`
    * `<actor><animation><filename>`

Gazebo will look for URIs (path / URL) in the following, in order:

1. Current running path / absolute path
2. All paths on the `IGN_GAZEBO_RESOURCE_PATH`\* environment variable (if path
   is URI, scheme is stripped)

\* The `IGN_FILE_PATH` environment variable also works in some scenarios, but
  it's not recommended when using Gazebo.

### GUI configuration

Gazebo Sim's
[GUI configuration](https://gazebosim.org/api/gui/4.2/config.html)
can come from the following, in order:

1. The command line option `--gui-config <file path>`
2. Plugins within SDF's `<world><gui>`
3. `$HOME/.gz/sim/<#>/gui.config` (if that file doesn't
exist, the default `gui.config` file that is installed with Gazebo
will be copied to that location)

