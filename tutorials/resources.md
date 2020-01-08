\page resources Finding resources

Setting up and running a simulation can involve loading various kinds of
resources, such as robot models and plugins, from different locations, which
can include a local filesystem and online servers. Ignition Gazebo offers
a few different mechanisms for locating required resources.

## Plugins

A plugin is a shared library that adheres to a specific API and is loaded at
runtime. Typically, plugins are scoped to perform a narrow set of features.
For example, the `diff_drive` plugin, provided by Ignition Gazebo, implements
a differential drive controller for mobile robots.

Ignition relies on plugins for rendering, physics simulation, sensor data
generation, and many of the capabilities. The following sections describe
how Ignition finds and loads different types of plugins.

### System Plugins

A system plugin is used by Ignition Gazebo, and provides an entry point for
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
  [PluginInfo](https://ignitionrobotics.org/api/gazebo/3.0/classignition_1_1gazebo_1_1ServerConfig_1_1PluginInfo.html)
  (within [ServerConfig](https://ignitionrobotics.org/api/gazebo/3.0/classignition_1_1gazebo_1_1ServerConfig.html))
  when instantiating the
  [Server](https://ignitionrobotics.org/api/gazebo/3.0/classignition_1_1gazebo_1_1Server.html#a084ef7616f5af42061a7aeded5651ab0).

Ignition will look for system plugins on the following paths, in order:

1. All paths on the `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` environment variable
2. `$HOME/.ignition/gazebo/plugins`
3. [Systems that are installed with Ignition Gazebo](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/default/src/systems/)

### Ignition GUI plugins

Each [Ignition GUI](https://ignitionrobotics.org/libs/rendering) plugin
defines a widget.

GUI plugins may be loaded through:

* Tags in SDF world files, where `filename` is the shared library:
    * `<world><gui><plugin>`
* Tags in [GUI config files](https://ignitionrobotics.org/api/gui/3.0/config.html),
  where `filename` is the shared library:
    * `<plugin>`
* The plugin menu on the top-right of the screen.

Ignition will look for GUI plugins on the following paths, in order:

1. All paths set on the `IGN_GUI_PLUGIN_PATH` environment variable
2. [GUI plugins that are installed with Ignition Gazebo](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/default/src/systems/)
3. Other paths added by calling `ignition::gui::App()->AddPluginPath`
4. `~/.ignition/gui/plugins`
5. [Plugins which are installed with Ignition GUI](https://ignitionrobotics.org/api/gui/3.0/namespaceignition_1_1gui_1_1plugins.html)

### Physics engines

[Ignition Physics](https://ignitionrobotics.org/libs/physics)
uses a plugin architecture and its physics engines are
built as plugins that are loaded at run time using
[Ignition Plugin](https://ignitionrobotics.org/libs/plugin).

At the moment, Ignition Physics will only look for physics engine plugin
shared libraries installed within its `<install_prefix>/lib` directory.

### Rendering engines

[Ignition Rendering](https://ignitionrobotics.org/libs/rendering)
uses a plugin architecture and its render engines are
built as plugins that are loaded at run time using
[Ignition Plugin](https://ignitionrobotics.org/libs/plugin).

At the moment, Ignition Rendering will only look for render engine plugin
shared libraries installed within its `<install_prefix>/lib` directory.
Likewise, the resources used by these engines are located in Ignition
Rendering's `<install_prefix>/share` directory.

### Sensors

Each unique type of sensor in
[Ignition Sensors](https://ignitionrobotics.org/libs/sensors) is a plugin. When
a particular sensor type is requested, the relevant plugin is loaded by
[Ignition Plugin](https://ignitionrobotics.org/libs/plugin) and a
sensor object is instantiated from it.

At the moment, Ignition Sensors will only look for sensor plugin
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
    * (TODO) `ignition::msgs::Model`, `ignition::msgs::Light`
* Within a system, using
  [SdfEntityCreator](https://ignitionrobotics.org/api/gazebo/3.0/classignition_1_1gazebo_1_1SdfEntityCreator.html)
  or directly creating components and entities.

Ignition will look for URIs (path / URL) in the following, in order:

1. All paths on the `SDF_PATH` environment variable (if path is URI,
   scheme is stripped)
2. Current running path / absolute path
3. [Ignition Fuel](https://app.ignitionrobotics.org/fuel/models)
    1. Cache (i.e. `$HOME/.ignition/fuel`)
    2. Web server

## Meshes

Mesh files may be loaded through:

* Tags in SDF files:
    * `<geometry><mesh><uri>`
    * `<actor><skin><filename>`
    * `<actor><animation><filename>`

Ignition will look for URIs (path / URL) in the following, in order:

1. Current running path / absolute path
2. All paths on the `IGN_FILE_PATH` environment variable (if path is URI,
   scheme is stripped)

### GUI configuration

Ignition Gazebo's
[GUI configuration](https://ignitionrobotics.org/api/gui/3.0/config.html)
can come from the following, in order:

1. The command line option `--gui-config <file path>`
2. Plugins within SDF's `<world><gui>`
3. `$HOME/.ignition/gazebo/gui.config` (if that file doesn't
exist, the default `gui.config` file that is installed with Ignition Gazebo
will be copied to that location)

