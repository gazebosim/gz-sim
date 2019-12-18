\page resources Finding resources

Setting up a simulation always involves loading various kinds of resources from
different places, such as robot models and plugins. Ignition Gazebo offers a
few different ways of looking up resources.

## Plugins

### Systems

System plugins may be loaded through:

* Tags in SDF files, where `filename` is the shared library and
  `name` is the class to be loaded.
    * Attached to world: `<world><plugin>`
    * Attached to model: `<model><plugin>`
    * Attached to sensor: `<sensor><plugin>`
* Passing their shared library and class to be loaded through
  [PluginInfo](https://ignitionrobotics.org/api/gazebo/3.0/classignition_1_1gazebo_1_1ServerConfig_1_1PluginInfo.html)
  (within [ServerConfig](https://ignitionrobotics.org/api/gazebo/3.0/classignition_1_1gazebo_1_1ServerConfig.html))
  when instantiating the
  [Server](https://ignitionrobotics.org/api/gazebo/3.0/classignition_1_1gazebo_1_1Server.html#a084ef7616f5af42061a7aeded5651ab0).

Ignition will look for system plugins on the following paths, in order:

1. All paths on the `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` environment variable
1. `$HOME/.ignition/gazebo/plugins`
1. [Systems that are installed with Ignition Gazebo](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/default/src/systems/)

### Ignition GUI plugins

GUI plugins may be loaded through:

* Tags in SDF world files, where `filename` is the shared library:
    * `<world><gui><plugin>`
* Tags in GUI config files, where `filename` is the shared library:
    * `<plugin>`
* The plugin menu on the top-right of the screen.

Ignition will look for GUI plugins on the following paths, in order:

1. All paths set on the `IGN_GUI_PLUGIN_PATH` environment variable
1. [GUI plugins that are installed with Ignition Gazebo](https://bitbucket.org/ignitionrobotics/ign-gazebo/src/default/src/systems/)
1. Other paths added by calling `ignition::gui::App()->AddPluginPath`
1. `~/.ignition/gui/plugins`
1. [Plugins which are installed with Ignition GUI](https://ignitionrobotics.org/api/gui/3.0/namespaceignition_1_1gui_1_1plugins.html)

### Physics engines

TODO

### Rendering engines

TODO

### Sensors

TODO

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
1. Current running path / absolute path
1. [Ignition Fuel](https://app.ignitionrobotics.org/fuel/models)
    1. Cache (i.e. `$HOME/.ignition/fuel`)
    1. Web server

## Meshes

Mesh files may be loaded through:

* Tags in SDF files:
    * `<geometry><mesh><uri>`
    * `<actor><skin><filename>`
    * `<actor><animation><filename>`

Ignition will look for URIs (path / URL) in the following, in order:

1. Current running path / absolute path
1. All paths on the `IGN_FILE_PATH` environment variable (if path is URI,
   scheme is stripped)

