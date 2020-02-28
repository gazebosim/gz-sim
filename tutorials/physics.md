\page physics Physics engines

Ignition Gazebo supports choosing what physics engine to use at runtime.
This is made possible by
[Ignition Physics](https://ignitionrobotics.org/libs/physics)' abstraction
layer.

Ignition Gazebo uses the [DART](https://dartsim.github.io/) physics engine
by default.

Downstream developers may also integrate other physics engines by creating new
Ignition Physics engine plugins. See
[Ignition Physics](https://ignitionrobotics.org/api/physics/2.0/tutorials.html)'s
tutorials to learn how to integrate a new engine.

## How Gazebo finds engines

Ignition Gazebo automatically looks for all physics engine plugins that are
installed with Ignition Physics. At the moment, that's only DART
(`ignition-physics-dartsim-plugin`).

If you've created a custom engine plugin, you can tell Gazebo where to find it
by setting the `IGN_GAZEBO_PHYSICS_ENGINE_PATH` environment variable to the
directory where the plugin's shared library can be found.

For example, if you've created the following physics engine shared library on
Linux:

`/home/physics_engines/libCustomEngine.so`

You should set the variable as follows:

`export IGN_GAZEBO_PHYSICS_ENGINE_PATH=/home/physics_engines`


If you have several libraries installed in different paths, you can add more
paths, for example:

`export IGN_GAZEBO_PHYSICS_ENGINE_PATH=/home/physics_engines:/home/more_engines`

## Tell Gazebo what engine to load

There are a few different ways of telling Gazebo which engine to load.

For any method, you should provide the name of your plugin's shared library,
but the `lib` prefix and the file extension are optional. So in this example,
the file is `libCustomEngine.so` but it's enough to set `CustomEngine`.

### From SDF

You can tell Gazebo what engine to load from the SDF world file by giving the
shared library name to the `Physics` plugin tag.

For the example above, you can load it like this:

```{.xml}
<plugin
  filename="libignition-gazebo-physics-system.so"
  name="ignition::gazebo::systems::Physics">
  <engine>
    <filename>CustomEngine</filename>
  </engine>
</plugin>
```

### From the command line

Alternatively, you can choose a plugin from the command line using the
`--physics-engine` option, for example:

`ign gazebo --physics-engine CustomEngine`

### From C++ API

All features available through the command line are also available through
[ignition::gazebo::ServerConfig](https://ignitionrobotics.org/api/gazebo/2.0/classignition_1_1gazebo_1_1ServerConfig.html).
When instantiating a server programmatically, a physics engine can be passed
to the constructor, for example:

```
ignition::gazebo::ServerConfig serverConfig;
serverConfig.SetPhysicsEngine("CustomEngine");

ignition::gazebo::Server server(serverConfig);
```

## Troubleshooting

> Failed to find plugin [libCustomEngine.so]. Have you checked the
> IGN_GAZEBO_PHYSICS_ENGINE_PATH environment variable?

Ignition Gazebo can't find out where `libCustomEngine.so` is located.

If that's an engine you believe should be installed with Ignition Physics,
check if the relevant plugin is installed.

If that's a 3rd party engine, find where the `.so` file is installed and add
that path to the environment variable as described above.

> Unable to load the [/home/physics_engines/libCustomEngine.so] library.

There was some problem loading that file. Check that it exists, that you have
permissions to access it, and that it's acually a physics engine plugin.

> No plugins with all required features found in library
> [/home/physics_engines/libCustomEngine.so]

This means that there are plugins on that library, but none of them satisfies
the minimum requirement of features needed to run an Ignition Gazebo simulation.
Be sure to implement all the necessary features.

> Failed to load a valid physics engine from
> [/home/physics_engines/libCustomEngine.so]

Some engines were found in that library, but none of them could be loaded. Check
that the the engines implement all the necessary features.

