\page createsystemplugins Create System Plugins

In Gazebo, all systems are loaded as plugins at runtime. Each system
is associated with an entity in simulation. Systems can be attached to the
following entity types:

* World
* Model
* Sensor
* Actor

To create a system plugin for use in the simulation environment, follow the
steps below.

## Decide on interfaces to implement

The first step of implementing a system plugin is to determine the subset of
available interfaces to implement. Aside from the base `System` object,
there are currently four additional available interfaces:

1. ISystemConfigure
  1. Has read-write access to world entities and components.
  2. Executed once the moment the plugin is loaded.
  3. Can be used to get custom configuration from the SDF file, register events
     with the event manager, as well as modifying entities and components.
2. ISystemPreUpdate
  1. Has read-write access to world entities and components.
  2. This is where systems say what they'd like to happen at time gz::sim::UpdateInfo::simTime.
  3. Can be used to modify state before physics runs, for example for applying control signals or performing network synchronization.
3. ISystemUpdate
  1. Has read-write access to world entities and components.
  2. Used for physics simulation step (i.e., simulates what happens at time gz::sim::UpdateInfo::simTime).
4. ISystemPostUpdate
  1. Has read-only access to world entities and components.
  2. Captures everything that happened at time gz::sim::UpdateInfo::simTime.
  3. Used to read out results at the end of a simulation step to be used for sensor or controller updates.
5. ISystemReset
  1. Has read-write access to world entities and components.
  2. Executed once the moment the plugin is reseted.

It's important to note that gz::sim::UpdateInfo::simTime does not refer to the current time, but the time reached after the `PreUpdate` and `Update` calls have finished.
So, if any of the `*Update` functions are called with simulation paused, time does not advance, which means the time reached after `PreUpdate` and `Update` is the same as the starting time.
This explains why gz::sim::UpdateInfo::simTime is initially 0 if simulation is started paused, while gz::sim::UpdateInfo::simTime is initially gz::sim::UpdateInfo::dt if simulation is started un-paused.

Systems that are only used to read the current state of the world (sensors,
graphics, and rendering) should implement `ISystemPostUpdate`.

Generally, systems that do not manage physics stepping will not need to
implement the `ISystemUpdate` interface.

Controllers and systems that provide feedback based on the state of the
world will need to implement `ISystemPostUpdate` to read the state at the
end of an update frame, as well as `ISystemPreUpdate` to provide feedback at
the beginning of the next frame.

## Implement Header

The header should include the `System` header:

Your System object should inherit from the `System` object as well as from
any interfaces that it provides.  It should then implement the corresponding
methods from the inherited interfaces.

\snippet examples/plugin/system_plugin/SampleSystem.hh header

## Register Plugin

If the library will only contain one plugin:

\snippet examples/plugin/system_plugin/SampleSystem.cc registerSampleSystem

If the library will contain multiple plugins, in one implementation do as
above, and then for each successive implementation use:

\snippet examples/plugin/system_plugin/SampleSystem2.cc registerSampleSystem2

## Implement Source

Implement the system class as usual, for example:

\snippet examples/plugin/system_plugin/SampleSystem.cc implementSampleSystem

## Setup the build

In your `CMakeLists.txt` add the following

```
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

# Add sources for each plugin to be registered.
add_library(SampleSystem SampleSystem.cc SampleSystem2.cc)
set_property(TARGET SampleSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(SampleSystem
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)
```

## Loading your plugin

In the SDF file representing your simulation, add the plugin to the `world` section:

```{.xml}
<sdf version="1.6">
  <world name="default">
    ...
    <plugin
      filename="SampleSystem"
      name="sample_system::SampleSystem">
    </plugin>
    <plugin
      filename="SampleSystem"
      name="sample_system::SampleSystem2">
    </plugin>
    ...
```
Plugins can be found using the `GZ_SIM_SYSTEM_PLUGIN_PATH` environment variable. If you
are using colcon to build your package consider using hooks to add your system to
`GZ_SIM_SYSTEM_PLUGIN_PATH`.
