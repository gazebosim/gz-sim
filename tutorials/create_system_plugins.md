\page createsystemplugins Create System Plugins

In Ignition Gazebo, all systems are loaded as plugins at runtime.  To create
a system plugin for use in the simulation environment, follow the steps
below.

## Decide on interfaces to implement

The first step of implementing a system plugin is to determine the subset of
available interfaces to implement.  Aside from the base `System` object,
there are currently three additional available interfaces:

1. ISystemPreUpdate
  1. Has read-write access to world entities and components.
  2. Executed with simulation time at (t0).
  3. Can be used to modify state before physics runs, for example for applying control signals or performing network synchronization.
2. ISystemUpdate
  1. Has read-write access to world entities and components.
  2. Responsible for propagating time from (t0) to (t0 + dt).
  3. Used for physics simulation step.
3. ISystemPostUpdate
  1. Has read-only access to world entities and components.
  2. Executed with simulation time at (t0 + dt).
  3. Used to read out results at the end of a simulation step to be used for sensor or controller updates.

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

## Implement Source

Implement the system class as normal

## Register Plugin

If the library will only contain one plugin:

\snippet examples/plugin/system_plugin/SampleSystem.cc registerSampleSystem

If the library will contain multiple plugins, in one implementation do as
above, and then for each successive implementation use:

\snippet examples/plugin/system_plugin/SampleSystem2.cc registerSampleSystem2

## Setup the build

In your `CMakeLists.txt` add the following

```
set(IGN_PLUGIN_VER 0)
ign_find_package(ignition-plugin0 REQUIRED COMPONENTS register)

# Add sources for each plugin to be registered.
add_library(SampleSystem SampleSystem.cc SampleSystem2.cc)
set_property(TARGET SampleSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(SampleSystem
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
)
```

## Loading your plugin

In the SDF file representing your simulation, add the plugin to the `world` section:

```{.xml}
<sdf version="1.6">
  <world name="default">
    ...
    <plugin
      filename="libSampleSystem.so"
      name="sample_system::SampleSystem">
    </plugin>
    <plugin
      filename="libSampleSystem.so"
      name="sample_system::SampleSystem2">
    </plugin>
    ...
```
