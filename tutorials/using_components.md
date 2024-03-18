\page usingcomponents Using Components in a System Plugin

Gazebo uses the entity component system (ECS) software architecture.
See the [Terminology](./terminology.html) page for the definitions of entity,
component, system, and entity component manager (ECM) used in this tutorial.
In short, a simulation world consists of many entities, each of which is
associated with a set of components.

System plugins can modify the simulation world by manipulating components.
The basic structure of a system plugin is outlined in the
[tutorial on creating system plugins](./createsystemplugins.html).

Here, we will explain how a system can use components to modify the world.
You can view the list of \ref gz::sim::components in the API.

Programmatic usage of components can be found in
[built-in systems](https://github.com/gazebosim/gz-sim/tree/gz-sim8/src/systems)
and [integration tests](https://github.com/gazebosim/gz-sim/blob/gz-sim8/test/integration)
in the source code.
Most of the built-in systems have a corresponding example SDF world.
You can find all the example worlds [here](https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds).

## Prerequisites

This is a tutorial for developers or advanced users.
It assumes that you are familiar with basic usage of Gazebo.

Prior to starting this tutorial, these other tutorials will help with
understanding:
- [Terminology](./terminology.html)
- [Create system plugins](./createsystemplugins.html)

## Resources

Quick access to resources mentioned in this tutorial:
- List of \ref gz::sim::components in the API
- List of \ref gz::sim::systems in the API
- Source code of [built-in systems](https://github.com/gazebosim/gz-sim/tree/gz-sim8/src/systems)
- Source code of [example worlds](https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds)
- Source code of [integration tests](https://github.com/gazebosim/gz-sim/blob/gz-sim8/test/integration)

## Entity Component Manager (ECM)

The gateway to interact with entities is through the
\ref gz::sim::EntityComponentManager
([source code](https://github.com/gazebosim/gz-sim/blob/gz-sim8/include/gz/sim/EntityComponentManager.hh)),
ECM for short.
The ECM gives us access to all the entities, each of which gives us access
to its associated components.

An ECM object is passed into all of the interfaces in a system, including
`ISystemConfigure()` and `ISystem*Update()`.
The signatures of those interfaces are specified in
\ref gz/sim/System.hh and explained in the
[tutorial on creating system plugins](./createsystemplugins.html).
We will assume that these interfaces are implemented in functions called
`Configure()` and `*Update()` in a system.

Note that when `Configure()` is called, all the elements in the parent element
of the plugin have been loaded.
For example, if the plugin is attached to a `<model>`, all the elements in that
`<model>` would have been loaded.
Similarly for `<world>`.
However, if you need to access entities outside the plugin's parent element,
they may not have finished loading at the time the plugin's `Configure()` is
called.
Then you may need to access those entities later, in `*Update()`.

## Case studies

The rest of the tutorial is case studies that walk through the usage of
specific components.

- \subpage jointforcecmdcomponent "JointForceCmd"
- \subpage posecomponent "Pose"
