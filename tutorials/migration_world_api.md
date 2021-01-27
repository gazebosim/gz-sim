\page migrationworldapi

# Migration from Gazebo-classic: World API

When migrating plugins from Gazebo-classic to Ignition Gazebo, developers will
notice that the C++ APIs for both simulators are quite different. Be sure to
check the [plugin migration tutorial](migrationplugins.html) to get a high-level
view of the architecture differences before using this guide.

This tutorial is meant to serve as a reference guide for developers migrating
functions from the
[gazebo::physics::World](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1World.html)
class.

If you're trying to use some API which doesn't have an equivalent on Ignition
yet, feel free to
[ticket an issue](https://github.com/ignitionrobotics/ign-gazebo/issues/).

## World API

Gazebo-classic's `gazebo::physics::World` provides lots of functionality, which
can be divided in these categories:

* **Properties**: Setting / getting properties
    * Example: [World::Gravity](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1World.html#a700b50d9b34e470cb1b5fbbd33625b1e) / [World::SetGravity](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1World.html#aff59ef61889e38ef3c2f09bda0c7cfbf)
* **Read family**: Getting children
    * Example: [World::Lights](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1World.html#a64a748a669cf5cb42bb3794afab6fa53)
* **Write family**: Adding children
    * Example: [World::Clear](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1World.html#aa71d36872f416feaa853788a7a7a7ef8)
* **Lifecycle**: Functions to control the world's lifecycle
    * Example: [World::Step](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1World.html#af272078d98d7f24a1f8949993d2d5493)
* **Others**: Functions that don't fit any of the categories above
    * Example: [World::UniqueModelName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1World.html#a05934164af0cf95eb5a4be70e726846d)

You'll find the Ignition APIs below on the following headers:

* [ignition/gazebo/World.hh](https://ignitionrobotics.org/api/gazebo/3.3/World_8hh.html)
* [ignition/gazebo/Util.hh](https://ignitionrobotics.org/api/gazebo/3.3/Util_8hh.html)
* [ignition/gazebo/SdfEntityCreator.hh](https://ignitionrobotics.org/api/gazebo/3.3/SdfEntityCreator_8hh.html)

It's worth remembering that most of this functionality can be performed using
the
[EntityComponentManager](https://ignitionrobotics.org/api/gazebo/3.3/classignition_1_1gazebo_1_1EntityComponentManager.html)
directly. The functions presented here exist for convenience and readability.

### Properties

Most of Gazebo-classic's World API is related to setting and getting
properties. These functions are great candidates to have equivalents on Ignition
Gazebo, because the Entity-Component-System architecture is perfect for setting
components (properties) into entities such as worlds.

Classic | Ignition
-- | --
Atmosphere | `ignition::gazebo::World::Atmosphere`
AtmosphereEnabled | TODO
DisableAllModels | TODO
EnableAllModels | TODO
GetSDFDom | TODO
Gravity | `ignition::gazebo::World::Gravity`
IsLoaded | Not applicable
IsPaused | Use `ignition::gazebo::UpdateInfo`
Iterations | Use `ignition::gazebo::UpdateInfo`
MagneticField | `ignition::gazebo::World::MagneticField`
Name | `ignition::gazebo::World::Name`
PauseTime | Use `ignition::gazebo::UpdateInfo`
Physics | TODO
PhysicsEnabled | TODO
PresetMgr | TODO
PublishLightPose | Use `ignition::gazebo::systems::PosePublisher`
PublishModelPose | Use `ignition::gazebo::systems::PosePublisher`
PublishModelScale | TODO
RealTime | Use `ignition::gazebo::UpdateInfo`
Running | Not applicable
SDF | TODO
SetAtmosphereEnabled | TODO
SetGravity | TODO
SetGravitySDF | TODO
SetMagneticField | TODO
SetPaused | Use world control service
SetPhysicsEnabled | TODO
SetSimTime | Use world control service
SetState | TODO
SetWindEnabled | TODO
SimTime | Use `ignition::gazebo::UpdateInfo`
SphericalCoords | TODO
StartTime | Use `ignition::gazebo::UpdateInfo`
URI | TODO
UpdateStateSDF | TODO
Wind | TODO
WindEnabled | TODO

## Read family

These APIs deal with reading information related to child / parent
relationships.

The main difference in these APIs across Gazebo generations is that
on classic, they deal with shared pointers to entities, while on Ignition,
they deal with entity IDs.

Classic | Ignition
-- | --
BaseByName | Use type-specific `ignition::gazebo::World::*ByName`
EntityByName | Use type-specific `ignition::gazebo::World::*ByName`
LightByName | `ignition::gazebo::World::LightByName`
LightCount |  `ignition::gazebo::World::LightCount`
Lights | `ignition::gazebo::World::Lights`
ModelByIndex |  `ignition::gazebo::World::ModelByName`
ModelByName | `ignition::gazebo::World::ModelByName`
ModelCount | `ignition::gazebo::World::ModelCount`
Models | `ignition::gazebo::World::Models`
PrintEntityTree | Use scene graph service

## Write family

These functions deal with modifying the entity tree, attaching children to new
parents.

Classic | Ignition
-- | --
Clear | TODO
ClearModels | TODO
InsertModelFile | TODO
InsertModelSDF | `ignition::gazebo::SdfEntityCreator::CreateEntities`
InsertModelString | TODO
RemoveModel | TODO
RemovePlugin | TODO

## Lifecycle

These functions aren't related to the state of a model, but perform some
processing related to the model's lifecycle, like initializing, updating or
terminating it.

Classic | Ignition
-- | --
Fini | N/A
Init | N/A
Load | `ignition::gazebo::SdfEntityCreator::CreateEntities`
LoadLight | `ignition::gazebo::SdfEntityCreator::CreateEntities`
LoadPlugin | TODO
Reset | TODO
ResetEntities | TODO
ResetPhysicsStates | TODO
ResetTime | Use world control service
Run | See server API
RunBlocking | See server API
SensorsInitialized | N/A
Step | See server API
Stop | See server API
_AddDirty | N/A
_SetSensorsInitialized | N/A

## Others

Miscelaneous functions that don't fit the other categories. Most of them involve
logic that should be performed from within a system.

Classic | Ignition
-- | --
EntityBelowPoint | Requires a system
ModelBelowPoint | Requires a system
SceneMsg | Use `ignition::gazebo::systems::SceneBoradcaster`
WorldPoseMutex | N/A
StripWorldName | N/A
UniqueModelName | TODO
Save | Use SDF generator
