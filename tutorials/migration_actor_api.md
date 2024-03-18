\page migrationactorapi Migration from Gazebo-classic: Actor API

When migrating plugins from Gazebo-classic to Gazebo, developers will
notice that the C++ APIs for both simulators are quite different. Be sure to
check the [plugin migration tutorial](migrationplugins.html) to get a high-level
view of the architecture differences before using this guide.

This tutorial is meant to serve as a reference guide for developers migrating
functions from the
[gazebo::phyiscs::Actor](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Actor.html)
class.

If you're trying to use some API which doesn't have an equivalent on Gazebo
yet, feel free to
[ticket an issue](https://github.com/gazebosim/gz-sim/issues/).

## Actor API

Gazebo-classic's `gazebo::physics::Actor` provides lots of functionality, which
can be divided in these categories:

* **Properties**: Setting / getting properties
    * Example: [Actor::GetName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#a9a98946a64f3893b085f650932c9dfee) / [Actor::SetName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Entity.html#a5d74ac4d7a230aed1ab4b11933b16e92)
* **Read family**: Getting children and parent
    * Example: [Actor::GetParentModel](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Entity.html#a94d6f01102b5d949006fba3628d9f355)
* **Write family**: Adding children, changing parent
    * Example: [Actor::RemoveChildren](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#aa85d2386e6fb02bdbec060a74b63238a)
* **Lifecycle**: Functions to control the actor's lifecycle
    * Example: [Actor::Init](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Actor.html#ae048ef824aaf614707c1496a2aefd415)
* **Others**: Functions that don't fit any of the categories above
    * Example: [Actor::PlaceOnEntity](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Entity.html#a9ecbfeb56940cacd75f55bed6aa9fcb4)

You'll find the Gazebo APIs below on the following headers:

* \ref gz/sim/Actor.hh
* \ref gz/sim/Util.hh
* \ref gz/sim/SdfEntityCreator.hh
* \ref gz/sim/EntityComponentManager.hh

It's worth remembering that most of this functionality can be performed using
the \ref gz::sim::EntityComponentManager "EntityComponentManager" directly.

As an example the `Actor::Pose()` is a convienient function for querying the `Pose` component from the `EntityComponentManager`, i.e.

```
  math::Pose3d pose = _ecm.Component<components::Pose>(actorEntityId)->Data();
```

The functions presented in the sections below exist for convenience and
readability. The items marked as `TODO` means that the equivalent API is not
implemented yet in Gazebo.

### Properties

Most of Gazebo-classic's Actor API is related to setting and getting
properties that are done via the Entity-Component-System by setting components
(properties) into entities such as actors. Note that a number of the public APIs
in Classic are inherited from the base Model / Entity class and they may not
have equivalents in Gazebo.

---

Classic | Gazebo
-- | --
AddType | `ecm.CreateComponent<Type>(entity, Type())`
AlignBvh | TODO
BoundingBox | TODO
CollisionBoundingBox | TODO
CustomTrajectory | see `SetCustomTrajectory`
DirtyPose | Not supported
FillMsg | TODO
GetAutoDisable | TODO
GetId | `gz::sim::Model::Entity`
GetName | `gz::sim::Actor::Name`
GetPluginCount | TODO
GetSaveable | Not supported
GetScopedName | `gz::sim::scopedName`
GetSDF | TODO
GetSDFDom | TODO
GetSelfCollide | TODO
GetType | `gz::sim::entityType`
GetWorldEnergy | TODO
GetWorldEnergyKinetic | TODO
GetWorldEnergyPotential | TODO
HasType | `gazebo::components::Link::typeId == entityTypeId(entity, ecm)`
InitialRelativePose | TODO
IsActive | TODO
IsCanonicalLink | Not applicable. See link API
IsSelected | Selection is client-specific, not porting
IsStatic | TODO
Mesh | TODO
Play | TODO
PluginInfo | TODO
Print | TODO
ProcessMsg | TODO
RelativeAngularAccel | TODO
RelativeAngularVel | TODO
RelativeLinearAccel | TODO
RelativeLinearVel | TODO
RelativePose | `gz::sim::Actor::Pose`
Scale | TODO
ScriptTime | `gz::sim::Actor::AnimationTime`
SDFPoseRelativeToParent | `gz::sim::Actor::Pose`
SDFSemanticPose | `gz::sim::Actor::Pose`
SensorScopedName | TODO
SetAngularVel | TODO
SetAnimation | use `gz::sim::Actor::SetTrajectoryPose`
SetAutoDisable | TODO
SetCollideMode | TODO
SetCustomTrajectory | use `gz::sim::Actor::SetTrajectoryPose`, `SetAnimationTime`, and `SetAnimationName` to achieve similar result.
SetEnabled | TODO
SetGravityMode | TODO
SetInitialRelativePose | TODO
SetJointAnimation | Not supported
SetJointPosition | Not supported
SetJointPositions | Not supported
SetLaserRetro | TODO
SetLinearVel | TODO
SetLinkWorldPose | TODO
SetName | TODO
SetRelativePose | TODO
SetSaveable | Not supported
SetScale | TODO
SetScriptTime | `gz::sim::Actor::SetAnimationTime`
SetSelected |  Selection is client-specific, not porting
SetSelfCollide | TODO
SetState | TODO
SetStatic | TODO
SetWindMode | TODO
SetWorldPose | TODO
SetWorldTwist | TODO
SkeletonAnimations | TODO
Stop | TODO
StopAnimation | use `gz::sim::Actor::SetTrajectoryPose`
TypeStr | `gz::sim::entityTypeStr`
UnscaledSDF | TODO
UpdateParamenters | TODO
URI | TODO
WindMode | TODO
WorldAngularAccel | TODO
WorldAngularVel | TODO
WorldLinearAccel | TODO
WorldLinearVel | TODO
WorldPose | `gz::sim::Actor::WorldPose`

---

## Read family

These APIs deal with reading information related to child / parent
relationships.

The main difference in these APIs across Gazebo generations is that
on classic, they deal with shared pointers to entities, while on Gazebo,
they deal with entity IDs.

---

Classic | Gazebo
-- | --
GetByName | TODO
GetChild | TODO
GetChildCollision |  TODO
GetChildCount | TODO
GetChildLink | TODO
GetGripper | Not supported
GetGripperCount | Not supported
GetJoint | Not supported
GetJointCount | Not supported
GetJoints | Not supported
GetLink | TODO
GetLinks | TODO
GetParent | `gz::sim::EntiyComponentManager::ParentEntity`
GetParentId | `gz::sim::EntiyComponentManager::ParentEntity`
GetParentModel | `gz::sim::EntiyComponentManager::ParentEntity`
GetSensorCount | TODO
GetWorld | const `gz::sim::worldEntity`
NestedModel | TODO
NestedModels | TODO

---

## Write family

These functions deal with modifying the entity tree, attaching children to new
parents. Note that APIs for changing an Actor's entity tree structure are
currently not implemented yet.

---

Classic | Gazebo
-- | --
AddChild | TODO
AttachStaticModel | TODO
DetachStaticModel | TODO
RemoveChild | TODO
RemoveChildren | TODO
RemoveJoint | TODO
SetCanonicalLink | TODO
SetParent | TODO
SetWorld | TODO

---

## Lifecycle

These functions aren't related to the state of a actor, but perform some
processing related to the actor's lifecycle, like initializing, updating or
terminating it.

---

Classic | Gazebo
-- | --
Fini | N/A
Init | N/A
Load | `gz::sim::SdfEntityCreator::CreateEntities`
LoadJoints | `gz::sim::SdfEntityCreator::CreateEntities`
LoadPlugins | TODO
Reset | TODO
ResetCustomTrajectory | TODO
ResetPhysicsStates | TODO
Update | Entities are updated by systems

---


## Others

Miscelaneous functions that don't fit the other categories. Most of them involve
logic that should be performed from within a system.

---

Classic | Gazebo
-- | --
GetJointController | Use this system: `gz::sim::systems::JointController`
GetNearestEntityBelow | Requires a system
PlaceOnEntity | Requires a system
PlaceOnNearestEntityBelow | Requires a system

---
