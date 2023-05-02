\page migrationjointapi

# Migration from Gazebo-classic: Joint API

When migrating plugins from Gazebo-classic to Gazebo, developers will
notice that the C++ APIs for both simulators are quite different. Be sure to
check the [plugin migration tutorial](migrationplugins.html) to get a high-level
view of the architecture differences before using this guide.

This tutorial is meant to serve as a reference guide for developers migrating
functions from the
[gazebo::phyiscs::Joint](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Joint.html)
class.

If you're trying to use some API which doesn't have an equivalent on Gazebo
yet, feel free to
[ticket an issue](https://github.com/gazebosim/gz-sim/issues/).

## Joint API

Gazebo-classic's `gazebo::physics::Joint` provides lots of functionality, which
can be divided in these categories:

* **Properties**: Setting / getting properties
    * Example: [Joint::Anchor](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Joint.html#a73e5d867a0cbe1dff76957425a58c9d0) / [Joint::SetAnchor](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Joint.html#a9698c1294d75ae76026842a35750afa5)
* **Read family**: Getting children and parent
    * Example: [Joint::GetChild](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#a3e8e42332400d79fbeb7f721350ab980)
* **Write family**: Adding children, changing parent
    * Example: [Joint::RemoveChildren](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#aa85d2386e6fb02bdbec060a74b63238a)
* **Lifecycle**: Functions to control the joint's lifecycle
    * Example: [Joint::Init](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Joint.html#ae048ef824aaf614707c1496a2aefd415)

You'll find the Gazebo APIs below on the following headers:

* [ignition/gazebo/Joint.hh](https://gazebosim.org/api/gazebo/7/Joint_8hh.html)
* [ignition/gazebo/Util.hh](https://gazebosim.org/api/gazebo/7/Util_8hh.html)
* [ignition/gazebo/SdfEntityCreator.hh](https://gazebosim.org/api/gazebo/7/SdfEntityCreator_8hh.html)
* [ignition/gazebo/EntityComponentManager.hh](https://gazebosim.org/api/gazebo/7/classignition_1_1gazebo_1_1EntityComponentManager.html)

It's worth remembering that most of this functionality can be performed using
the
[EntityComponentManager](https://gazebosim.org/api/gazebo/7/classignition_1_1gazebo_1_1EntityComponentManager.html)
directly.

As an example the `Join::Pose()` is a convienient function for querying the `Pose` component from the `EntityComponentManager`, i.e.

```
  math::Pose3d pose = _ecm.Component<components::Pose>(jointEntityId)->Data();
```

The functions presented in the sections below exist for convenience and
readability. The items marked as `TODO` means that the equivalent API is not
implemented yet in Gazebo.

### Properties

Most of Gazebo-classic's Joint API is related to setting and getting
properties. These functions are great candidates to have equivalents on
Gazebo, because the Entity-Component-System architecture is perfect for setting
components (properties) into entities such as joints.

---

Classic | Gazebo
-- | --
AddType | `ecm.CreateComponent<Type>(entity, Type())`
Anchor | TODO
AnchorErrorPose | TODO
ApplyStiffnessDamping | TODO
AreConnected | TODO
AxisFrame | TODO
AxisFrameOffset | TODO
CacheForceTorque | TODO
CheckAndTruncateForce | TODO
DOF | TODO
FillMsg | TODO
GetDamping | TODO
GetEffortLimit | TODO
GetForce | `ignition::gazebo::Joint::TransmittedWrench`
GetForceTorque |  `ignition::gazebo::Joint::TransmittedWrench`
GetId | `ignition::gazebo::Joint::Entity`
GetInertiaRatio | TODO
GetJointLink | use `ignition::gazebo::Joint::*LinkName`
GetMsgType | TODO
GetName | `ignition::gazebo::Joint::Name`
GetParam | TODO
GetSaveable | Not supported
GetScopedName | `ignition::gazebo::scopedName`
GetSDF | TODO
GetSDFDom | TODO
GetSpringReferencePosition | TODO
GetStiffness | TODO
GetStopDissipation | TODO
GetStopStiffness | TODO
GetType | `ignition::gazebo::Joint::Type`
GetVelocity | `ignition::gazebo::Joint::Velocity`
GetVelocityLimit | TODO
GetWorldEnergyPotentialSpring | TODO
GlobalAxis | TODO
HasType | `gazebo::components::Joint::typeId == entityTypeId(entity, ecm)`
InertiaRatio | TODO
InitialAnchorPose | TODO
IsSelected | Selection is client-specific, not porting
LinkForce | TODO
LinkTorque | TODO
LocalAxis | `ignition::gazebo::Joint::Axis`
LowerLimit | TODO
ParentWorldPose | TODO
Position | `ignition::gazebo::Joint::Position`
Print | TODO
ResolveAxisXyz | TODO
SDFPoseRelativeToParent | `ignition::gazebo::Joint::Pose`
SDFSemanticPose | `ignition::gazebo::Joint::Pose`
SetAnchor | TODO
SetAxis | TODO
SetDamping | TODO
SetEffortLimit | `ignition::gazebo::Joint::SetEffortLimits`
SetForce | `ignition::gazebo::Joint::SetForce`
SetLowerLimit | `ignition::gazebo::Joint::SetPositionLimits`
SetName | TODO
SeParam | TODO
SetPosition | `ignition::gazebo::Joint::ResetPosition`
SetProvideFeedback | `ignition::gazebo::Joint::EnableTransmittedWrenchCheck`
SetSaveable | Not supported
SetSelected |  Selection is client-specific, not porting
SetState | TODO
SetStiffness | TODO
SetStiffnessDamping | TODO
SetStopDissipation | TODO
SetStopStiffness | TODO
SetUpperLimit | `ignition::gazebo::Joint::SetPositionLimits`
SetVelocity | `ignition::gazebo::Joint::SetVelocity`
SetVelocityLimit | `ignition::gazebo::Joint::SetVelocityLimits`
TypeStr | `ignition::gazebo::Joint::Type`
UpdateParameters | TODO
UpperLimit | TODO
URI | TODO
WorldPose | TODO

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
GetByName | Use type-specific `ignition::gazebo::Joint::*ByName`
GetChild | Use type-specific `ignition::gazebo::Joint::*ByName`
GetChild (Child link) | `ignition::gazebo::Joint::ChildLinkName`
GetChildCount | Use type-specific `ignition::gazebo::Joint::*Count`
GetParent | `ignition::gazebo::Joint::ParentModel`
GetParent (Parent link) | `ignition::gazebo::Joint::ParentLinkName`
GetParentId | `ignition::gazebo::EntiyComponentManager::ParentEntity`
GetWorld |  `ignition::gazebo::worldEntity`

---

## Write family

These functions deal with modifying the entity tree, attaching children to new
parents. Note that APIs for changing a Joint's entity tree structure are
currently not implemented yet.

---

Classic | Gazebo
-- | --
Attach | TODO
Detach | TODO
RemoveChild | TODO
RemoveChildren | TODO
SetModel | TODO
SetParent | TODO
SetWorld | TODO

---

## Lifecycle

These functions aren't related to the state of a joint, but perform some
processing related to the joint's lifecycle, like initializing, updating or
terminating it.

---

Classic | Gazebo
-- | --
ConnectJointUpdate | TODO
Fini | N/A
Init | N/A
Load | `ignition::gazebo::SdfEntityCreator::CreateEntities`
Reset | `ignition::gazebo::Joint::ResetPosition` / `ignition::gazebo::Joint::ResetVelocity`
Update | Entities are updated by systems

---
