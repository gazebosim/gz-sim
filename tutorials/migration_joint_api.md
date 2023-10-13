\page migrationjointapi Migration from Gazebo-classic: Joint API

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

* \ref gz/sim/Joint.hh
* \ref gz/sim/Util.hh
* \ref gz/sim/SdfEntityCreator.hh
* \ref gz/sim/EntityComponentManager.hh

It's worth remembering that most of this functionality can be performed using
the \ref gz::sim::EntityComponentManager "EntityComponentManager" directly.

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
GetForce | `gz::sim::Joint::TransmittedWrench`
GetForceTorque |  `gz::sim::Joint::TransmittedWrench`
GetId | `gz::sim::Joint::Entity`
GetInertiaRatio | TODO
GetJointLink | use `gz::sim::Joint::*LinkName`
GetMsgType | TODO
GetName | `gz::sim::Joint::Name`
GetParam | TODO
GetSaveable | Not supported
GetScopedName | `gz::sim::scopedName`
GetSDF | TODO
GetSDFDom | TODO
GetSpringReferencePosition | TODO
GetStiffness | TODO
GetStopDissipation | TODO
GetStopStiffness | TODO
GetType | `gz::sim::Joint::Type`
GetVelocity | `gz::sim::Joint::Velocity`
GetVelocityLimit | TODO
GetWorldEnergyPotentialSpring | TODO
GlobalAxis | TODO
HasType | `gazebo::components::Joint::typeId == entityTypeId(entity, ecm)`
InertiaRatio | TODO
InitialAnchorPose | TODO
IsSelected | Selection is client-specific, not porting
LinkForce | TODO
LinkTorque | TODO
LocalAxis | `gz::sim::Joint::Axis`
LowerLimit | TODO
ParentWorldPose | TODO
Position | `gz::sim::Joint::Position`
Print | TODO
ResolveAxisXyz | TODO
SDFPoseRelativeToParent | `gz::sim::Joint::Pose`
SDFSemanticPose | `gz::sim::Joint::Pose`
SetAnchor | TODO
SetAxis | TODO
SetDamping | TODO
SetEffortLimit | `gz::sim::Joint::SetEffortLimits`
SetForce | `gz::sim::Joint::SetForce`
SetLowerLimit | `gz::sim::Joint::SetPositionLimits`
SetName | TODO
SeParam | TODO
SetPosition | `gz::sim::Joint::ResetPosition`
SetProvideFeedback | `gz::sim::Joint::EnableTransmittedWrenchCheck`
SetSaveable | Not supported
SetSelected |  Selection is client-specific, not porting
SetState | TODO
SetStiffness | TODO
SetStiffnessDamping | TODO
SetStopDissipation | TODO
SetStopStiffness | TODO
SetUpperLimit | `gz::sim::Joint::SetPositionLimits`
SetVelocity | `gz::sim::Joint::SetVelocity`
SetVelocityLimit | `gz::sim::Joint::SetVelocityLimits`
TypeStr | `gz::sim::Joint::Type`
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
GetByName | Use type-specific `gz::sim::Joint::*ByName`
GetChild | Use type-specific `gz::sim::Joint::*ByName`
GetChild (Child link) | `gz::sim::Joint::ChildLinkName`
GetChildCount | Use type-specific `gz::sim::Joint::*Count`
GetParent | `gz::sim::Joint::ParentModel`
GetParent (Parent link) | `gz::sim::Joint::ParentLinkName`
GetParentId | `gz::sim::EntiyComponentManager::ParentEntity`
GetWorld |  `gz::sim::worldEntity`

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
Load | `gz::sim::SdfEntityCreator::CreateEntities`
Reset | `gz::sim::Joint::ResetPosition` / `gz::sim::Joint::ResetVelocity`
Update | Entities are updated by systems

---
