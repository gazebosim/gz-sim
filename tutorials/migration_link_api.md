\page migrationlinkapi

# Migration from Gazebo-classic: Link API

When migrating plugins from Gazebo-classic to Gazebo, developers will
notice that the C++ APIs for both simulators are quite different. Be sure to
check the [plugin migration tutorial](migrationplugins.html) to get a high-level
view of the architecture differences before using this guide.

This tutorial is meant to serve as a reference guide for developers migrating
functions from the
[sim::phyiscs::Link](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Link.html)
class.

If you're trying to use some API which doesn't have an equivalent on Gazebo
yet, feel free to
[ticket an issue](https://github.com/gazebosim/gz-sim/issues/).

## Link API

Gazebo-classic's `sim::physics::Link` provides lots of functionality, which
can be divided in these categories:

* **Properties**: Setting / getting properties
    * Example: [Link::GetName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#a9a98946a64f3893b085f650932c9dfee) / [Link::SetName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Entity.html#a5d74ac4d7a230aed1ab4b11933b16e92)
* **Read family**: Getting children and parent
    * Example: [Link::GetCollision](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Link.html#ae52be77915eb1e972e7571a20e4ab562)
* **Write family**: Adding children, changing parent
    * Example: [Link::RemoveChildren](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#aa85d2386e6fb02bdbec060a74b63238a)
* **Lifecycle**: Functions to control the link's lifecycle
    * Example: [Link::Init](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Link.html#ae048ef824aaf614707c1496a2aefd415)
* **Others**: Functions that don't fit any of the categories above
    * Example: [Link::PlaceOnEntity](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Entity.html#a9ecbfeb56940cacd75f55bed6aa9fcb4)

You'll find the Gazebo APIs below on the following headers:

* [gz/sim/Link.hh](https://gazebosim.org/api/gazebo/7/Link_8hh.html)
* [gz/sim/Util.hh](https://gazebosim.org/api/gazebo/7/Util_8hh.html)
* [gz/sim/SdfEntityCreator.hh](https://gazebosim.org/api/gazebo/7/SdfEntityCreator_8hh.html)
* [gz/sim/EntityComponentManager.hh](https://gazebosim.org/api/gazebo/7/classignition_1_1gazebo_1_1EntityComponentManager.html)

It's worth remembering that most of this functionality can be performed using
the
[EntityComponentManager](https://gazebosim.org/api/gazebo/7/classignition_1_1gazebo_1_1EntityComponentManager.html)
directly. The functions presented here exist for convenience and readability.

### Properties

Most of Gazebo-classic's Link API is related to setting and getting
properties. These functions are great candidates to have equivalents on Gazebo
Gazebo, because the Entity-Component-System architecture is perfect for setting
components (properties) into entities such as links.

---

Classic | Gazebo
-- | --
AddForce | `gz::sim::Link::AddWorldForce`
AddForceAtRelativePosition | TODO
AddForceAtWorldPosition | TODO
AddLinkForce | TODO
AddRelativeForce | TODO
AddRelativeTorque | TODO
AddTorque | `gz::sim::Link::AddWorldWrench`
AddType | `ecm.CreateComponent<Type>(entity, Type())`
Battery | Use this system: `gz::sim::systems::LinearBatteryPlugin`
BoundingBox | TODO
CollisionBoundingBox | TODO
DirtyPose | Not supported
FillMsg | TODO
GetAngularDamping | TODO
GetEnabled | TODO
GetGravityMode | TODO
GetId | `gz::sim::Link::Entity`
GetInertial | `gz::sim::Link::WorldInertialPose` / `gz::sim::Link::WorldInertialMatrix`
GetKinematic | TODO
GetLinearDamping | TODO
GetName | `gz::sim::Link::Name`
GetSDF | TODO
GetSDFDom | TODO
GetSaveable | Not supported
GetScopedName | `gz::sim::scopedName`
GetSelfCollide | See model API
GetSensorName | See sensor API
GetType | `gz::sim::entityType`
GetWorldEnergy | TODO
GetWorldEnergyKinetic | `gz::sim::Link::WorldKineticEnergy`
GetWorldEnergyPotential | TODO
HasType | `sim::components::Link::typeId == entityTypeId(entity, ecm)`
InitialRelativePose | TODO
IsCanonicalLink | `gz::sim::Link::IsCanonical`
IsSelected | Selection is client-specific, not porting
IsStatic | See model API
MoveFrame | TODO
Print | TODO
ProcessMsg | TODO
RelativeAngularAccel | TODO
RelativeAngularVel | TODO
RelativeForce | TODO
RelativeLinearAccel | TODO
RelativeLinearVel | TODO
RelativePose | TODO
RelativeTorque | TODO
RelativeWindLinearVel | TODO
SDFPoseRelativeToParent | TODO
SDFSemanticPose | TODO
SetAngularDamping | TODO
SetAngularVel | TODO
SetAnimation | TODO
SetAutoDisable | TODO
SetCanonicalLink | TODO
SetCollideMode | TODO
SetEnabled | TODO
SetForce | TODO
SetGravityMode | TODO
SetInertial | TODO
SetInitialRelativePose | TODO
SetKinematic | TODO
SetLaserRetro | TODO
SetLinearDamping | TODO
SetLinearVel | TODO
SetLinkStatic | TODO
SetName | TODO
SetPublishData | TODO
SetRelativePose | TODO
SetSaveable | Not supported
SetScale | TODO
SetSelected |  Selection is client-specific, not porting
SetSelfCollide | TODO
SetState | TODO
SetStatic | TODO
SetTorque | TODO
SetVisualPose | See visual API
SetWindEnabled | TODO
SetWindMode | TODO
SetWorldPose | TODO
SetWorldTwist | TODO
StopAnimation | TODO
TypeStr | `gz::sim::entityTypeStr`
URI | TODO
UpdateParameters | TODO
VisualPose | See visual API
WindMode | `gz::sim::Link::WindMode`
WorldAngularAccel | `gz::sim::Link::WorldAngularAcceleration`
WorldAngularMomentum | TODO
WorldAngularVel | `gz::sim::Link::WorldAngularVelocity`
WorldCoGLinearVel | TODO
WorldCoGPose | TODO
WorldForce | TODO
WorldInertiaMatrix | `gz::sim::Link::WorldInertialMatrix`
WorldInertialPose | `gz::sim::Link::WorldInertialPose`
WorldLinearAccel | `gz::sim::Link::WorldLinearAcceleration`
WorldLinearVel | `gz::sim::Link::WorldLinearVelocity`
WorldPose |  `gz::sim::Link::WorldPose`
WorldTorque | TODO
WorldWindLinearVel | TODO

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
BatteryCount | Use this system: `gz::sim::systems::LinearBatteryPlugin`
FindAllConnectedLinksHelper | TODO
GetByName | Use type-specific `gz::sim::Link::*ByName`
GetChild | Use type-specific `gz::sim::Link::*ByName`
GetChildCollision | `gz::sim::Link::CollisionByName`
GetChildCount | Use type-specific `gz::sim::Link::*Count`
GetChildJoint | TODO
GetChildJointsLinks | See joint API
GetChildLink | Not supported
GetCollision | `gz::sim::Link::CollisionByName`
GetCollisions | `gz::sim::Link::Collisions`
GetModel | `gz::sim::Link::ParentModel`
GetParent | `gz::sim::EntiyComponentManager::ParentEntity`
GetParentId | `gz::sim::EntiyComponentManager::ParentEntity`
GetParentJoints | TODO
GetParentJointsLinks | See joint API
GetParentModel | `gz::sim::Link::ParentModel`
GetSensorCount | `gz::sim::Link::SensorCount`
GetVisualMessage | See visual API
GetWorld |  `gz::sim::worldEntity`
VisualId | `gz::sim::Link::VisualByName`
Visuals | `gz::sim::Link::Visuals`

---

## Write family

These functions deal with modifying the entity tree, attaching children to new
parents.

---

Classic | Gazebo
-- | --
AddChild | TODO
AddChildJoint | TODO
AddParentJoint | TODO
AttachStaticLink | TODO
AttachStaticModel | TODO
CreateJoint | TODO
CreateLink | TODO
DetachAllStaticModels | TODO
DetachStaticLink | TODO
DetachStaticModel | TODO
RemoveChild | TODO
RemoveChildJoint | TODO
RemoveChildren | TODO
RemoveCollision | TODO
RemoveJoint | TODO
RemoveParentJoint | TODO
SetCanonicalLink | TODO
SetParent | TODO
SetWorld | TODO

---

## Lifecycle

These functions aren't related to the state of a link, but perform some
processing related to the link's lifecycle, like initializing, updating or
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
ResetPhysicsStates | TODO
Update | Entities are updated by systems
UpdateMass | Entities are updated by systems
UpdateSurface | Entities are updated by systems
UpdateWind | Entities are updated by systems
OnPoseChange | TODO

---

## Others

Miscelaneous functions that don't fit the other categories. Most of them involve
logic that should be performed from within a system.

---

Classic | Gazebo
-- | --
GetNearestEntityBelow | Requires a system
PlaceOnEntity | Requires a system
PlaceOnNearestEntityBelow | Requires a system

---
