\page migrationlinkapi

# Migration from Gazebo-classic: Link API

When migrating plugins from Gazebo-classic to Ignition Gazebo, developers will
notice that the C++ APIs for both simulators are quite different. Be sure to
check the [plugin migration tutorial](migrationplugins.html) to get a high-level
view of the architecture differences before using this guide.

This tutorial is meant to serve as a reference guide for developers migrating
functions from the
[gazebo::phyiscs::Link](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Link.html)
class.

If you're trying to use some API which doesn't have an equivalent on Ignition
yet, feel free to
[ticket an issue](https://github.com/ignitionrobotics/ign-gazebo/issues/).

## Link API

Gazebo-classic's `gazebo::physics::Link` provides lots of functionality, which
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

You'll find the Ignition APIs below on the following headers:

* [ignition/gazebo/Link.hh](https://ignitionrobotics.org/api/gazebo/3.3/Link_8hh.html)
* [ignition/gazebo/Util.hh](https://ignitionrobotics.org/api/gazebo/3.3/Util_8hh.html)
* [ignition/gazebo/SdfEntityCreator.hh](https://ignitionrobotics.org/api/gazebo/3.3/SdfEntityCreator_8hh.html)
* [ignition/gazebo/EntityComponentManager.hh](https://ignitionrobotics.org/api/gazebo/3.3/classignition_1_1gazebo_1_1EntityComponentManager.html)

It's worth remembering that most of this functionality can be performed using
the
[EntityComponentManager](https://ignitionrobotics.org/api/gazebo/3.3/classignition_1_1gazebo_1_1EntityComponentManager.html)
directly. The functions presented here exist for convenience and readability.

### Properties

Most of Gazebo-classic's Link API is related to setting and getting
properties. These functions are great candidates to have equivalents on Ignition
Gazebo, because the Entity-Component-System architecture is perfect for setting
components (properties) into entities such as links.

---

Classic | Ignition
-- | --
AddForce | `ignition::gazebo::Link::AddWorldForce`
AddForceAtRelativePosition | TODO
AddForceAtWorldPosition | TODO
AddLinkForce | TODO
AddRelativeForce | TODO
AddRelativeTorque | TODO
AddTorque | `ignition::gazebo::Link::AddWorldWrench`
AddType | `ecm.CreateComponent<Type>(entity, Type())`
Battery | Use this system: `ignition::gazebo::systems::LinearBatteryPlugin`
BoundingBox | TODO
CollisionBoundingBox | TODO
DirtyPose | Not supported
FillMsg | TODO
GetAngularDamping | TODO
GetEnabled | TODO
GetGravityMode | TODO
GetId | `ignition::gazebo::Link::Entity`
GetInertial | `ignition::gazebo::Link::WorldInertialPose` / `ignition::gazebo::Link::WorldInertialMatrix`
GetKinematic | TODO
GetLinearDamping | TODO
GetName | `ignition::gazebo::Link::Name`
GetSDF | TODO
GetSDFDom | TODO
GetSaveable | Not supported
GetScopedName | `ignition::gazebo::scopedName`
GetSelfCollide | See model API
GetSensorName | See sensor API
GetType | `ignition::gazebo::entityType`
GetWorldEnergy | TODO
GetWorldEnergyKinetic | `ignition::gazebo::Link::WorldKineticEnergy`
GetWorldEnergyPotential | TODO
HasType | `gazebo::components::Link::typeId == entityTypeId(entity, ecm)`
InitialRelativePose | TODO
IsCanonicalLink | `ignition::gazebo::Link::IsCanonical`
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
TypeStr | `ignition::gazebo::entityTypeStr`
URI | TODO
UpdateParameters | TODO
VisualPose | See visual API
WindMode | `ignition::gazebo::Link::WindMode`
WorldAngularAccel | TODO
WorldAngularMomentum | TODO
WorldAngularVel | `ignition::gazebo::Link::WorldAngularVelocity`
WorldCoGLinearVel | TODO
WorldCoGPose | TODO
WorldForce | TODO
WorldInertiaMatrix | `ignition::gazebo::Link::WorldInertialMatrix`
WorldInertialPose | `ignition::gazebo::Link::WorldInertialPose`
WorldLinearAccel | `ignition::gazebo::Link::WorldLinearAcceleration`
WorldLinearVel | `ignition::gazebo::Link::WorldLinearVelocity`
WorldPose |  `ignition::gazebo::Link::WorldPose`
WorldTorque | TODO
WorldWindLinearVel | TODO

---

## Read family

These APIs deal with reading information related to child / parent
relationships.

The main difference in these APIs across Gazebo generations is that
on classic, they deal with shared pointers to entities, while on Ignition,
they deal with entity IDs.

---

Classic | Ignition
-- | --
BatteryCount | Use this system: `ignition::gazebo::systems::LinearBatteryPlugin`
FindAllConnectedLinksHelper | TODO
GetByName | Use type-specific `ignition::gazebo::Link::*ByName`
GetChild | Use type-specific `ignition::gazebo::Link::*ByName`
GetChildCollision | `ignition::gazebo::Link::CollisionByName`
GetChildCount | Use type-specific `ignition::gazebo::Link::*Count`
GetChildJoint | TODO
GetChildJointsLinks | See joint API
GetChildLink | Not supported
GetCollision | `ignition::gazebo::Link::CollisionByName`
GetCollisions | `ignition::gazebo::Link::Collisions`
GetModel | `ignition::gazebo::Link::ParentModel`
GetParent | `ignition::gazebo::EntiyComponentManager::ParentEntity`
GetParentId | `ignition::gazebo::EntiyComponentManager::ParentEntity`
GetParentJoints | TODO
GetParentJointsLinks | See joint API
GetParentModel | `ignition::gazebo::Link::ParentModel`
GetSensorCount | `ignition::gazebo::Link::SensorCount`
GetVisualMessage | See visual API
GetWorld |  `ignition::gazebo::worldEntity`
VisualId | `ignition::gazebo::Link::VisualByName`
Visuals | `ignition::gazebo::Link::Visuals`

---

## Write family

These functions deal with modifying the entity tree, attaching children to new
parents.

---

Classic | Ignition
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

Classic | Ignition
-- | --
Fini | N/A
Init | N/A
Load | `ignition::gazebo::SdfEntityCreator::CreateEntities`
LoadJoints | `ignition::gazebo::SdfEntityCreator::CreateEntities`
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

Classic | Ignition
-- | --
GetNearestEntityBelow | Requires a system
PlaceOnEntity | Involves Requires a system
PlaceOnNearestEntityBelow | Requires a system

---
