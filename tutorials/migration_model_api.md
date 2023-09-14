\page migrationmodelapi Migration from Gazebo-classic: Model API

When migrating plugins from Gazebo-classic to Gazebo, developers will
notice that the C++ APIs for both simulators are quite different. Be sure to
check the [plugin migration tutorial](migrationplugins.html) to get a high-level
view of the architecture differences before using this guide.

This tutorial is meant to serve as a reference guide for developers migrating
functions from the
[sim::physics::Model](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Model.html)
class.

If you're trying to use some API which doesn't have an equivalent on Gazebo
yet, feel free to
[ticket an issue](https://github.com/gazebosim/gz-sim/issues/).

## Model API

Gazebo-classic's `sim::physics::Model` provides lots of functionality, which
can be divided in these categories:

* **Properties**: Setting / getting properties
    * Example: [Model::GetName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#a9a98946a64f3893b085f650932c9dfee) / [Model::SetName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Entity.html#a5d74ac4d7a230aed1ab4b11933b16e92)
* **Read family**: Getting children and parent
    * Example: [Model::GetLink](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Model.html#a7a923baddd29445b23740ca06e153c06)
* **Write family**: Adding children, changing parent
    * Example: [Model::RemoveChildren](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#aa85d2386e6fb02bdbec060a74b63238a)
* **Lifecycle**: Functions to control the model's lifecycle
    * Example: [Model::Init](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Model.html#ae048ef824aaf614707c1496a2aefd415)
* **Others**: Functions that don't fit any of the categories above
    * Example: [Model::PlaceOnEntity](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Entity.html#a9ecbfeb56940cacd75f55bed6aa9fcb4)

You'll find the Gazebo APIs below on the following headers:

* \ref gz/sim/Model.hh
* \ref gz/sim/Util.hh
* \ref gz/sim/SdfEntityCreator.hh
* \ref gz/sim/EntityComponentManager.hh

It's worth remembering that most of this functionality can be performed using
the \ref gz::sim::EntityComponentManager "EntityComponentManager" directly.
The functions presented here exist for convenience and readability.

### Properties

Most of Gazebo-classic's Model API is related to setting and getting
properties. These functions are great candidates to have equivalents on Gazebo
Gazebo, because the Entity-Component-System architecture is perfect for setting
components (properties) into entities such as models.

---

Classic | Gazebo
-- | --
AddType | `ecm.CreateComponent<Type>(entity, Type())`
BoundingBox | TODO
CollisionBoundingBox | TODO
DirtyPose | Not supported
FillMsg | TODO
GetAutoDisable | TODO
GetId | `gz::sim::Model::Entity`
GetName | `gz::sim::Model::Name`
GetPluginCount | TODO
GetSDF | TODO
GetSDFDom | TODO
GetSaveable | Not supported
GetScopedName | `gz::sim::scopedName`
GetSelfCollide | `gz::sim::Model::SelfCollide`
GetType | `gz::sim::entityType`
GetWorldEnergy | TODO
GetWorldEnergyKinetic | TODO
GetWorldEnergyPotential | TODO
HasType | `sim::components::Model::typeId == entityTypeId(entity, ecm)`
InitialRelativePose | TODO
IsCanonicalLink | See link API
IsSelected | Selection is client-specific, not porting
IsStatic | `gz::sim::Model::Static`
PluginInfo | TODO
Print | TODO
ProcessMsg | TODO
RelativeAngularAccel | TODO
RelativeAngularVel | TODO
RelativeLinearAccel | TODO
RelativeLinearVel | TODO
RelativePose | TODO
SDFPoseRelativeToParent | TODO
SDFSemanticPose | TODO
Scale | TODO
SensorScopedName | TODO
SetAngularVel | TODO
SetAnimation | TODO
SetAutoDisable | TODO
SetCollideMode | TODO
SetEnabled | TODO
SetGravityMode | TODO
SetInitialRelativePose | TODO
SetJointAnimation | TODO
SetJointPosition | See joint API
SetJointPositions | See joint API
SetLaserRetro | TODO
SetLinearVel | TODO
SetLinkWorldPose | See link API
SetName | TODO
SetRelativePose | TODO
SetSaveable | Not supported
SetScale | TODO
SetSelected |  Selection is client-specific, not porting
SetSelfCollide | TODO
SetState | TODO
SetStatic | TODO
SetWindMode | TODO
SetWorldPose | `gz::sim::Model::SetWorldPoseCmd`
SetWorldTwist | TODO
StopAnimation | TODO
TypeStr | `gz::sim::entityTypeStr`
URI | TODO
UnscaledSDF | TODO
UpdateParameters | TODO
WindMode | `gz::sim::Model::WindMode`
WorldAngularAccel | TODO
WorldAngularVel | TODO
WorldLinearAccel | TODO
WorldLinearVel | TODO
WorldPose |  `gz::sim::worldPose`

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
GetByName | Use type-specific `gz::sim::Model::*ByName`
GetChild | Use type-specific `gz::sim::Model::*ByName`
GetChildCollision |  See link API
GetChildCount | Use type-specific `gz::sim::Model::*Count`
GetChildLink | `gz::sim::Model::LinkByName`
GetGripper | TODO
GetGripperCount | TODO
GetJoint |  `gz::sim::Model::JointByName`
GetJointCount | `gz::sim::Model::JointCount`
GetJoints | `gz::sim::Model::Joints`
GetLink | `gz::sim::Model::LinkByName`
GetLinks | const `gz::sim::Model::Links`
GetParent | `gz::sim::EntiyComponentManager::ParentEntity`
GetParentId | `gz::sim::EntiyComponentManager::ParentEntity`
GetParentModel | `gz::sim::EntiyComponentManager::ParentEntity`
GetSensorCount | See link API
GetWorld | const `gz::sim::Model::World`
NestedModel | `gz::sim::Model::NestedModelByName`
NestedModels | const `gz::sim::Model::NestedModels`

---

## Write family

These functions deal with modifying the entity tree, attaching children to new
parents.

---

Classic | Gazebo
-- | --
AddChild | TODO
AttachStaticModel | TODO
CreateJoint | TODO
CreateLink | TODO
DetachStaticModel | TODO
RemoveChild | TODO
RemoveChildren | TODO
RemoveJoint | TODO
SetCanonicalLink | TODO
SetParent | TODO
SetWorld | TODO

---

## Lifecycle

These functions aren't related to the state of a model, but perform some
processing related to the model's lifecycle, like initializing, updating or
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
