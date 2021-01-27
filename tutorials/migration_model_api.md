\page migrationmodelapi

# Migration from Gazebo-classic: Model API

When migrating plugins from Gazebo-classic to Ignition Gazebo, developers will
notice that the C++ APIs for both simulators are quite different. Be sure to
check the [plugin migration tutorial](migrationplugins.html) to get a high-level
view of the architecture differences before using this guide.

This tutorial is meant to serve as a reference guide for developers migrating
functions from the
[gazebo::physics::Model](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Model.html)
class.

If you're trying to use some API which doesn't have an equivalent on Ignition
yet, feel free to
[ticket an issue](https://github.com/ignitionrobotics/ign-gazebo/issues/).

## Model API

Gazebo-classic's `gazebo::physics::Model` provides lots of functionality, which
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

You'll find the Ignition APIs below on the following headers:

* [ignition/gazebo/Model.hh](https://ignitionrobotics.org/api/gazebo/3.3/Model_8hh.html)
* [ignition/gazebo/Util.hh](https://ignitionrobotics.org/api/gazebo/3.3/Util_8hh.html)
* [ignition/gazebo/SdfEntityCreator.hh](https://ignitionrobotics.org/api/gazebo/3.3/SdfEntityCreator_8hh.html)
* [ignition/gazebo/EntityComponentManager.hh](https://ignitionrobotics.org/api/gazebo/3.3/classignition_1_1gazebo_1_1EntityComponentManager.html)

It's worth remembering that most of this functionality can be performed using
the
[EntityComponentManager](https://ignitionrobotics.org/api/gazebo/3.3/classignition_1_1gazebo_1_1EntityComponentManager.html)
directly. The functions presented here exist for convenience and readability.

### Properties

Most of Gazebo-classic's Model API is related to setting and getting
properties. These functions are great candidates to have equivalents on Ignition
Gazebo, because the Entity-Component-System architecture is perfect for setting
components (properties) into entities such as models.

---

Classic | Ignition
-- | --
AddType | `ecm.CreateComponent<Type>(entity, Type())`
BoundingBox | TODO
CollisionBoundingBox | TODO
DirtyPose | Not supported
FillMsg | TODO
GetAutoDisable | TODO
GetId | `ignition::gazebo::Model::Entity`
GetName | `ignition::gazebo::Model::Name`
GetPluginCount | TODO
GetSDF | TODO
GetSDFDom | TODO
GetSaveable | Not supported
GetScopedName | `ignition::gazebo::scopedName`
GetSelfCollide | `ignition::gazebo::Model::SelfCollide`
GetType | `ignition::gazebo::entityType`
GetWorldEnergy | TODO
GetWorldEnergyKinetic | TODO
GetWorldEnergyPotential | TODO
HasType | `gazebo::components::Model::typeId == entityTypeId(entity, ecm)`
InitialRelativePose | TODO
IsCanonicalLink | See link API
IsSelected | Selection is client-specific, not porting
IsStatic | `ignition::gazebo::Model::Static`
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
SetWorldPose | `ignition::gazebo::Model::SetWorldPoseCmd`
SetWorldTwist | TODO
StopAnimation | TODO
TypeStr | `ignition::gazebo::entityTypeStr`
URI | TODO
UnscaledSDF | TODO
UpdateParameters | TODO
WindMode | `ignition::gazebo::Model::WindMode`
WorldAngularAccel | TODO
WorldAngularVel | TODO
WorldLinearAccel | TODO
WorldLinearVel | TODO
WorldPose |  `ignition::gazebo::worldPose`

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
GetByName | Use type-specific `ignition::gazebo::Model::*ByName`
GetChild | Use type-specific `ignition::gazebo::Model::*ByName`
GetChildCollision |  See link API
GetChildCount | Use type-specific `ignition::gazebo::Model::*Count`
GetChildLink | `ignition::gazebo::Model::LinkByName`
GetGripper | TODO
GetGripperCount | TODO
GetJoint |  `ignition::gazebo::Model::JointByName`
GetJointCount | `ignition::gazebo::Model::JointCount`
GetJoints | `ignition::gazebo::Model::Joints`
GetLink | `ignition::gazebo::Model::LinkByName`
GetLinks | const `ignition::gazebo::Model::Links`
GetParent | `ignition::gazebo::EntiyComponentManager::ParentEntity`
GetParentId | `ignition::gazebo::EntiyComponentManager::ParentEntity`
GetParentModel | `ignition::gazebo::EntiyComponentManager::ParentEntity`
GetSensorCount | See link API
GetWorld | const `ignition::gazebo::Model::World`
NestedModel | `ignition::gazebo::Model::NestedModelByName`
NestedModels | const `ignition::gazebo::Model::NestedModels`

---

## Write family

These functions deal with modifying the entity tree, attaching children to new
parents.

---

Classic | Ignition
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

---

## Others

Miscelaneous functions that don't fit the other categories. Most of them involve
logic that should be performed from within a system.

---

Classic | Ignition
-- | --
GetJointController | Use this system: `ignition::gazebo::systems::JointController`
GetNearestEntityBelow | Requires a system
PlaceOnEntity | Involves Requires a system
PlaceOnNearestEntityBelow | Requires a system

---
