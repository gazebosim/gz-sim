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

It's worth remembering that most of this functionality can be performed using
the
[EntityComponentManager](https://ignitionrobotics.org/api/gazebo/3.3/classignition_1_1gazebo_1_1EntityComponentManager.html)
directly. The functions presented here exist for convenience and readability.

### Properties

Most of Gazebo-classic's Model API is related to setting and getting
properties. These functions are great candidates to have equivalents on Ignition
Gazebo, because the Entity-Component-System architecture is perfect for setting
components (properties) into entities such as models.

Classic | Ignition
-- | --
AddType | `ecm.CreateComponent<Type>(entity, Type())`
BoundingBox |
CollisionBoundingBox |
DirtyPose | Not supported
FillMsg |
GetAutoDisable |
GetId | `ignition::gazebo::Model::Entity`
GetName | `ignition::gazebo::Model::Name`
GetPluginCount |
GetSDF |
GetSDFDom |
GetSaveable | Not supported
GetScopedName | `ignition::gazebo::scopedName`
GetSelfCollide | `ignition::gazebo::Model::SelfCollide`
GetType | `ignition::gazebo::entityType`
GetWorldEnergy |
GetWorldEnergyKinetic |
GetWorldEnergyPotential |
HasType |  `ignition::gazebo::isType`
InitialRelativePose |
IsCanonicalLink | See link API
IsSelected | Selection is client-specific, not porting
IsStatic | `ignition::gazebo::Model::Static`
PluginInfo |
Print |
ProcessMsg |
RelativeAngularAccel |
RelativeAngularVel |
RelativeLinearAccel |
RelativeLinearVel |
RelativePose |
SDFPoseRelativeToParent |
SDFSemanticPose |
Scale |
SensorScopedName |
SetAngularVel |
SetAnimation |
SetAutoDisable |
SetCollideMode |
SetEnabled |
SetGravityMode |
SetInitialRelativePose |
SetJointAnimation |
SetJointPosition | See joint API
SetJointPositions | See joint API
SetLaserRetro |
SetLinearVel |
SetLinkWorldPose | See link API
SetName |
SetRelativePose |
SetSaveable | Not supported
SetScale |
SetSelected |  Selection is client-specific, not porting
SetSelfCollide |
SetState |
SetStatic |
SetWindMode |
SetWorldPose | `ignition::gazebo::Model::SetWorldPoseCmd`
SetWorldTwist |
StopAnimation |
TypeStr | `ignition::gazebo::entityTypeStr`
URI |
UnscaledSDF |
UpdateParameters |
WindMode | `ignition::gazebo::Model::WindMode`
WorldAngularAccel |
WorldAngularVel |
WorldLinearAccel |
WorldLinearVel |
WorldPose |  `ignition::gazebo::worldPose`


## Read family

These APIs deal with reading information related to child / parent
relationships.

The main difference in these APIs across Gazebo generations is that
on classic, they deal with shared pointers to entities, while on Ignition,
they deal with entity IDs.

Classic | Ignition
-- | --
GetByName | Use type-specific `ignition::gazebo::Model::*ByName`
GetChild | Use type-specific `ignition::gazebo::Model::*ByName`
GetChildCollision |  See link API
GetChildCount | Use type-specific `ignition::gazebo::Model::*Count`
GetChildLink | `ignition::gazebo::Model::LinkByName`
GetGripper |
GetGripperCount |
GetJoint |  `ignition::gazebo::Model::JointByName`
GetJointCount | `ignition::gazebo::Model::JointCount`
GetJoints | `ignition::gazebo::Model::Joints`
GetLink | `ignition::gazebo::Model::LinkByName`
GetLinks | const `ignition::gazebo::Model::Links`
GetParent | `ignition::gazebo::Model::Parent`
GetParentId | `ignition::gazebo::Model::Parent`
GetParentModel | `ignition::gazebo::Model::Parent`
GetSensorCount | See link API
GetWorld | const `ignition::gazebo::Model::World`
NestedModel | `ignition::gazebo::Model::NestedModelByName`
NestedModels | const `ignition::gazebo::Model::NestedModels`


## Write family

These functions deal with modifying the entity tree, attaching children to new
parents.

Classic | Ignition
-- | --
AddChild |
AttachStaticModel |
CreateJoint |
CreateLink |
DetachStaticModel |
RemoveChild |
RemoveChildren |
RemoveJoint |
SetCanonicalLink |
SetParent |
SetWorld |

## Lifecycle

These functions aren't related to the state of a model, but perform some
processing related to the model's lifecycle, like initializing, updating or
terminating it.

Classic | Ignition
-- | --
Fini | N/A
Init | N/A
Load | `ignition::gazebo::SdfEntityCreator::CreateEntities`
LoadJoints | `ignition::gazebo::SdfEntityCreator::CreateEntities`
LoadPlugins |
Reset |
ResetPhysicsStates |
Update | Entities are updated by systems

## Others

Miscelaneous functions that don't fit the other categories. Most of them involve
logic that should be performed from within a system.

Classic | Ignition
-- | --
GetJointController | Use this system: `ignition::gazebo::systems::JointController`
GetNearestEntityBelow | Requires a system
PlaceOnEntity | Involves Requires a system
PlaceOnNearestEntityBelow | Requires a system
