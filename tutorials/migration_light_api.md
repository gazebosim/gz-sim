\page migrationlightapi Migration from Gazebo-classic: Light API

When migrating plugins from Gazebo-classic to Gazebo, developers will
notice that the C++ APIs for both simulators are quite different. Be sure to
check the [plugin migration tutorial](migrationplugins.html) to get a high-level
view of the architecture differences before using this guide.

This tutorial is meant to serve as a reference guide for developers migrating
functions from the
[gazebo::rendering::Light](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/rendering_1_1Light.html)
class.

If you're trying to use some API which doesn't have an equivalent on Gazebo
yet, feel free to
[ticket an issue](https://github.com/gazebosim/gz-sim/issues/).

## Light API

Gazebo-classic has 2 Light classes:
* [`gazebo::rendering::Light`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1rendering_1_1Light.html) - responsible for accessing and setting light properties.
    * Example: [Light::DiffuseColor](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1rendering_1_1Light.html#a0deb81873bee2c7bc883a10c373501d0) / [Light::SetDiffuseColor](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1rendering_1_1Light.html#a9208ba6d4cb8e0972e046e735dc26976)
* [`gazebo::physics::Light`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Light.html) - responsible for accessing and setting generic entity properties, and parent / child relationship.
    * Example: [Light::GetName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#a9a98946a64f3893b085f650932c9dfee) / [Light::SetName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Entity.html#a5d74ac4d7a230aed1ab4b11933b16e92)
    * Example: [Light::GetParent](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#af87578478aeeb2b176de010d1d639fd9) / [Light::SetParent](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Base.html#a736efe8278da4ecb1640100a2857756f)

In Gazebo, the light APIs has been consolidated into a single Light class with
some of generic functions available through other utility / core classes.
You'll find the APIs below on the following headers:

* \ref gz/sim/Light.hh
* \ref gz/sim/Util.hh
* \ref gz/sim/SdfEntityCreator.hh
* \ref gz/sim/EntityComponentManager.hh

It's worth remembering that most of this functionality can be performed using
the \ref gz::sim::EntityComponentManager "EntityComponentManager" directly.

As an example the `Light::Pose()` is a convienient function for querying the `Pose` component from the `EntityComponentManager`, i.e.

```
  math::Pose3d pose = _ecm.Component<components::Pose>(lightEntityId)->Data();
```

The functions presented in the sections below exist for convenience and
readability. The items marked as `TODO` means that the equivalent API is not
implemented yet in Gazebo.

### Properties

Most of Gazebo-classic's Light API is related to setting
and getting properties. These functions are great candidates to have
equivalents on Gazebo because the Entity-Component-System architecture is
perfect for setting components (properties) into entities such as lights.
This section focuses on migrating from APIs provided through the
`gazebo::rendering::Light` class.

---

Classic | Gazebo
-- | --
CastShadows | `gz::sim::Light::CastShadows`
Clone | TODO
DiffuseColor | `gz::sim::Light::DiffuseColor`
Direction | `gz::sim::Light::Direction`
FillMsg | TODO
Id | `gz::sim::Light::Entity`
LightType | `gz::sim::Light::Type`
Name | `gz::sim::Light::Name`
Position | `gz::sim::Light::Pose`
Rotation | `gz::sim::Light::Pose`
SetAttenuation | use `gz::sim::Light::SetAttenuation*`
SetCastShadows | `gz::sim::Light::SetCastShadows`
SetDiffuseColor | `gz::sim::Light::SetDiffuseColor`
SetDirection | `gz::sim::Light::SetDirection`
SetLightType | TODO
SetName | TODO
SetPosition | TODO
SetRange | `gz::sim::Light::SetAttenuationRange`
SetRotation | TODO
SetSelected |  Selection is client-specific, not porting
SetSpecularColor | `gz::sim::Light::SetSpecularColor`
SetSpotFalloff | `gz::sim::Light::SetSpotFalloff`
SetSpotInnerAngle | `gz::sim::Light::SetSpotInnerAngle`
SetSpotOuterAngle | `gz::sim::Light::SetSpotOuterAngle`
SetVisible | TODO
ShowVisual | TODO
SpecularColor | `gz::sim::Light::SetSpecularColor`
ToggleShowVisual | TODO
Type | `gz::sim::Light::Type`
Visible | TODO
WorldPose | TODO
---


## Read family

These APIs deal with reading information related to child / parent
relationships. As mentioned earlier, these APIs in Gazebo
classic are provided by the `gazebo::physics::Light` class. Most of the
APIs are inherited from the base `gazebo::physics::Entity` class in classic.
We will only list the relevant ones here.

The main difference in these APIs across Gazebo generations is that
on classic, they deal with shared pointers to entities, while on Gazebo,
they deal with entity IDs.

---

Classic | Gazebo
-- | --
GetParent | `gz::sim::Light::Parent`
GetParentId | `gz::sim::Light::Parent`
GetWorld | `gz::sim::worldEntity`

---

## Write family

These functions deal with modifying the entity tree, attaching children to new
parents. As mentioned earlier, these APIs in Gazebo classic are provided by the
`gazebo::physics::Light` class. Most of the APIs are inherited from the base
`gazebo::physics::Entity` class in classic. We will only list the relevant ones
here. As seen below, APIs for changing a Light's entity tree structure are
currently not implemented yet.

---

Classic | Gazebo
-- | --
AddChild | Not supported
RemoveChild | Not supported
RemoveChildren | Not supported
SetParent | TODO
SetWorld | TODO

---

## Lifecycle

These functions are related to the light's lifecycle, like loading and updating
its properties.

---

Classic | Gazebo
-- | --
Load | `gz::sim::SdfEntityCreator::CreateEntities`
LoadFromMsg | `gz::sim::SdfEntityCreator::CreateEntities`
UpdateFromMsg | TODO

---
