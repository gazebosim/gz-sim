\page migrationsensorapi Migration from Gazebo-classic: Sensor API

When migrating plugins from Gazebo-classic to Gazebo, developers will
notice that the C++ APIs for both simulators are quite different. Be sure to
check the [plugin migration tutorial](migrationplugins.html) to get a high-level
view of the architecture differences before using this guide.

This tutorial is meant to serve as a reference guide for developers migrating
functions from the
[gazebo::sensors::Sensor](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1sensors_1_1Sensor.html)
class.

If you're trying to use some API which doesn't have an equivalent on Gazebo
yet, feel free to
[ticket an issue](https://github.com/gazebosim/gz-sim/issues/).

## Sensor API

Gazebo-classic's `gazebo::sensors::Sensor` provides lots of functionality, which
can be divided in these categories:

* **Properties**: Setting / getting properties
    * Example: [Sensor::Name](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1sensors_1_1Sensor.html#a41087c5f2f732f7a2f336b45b952f199)
* **Read family**: Getting parent
    * Example: [Sensor::ParentName](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1sensors_1_1Sensor.html#ac39481d8faba2202d0212ef018595de3)
* **Write family**: Changing parent
    * Example: [Sensor::SetParent](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1sensors_1_1Sensor.html#a8d07a3535e558a172e212f73b942d39d)
* **Lifecycle**: Functions to control the sensor's lifecycle
    * Example: [Sensor::Init](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1sensors_1_1Sensor.html#a3e0b39e1326de703012f81ac2be7feba)

You'll find the Gazebo APIs below on the following headers:

* \ref gz/sim/Sensor.hh
* \ref gz/sim/Util.hh
* \ref gz/sim/SdfEntityCreator.hh
* \ref gz/sim/EntityComponentManager.hh

It's worth remembering that most of this functionality can be performed using
the \ref gz::sim::EntityComponentManager "EntityComponentManager" directly.

As an example the `Sensor::Pose()` is a convienient function for querying the `Pose` component from the `EntityComponentManager`, i.e.

```
  math::Pose3d pose = _ecm.Component<components::Pose>(sensorEntityId)->Data();
```

The functions presented in the sections below exist for convenience and
readability. The items marked as `TODO` means that the equivalent API is not
implemented yet in Gazebo.

### Properties

Most of Gazebo-classic's Sensor API is related to setting and getting
properties. These functions are great candidates to have equivalents on Gazebo
Gazebo, because the Entity-Component-System architecture is perfect for setting
components (properties) into entities such as sensors.

---

Classic | Gazebo
-- | --
Category | TODO
FillMsg | TODO
Id | `ignition::gazebo::Sensor::Entity`
IsActive | TODO
LastMeasurementTime | TODO
LastUpdateTime | TODO
Name | `ignition::gazebo::Sensor::Name`
NextRequiredTimestamp | TODO
Noise | TODO
Pose | `ignition::gazebo::Sensor::Pose`
ResetLastUpdateTime | TODO
ScopedName | `ignition::gazebo::scopedName`
SetActive | TODO
SetPose | TODO
SetUpdateRate | TODO
Topic | `ignition::gazebo::Sensor::Topic`
Type | `ignition::gazebo::entityType`
UpdateRate | TODO
Visualize | TODO
WorldName | `ignition::gazebo::worldEntity`

---

## Read family

These APIs deal with reading information related to parent relationship.

The main difference in these APIs across Gazebo generations is that
on classic, they deal with shared pointers to entities, while on Gazebo,
they deal with entity IDs.

---

Classic | Gazebo
-- | --
ParentId | `ignition::gazebo::Sensor::Parent`
ParentName | `ignition::gazebo::Sensor::Parent`

---

## Write family

These functions deal with modifying the entity tree, attaching children to new
parents.

---

Classic | Gazebo
-- | --
SetParent | TODO
---

## Lifecycle

These functions aren't related to the state of a sensor, but perform some
processing related to the sensor's lifecycle, like initializing, updating or
terminating it.

---

Classic | Gazebo
-- | --
ConnectUpdated | TODO
Fini | N/A
Init | N/A
Load | `ignition::gazebo::SdfEntityCreator::CreateEntities`
Update | Entities are updated by systems
---
