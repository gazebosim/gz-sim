\page ardupilot Case study: migrating the ArduPilot ModelPlugin from Gazebo classic to Gazebo

A variety of changes are required when migrating a plugin from Gazebo classic
to Gazebo. In this tutorial we offer as a case
study the migration of one particular `ModelPlugin`,
[ardupilot_gazebo](https://github.com/khancyr/ardupilot_gazebo). We hope that
this example provides useful tips to others who are migrating their existing
plugins from classic to Gazebo.

The complete, migrated version of the `ardupilot_gazebo` plugin covered in this tutorial
can be found in [this fork](https://github.com/gerkey/ardupilot_gazebo/tree/ignition).

## Background

The `ardupilot_gazebo` plugin is used with Gazebo to assist with simulating
aerial vehicles (aka drones). For more information on how to use
it, check the [ArduPilot
documentation](https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html).

As context to understand what we're migrating, here's a system diagram for how
the ArduPilot Gazebo plugin works is used:

<img src="https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/ardupilot_diagram.png"/>

*UAV icon credit: By Julian Herzog, CC BY 4.0, https://commons.wikimedia.org/w/index.php?curid=60965475*

For each UAV model in simulation, there is one instance of ArduPilotPlugin
loaded into the simulation process. That plugin uses internal simulation APIs
to retrieve the UAV's current state, which it sends to an external ArduPilot
process via a custom UDP protocol (it's called Flight Dynamics Model, or FDM).
The ArduPilot process in turn makes the vehicle state available via the MAVLink
protocol to other processes, such as QGroundControl (QGC). The user can issue
commands in QGC like "take off" or "goto waypoint", which are sent via MAVLink
to ArduPilot, which computes motor commands and sends them to the plugin, which
passes them onto the vehicle via internal simulation APIs.

To be clear, this structure is pre-existing and widely used in UAV simulation.
Our contribution in this tutorial is port the plugin from Gazebo classic to
Gazebo, preserving the rest of the setup.

## Structure of the migration

Migration of this plugin involves modifications to multiple parts of the associated code:

1. The plugin header file, `ArduPilotPlugin.hh`
2. The plugin source file, `ArduPilotPlugin.cc`
3. The plugin's CMake build recipe, `CMakeLists.txt`
4. The custom model in which the plugin is used

We'll take them each in turn in the following sections.

## Plugin header file (ArduPilotPlugin.hh)

### Headers

The old code includes these Gazebo classic headers:

```cpp
// OLD
#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
```

In the new code, we still need `<sdf/sdf.hh>`, because the underlying [SDFormat
library](http://sdformat.org/) is used by both classic and Gazebo. But in place of the `<gazebo/...>` headers, we'll pull in one from Gazebo:

```cpp
// NEW
#include <gz/sim/System.hh>
#include <sdf/sdf.hh>
```

### Class declaration

In the old code, the plugin class `ArduPilotPlugin` is declared in the `gazebo` namespace:
```cpp
// OLD
namespace sim
{
```

In the new code we declare the class in the `gz::sim::systems` namespace:

```cpp
// NEW
namespace gz
{
namespace sim
{
namespace systems
{
```

In the old code, the plugin class inherits from `ModelPlugin`:

```cpp
// OLD
class GAZEBO_VISIBLE ArduPilotPlugin : public ModelPlugin
```

In the new code, we use multiple inheritance to declare that our plugin will
act as a *system* (in the entity-component-system, or ECS, pattern used by
Gazebo), and further which interfaces of a system it will use (we also update
the symbol visibility macro):

```cpp
// NEW
class GZ_SIM_VISIBLE ArduPilotPlugin:
       public gz::sim::System,
       public gz::sim::ISystemConfigure,
       public gz::sim::ISystemPostUpdate,
       public gz::sim::ISystemPreUpdate
```

With this declaration we're indicating that our plugin will supply implementation of the `Configure()`, `PreUpdate()`, and `PostUpdate()` methods.

In the old code, the `Load()` method is called once for each instance of the
plugin that is loaded, allowing for startup configuration, like pulling
parameters out of the plugin's SDF configuration:

```cpp
// OLD
virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
```

In the new code, we use `Configure()` for the same purpose (if a different signature):

```cpp
// NEW
void Configure(const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &_eventMgr);
```

Similarly, the old code provides `OnUpdate()`, which is called once per time step while simulation is running:

```cpp
// OLD
void OnUpdate();
```

In the new code, this method is replaced by two methods, `PreUpdate()` and
`PostUpdate()`:


```cpp
// NEW
void PreUpdate(const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm);

void PostUpdate(const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm);
```

As the names suggest, the former is called before each time step, while the
latter is called after each time step. Note the subtle difference in signature:
`PreUpdate()` takes a non-`const` reference to the `EntityComponentManager`,
while `PostUpdate()` takes a `const` reference to it. We'll make any changes to
the state of simulation (e.g., setting torques on joints) in `PreUpdate()` and
we'll read out results from simulation (e.g., getting the pose of a link) in
`PostUpdate()`.

The remaining changes in the header are just bookkeeping, to allow us to have
access to the right objects with the right types in other class methods. These three helpers:

```cpp
// OLD
void ApplyMotorForces(const double _dt);
void SendState();
bool InitArduPilotSockets(sdf::ElementPtr _sdf);
```

become:

```cpp
// NEW
void ApplyMotorForces(const double _dt,
  gz::sim::EntityComponentManager &_ecm);
void SendState(double _simTime,
  const gz::sim::EntityComponentManager &_ecm);
bool InitArduPilotSockets(const std::shared_ptr<const sdf::Element> &_sdf);
```

## Plugin source file (ArduPilotPlugin.cc)

### Headers

The old code includes these Gazebo-related headers:

```cpp
// OLD
#include <sdf/sdf.hh>
#include <gz/math/Filter.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
```

Like we did in `ArduPilotPlugin.hh`, we'll keep `<sdf/sdf.hh>`. The others are
replaced with Gazebo equivalents, and where possible we narrow the inclusion
to exactly what we need. We start by enumerating those *components* (part of the
ECS pattern used by Gazebo) that we're using:

```cpp
// NEW
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
```

To better understand the ECS pattern as it is used in Gazebo, it's helpful to
learn about the EntityComponentManager (ECM), which is responsible for managing
the ECS graph. A great resource to understand the logic under the hood of the
ECM is the `SdfEntityCreator` class
([header](https://github.com/gazebosim/gz-sim/blob/main/include/gz/sim/SdfEntityCreator.hh),
[source](https://github.com/gazebosim/gz-sim/blob/main/src/SdfEntityCreator.cc)).
This class is responsible for mapping the content of an SDF file to the
entities and components that form the graph handled by the ECM. For example, if
you wonder which components can be accessed by default from the plugin, this
class is the best entry point.

Next we include the parts of `gz-sim` itself that we're using:

```cpp
// NEW
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
```

We need a few things from `gz-math`:

```cpp
// NEW
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/PID.hh>
#include <gz/math/Vector3.hh>
```

To use the `GZ_ADD_PLUGIN()` and `GZ_ADD_PLUGIN_ALIAS()` macros, we
need a header from `gz-plugin`:

```cpp
// NEW
#include <gz/plugin/Register.hh>
```

Because we'll be subscribing to data published by a sensor, we need a header from `gz-transport`:

```cpp
// NEW
#include <gz/transport/Node.hh>
```

And we keep the SDFormat header:

```cpp
// NEW
#include <sdf/sdf.hh>
```

### Class members

Now let's get into the class member declarations. The `PID` class has moved from `common`:

```cpp
// OLD
common::PID pid;
```

to `gz::math`:

```cpp
// NEW
gz::math::PID pid;
```

In the old code we store a `physics::JointPtr` for each propeller joint we're controlling:

```cpp
// OLD
physics::JointPtr joint;
```

In the new code we store an `gz::sim::Entity` instead:

```cpp
// NEW
gz::sim::Entity joint;
```

In the old code we store an `event::ConnectionPtr` to manage periodic calls to the `OnUpdate()` method:

```cpp
// OLD
event::ConnectionPtr updateConnection;
```

There's no equivalent class member in the new code. Instead we declared our
intent to have this class's update methods called via its inheritance.

In the old code we store a `physics::ModelPtr` for the model we're acting on:

```cpp
// OLD
physics::ModelPtr model;
```

In the new code we instead store references to the model, the entity underlying
the model, and the entity underyling one of the links in the model:

```cpp
// NEW
gz::sim::Entity entity{gz::sim::kNullEntity};
gz::sim::Model model{gz::sim::kNullEntity};
gz::sim::Entity modelLink{gz::sim::kNullEntity};
```

The old code uses a custom time class:

```cpp
// OLD
sim::common::Time lastControllerUpdateTime;
```

while the new code uses `std::chrono`:

```cpp
// NEW
std::chrono::steady_clock::duration lastControllerUpdateTime{0};
```

In this plugin we need to read data from an IMU sensor attached to the UAV. In
the old code we store a pointer to the sensor:

```cpp
// OLD
sensors::ImuSensorPtr imuSensor;
```

In the new code, instead of accessing the sensor object directly we must
subscribe to a topic published by the sensor (you might be tempted to try
retrieving the sensor data via components attached to the IMU entity, but that
won't work because the logic to produce the data lives in the IMU system and
its output can only be consumed via subscription). So we need a few more
variables to track the state of subscription, data receipt via subscription,
and so on:

```cpp
// NEW
std::string imuName;
bool imuInitialized;
gz::transport::Node node;
gz::msgs::IMU imuMsg;
bool imuMsgValid;
std::mutex imuMsgMutex;
```

We also need a callback function that will be invoked upon receipt of newly
published data from the IMU sensor. The callback just latches the latest
message in a mutex-controlled fashion:

```cpp
// NEW
void imuCb(const gz::msgs::IMU &_msg)
{
  std::lock_guard<std::mutex> lock(this->imuMsgMutex);
  imuMsg = _msg;
  imuMsgValid = true;
}
```

### Plugin interface: Configure()

Recall that `Configure()` replaces `Load()`.

In the old code, we store the model pointer and name:

```cpp
// OLD
this->dataPtr->model = _model;
this->dataPtr->modelName = this->dataPtr->model->GetName();
```

In the new code, we store the entity, model, and name a bit differently:

```cpp
// NEW
this->dataPtr->entity = _entity;
this->dataPtr->model = gz::sim::Model(_entity);
this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);
```

Also in the new code we need to make sure of the existence of the specific
*components* that we need. In our case, we're going to access the `WorldPose`
and `WorldLinearVelocity` components of the *entity* representing one of the
UAV model's links. The data in those components will be periodically updated by
the physics *system*. But the physics system will not necessarily
create the components, so before accessing them later in our code, we need to
ensure that the components exist:

```cpp
// NEW
if(!_ecm.EntityHasComponentType(this->dataPtr->modelLink, components::WorldPose::typeId))
{
  _ecm.CreateComponent(this->dataPtr->modelLink, gz::sim::components::WorldPose());
}
if(!_ecm.EntityHasComponentType(this->dataPtr->modelLink, components::WorldLinearVelocity::typeId))
{
  _ecm.CreateComponent(this->dataPtr->modelLink, gz::sim::components::WorldLinearVelocity());
}
```

We'll see this pattern elsewhere in the new code: check for a component's
existence, create it if necessary, then proceed with using it.

We also clone the `const sdf::Element` that we've passed so that we can call
non-`const` methods on it:

```cpp
// NEW
sdf::ElementPtr sdfClone = _sdf->Clone();
```

In the old code we retrieve a pointer to each joint that we're controlling:

```cpp
// OLD
control.joint = _model->GetJoint(control.jointName);
```

In the new code we retrieve the entity that represents the joint:

```cpp
// NEW
control.joint = this->dataPtr->model.JointByName(_ecm, control.jointName);
```

The accessor methods for members in the `PID` class have changed. The old code uses a `Get` prefix, e.g.:

```cpp
// OLD
param = controlSDF->Get("vel_p_gain", control.pid.GetPGain()).first;
param = controlSDF->Get("vel_i_gain", control.pid.GetIGain()).first;
param = controlSDF->Get("vel_d_gain", control.pid.GetDGain()).first;
```

In the new code, the `Get` prefix is gone:

```cpp
// NEW
param = controlSDF->Get("vel_p_gain", control.pid.PGain()).first;
param = controlSDF->Get("vel_i_gain", control.pid.IGain()).first;
param = controlSDF->Get("vel_d_gain", control.pid.DGain()).first;
```

The old code does a bunch of lookups to get a pointer to the IMU sensor. In the
new code, we just store the name of the sensors from the user-supplied SDF
configuration:

```cpp
// NEW
this->dataPtr->imuName = _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;
```

and we do the equivalent lookup later, in `PreUpdate()`, which we'll cover next.

### Plugin interface: OnUpdate() -> PreUpdate() + PostUpdate()

The old code does the following each time step in its `OnUpdate()` method:

```cpp
// OLD
const sim::common::Time curTime =
  this->dataPtr->model->GetWorld()->SimTime();

if (curTime > this->dataPtr->lastControllerUpdateTime)
{
  this->ReceiveMotorCommand();
  if (this->dataPtr->arduPilotOnline)
  {
    this->ApplyMotorForces((curTime -
      this->dataPtr->lastControllerUpdateTime).Double());
    this->SendState();
  }
}

this->dataPtr->lastControllerUpdateTime = curTime;
```

As mentioned above, in the new code we're splitting that work into two halves:
the "write" part should happen in `PreUpdate()` and the "read" part should
happen in `PostUpdate()`.

In `PreUpdate()` we receive new commands from the external ArduPilot process
and write the resulting forces to propeller joints in simulation:

```cpp
// NEW
if (_info.simTime > this->dataPtr->lastControllerUpdateTime)
{
  this->ReceiveMotorCommand();
  if (this->dataPtr->arduPilotOnline)
  {
    this->ApplyMotorForces(std::chrono::duration_cast<std::chrono::duration<double> >(_info.simTime -
      this->dataPtr->lastControllerUpdateTime).count(), _ecm);
  }
```

Then in `PostUpdate()` we read the latest state (e.g., IMU sensor data, UAV
pose and velocity) from simulation and send it out to ArduPilot:

```cpp
// NEW
if (_info.simTime > this->dataPtr->lastControllerUpdateTime)
{
  if (this->dataPtr->arduPilotOnline)
  {
    this->SendState(std::chrono::duration_cast<std::chrono::duration<double> >(_info.simTime).count(),
            _ecm);
  }
}

this->dataPtr->lastControllerUpdateTime = _info.simTime;
```

Note the differences in both methods with regard to time-handling: (i) the
current simulation time is passed in as part of an
`gz::sim::UpdateInfo` object; and (ii) we operate on time values using
`std::chrono`.

#### One-time initialization in PreUpdate(): subscribing to sensor data

Though it's not part of the regular update loop, we subscribe to the IMU sensor
data in `PreUpdate()` because the information that we need for that
subscription isn't available when we're in `Configure()`.

That one-time subscription logic looks like this, starting with determination
of the right topic name and ending with registering our previously defined
`imuCb()` method as the callback to receive new IMU data:

```cpp
// NEW
if(!this->dataPtr->imuInitialized)
{
  // Set unconditionally because we're only going to try this once.
  this->dataPtr->imuInitialized = true;

  auto imuEntity = _ecm.EntityByComponents(
      components::Name(this->dataPtr->imuName),
      components::Imu(),
      components::ParentEntity(this->dataPtr->modelLink));
  auto imuTopicName = _ecm.ComponentData<components::SensorTopic>(imuEntity);

  if(imuTopicName.empty())
  {
    gzerr << "[" << this->dataPtr->modelName << "] "
          << "imu_sensor [" << this->dataPtr->imuName
          << "] not found, abort ArduPilot plugin." << "\n";
    return;
  }

  this->dataPtr->node.Subscribe(imuTopicName, &gz::sim::systems::ArduPilotPluginPrivate::imuCb, this->dataPtr.get());
}
```

### Writing to simulation

Based on commands received from ArduPilot, new forces are applied to the
propeller joints in `ApplyMotorForces()`, using the joints' current velocities
as feedback. In the old code that's done by calling `GetVelocity()` and
`SetForce()` on each joint `i`:

```cpp
// OLD
const double vel = this->dataPtr->controls[i].joint->GetVelocity(0);
// ...do some feedback control math to compute force from vel...
this->dataPtr->controls[i].joint->SetForce(0, force);
```

In the new code, for each joint `i` we read from the `JointVelocity` component
attached to the corresponding entity, and we write to the `JointForceCmd`
component attached to the same entity (creating it first in case it doesn't yet
exist):

```cpp
// NEW
const double vel = _ecm.ComponentData<gz::sim::components::JointVelocity>(
    this->dataPtr->controls[i].joint);
// ...do some feedback control math to compute force from vel...
_ecm.SetComponentData(this->dataPtr->controls[i].joint,
    gz::sim::components::JointForceCmd({force}));
```

A similar pattern is used for the case of setting a velocity on a joint;
instead of calling `SetVelocity()` on the joint, we write to the
`JointVelocityCmd` component of the joint entity.

### Reading from simulation

To prepare the data that will be sent to ArduPilot, in `SendState()` we need to
read some information from simulation, specifically: linear acceleration and
angular velocity from the IMU, and the UAV's pose and linear velocity in the
world frame.

In the old code, we get the IMU data by calling methods on the sensor object
and copying the result into the packet that we're going to send to ArduPilot:

```cpp
// OLD
const gz::math::Vector3d linearAccel =
  this->dataPtr->imuSensor->LinearAcceleration();
pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();

const gz::math::Vector3d angularVel =
  this->dataPtr->imuSensor->AngularVelocity();
pkt.imuAngularVelocityRPY[0] = angularVel.X();
pkt.imuAngularVelocityRPY[1] = angularVel.Y();
pkt.imuAngularVelocityRPY[2] = angularVel.Z();
```

In the new code, as previously mentioned, these data are accessed by
subscribing to the sensor via gz-transport. In that subscription we registered
a callback that just copies the latest IMU message to `imuMsg` and sets the
flag `imuMsgValid`, using `imuMsgMutex` to exclude concurrent access to those
variables. So we access the latest IMU sensor by copying and reading from that
message:

```cpp
// NEW
gz::msgs::IMU imuMsg;
{
  std::lock_guard<std::mutex> lock(this->dataPtr->imuMsgMutex);
  if(!this->dataPtr->imuMsgValid)
  {
    return;
  }
  imuMsg = this->dataPtr->imuMsg;
}

pkt.imuLinearAccelerationXYZ[0] = imuMsg.linear_acceleration().x();
pkt.imuLinearAccelerationXYZ[1] = imuMsg.linear_acceleration().y();
pkt.imuLinearAccelerationXYZ[2] = imuMsg.linear_acceleration().z();

pkt.imuAngularVelocityRPY[0] = imuMsg.angular_velocity().x();
pkt.imuAngularVelocityRPY[1] = imuMsg.angular_velocity().y();
pkt.imuAngularVelocityRPY[2] = imuMsg.angular_velocity().z();
```

In the old code, we access the UAV's pose linear velocity in the world frame by
calling `WorldPose()` and `WorldLinearVelocity()`, respectively, on the model
object:

```cpp
// OLD
const gz::math::Pose3d gazeboXYZToModelXForwardZDown =
  this->modelXYZToAirplaneXForwardZDown +
  this->dataPtr->model->WorldPose();

const gz::math::Vector3d velGazeboWorldFrame =
  this->dataPtr->model->GetLink()->WorldLinearVel();
```

In the new code we instead read from the `WorldPose` and `WorldLinearVelocity`
components attached to the entity representing one of the UAV model's links:

```cpp
// NEW
const gz::sim::components::WorldPose* pComp =
    _ecm.Component<gz::sim::components::WorldPose>(this->dataPtr->modelLink);
const gz::math::Pose3d gazeboXYZToModelXForwardZDown =
  this->modelXYZToAirplaneXForwardZDown +
  pComp->Data();

const gz::sim::components::WorldLinearVelocity* vComp =
  _ecm.Component<gz::sim::components::WorldLinearVelocity>(this->dataPtr->modelLink);
const gz::math::Vector3d velGazeboWorldFrame = vComp->Data();
```

### Registering the plugin

In the old code we register our plugin via the macro `GZ_REGISTER_PLUGIN()`:

```cpp
// OLD
GZ_REGISTER_MODEL_PLUGIN(ArduPilotPlugin)
```

In the new code we instead use two macros: `GZ_ADD_PLUGIN()` and `GZ_ADD_PLUGIN_ALIAS()`:

```cpp
// NEW
GZ_ADD_PLUGIN(gz::sim::systems::ArduPilotPlugin,
                    gz::sim::System,
                    gz::sim::systems::ArduPilotPlugin::ISystemConfigure,
                    gz::sim::systems::ArduPilotPlugin::ISystemPostUpdate,
                    gz::sim::systems::ArduPilotPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::ArduPilotPlugin,"ArduPilotPlugin")
```

## Build recipe: `CMakeLists.txt`

Compared to the code changes, the updates in the CMake configuration are pretty
minor and primarily result from the fact that the formerly monolithic Gazebo
project is now a set of Gazebo libraries.

In the old code we retrieve all the required build configuration by finding the Gazebo package:

```
# OLD
find_package(gazebo REQUIRED)
```

In the new code we explicitly reference each Gazebo package that we use:

```
# NEW
find_package(sdformat14 REQUIRED)
find_package(gz-common5-all REQUIRED)
find_package(gz-sim8-all REQUIRED)
find_package(gz-math7-all REQUIRED)
find_package(gz-msgs10-all REQUIRED)
find_package(gz-physics7-all REQUIRED)
find_package(gz-sensors8-all REQUIRED)
find_package(gz-transport13-all REQUIRED)
```

In the old code we need only refer to the build configuration retrieved from the Gazebo package:

```
include_directories(
        ${PROJECT_SOURCE_DIR}
        include
        ${GAZEBO_INCLUDE_DIRS})

link_libraries(
        ${GAZEBO_LIBRARIES}
)
```

Whereas in the new code we refer to build configuration from each Gazebo package:

```
include_directories(
        ${PROJECT_SOURCE_DIR}
        include
        ${SDFORMAT-INCLUDE_DIRS}
        ${GZ-COMMON_INCLUDE_DIRS}
        ${GZ-SIM_INCLUDE_DIRS}
        ${GZ-MATH_INCLUDE_DIRS}
        ${GZ-MSGS_INCLUDE_DIRS}
        ${GZ-TRANSPORT_INCLUDE_DIRS}
        )

link_libraries(
        ${SDFORMAT-LIBRARIES}
        ${GZ-COMMON_LIBRARIES}
        ${GZ-SIM_LIBRARIES}
        ${GZ-MATH_LIBRARIES}
        ${GZ-MSGS_LIBRARIES}
        ${GZ-TRANSPORT_LIBRARIES}
        )
```

## The model

The old UAV is defined in two parts: (i) the `iris_with_standoffs` model, which
defines the vehicle structure; and (ii) the `iris_with_ardupilot` model, which
includes the extends the `iris_with_standoffs` model by adding plugins needed to
fly it.

Because model inclusion is not (yet?) supported, the new model just combines
the additional plugin configuration from `iris_with_ardupilot` into
`iris_with_standoffs`. Along the way a few changes are made, as follows.

The `<script>` tag for visual material is not (yet?) supported, so in the
model, instances of `<script>` are just commented out (which leaves the UAV
visually untextured, but functional).

In the old model, loading an instance of the LiftDrag plugin for each half of
each propeller looks like this:

```xml
<!-- OLD -->
<plugin
    name="rotor_0_blade_1"
    filename="LiftDragPlugin">
  <!-- ...configuration goes here... -->
  <link_name>iris::rotor_0</link_name>
</plugin>
```

In the new model, we do this instead:

```xml
<!-- NEW -->
<plugin
    name="gz::sim::systems::LiftDrag"
    filename="gz-sim-lift-drag-system">
  <!-- ...configuration goes here... -->
  <link_name>rotor_0</link_name>
</plugin>
```

In the old model, it's possible to read joint state and apply joint forces
automatically. In the new model, we must instantiate the `JointStatePublisher`
plugin once for the entire model and the `ApplyJointForce` plugin once for each propeller joint:

```xml
<!-- NEW -->
<plugin
  filename="gz-sim-joint-state-publisher-system"
  name="gz::sim::systems::JointStatePublisher"></plugin>
<plugin
  filename="gz-sim-apply-joint-force-system"
  name="gz::sim::systems::ApplyJointForce">
  <joint_name>rotor_0_joint</joint_name>
</plugin>
<plugin
  filename="gz-sim-apply-joint-force-system"
  name="gz::sim::systems::ApplyJointForce">
  <joint_name>rotor_1_joint</joint_name>
</plugin>
<plugin
  filename="gz-sim-apply-joint-force-system"
  name="gz::sim::systems::ApplyJointForce">
  <joint_name>rotor_2_joint</joint_name>
</plugin>
<plugin
  filename="gz-sim-apply-joint-force-system"
  name="gz::sim::systems::ApplyJointForce">
  <joint_name>rotor_3_joint</joint_name>
</plugin>
```

## What's next

You should be able to apply the same general changes covered in this tutorial
to your Gazebo plugins to migrate them to Gazebo.

Check out [these
instructions](https://github.com/gerkey/ardupilot_gazebo/tree/ignition#using-with-ignition)
if you'd like to learn more about using ardupilot_gazebo with Gazebo.
