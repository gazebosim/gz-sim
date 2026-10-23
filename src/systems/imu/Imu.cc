/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "Imu.hh"

#include <memory>
#include <string>
#include <utility>

#include <gz/plugin/Register.hh>

#include <sdf/Element.hh>

#include <gz/common/Profiler.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/ImuSensor.hh>

#include "gz/sim/World.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Component.hh"
#include "gz/sim/components/Factory.hh"
#include "gz/sim/components/Imu.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

namespace gz::sim::systems::imu
{
/// \brief Private component that holds the gz-sensors IMU instance for an
/// entity.
using ImuSensor = components::Component<
    std::shared_ptr<sensors::ImuSensor>, class ImuSensorTag>;
GZ_SIM_REGISTER_PRIVATE_COMPONENT("gz_sim_systems_imu.ImuSensor", ImuSensor)
}  // namespace gz::sim::systems::imu

using ImuSensor = gz::sim::systems::imu::ImuSensor;

/// \brief Private Imu data class.
class gz::sim::systems::ImuPrivate
{
  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep track of world ID, which is equivalent to the scene's
  /// root visual.
  /// Defaults to zero, which is considered invalid by Gazebo.
  public: Entity worldEntity = kNullEntity;

  /// \brief Create IMU sensors in gz-sensors
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateSensors(EntityComponentManager &_ecm);

  /// \brief Update IMU sensor data based on physics data
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const UpdateInfo &_info,
                      const EntityComponentManager &_ecm);

  /// \brief Create sensor
  /// \param[in] _ecm Mutable reference to ECM.
  /// \param[in] _entity Entity of the IMU
  /// \param[in] _imu IMU component.
  /// \param[in] _parent Parent entity component.
  public: void AddSensor(
    EntityComponentManager &_ecm,
    const Entity _entity,
    const components::Imu *_imu,
    const components::ParentEntity *_parent);
};

//////////////////////////////////////////////////
Imu::Imu() : System(), dataPtr(std::make_unique<ImuPrivate>())
{
}

//////////////////////////////////////////////////
Imu::~Imu() = default;

//////////////////////////////////////////////////
void Imu::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Imu::PreUpdate");
  this->dataPtr->CreateSensors(_ecm);
}

//////////////////////////////////////////////////
void Imu::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Imu::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    this->dataPtr->Update(_info, _ecm);
  }
}

//////////////////////////////////////////////////
void ImuPrivate::AddSensor(
  EntityComponentManager &_ecm,
  const Entity _entity,
  const components::Imu *_imu,
  const components::ParentEntity *_parent)
{
  // Get the world acceleration (defined in world frame)
  auto gravity = _ecm.Component<components::Gravity>(worldEntity);
  if (nullptr == gravity)
  {
    gzerr << "World missing gravity." << std::endl;
    return;
  }

  // create sensor
  std::string sensorScopedName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _imu->Data();
  data.SetName(sensorScopedName);
  // check topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/imu";
    data.SetTopic(topic);
  }
  std::shared_ptr<sensors::ImuSensor> sensor =
      this->sensorFactory.CreateSensor<
      sensors::ImuSensor>(data);
  if (nullptr == sensor)
  {
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
  }

  // set sensor parent
  std::string parentName = _ecm.Component<components::Name>(
      _parent->Data())->Data();
  sensor->SetParent(parentName);

  // set gravity - assume it remains fixed
  sensor->SetGravity(gravity->Data());

  // Get initial pose of sensor and set the reference z pos
  // The WorldPose component was just created and so it's empty
  // We'll compute the world pose manually here
  math::Pose3d p = worldPose(_entity, _ecm);
  sensor->SetOrientationReference(p.Rot());

  // Get world frame orientation and heading.
  // If <orientation_reference_frame> includes a named
  // frame like NED, that must be supplied to the IMU sensor,
  // otherwise orientations are reported w.r.t to the initial
  // orientation.
  if (data.Element()->HasElement("imu")) {
    auto imuElementPtr = data.Element()->GetElement("imu");
    if (imuElementPtr->HasElement("orientation_reference_frame")) {
      double heading = 0.0;

      gz::sim::World world(worldEntity);
      if (world.SphericalCoordinates(_ecm))
      {
        auto sphericalCoordinates = world.SphericalCoordinates(_ecm).value();
        heading = sphericalCoordinates.HeadingOffset().Radian();
      }

      sensor->SetWorldFrameOrientation(math::Quaterniond(0, 0, heading),
        gz::sensors::WorldFrameEnumType::ENU);
    }
  }

  // Set whether orientation is enabled
  if (data.ImuSensor())
  {
    sensor->SetOrientationEnabled(
        data.ImuSensor()->OrientationEnabled());
  }

  _ecm.CreateComponent(_entity, components::SensorTopic(sensor->Topic()));
  _ecm.CreateComponent(_entity, ImuSensor(std::move(sensor)));
}

//////////////////////////////////////////////////
void ImuPrivate::CreateSensors(EntityComponentManager &_ecm)
{
  GZ_PROFILE("ImuPrivate::CreateImuEntities");
  // Get World Entity
  if (kNullEntity == this->worldEntity)
    this->worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == this->worldEntity)
  {
    gzerr << "Missing world entity." << std::endl;
    return;
  }

  // Create IMUs for any IMU entity that does not yet have an ImuSensor
  // component.
  _ecm.Each<components::Imu, components::ParentEntity, Exclude<ImuSensor>>(
    [&](const Entity &_entity,
        components::Imu *_imu,
        components::ParentEntity *_parent)->bool
      {
        this->AddSensor(_ecm, _entity, _imu, _parent);
        return true;
      });
}

//////////////////////////////////////////////////
void ImuPrivate::Update(const UpdateInfo &_info,
                        const EntityComponentManager &_ecm)
{
  GZ_PROFILE("ImuPrivate::Update");
  auto gravity = _ecm.Component<components::Gravity>(this->worldEntity);

  // We only update if a sensor needs data and has subscribers.
  // Note: gz-sensors does its own throttling; checking here avoids unnecessary
  // work when the sensor is not due for an update.
  _ecm.Each<ImuSensor,
            components::WorldPose,
            components::AngularVelocity,
            components::LinearAcceleration>(
    [&](const Entity &/*_entity*/,
        const ImuSensor *_imuSensor,
        const components::WorldPose *_worldPose,
        const components::AngularVelocity *_angularVel,
        const components::LinearAcceleration *_linearAccel)->bool
      {
        const auto &sensor = _imuSensor->Data();
        if (sensor->NextDataUpdateTime() > _info.simTime ||
            !sensor->HasConnections())
        {
          return true;
        }

        const auto &imuWorldPose = _worldPose->Data();
        sensor->SetWorldPose(imuWorldPose);

        // Update World Gravity
        if (nullptr != gravity)
        {
          sensor->SetGravity(gravity->Data());
        }
        else
        {
          gzerr << "World missing gravity."
                << "Gravity used by imu sensor may be outdated" << std::endl;
        }
        // Set the IMU angular velocity (defined in imu's local frame)
        sensor->SetAngularVelocity(_angularVel->Data());

        // Set the IMU linear acceleration in the imu local frame
        sensor->SetLinearAcceleration(_linearAccel->Data());

        // Update measurement time
        sensor->Update(_info.simTime, false);

        return true;
      });
}

GZ_ADD_PLUGIN(Imu, System,
  Imu::ISystemPreUpdate,
  Imu::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(Imu, "gz::sim::systems::Imu")
