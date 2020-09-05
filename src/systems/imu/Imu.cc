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

#include <ignition/plugin/Register.hh>

#include <sdf/Element.hh>

#include <ignition/common/Profiler.hh>

#include <ignition/transport/Node.hh>

#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/ImuSensor.hh>

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Imu.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

#include "Imu.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private Imu data class.
class ignition::gazebo::systems::ImuPrivate
{
  /// \brief A map of IMU entity to its IMU sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<ignition::sensors::ImuSensor>> entitySensorMap;

  /// \brief Ign-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  public: Entity worldEntity = kNullEntity;

  /// \brief Create IMU sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateImuEntities(EntityComponentManager &_ecm);

  /// \brief Update IMU sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Remove IMU sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveImuEntities(const EntityComponentManager &_ecm);
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
  IGN_PROFILE("Imu::PreUpdate");
  this->dataPtr->CreateImuEntities(_ecm);
}

//////////////////////////////////////////////////
void Imu::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  IGN_PROFILE("Imu::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    this->dataPtr->Update(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      auto time = math::durationToSecNsec(_info.simTime);
      dynamic_cast<sensors::Sensor *>(it.second.get())->Update(
          common::Time(time.first, time.second), false);
    }
  }

  this->dataPtr->RemoveImuEntities(_ecm);
}

//////////////////////////////////////////////////
void ImuPrivate::CreateImuEntities(EntityComponentManager &_ecm)
{
  IGN_PROFILE("ImuPrivate::CreateImuEntities");
  // Get World Entity
  if (kNullEntity == this->worldEntity)
    this->worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == this->worldEntity)
  {
    ignerr << "Missing world entity." << std::endl;
    return;
  }

  // Get the world acceleration (defined in world frame)
  auto gravity = _ecm.Component<components::Gravity>(worldEntity);
  if (nullptr == gravity)
  {
    ignerr << "World missing gravity." << std::endl;
    return;
  }

  // Create IMUs
  _ecm.EachNew<components::Imu, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::Imu *_imu,
        const components::ParentEntity *_parent)->bool
      {
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
        std::unique_ptr<sensors::ImuSensor> sensor =
            this->sensorFactory.CreateSensor<
            sensors::ImuSensor>(data);
        if (nullptr == sensor)
        {
          ignerr << "Failed to create sensor [" << sensorScopedName << "]"
                 << std::endl;
          return true;
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

        // Set topic
        _ecm.CreateComponent(_entity, components::SensorTopic(sensor->Topic()));

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void ImuPrivate::Update(const EntityComponentManager &_ecm)
{
  IGN_PROFILE("ImuPrivate::Update");
  _ecm.Each<components::Imu,
            components::WorldPose,
            components::AngularVelocity,
            components::LinearAcceleration>(
    [&](const Entity &_entity,
        const components::Imu * /*_imu*/,
        const components::WorldPose *_worldPose,
        const components::AngularVelocity *_angularVel,
        const components::LinearAcceleration *_linearAccel)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          const auto &imuWorldPose = _worldPose->Data();
          it->second->SetWorldPose(imuWorldPose);

          // Set the IMU angular velocity (defined in imu's local frame)
          it->second->SetAngularVelocity(_angularVel->Data());

          // Set the IMU linear acceleration in the imu local frame
          it->second->SetLinearAcceleration(_linearAccel->Data());
         }
        else
        {
          ignerr << "Failed to update IMU: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void ImuPrivate::RemoveImuEntities(
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("ImuPrivate::RemoveImuEntities");
  _ecm.EachRemoved<components::Imu>(
    [&](const Entity &_entity,
        const components::Imu *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing IMU sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

IGNITION_ADD_PLUGIN(Imu, System,
  Imu::ISystemPreUpdate,
  Imu::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(Imu, "ignition::gazebo::systems::Imu")
