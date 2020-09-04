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

#include <sdf/Sensor.hh>

#include <ignition/common/Profiler.hh>

#include <ignition/transport/Node.hh>

#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/MagnetometerSensor.hh>

#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/Magnetometer.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

#include "Magnetometer.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private Magnetometer data class.
class ignition::gazebo::systems::MagnetometerPrivate
{
  /// \brief A map of magnetometer entity to its sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<ignition::sensors::MagnetometerSensor>> entitySensorMap;

  /// \brief Ign-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Create magnetometer sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateMagnetometerEntities(EntityComponentManager &_ecm);

  /// \brief Update magnetometer sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Remove magnetometer sensors if their entities have been removed
  /// from simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveMagnetometerEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
Magnetometer::Magnetometer() : System(), dataPtr(
    std::make_unique<MagnetometerPrivate>())
{
}

//////////////////////////////////////////////////
Magnetometer::~Magnetometer() = default;

//////////////////////////////////////////////////
void Magnetometer::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("Magnetometer::PreUpdate");
  this->dataPtr->CreateMagnetometerEntities(_ecm);
}

//////////////////////////////////////////////////
void Magnetometer::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  IGN_PROFILE("Magnetometer::PostUpdate");

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

  this->dataPtr->RemoveMagnetometerEntities(_ecm);
}

//////////////////////////////////////////////////
void MagnetometerPrivate::CreateMagnetometerEntities(
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("MagnetometerPrivate::CreateMagnetometerEntities");
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == worldEntity)
  {
    ignerr << "Missing world entity." << std::endl;
    return;
  }

  // Get the world magnetic field (defined in world frame)
  auto worldField = _ecm.Component<components::MagneticField>(worldEntity);
  if (nullptr == worldField)
  {
    ignerr << "World missing magnetic field." << std::endl;
    return;
  }

  // Create magnetometers
  _ecm.EachNew<components::Magnetometer, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::Magnetometer *_magnetometer,
        const components::ParentEntity *_parent)->bool
      {
        // create sensor
        std::string sensorScopedName =
            removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _magnetometer->Data();
        data.SetName(sensorScopedName);
        // check topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/magnetometer";
          data.SetTopic(topic);
        }
        std::unique_ptr<sensors::MagnetometerSensor> sensor =
            this->sensorFactory.CreateSensor<
            sensors::MagnetometerSensor>(data);
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

        // set world magnetic field. Assume uniform in world and does not
        // change throughout simulation
        sensor->SetWorldMagneticField(worldField->Data());

        // Get initial pose of sensor and set the reference z pos
        // The WorldPose component was just created and so it's empty
        // We'll compute the world pose manually here
        math::Pose3d p = worldPose(_entity, _ecm);
        sensor->SetWorldPose(p);

        // Set topic
        _ecm.CreateComponent(_entity, components::SensorTopic(sensor->Topic()));

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void MagnetometerPrivate::Update(
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("MagnetometerPrivate::Update");
  _ecm.Each<components::Magnetometer,
            components::WorldPose>(
    [&](const Entity &_entity,
        const components::Magnetometer * /*_magnetometer*/,
        const components::WorldPose *_worldPose)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          // Get the magnetometer physical position
          const math::Pose3d &magnetometerWorldPose = _worldPose->Data();
          it->second->SetWorldPose(magnetometerWorldPose);
        }
        else
        {
          ignerr << "Failed to update magnetometer: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void MagnetometerPrivate::RemoveMagnetometerEntities(
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("MagnetometerPrivate::RemoveMagnetometerEntities");
  _ecm.EachRemoved<components::Magnetometer>(
    [&](const Entity &_entity,
        const components::Magnetometer *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing magnetometer sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

IGNITION_ADD_PLUGIN(Magnetometer, System,
  Magnetometer::ISystemPreUpdate,
  Magnetometer::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(Magnetometer,
                          "ignition::gazebo::systems::Magnetometer")
