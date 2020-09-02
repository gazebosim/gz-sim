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
#include <ignition/msgs/air_pressure_sensor.pb.h>

#include <ignition/plugin/Register.hh>

#include <ignition/common/Profiler.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>

#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/AirPressureSensor.hh>

#include "ignition/gazebo/components/AirPressureSensor.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

#include "AirPressure.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private AirPressure data class.
class ignition::gazebo::systems::AirPressurePrivate
{
  /// \brief A map of air pressure entity to its vertical reference
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::AirPressureSensor>> entitySensorMap;

  /// \brief Ign-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Create air pressure sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateAirPressureEntities(EntityComponentManager &_ecm);

  /// \brief Update air pressure sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateAirPressures(const EntityComponentManager &_ecm);

  /// \brief Remove air pressure sensors if their entities have been removed
  /// from simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveAirPressureEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
AirPressure::AirPressure() :
  System(), dataPtr(std::make_unique<AirPressurePrivate>())
{
}

//////////////////////////////////////////////////
AirPressure::~AirPressure() = default;

//////////////////////////////////////////////////
void AirPressure::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("AirPressure::PreUpdate");
  this->dataPtr->CreateAirPressureEntities(_ecm);
}

//////////////////////////////////////////////////
void AirPressure::PostUpdate(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  IGN_PROFILE("AirPressure::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  if (!_info.paused)
  {
    this->dataPtr->UpdateAirPressures(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      auto time = math::durationToSecNsec(_info.simTime);
      dynamic_cast<sensors::Sensor *>(it.second.get())->Update(
          common::Time(time.first, time.second), false);
    }
  }

  this->dataPtr->RemoveAirPressureEntities(_ecm);
}

//////////////////////////////////////////////////
void AirPressurePrivate::CreateAirPressureEntities(EntityComponentManager &_ecm)
{
  IGN_PROFILE("AirPressurePrivate::CreateAirPressureEntities");
  // Create air pressure sensors
  _ecm.EachNew<components::AirPressureSensor, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::AirPressureSensor *_airPressure,
        const components::ParentEntity *_parent)->bool
      {
        // create sensor
        std::string sensorScopedName =
            removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _airPressure->Data();
        data.SetName(sensorScopedName);
        // check topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/air_pressure";
          data.SetTopic(topic);
        }
        std::unique_ptr<sensors::AirPressureSensor> sensor =
            this->sensorFactory.CreateSensor<
            sensors::AirPressureSensor>(data);
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

        // The WorldPose component was just created and so it's empty
        // We'll compute the world pose manually here
        // set sensor world pose
        math::Pose3d sensorWorldPose = worldPose(_entity, _ecm);
        sensor->SetPose(sensorWorldPose);

        // Set topic
        _ecm.CreateComponent(_entity, components::SensorTopic(sensor->Topic()));

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void AirPressurePrivate::UpdateAirPressures(const EntityComponentManager &_ecm)
{
  IGN_PROFILE("AirPressurePrivate::UpdateAirPressures");
  _ecm.Each<components::AirPressureSensor, components::WorldPose>(
    [&](const Entity &_entity,
        const components::AirPressureSensor *,
        const components::WorldPose *_worldPose)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          const math::Pose3d &worldPose = _worldPose->Data();
          it->second->SetPose(worldPose);
        }
        else
        {
          ignerr << "Failed to update air pressure: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void AirPressurePrivate::RemoveAirPressureEntities(
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("AirPressurePrivate::RemoveAirPressureEntities");
  _ecm.EachRemoved<components::AirPressureSensor>(
    [&](const Entity &_entity,
        const components::AirPressureSensor *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing air pressure sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

IGNITION_ADD_PLUGIN(AirPressure, System,
  AirPressure::ISystemPreUpdate,
  AirPressure::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(AirPressure, "ignition::gazebo::systems::AirPressure")
