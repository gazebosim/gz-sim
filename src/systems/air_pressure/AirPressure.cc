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

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/AirPressureSensor.hh>

#include "ignition/gazebo/components/AirPressure.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
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
  this->dataPtr->CreateAirPressureEntities(_ecm);
}

//////////////////////////////////////////////////
void AirPressure::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  if (!_info.paused)
  {
    this->dataPtr->UpdateAirPressures(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      auto time = math::durationToSecNsec(_info.simTime);
      it.second->Update(common::Time(time.first, time.second));
    }
  }

  this->dataPtr->RemoveAirPressureEntities(_ecm);
}

//////////////////////////////////////////////////
void AirPressurePrivate::CreateAirPressureEntities(EntityComponentManager &_ecm)
{
  // Create air pressure sensors
  _ecm.EachNew<components::AirPressure, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::AirPressure *_airPressure,
        const components::ParentEntity *_parent)->bool
      {
        // create sensor
        std::string sensorScopedName = scopedName(_entity, _ecm, "::", false);
        sdf::Sensor data = _airPressure->Data();
        data.SetName(sensorScopedName);
        // check topic
        if (!data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/air_pressure";
          data.SetTopic(topic);
        }
        std::unique_ptr<sensors::AirPressureSensor> sensor =
            this->sensorFactory.CreateSensor<
            sensors::AirPressureSensor>(data);
        // set sensor parent
        std::string parentName = _ecm.Component<components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // Get initial pose of sensor and set the reference z pos
        // The WorldPose component was just created and so it's empty
        // We'll compute the world pose manually here
        sensor->SetVerticalPosition(worldPose(_entity, _ecm).Pos().Z());

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void AirPressurePrivate::UpdateAirPressures(const EntityComponentManager &_ecm)
{
  _ecm.Each<components::AirPressure, components::WorldPose,
            components::WorldLinearVelocity>(
    [&](const Entity &_entity,
        const components::AirPressure * /*_airPressure*/,
        const components::WorldPose *_worldPose,
        const components::WorldLinearVelocity *_worldLinearVel)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          it->second->SetVerticalPosition(_worldPose->Data().Pos().Z());
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
  _ecm.EachRemoved<components::AirPressure>(
    [&](const Entity &_entity,
        const components::AirPressure *)->bool
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
