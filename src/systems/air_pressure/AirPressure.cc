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

#include "AirPressure.hh"

#include <gz/msgs/air_pressure_sensor.pb.h>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/plugin/Register.hh>

#include <gz/common/Profiler.hh>

#include <sdf/Sensor.hh>

#include <gz/math/Helpers.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/AirPressureSensor.hh>

#include "gz/sim/components/AirPressureSensor.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private AirPressure data class.
class gz::sim::systems::AirPressurePrivate
{
  /// \brief A map of air pressure entity to its sensor
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::AirPressureSensor>> entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// True if the rendering component is initialized
  public: bool initialized = false;

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the IMU
  /// \param[in] _airPressure AirPressureSensor component.
  /// \param[in] _parent Parent entity component.
  public: void AddAirPressure(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::AirPressureSensor *_airPressure,
    const components::ParentEntity *_parent);

  /// \brief Create air pressure sensor
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

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
  GZ_PROFILE("AirPressure::PreUpdate");

  // Create components
  for (auto entity : this->dataPtr->newSensors)
  {
    auto it = this->dataPtr->entitySensorMap.find(entity);
    if (it == this->dataPtr->entitySensorMap.end())
    {
      gzerr << "Entity [" << entity
             << "] isn't in sensor map, this shouldn't happen." << std::endl;
      continue;
    }
    // Set topic
    _ecm.CreateComponent(entity, components::SensorTopic(it->second->Topic()));
  }
  this->dataPtr->newSensors.clear();
}

//////////////////////////////////////////////////
void AirPressure::PostUpdate(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  GZ_PROFILE("AirPressure::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);

  if (!_info.paused)
  {
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the AirPressurePrivate::UpdatePressures function
    bool needsUpdate = false;
    for (auto &it : this->dataPtr->entitySensorMap)
    {
      if (it.second->NextDataUpdateTime() <= _info.simTime &&
          it.second->HasConnections())
      {
        needsUpdate = true;
        break;
      }
    }
    if (!needsUpdate)
      return;

    this->dataPtr->UpdateAirPressures(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      it.second->Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveAirPressureEntities(_ecm);
}

//////////////////////////////////////////////////
void AirPressurePrivate::AddAirPressure(
  const EntityComponentManager &_ecm,
  const Entity _entity,
  const components::AirPressureSensor *_airPressure,
  const components::ParentEntity *_parent)
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
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
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

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void AirPressurePrivate::CreateSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("AirPressurePrivate::CreateAirPressureEntities");
  if (!this->initialized)
  {
    // Create air pressure sensors
    _ecm.Each<components::AirPressureSensor, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::AirPressureSensor *_airPressure,
          const components::ParentEntity *_parent)->bool
        {
          this->AddAirPressure(_ecm, _entity, _airPressure, _parent);
          return true;
        });
    this->initialized = true;
  }
  else
  {
    // Create air pressure sensors
    _ecm.EachNew<components::AirPressureSensor, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::AirPressureSensor *_airPressure,
          const components::ParentEntity *_parent)->bool
        {
          this->AddAirPressure(_ecm, _entity, _airPressure, _parent);
          return true;
        });
  }
}

//////////////////////////////////////////////////
void AirPressurePrivate::UpdateAirPressures(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("AirPressurePrivate::UpdateAirPressures");
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
          gzerr << "Failed to update air pressure: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void AirPressurePrivate::RemoveAirPressureEntities(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("AirPressurePrivate::RemoveAirPressureEntities");
  _ecm.EachRemoved<components::AirPressureSensor>(
    [&](const Entity &_entity,
        const components::AirPressureSensor *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing air pressure sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

GZ_ADD_PLUGIN(AirPressure, System,
  AirPressure::ISystemPreUpdate,
  AirPressure::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(AirPressure, "gz::sim::systems::AirPressure")
