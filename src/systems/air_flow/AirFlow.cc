/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include "AirFlow.hh"

#include <gz/msgs/fluid_pressure.pb.h>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/plugin/Register.hh>

#include <gz/common/Profiler.hh>

#include <sdf/Sensor.hh>

#include <gz/math/Helpers.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/AirFlowSensor.hh>

#include "gz/sim/components/AirFlowSensor.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private AirFlow data class.
class gz::sim::systems::AirFlowPrivate
{
  /// \brief A map of air flow entity to its sensor
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::AirFlowSensor>> entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// True if the rendering component is initialized
  public: bool initialized = false;

  public: Entity entity;

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the IMU
  /// \param[in] _AirFlow AirFlowSensor component.
  /// \param[in] _parent Parent entity component.
  public: void AddAirFlow(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::AirFlowSensor *_AirFlow,
    const components::ParentEntity *_parent);

  /// \brief Create air flow sensor
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

  /// \brief Update air flow sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateAirFlows(const EntityComponentManager &_ecm);

  /// \brief Remove air flow sensors if their entities have been removed
  /// from simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveAirFlowEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
AirFlow::AirFlow() :
  System(), dataPtr(std::make_unique<AirFlowPrivate>())
{
}

//////////////////////////////////////////////////
AirFlow::~AirFlow() = default;

//////////////////////////////////////////////////
void AirFlow::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("AirFlow::PreUpdate");

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
void AirFlow::PostUpdate(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  GZ_PROFILE("AirFlow::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);

  if (!_info.paused)
  {
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the AirFlowPrivate::UpdateSpeeds function
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

    this->dataPtr->UpdateAirFlows(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      it.second->Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveAirFlowEntities(_ecm);
}

//////////////////////////////////////////////////
void AirFlowPrivate::AddAirFlow(
  const EntityComponentManager &_ecm,
  const Entity _entity,
  const components::AirFlowSensor *_AirFlow,
  const components::ParentEntity *_parent)
{
  this->entity = _entity;
  // create sensor
  std::string sensorScopedName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _AirFlow->Data();
  data.SetName(sensorScopedName);
  // check topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/air_flow";
    data.SetTopic(topic);
  }
  std::unique_ptr<sensors::AirFlowSensor> sensor =
      this->sensorFactory.CreateSensor<
      sensors::AirFlowSensor>(data);
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
void AirFlowPrivate::CreateSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("AirFlowPrivate::CreateAirFlowEntities");
  if (!this->initialized)
  {
    // Create air flow sensors
    _ecm.Each<components::AirFlowSensor, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::AirFlowSensor *_AirFlow,
          const components::ParentEntity *_parent)->bool
        {
          this->AddAirFlow(_ecm, _entity, _AirFlow, _parent);
          return true;
        });
    this->initialized = true;
  }
  else
  {
    // Create air flow sensors
    _ecm.EachNew<components::AirFlowSensor, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::AirFlowSensor *_AirFlow,
          const components::ParentEntity *_parent)->bool
        {
          this->AddAirFlow(_ecm, _entity, _AirFlow, _parent);
          return true;
        });
  }
}

//////////////////////////////////////////////////
void AirFlowPrivate::UpdateAirFlows(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("AirFlowPrivate::UpdateAirFlows");
  _ecm.Each<components::AirFlowSensor, components::WorldPose>(
    [&](const Entity &_entity,
        const components::AirFlowSensor *,
        const components::WorldPose *_worldPose)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          const math::Pose3d &worldPose = _worldPose->Data();
          it->second->SetPose(worldPose);

          math::Vector3d sensorRelativeVel = relativeVel(_entity, _ecm);
          it->second->SetVelocity(sensorRelativeVel);
        }
        else
        {
          gzerr << "Failed to update air flow: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void AirFlowPrivate::RemoveAirFlowEntities(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("AirFlowPrivate::RemoveAirFlowEntities");
  _ecm.EachRemoved<components::AirFlowSensor>(
    [&](const Entity &_entity,
        const components::AirFlowSensor *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing air flow sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

GZ_ADD_PLUGIN(AirFlow, System,
  AirFlow::ISystemPreUpdate,
  AirFlow::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(AirFlow, "gz::sim::systems::AirFlow")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(AirFlow, "ignition::gazebo::systems::AirFlow")
