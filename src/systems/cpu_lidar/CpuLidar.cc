/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "CpuLidar.hh"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/plugin/Register.hh>

#include <sdf/Element.hh>

#include <gz/common/Profiler.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/CpuLidarSensor.hh>

#include "gz/sim/components/CpuLidar.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/RaycastData.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private CpuLidar data class.
class gz::sim::systems::CpuLidarPrivate
{
  /// \brief A map of CPU lidar entity to its sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::CpuLidarSensor>> entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// \brief Keep track of world ID.
  /// Defaults to kNullEntity.
  public: Entity worldEntity = kNullEntity;

  /// True if the system has been initialized
  public: bool initialized = false;

  /// \brief Create CPU lidar sensors in gz-sensors
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

  /// \brief Update CPU lidar sensor data based on physics raycast data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the CPU lidar
  /// \param[in] _cpuLidar CpuLidar component.
  /// \param[in] _parent Parent entity component.
  public: void AddSensor(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::CpuLidar *_cpuLidar,
    const components::ParentEntity *_parent);

  /// \brief Remove CPU lidar sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveSensorEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
CpuLidar::CpuLidar() : System(), dataPtr(std::make_unique<CpuLidarPrivate>())
{
}

//////////////////////////////////////////////////
CpuLidar::~CpuLidar() = default;

//////////////////////////////////////////////////
void CpuLidar::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("CpuLidar::PreUpdate");

  for (auto entity : this->dataPtr->newSensors)
  {
    auto it = this->dataPtr->entitySensorMap.find(entity);
    if (it == this->dataPtr->entitySensorMap.end())
    {
      gzerr << "Entity [" << entity
             << "] isn't in sensor map, this shouldn't happen." << std::endl;
      continue;
    }
    _ecm.CreateComponent(entity, components::SensorTopic(it->second->Topic()));

    auto rays = it->second->GenerateRays();

    components::RaycastDataInfo raycastData;
    raycastData.rays.reserve(rays.size());
    for (const auto &ray : rays)
    {
      raycastData.rays.push_back({ray.first, ray.second});
    }

    _ecm.CreateComponent(entity, components::RaycastData(raycastData));
  }
  this->dataPtr->newSensors.clear();

  // Only raycast when the sensor needs fresh data.
  if (!_info.paused)
  {
    for (auto &it : this->dataPtr->entitySensorMap)
    {
      auto *comp = _ecm.Component<components::RaycastData>(it.first);
      if (comp)
      {
        comp->Data().needsRaycast =
          (it.second->NextDataUpdateTime() <= _info.simTime &&
           it.second->HasConnections());
      }
    }
  }
}

//////////////////////////////////////////////////
void CpuLidar::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  GZ_PROFILE("CpuLidar::PostUpdate");

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);

  if (!_info.paused)
  {
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

    this->dataPtr->Update(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      it.second->Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveSensorEntities(_ecm);
}

//////////////////////////////////////////////////
void CpuLidarPrivate::AddSensor(
  const EntityComponentManager &_ecm,
  const Entity _entity,
  const components::CpuLidar *_cpuLidar,
  const components::ParentEntity *_parent)
{
  std::string sensorScopedName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _cpuLidar->Data();
  data.SetName(sensorScopedName);
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/cpu_lidar";
    data.SetTopic(topic);
  }
  std::unique_ptr<sensors::CpuLidarSensor> sensor =
      this->sensorFactory.CreateSensor<
      sensors::CpuLidarSensor>(data);
  if (nullptr == sensor)
  {
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
  }

  std::string parentName = _ecm.Component<components::Name>(
      _parent->Data())->Data();
  sensor->SetParent(parentName);

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void CpuLidarPrivate::CreateSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("CpuLidarPrivate::CreateSensors");
  if (kNullEntity == this->worldEntity)
    this->worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == this->worldEntity)
  {
    gzerr << "Missing world entity." << std::endl;
    return;
  }

  if (!this->initialized)
  {
    _ecm.Each<components::CpuLidar, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::CpuLidar *_cpuLidar,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _cpuLidar, _parent);
          return true;
        });
      this->initialized = true;
  }
  else
  {
    _ecm.EachNew<components::CpuLidar, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::CpuLidar *_cpuLidar,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _cpuLidar, _parent);
          return true;
      });
  }
}

//////////////////////////////////////////////////
void CpuLidarPrivate::Update(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("CpuLidarPrivate::Update");
  _ecm.Each<components::CpuLidar,
            components::RaycastData,
            components::WorldPose>(
    [&](const Entity &_entity,
        const components::CpuLidar * /*_cpuLidar*/,
        const components::RaycastData *_raycastData,
        const components::WorldPose *_worldPose)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          it->second->SetPose(_worldPose->Data());

          const auto &results = _raycastData->Data().results;
          if (!results.empty())
          {
            std::vector<sensors::CpuLidarSensor::RayResult> rayResults;
            rayResults.reserve(results.size());
            for (const auto &result : results)
            {
              sensors::CpuLidarSensor::RayResult r;
              r.point = result.point;
              r.fraction = result.fraction;
              r.normal = result.normal;
              rayResults.push_back(r);
            }
            it->second->SetRaycastResults(rayResults);
          }
        }
        else
        {
          gzerr << "Failed to update CpuLidar: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void CpuLidarPrivate::RemoveSensorEntities(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("CpuLidarPrivate::RemoveSensorEntities");
  _ecm.EachRemoved<components::CpuLidar>(
    [&](const Entity &_entity,
        const components::CpuLidar *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing CpuLidar sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

GZ_ADD_PLUGIN(CpuLidar, System,
  CpuLidar::ISystemPreUpdate,
  CpuLidar::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(CpuLidar, "gz::sim::systems::CpuLidar")
