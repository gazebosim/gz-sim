/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef IGNITION_GAZEBO_SYSTEMS_SENSORIMPL_HH_
#define IGNITION_GAZEBO_SYSTEMS_SENSORIMPL_HH_

#include <map>
#include <memory>
#include <set>

#include <ignition/common/Profiler.hh>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/ParentEntity.hh>

#include <ignition/sensors/SensorFactory.hh>

namespace ignition 
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{

template <typename SensorT, typename ComponentT>
class SensorImpl
{
  public: using SensorPtr = std::unique_ptr<SensorT>;

  public: using EntitySensorMap = std::unordered_map<Entity, SensorPtr>;

  /// \brief A map of Sensor entity to its gz-sensors object.
  public: EntitySensorMap entitySensorMap; 

  /// \brief gz-sensors sensor factory for creating sensors
  public: ignition::sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  public: bool initialized = false;

  /// \brief Create sensors in gz-sensors
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm) 
  {
    IGN_PROFILE("SensorImpl::CreateSensors");

    if (!this->initialized)
    {
      // Create Sensors
      _ecm.Each<ComponentT, components::ParentEntity>(
      [&](const Entity &_entity,
          const ComponentT *_sensor,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _sensor, _parent);
          return true;
        });
      this->initialized = true;
    }
    else
    {
      _ecm.EachNew<ComponentT, components::ParentEntity>(
        [&](const Entity &_entity,
            const ComponentT *_sensor,
            const components::ParentEntity *_parent)->bool
          {
            this->AddSensor(_ecm, _entity, _sensor, _parent);
            return true;
        });
    }
  }

  public: void PreUpdate(const UpdateInfo &/*_info*/,
                         EntityComponentManager &_ecm)
  {
    IGN_PROFILE("SensorImpl::PreUpdate");

    // Create components
    for (auto entity : this->newSensors)
    {
      auto it = this->entitySensorMap.find(entity);
      if (it == this->entitySensorMap.end())
      {
        ignerr << "Entity [" << entity
               << "] isn't in sensor map, this shouldn't happen." << std::endl;
        continue;
      }

      this->CreateComponents(_ecm, entity, it->second.get());
    }
    this->newSensors.clear();
  }

  /// \brief Update sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm)
  {
    IGN_PROFILE("SensorImpl::PostUpdate");

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
      this->Update(_ecm);

      for (auto &it : this->entitySensorMap)
      {
        it.second->Update(_info.simTime, false);
      }
    }

    this->RemoveSensorEntities(_ecm);
  }

  /// \brief Remove sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveSensorEntities(const EntityComponentManager &_ecm)
  {
    IGN_PROFILE("SensorImpl::RemoveSensorEntities");
    _ecm.EachRemoved<ComponentT>(
      [&](const Entity &_entity,
        const ComponentT *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
  }

  public: virtual void CreateComponents(
              EntityComponentManager &_ecm,
              const Entity _entity,
              const SensorT *_sensor) = 0;

  public: virtual void AddSensor(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const ComponentT *_sensor,
    const components::ParentEntity *_parent) = 0;

  public: virtual void Update(const EntityComponentManager &_ecm) = 0;
};
}  // namespace systems
}
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_SYSTEMS_SENSORIMPL_HH_
