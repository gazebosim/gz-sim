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
#ifndef GZ_SIM_SYSTEMS_CPU_LIDAR_HH_
#define GZ_SIM_SYSTEMS_CPU_LIDAR_HH_

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <gz/sensors/CpuLidarSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include "gz/sim/components/CpuLidar.hh"
#include "gz/sim/components/ParentEntity.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \class CpuLidar CpuLidar.hh gz/sim/systems/CpuLidar.hh
  /// \brief This system manages all CPU-based lidar sensors in simulation.
  /// Each sensor publishes lidar scan data over Gazebo Transport.
  class CpuLidar:
    public System,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: CpuLidar();

    /// \brief Destructor
    public: ~CpuLidar() override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    private: void CreateSensors(const EntityComponentManager &_ecm);

    private: void Update(const EntityComponentManager &_ecm);

    private: void AddSensor(
      const EntityComponentManager &_ecm,
      const Entity _entity,
      const components::CpuLidar *_cpuLidar,
      const components::ParentEntity *_parent);

    private: void RemoveSensorEntities(const EntityComponentManager &_ecm);

    private: std::unordered_map<Entity,
        std::unique_ptr<sensors::CpuLidarSensor>> entitySensorMap;

    private: sensors::SensorFactory sensorFactory;

    private: std::unordered_set<Entity> newSensors;

    private: Entity worldEntity = kNullEntity;

    private: bool initialized = false;
  };
  }
}
}
}
#endif
