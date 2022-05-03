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
#include "gz_sensor/GzSensorImpl.hh"

#include <ignition/msgs/air_pressure_sensor.pb.h>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <ignition/plugin/Register.hh>

#include <ignition/common/Profiler.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>

#include <ignition/sensors/AirPressureSensor.hh>

#include "ignition/gazebo/components/AirPressureSensor.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private AirPressure data class.
class ignition::gazebo::systems::AirPressurePrivate
: public GzSensorImpl<sensors::AirPressureSensor, components::AirPressureSensor>
{
  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the IMU
  /// \param[in] _airPressure AirPressureSensor component.
  /// \param[in] _parent Parent entity component.
  public: void AddSensor(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::AirPressureSensor *_airPressure,
    const components::ParentEntity *_parent) override;

  public: void CreateComponents(
    EntityComponentManager &_ecm,
    const Entity _entity,
    const sensors::AirPressureSensor *_sensor) override;

  public: void Update(const EntityComponentManager &_ecm) override;
};

//////////////////////////////////////////////////
AirPressure::AirPressure() :
  System(), dataPtr(std::make_unique<AirPressurePrivate>())
{
}

//////////////////////////////////////////////////
AirPressure::~AirPressure() = default;

//////////////////////////////////////////////////
void AirPressure::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  this->dataPtr->PreUpdate(_info, _ecm);
}

//////////////////////////////////////////////////
void AirPressure::PostUpdate(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
{
  this->dataPtr->PostUpdate(_info, _ecm);
}

//////////////////////////////////////////////////
void AirPressurePrivate::CreateComponents(
    EntityComponentManager &_ecm,
    const Entity _entity,
    const sensors::AirPressureSensor *_sensor)
{
  // Set topic
  _ecm.CreateComponent(_entity, components::SensorTopic(_sensor->Topic()));
}

//////////////////////////////////////////////////
void AirPressurePrivate::AddSensor(
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
    ignerr << "Failed to create sensor [" << sensorScopedName << "]"
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
void AirPressurePrivate::Update(const EntityComponentManager &_ecm)
{
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

IGNITION_ADD_PLUGIN(AirPressure, System,
  AirPressure::ISystemPreUpdate,
  AirPressure::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(AirPressure, "ignition::gazebo::systems::AirPressure")
