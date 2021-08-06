/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/msgs/double.pb.h>

#include <string>
#include <unordered_map>
#include <utility>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/sensors/Noise.hh>
#include <ignition/sensors/SensorFactory.hh>

#include <sdf/Sensor.hh>

#include <ignition/gazebo/components/CustomSensor.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>

#include "Odometer.hh"
#include "OdometerSystem.hh"

using namespace custom;

//////////////////////////////////////////////////
void OdometerSystem::PreUpdate(const ignition::gazebo::UpdateInfo &,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  _ecm.EachNew<ignition::gazebo::components::CustomSensor,
               ignition::gazebo::components::ParentEntity>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *_custom,
        const ignition::gazebo::components::ParentEntity *_parent)->bool
      {
        // Get sensor's scoped name without the world
        auto sensorScopedName = ignition::gazebo::removeParentScope(
            ignition::gazebo::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _custom->Data();
        data.SetName(sensorScopedName);

        // Default to scoped name as topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/odometer";
          data.SetTopic(topic);
        }

        ignition::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<custom::Odometer>(data);
        if (nullptr == sensor)
        {
          ignerr << "Failed to create odometer [" << sensorScopedName << "]"
                 << std::endl;
          return false;
        }

        // Set sensor parent
        auto parentName = _ecm.Component<ignition::gazebo::components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic on Gazebo
        _ecm.CreateComponent(_entity,
            ignition::gazebo::components::SensorTopic(sensor->Topic()));

        // Keep track of this sensor
        this->entitySensorMap.insert(std::make_pair(_entity,
            std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void OdometerSystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  if (!_info.paused)
  {
    for (auto &[entity, sensor] : this->entitySensorMap)
    {
      sensor->NewPosition(ignition::gazebo::worldPose(entity, _ecm).Pos());
      sensor->Update(_info.simTime);
    }
  }

  this->RemoveSensorEntities(_ecm);
}

//////////////////////////////////////////////////
void OdometerSystem::RemoveSensorEntities(
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<ignition::gazebo::components::CustomSensor>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing odometer for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

IGNITION_ADD_PLUGIN(OdometerSystem, ignition::gazebo::System,
  OdometerSystem::ISystemPreUpdate,
  OdometerSystem::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(OdometerSystem, "custom::OdometerSystem")
