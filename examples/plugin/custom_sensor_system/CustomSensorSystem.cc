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

#include "CustomSensorSystem.hh"

using namespace custom;

//////////////////////////////////////////////////
bool CustomSensor::Load(const sdf::Sensor &_sdf)
{
  auto type = _sdf.Type();
  if (type != sdf::SensorType::CUSTOM)
  {
    ignerr << "Trying to load custom sensor, got [" << static_cast<int>(type)
           << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  ignition::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  this->pub = this->node.Advertise<ignition::msgs::Pose>(this->Topic());

  // Load custom sensor params
  auto elem = _sdf.Element();
  if (nullptr == elem)
  {
    ignerr << "SDF missing element pointer." << std::endl;
    return false;
  }

  if (!elem->HasElement("ignition:custom_sensor"))
  {
    igndbg << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return true;
  }

  auto customElem = elem->GetElement("ignition:custom_sensor");

  if (!customElem->HasElement("noise"))
  {
    igndbg << "No noise for [" << this->Topic() << "]" << std::endl;
    return true;
  }

  sdf::Noise noiseSdf;
  noiseSdf.Load(customElem->GetElement("noise"));
  this->noise = ignition::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
  if (nullptr == this->noise)
  {
    ignerr << "Failed to load noise." << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void CustomSensor::SetWorldPose(const ignition::math::Pose3d &_worldPose)
{
  this->data = _worldPose;
}

//////////////////////////////////////////////////
bool CustomSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  ignition::msgs::Pose msg;
  *msg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  if (nullptr != this->noise)
  {
    this->data.Pos().X(this->noise->Apply(this->data.Pos().X()));
    this->data.Pos().Y(this->noise->Apply(this->data.Pos().Y()));
    this->data.Pos().Z(this->noise->Apply(this->data.Pos().Z()));
    this->data.Rot().Euler(
        this->noise->Apply(this->data.Rot().Roll()),
        this->noise->Apply(this->data.Rot().Pitch()),
        this->noise->Apply(this->data.Rot().Yaw()));
  }

  ignition::msgs::Set(&msg, this->data);

  this->AddSequence(msg.mutable_header());
  this->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
void CustomSensorSystem::PreUpdate(const ignition::gazebo::UpdateInfo &,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  _ecm.EachNew<ignition::gazebo::components::CustomSensor,
               ignition::gazebo::components::ParentEntity>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *_custom,
        const ignition::gazebo::components::ParentEntity *_parent)->bool
      {
        // create sensor
        auto sensorScopedName = ignition::gazebo::removeParentScope(
            ignition::gazebo::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _custom->Data();
        data.SetName(sensorScopedName);

        // check topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/custom";
          data.SetTopic(topic);
        }

        auto sensor = std::make_unique<CustomSensor>();
        if (!sensor->Load(data))
        {
          ignerr << "Sensor::Load failed for plugin [CustomSensor]\n";
          return false;
        }
        if (!sensor->Init())
        {
          ignerr << "Sensor::Init failed for plugin [CustomSensor]\n";
          return false;
        }

        // set sensor parent
        std::string parentName =
            _ecm.Component<ignition::gazebo::components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic
        _ecm.CreateComponent(_entity,
            ignition::gazebo::components::SensorTopic(sensor->Topic()));

        this->entitySensorMap.insert(std::make_pair(_entity,
            std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void CustomSensorSystem::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                           const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  if (!_info.paused)
  {
    for (auto &[entity, sensor] : this->entitySensorMap)
    {
      sensor->SetWorldPose(ignition::gazebo::worldPose(entity, _ecm));
      sensor->Update(_info.simTime);
    }
  }

  this->RemoveSensorEntities(_ecm);
}

//////////////////////////////////////////////////
void CustomSensorSystem::RemoveSensorEntities(
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<ignition::gazebo::components::CustomSensor>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing custom sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

IGNITION_ADD_PLUGIN(CustomSensorSystem, ignition::gazebo::System,
  CustomSensorSystem::ISystemPreUpdate,
  CustomSensorSystem::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(CustomSensorSystem, "custom::CustomSensorSystem")
