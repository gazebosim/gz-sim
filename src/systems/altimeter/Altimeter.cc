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

#include <ignition/msgs/altimeter.pb.h>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/AltimeterSensor.hh>

#include "ignition/gazebo/components/Altimeter.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

#include "Altimeter.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private Altimeter data class.
class ignition::gazebo::systems::AltimeterPrivate
{
  /// \brief A map of altimeter entity to its vertical reference
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::AltimeterSensor>> entitySensorMap;

  /// \brief Ign-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Create altimeter sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateAltimeterEntities(EntityComponentManager &_ecm);

  /// \brief Update altimeter sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateAltimeters(const EntityComponentManager &_ecm);

  /// \brief Remove altimeter sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveAltimeterEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
Altimeter::Altimeter() : System(), dataPtr(std::make_unique<AltimeterPrivate>())
{
}

//////////////////////////////////////////////////
Altimeter::~Altimeter() = default;

//////////////////////////////////////////////////
void Altimeter::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("Altimeter::PreUpdate");
  this->dataPtr->CreateAltimeterEntities(_ecm);
}

//////////////////////////////////////////////////
void Altimeter::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  IGN_PROFILE("Altimeter::PostUpdate");

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
    this->dataPtr->UpdateAltimeters(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      auto time = math::durationToSecNsec(_info.simTime);
      dynamic_cast<sensors::Sensor *>(it.second.get())->Update(
          common::Time(time.first, time.second), false);
    }
  }

  this->dataPtr->RemoveAltimeterEntities(_ecm);
}

//////////////////////////////////////////////////
void AltimeterPrivate::CreateAltimeterEntities(EntityComponentManager &_ecm)
{
  IGN_PROFILE("Altimeter::CreateAltimeterEntities");
  // Create altimeters
  _ecm.EachNew<components::Altimeter, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::Altimeter *_altimeter,
        const components::ParentEntity *_parent)->bool
      {
        // create sensor
        std::string sensorScopedName =
            removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _altimeter->Data();
        data.SetName(sensorScopedName);
        // check topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/altimeter";
          data.SetTopic(topic);
        }
        std::unique_ptr<sensors::AltimeterSensor> sensor =
            this->sensorFactory.CreateSensor<
            sensors::AltimeterSensor>(data);
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

        // Get initial pose of sensor and set the reference z pos
        // The WorldPose component was just created and so it's empty
        // We'll compute the world pose manually here
        double verticalReference = worldPose(_entity, _ecm).Pos().Z();
        sensor->SetVerticalReference(verticalReference);
        sensor->SetPosition(verticalReference);

        // Set topic
        _ecm.CreateComponent(_entity, components::SensorTopic(sensor->Topic()));

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void AltimeterPrivate::UpdateAltimeters(const EntityComponentManager &_ecm)
{
  IGN_PROFILE("Altimeter::UpdateAltimeters");
  _ecm.Each<components::Altimeter, components::WorldPose,
            components::WorldLinearVelocity>(
    [&](const Entity &_entity,
        const components::Altimeter * /*_altimeter*/,
        const components::WorldPose *_worldPose,
        const components::WorldLinearVelocity *_worldLinearVel)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          math::Vector3d linearVel;
          math::Pose3d worldPose = _worldPose->Data();
          it->second->SetPosition(worldPose.Pos().Z());
          it->second->SetVerticalVelocity(_worldLinearVel->Data().Z());
        }
        else
        {
          ignerr << "Failed to update altimeter: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void AltimeterPrivate::RemoveAltimeterEntities(
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("Altimeter::RemoveAltimeterEntities");
  _ecm.EachRemoved<components::Altimeter>(
    [&](const Entity &_entity,
        const components::Altimeter *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing altimeter sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

IGNITION_ADD_PLUGIN(Altimeter, System,
  Altimeter::ISystemPreUpdate,
  Altimeter::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(Altimeter, "ignition::gazebo::systems::Altimeter")
