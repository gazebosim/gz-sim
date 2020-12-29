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

#include "Gps.hh"

#include <ignition/msgs/gps.pb.h>

#include <string>
#include <unordered_map>
#include <utility>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/GpsSensor.hh>

#include "ignition/gazebo/components/Gps.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private Gps data class.
class ignition::gazebo::systems::GpsPrivate
{
  /// \brief A map of gps entity to its vertical reference
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::GpsSensor>> entitySensorMap;

  /// \brief Ign-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Create gps sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateGpsEntities(EntityComponentManager &_ecm);

  /// \brief Update gps sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateGps(const EntityComponentManager &_ecm);

  /// \brief Remove gps sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveGpsEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
Gps::Gps() : System(), dataPtr(std::make_unique<GpsPrivate>())
{
}

//////////////////////////////////////////////////
Gps::~Gps() = default;

//////////////////////////////////////////////////
void Gps::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("Gps::PreUpdate");
  this->dataPtr->CreateGpsEntities(_ecm);
}

//////////////////////////////////////////////////
void Gps::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  IGN_PROFILE("Gps::PostUpdate");

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
    this->dataPtr->UpdateGps(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      auto time = math::durationToSecNsec(_info.simTime);
      dynamic_cast<sensors::Sensor *>(it.second.get())->Update(
          math::secNsecToDuration(time.first, time.second), false);
    }
  }

  this->dataPtr->RemoveGpsEntities(_ecm);
}

//////////////////////////////////////////////////
void GpsPrivate::CreateGpsEntities(EntityComponentManager &_ecm)
{
  IGN_PROFILE("Gps::CreateGpsEntities");
  // Create gps
  _ecm.EachNew<components::Gps, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::Gps *_gps,
        const components::ParentEntity *_parent)->bool
      {
        // create sensor
        std::string sensorScopedName =
            removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _gps->Data();
        data.SetName(sensorScopedName);
        // check topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/gps";
          data.SetTopic(topic);
        }
        std::unique_ptr<sensors::GpsSensor> sensor =
            this->sensorFactory.CreateSensor<
            sensors::GpsSensor>(data);
        if (nullptr == sensor)
        {
          ignerr << "Failed to create sensor [" << sensorScopedName << "]"
                 << std::endl;
          return true;
        }

        igndbg << "Creating GPS Entity" << std::endl;

        // set sensor parent
        std::string parentName = _ecm.Component<components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        sensor->SetLatitude(0.0);
        sensor->SetLongitude(0.0);

        // Set topic
        _ecm.CreateComponent(_entity, components::SensorTopic(sensor->Topic()));

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void GpsPrivate::UpdateGps(const EntityComponentManager &_ecm)
{
  IGN_PROFILE("Gps::UpdateGps");
  _ecm.Each<components::Gps, components::WorldPose>(
    [&](const Entity &_entity,
        const components::Gps * /*_gps*/,
        const components::WorldPose *_worldPose)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          math::Vector3d linearVel;
          math::Pose3d worldPose = _worldPose->Data();
          it->second->SetLatitude(worldPose.Pos().X());
          it->second->SetLongitude(worldPose.Pos().Y());
          //it->second->SetPosition(worldPose.Pos().Z());
          //it->second->SetVerticalVelocity(_worldLinearVel->Data().Z());
        }
        else
        {
          ignerr << "Failed to update gps: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void GpsPrivate::RemoveGpsEntities(
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("Gps::RemoveGpsEntities");
  _ecm.EachRemoved<components::Gps>(
    [&](const Entity &_entity,
        const components::Gps *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing gps sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

IGNITION_ADD_PLUGIN(Gps, System,
  Gps::ISystemPreUpdate,
  Gps::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(Gps, "ignition::gazebo::systems::Gps")
