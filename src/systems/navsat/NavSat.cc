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

#include "NavSat.hh"

#include <ignition/msgs/navsat.pb.h>

#include <string>
#include <unordered_map>
#include <utility>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include <ignition/sensors/SensorFactory.hh>
#include <ignition/sensors/NavSatSensor.hh>

#include "ignition/gazebo/components/NavSat.hh"
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

/// \brief Private NavSat data class.
class ignition::gazebo::systems::NavSatPrivate
{
  /// \brief A map of NavSat entity to its vertical reference
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::NavSatSensor>> entitySensorMap;

  /// \brief Ign-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Create NavSat sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateNavSatEntities(EntityComponentManager &_ecm);

  /// \brief Update NavSat sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateNavSat(const EntityComponentManager &_ecm);

  /// \brief Remove NavSat sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveNavSatEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
NavSat::NavSat() : System(), dataPtr(std::make_unique<NavSatPrivate>())
{
}

//////////////////////////////////////////////////
NavSat::~NavSat() = default;

//////////////////////////////////////////////////
void NavSat::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("NavSat::PreUpdate");
  this->dataPtr->CreateNavSatEntities(_ecm);
}

//////////////////////////////////////////////////
void NavSat::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  IGN_PROFILE("NavSat::PostUpdate");

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
    this->dataPtr->UpdateNavSat(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      auto time = math::durationToSecNsec(_info.simTime);
      dynamic_cast<sensors::Sensor *>(it.second.get())->Update(
          math::secNsecToDuration(time.first, time.second), false);
    }
  }

  this->dataPtr->RemoveNavSatEntities(_ecm);
}

//////////////////////////////////////////////////
void NavSatPrivate::CreateNavSatEntities(EntityComponentManager &_ecm)
{
  IGN_PROFILE("NavSat::CreateNavSatEntities");
  // Create NavSat
  _ecm.EachNew<components::NavSat, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::NavSat *_navsat,
        const components::ParentEntity *_parent)->bool
      {
        // create sensor
        std::string sensorScopedName =
            removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _navsat->Data();
        data.SetName(sensorScopedName);
        // check topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/navsat";
          data.SetTopic(topic);
        }
        std::unique_ptr<sensors::NavSatSensor> sensor =
            this->sensorFactory.CreateSensor<
            sensors::NavSatSensor>(data);
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
        // Set topic
        _ecm.CreateComponent(_entity, components::SensorTopic(sensor->Topic()));

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        igndbg << "Creating NavSat Entity" << std::endl;

        return true;
      });
}

//////////////////////////////////////////////////
void NavSatPrivate::UpdateNavSat(const EntityComponentManager &_ecm)
{
  IGN_PROFILE("NavSat::UpdateNavSat");

  _ecm.Each<components::NavSat, components::WorldPose,
   components::WorldLinearVelocity>(
    [&](const Entity &_entity,
        const components::NavSat * /*_navsat*/,
        const components::WorldPose *_worldPose,
        const components::WorldLinearVelocity *_worldLinearVel
        )->bool
      {
        auto it = this->entitySensorMap.find(_entity);

        if (it != this->entitySensorMap.end())
        {
          math::Vector3d worldCoords;
          math::SphericalCoordinates sphericalCoords;
          math::Vector3d worldvel = _worldLinearVel->Data();

          worldCoords = sphericalCoords.PositionTransform(_worldPose->Data().Pos(),
          math::SphericalCoordinates::GLOBAL,
          math::SphericalCoordinates::SPHERICAL);

          it->second->SetLatitude(worldCoords.X());
          it->second->SetLongitude(worldCoords.Y());
          it->second->SetAltitude(worldCoords.Z());
          it->second->SetVelocity(worldvel);
        }
        else
        {
          ignerr << "Failed to update NavSat: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void NavSatPrivate::RemoveNavSatEntities(
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("NavSat::RemoveNavSatEntities");
  _ecm.EachRemoved<components::NavSat>(
    [&](const Entity &_entity,
        const components::NavSat *)->bool
      {
        igndbg << "Removing NavSat entity" << std::endl;
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing NavSat sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

IGNITION_ADD_PLUGIN(NavSat, System,
  NavSat::ISystemPreUpdate,
  NavSat::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(NavSat, "ignition::gazebo::systems::NavSat")
