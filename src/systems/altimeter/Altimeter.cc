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

#include "Altimeter.hh"

#include <gz/msgs/altimeter.pb.h>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <gz/math/Helpers.hh>
#include <gz/transport/Node.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/AltimeterSensor.hh>

#include "gz/sim/components/Altimeter.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private Altimeter data class.
class gz::sim::systems::AltimeterPrivate
{
  /// \brief A map of altimeter entity to its sensor
  public: std::unordered_map<Entity,
      std::unique_ptr<sensors::AltimeterSensor>> entitySensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// True if the rendering component is initialized
  public: bool initialized = false;

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the IMU
  /// \param[in] _altimeter Altimeter component.
  /// \param[in] _parent Parent entity component.
  public: void AddAltimeter(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::Altimeter *_altimeter,
    const components::ParentEntity *_parent);

  /// \brief Create altimeter sensor
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

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
  GZ_PROFILE("Altimeter::PreUpdate");

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
void Altimeter::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Altimeter::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the AltimeterPrivate::UpdateAltimeters function
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

    this->dataPtr->UpdateAltimeters(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      it.second->Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveAltimeterEntities(_ecm);
}

//////////////////////////////////////////////////
void AltimeterPrivate::AddAltimeter(
  const EntityComponentManager &_ecm,
  const Entity _entity,
  const components::Altimeter *_altimeter,
  const components::ParentEntity *_parent)
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
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
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

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void AltimeterPrivate::CreateSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Altimeter::CreateAltimeterEntities");
  if (!this->initialized)
  {
    // Create altimeters
    _ecm.Each<components::Altimeter, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Altimeter *_altimeter,
          const components::ParentEntity *_parent)->bool
        {
          this->AddAltimeter(_ecm, _entity, _altimeter, _parent);
          return true;
        });
    this->initialized = true;
  }
  else
  {
    // Create altimeters
    _ecm.EachNew<components::Altimeter, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Altimeter *_altimeter,
          const components::ParentEntity *_parent)->bool
        {
          this->AddAltimeter(_ecm, _entity, _altimeter, _parent);
          return true;
        });
  }
}

//////////////////////////////////////////////////
void AltimeterPrivate::UpdateAltimeters(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Altimeter::UpdateAltimeters");
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
          math::Pose3d worldPose = _worldPose->Data();
          it->second->SetPosition(worldPose.Pos().Z());
          it->second->SetVerticalVelocity(_worldLinearVel->Data().Z());
        }
        else
        {
          gzerr << "Failed to update altimeter: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void AltimeterPrivate::RemoveAltimeterEntities(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Altimeter::RemoveAltimeterEntities");
  _ecm.EachRemoved<components::Altimeter>(
    [&](const Entity &_entity,
        const components::Altimeter *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing altimeter sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

GZ_ADD_PLUGIN(Altimeter, System,
  Altimeter::ISystemPreUpdate,
  Altimeter::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(Altimeter, "gz::sim::systems::Altimeter")
