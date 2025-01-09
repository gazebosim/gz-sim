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

#include "ForceTorque.hh"

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <string>

#include <gz/plugin/Register.hh>

#include <sdf/Element.hh>

#include <gz/common/Profiler.hh>

#include <gz/transport/Node.hh>

#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/ForceTorqueSensor.hh>

#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/ForceTorque.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointTransmittedWrench.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private ForceTorque data class.
class gz::sim::systems::ForceTorquePrivate
{
  /// \brief A map of FT entity to its FT sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<gz::sensors::ForceTorqueSensor>> entitySensorMap;

  /// \brief A struct to hold the joint and link entities associated with a
  /// sensor
  public: struct SensorJointAndLinks
  {
    /// \brief The parent joint of the sensor
    Entity joint;
    /// \breif The parent link of the joint
    Entity jointParentLink;
    /// \breif The child link of the joint
    Entity jointChildLink;
  };

  /// \brief Cache of the entities associated with the sensor
  public: std::unordered_map<Entity, SensorJointAndLinks> sensorJointLinkMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<Entity> newSensors;

  /// True if the sensor is initialized
  public: bool initialized = false;

  /// \brief Get the link entity identified by the given scoped name
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _name Scoped name of the link
  /// \param[in] _parentModel The model entity in which the scope of the given
  /// name starts.
  /// \return The link entity if found, otherwise kNullEntity
  public: Entity GetLinkFromScopedName(const EntityComponentManager &_ecm,
                                       const std::string &_name,
                                       Entity _parentModel) const;

  /// \brief Create force-torque sensor
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const EntityComponentManager &_ecm);

  /// \brief Update FT sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the force-torque sensor
  /// \param[in] _forceTorque ForceTorqueSensor component.
  /// \param[in] _parent Parent entity component.
  public: void AddSensor(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::ForceTorque *_forceTorque,
    const components::ParentEntity *_parent);

  /// \brief Remove FT sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveForceTorqueEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
ForceTorque::ForceTorque()
    : dataPtr(std::make_unique<ForceTorquePrivate>())
{
}

//////////////////////////////////////////////////
ForceTorque::~ForceTorque() = default;

//////////////////////////////////////////////////
void ForceTorque::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ForceTorque::PreUpdate");

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
    // Enable JointTransmittedWrench to get force-torque measurements
    auto jointEntity =
        _ecm.Component<components::ParentEntity>(entity)->Data();
    gzdbg << "Adding JointTransmittedWrench to: " << jointEntity << std::endl;
    _ecm.CreateComponent(jointEntity, components::JointTransmittedWrench());
  }
  this->dataPtr->newSensors.clear();
}

//////////////////////////////////////////////////
void ForceTorque::PostUpdate(const UpdateInfo &_info,
                     const EntityComponentManager &_ecm)
{
  GZ_PROFILE("ForceTorque::PostUpdate");

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
    // to avoid doing work in the ForceTorquePrivate::Update function
    bool needsUpdate = false;
    for (const auto &[sensorEntity, sensor] : this->dataPtr->entitySensorMap)
    {
      if (sensor->NextDataUpdateTime() <= _info.simTime &&
          sensor->HasConnections())
      {
        needsUpdate = true;
        break;
      }
    }
    if (!needsUpdate)
      return;

    // Transform joint wrench to sensor wrench and write to sensor
    this->dataPtr->Update(_ecm);

    for (auto &[sensorEntity, sensor] : this->dataPtr->entitySensorMap)
    {
      // Call gz::sensors::ForceTorqueSensor::Update
      // * Convert to user-specified frame
      // * Apply noise
      // * Publish to gz-transport topic
      sensor->Update(_info.simTime, false);
    }
  }

  this->dataPtr->RemoveForceTorqueEntities(_ecm);
}

//////////////////////////////////////////////////
Entity ForceTorquePrivate::GetLinkFromScopedName(
    const EntityComponentManager &_ecm, const std::string &_name,
    Entity _parentModel) const
{
  auto entities = entitiesFromScopedName(_name, _ecm, _parentModel);
  for (const auto & entity : entities)
  {
    if (_ecm.EntityHasComponentType(entity, components::Link::typeId))
    {
      return entity;
    }
  }
  return kNullEntity;

}
//////////////////////////////////////////////////
void ForceTorquePrivate::CreateSensors(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("ForceTorquePrivate::CreateSensors");
  if (!this->initialized)
  {
    // Create force-torque sensors
    _ecm.Each<components::ForceTorque, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::ForceTorque *_forceTorque,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _forceTorque, _parent);
          return true;
        });
    this->initialized = true;
  }
  else
  {
    // Create force-torque sensors
    _ecm.EachNew<components::ForceTorque, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::ForceTorque *_forceTorque,
          const components::ParentEntity *_parent)->bool
        {
          this->AddSensor(_ecm, _entity, _forceTorque, _parent);
          return true;
        });
  }
}

//////////////////////////////////////////////////
void ForceTorquePrivate::Update(const EntityComponentManager &_ecm)
{
  GZ_PROFILE("ForceTorquePrivate::Update");
  _ecm.Each<components::ForceTorque>(
      [&](const Entity &_entity, const components::ForceTorque *) -> bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          auto jointLinkIt = this->sensorJointLinkMap.find(_entity);
          if (jointLinkIt == this->sensorJointLinkMap.end())
          {
            gzerr << "Failed to update Force/Torque Sensor: " << _entity
                   << ". Associated entities not found." << std::endl;
            return true;
          }

          // Return early if JointTransmittedWrench component has not yet been
          // populated by the Physics system
          auto jointWrench = _ecm.Component<components::JointTransmittedWrench>(
              jointLinkIt->second.joint);
          if (nullptr == jointWrench)
          {
            return true;
          }

          // Notation:
          // X_WJ: Pose of joint in world
          // X_WP: Pose of parent link in world
          // X_WC: Pose of child link in world
          // X_WS: Pose of sensor in world
          // X_SP: Pose of parent link in sensors frame
          // X_SC: Pose of child link in sensors frame
          const auto X_WP =
              worldPose(jointLinkIt->second.jointParentLink, _ecm);
          const auto X_WC = worldPose(jointLinkIt->second.jointChildLink, _ecm);
          // There appears to be a bug worldPose for computing poses of //joint
          // and its children, so we do it manually here.
          const auto X_CJ =
              _ecm.Component<components::Pose>(jointLinkIt->second.joint)
                  ->Data();
          auto X_WJ = X_WC * X_CJ;

          auto X_JS = _ecm.Component<components::Pose>(_entity)->Data();
          auto X_WS = X_WJ * X_JS;
          auto X_SP = X_WS.Inverse() * X_WP;

          // The joint wrench is computed at the joint frame. We need to
          // transform it the sensor frame.
          math::Vector3d force =
              X_JS.Rot().Inverse() * msgs::Convert(jointWrench->Data().force());

          math::Vector3d torque =
              X_JS.Rot().Inverse() *
                  msgs::Convert(jointWrench->Data().torque()) -
              X_JS.Pos().Cross(force);

          it->second->SetForce(force);
          it->second->SetTorque(torque);
          it->second->SetRotationParentInSensor(X_SP.Rot());
        }
        else
        {
          gzerr << "Failed to update Force/Torque Sensor: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}


//////////////////////////////////////////////////
void ForceTorquePrivate::AddSensor(
    const EntityComponentManager &_ecm,
    const Entity _entity,
    const components::ForceTorque *_forceTorque,
    const components::ParentEntity *_parent)
{
  // create sensor
  std::string sensorScopedName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _forceTorque->Data();
  data.SetName(sensorScopedName);
  // check topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/forcetorque";
    data.SetTopic(topic);
  }
  std::unique_ptr<sensors::ForceTorqueSensor> sensor =
      this->sensorFactory.CreateSensor<
      sensors::ForceTorqueSensor>(data);
  if (nullptr == sensor)
  {
    gzerr << "Failed to create sensor [" << sensorScopedName << "]"
           << std::endl;
    return;
  }

  auto jointEntity = _parent->Data();
  const std::string jointName =
      _ecm.Component<components::Name>(jointEntity)->Data();

  // Parent has to be a joint
  if (!_ecm.EntityHasComponentType(jointEntity,
                                   components::Joint::typeId))
  {
    gzerr << "Parent entity of sensor [" << sensorScopedName
           << "] must be a joint. Failed to create sensor." << std::endl;
    return;
  }

  const auto modelEntity =
      _ecm.Component<components::ParentEntity>(jointEntity)->Data();

  // Find the joint parent and child links
  const auto jointParentName =
      _ecm.Component<components::ParentLinkName>(jointEntity)->Data();
  auto jointParentLinkEntity =
      this->GetLinkFromScopedName(_ecm, jointParentName, modelEntity);

  if (kNullEntity == jointParentLinkEntity)
  {
    gzerr << "Parent link with name [" << jointParentName
           << "] of joint with name [" << jointName
           << "] not found. Failed to create sensor [" << sensorScopedName
           << "]" << std::endl;
    return;
  }

  const auto jointChildName =
      _ecm.Component<components::ChildLinkName>(jointEntity)->Data();
  auto jointChildLinkEntity =
      this->GetLinkFromScopedName(_ecm, jointChildName, modelEntity);
  if (kNullEntity == jointChildLinkEntity)
  {
    gzerr << "Child link with name [" << jointChildName
           << "] of joint with name [" << jointName
           << "] not found. Failed to create sensor [" << sensorScopedName
           << "]" << std::endl;
    return;
  }

  SensorJointAndLinks sensorJointLinkEntry;
  sensorJointLinkEntry.joint = jointEntity;
  sensorJointLinkEntry.jointParentLink = jointParentLinkEntity;
  sensorJointLinkEntry.jointChildLink = jointChildLinkEntity;
  this->sensorJointLinkMap[_entity] = sensorJointLinkEntry;

  const auto X_WC = worldPose(jointChildLinkEntity, _ecm);
  const auto X_CJ = _ecm.Component<components::Pose>(jointEntity)->Data();
  const auto X_WJ = X_WC * X_CJ;
  const auto X_JS = _ecm.Component<components::Pose>(_entity)->Data();
  const auto X_WS = X_WJ * X_JS;
  const auto X_SC = X_WS.Inverse() * X_WC;
  sensor->SetRotationChildInSensor(X_SC.Rot());

  this->entitySensorMap.insert(
      std::make_pair(_entity, std::move(sensor)));
  this->newSensors.insert(_entity);
}

//////////////////////////////////////////////////
void ForceTorquePrivate::RemoveForceTorqueEntities(
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("ForceTorquePrivate::RemoveForceTorqueEntities");
  _ecm.EachRemoved<components::ForceTorque>(
    [&](const Entity &_entity,
        const components::ForceTorque *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          gzerr << "Internal error, missing FT sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

GZ_ADD_PLUGIN(ForceTorque, System,
  ForceTorque::ISystemPreUpdate,
  ForceTorque::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(ForceTorque, "gz::sim::systems::ForceTorque")
