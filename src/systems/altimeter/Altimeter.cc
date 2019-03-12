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

#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Altimeter.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

#include "Altimeter.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Altimeter sensor class
class ignition::gazebo::systems::AltimeterSensor
{
  /// \brief Constructor
  public: AltimeterSensor();

  /// \brief Destructor
  public: ~AltimeterSensor();

  /// \brief Load the altimeter from an sdf element
  /// \param[in] _sdf SDF element describing the altimeter
  public: void Load(const sdf::ElementPtr &_sdf);

  /// \brief Publish altimeter data over ign transport
  public: void Publish();

  /// \brief Topic to publish data to
  public: std::string topic;

  /// \brief Vertical position in meters
  public: double verticalPosition = 0.0;

  /// \brief Vertical velocity in meters per second
  public: double verticalVelocity = 0.0;

  /// \brief Vertical reference, i.e. initial sensor position
  public: double verticalReference = 0.0;

  /// \brief Ign transport node
  public: transport::Node node;

  /// \brief publisher for altimeter data
  public: transport::Node::Publisher pub;

  /// \brief common::Time from when the sensor was updated
  public: common::Time lastMeasurementTime;
};

/// \brief Private Altimeter data class.
class ignition::gazebo::systems::AltimeterPrivate
{
  /// \brief A map of altimeter entity to its vertical reference
  public: std::unordered_map<Entity, std::unique_ptr<AltimeterSensor>>
      entitySensorMap;

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
AltimeterSensor::AltimeterSensor() = default;

//////////////////////////////////////////////////
AltimeterSensor::~AltimeterSensor() = default;

//////////////////////////////////////////////////
void AltimeterSensor::Load(const sdf::ElementPtr &_sdf)
{
  if (_sdf->HasElement("topic"))
    this->topic = _sdf->Get<std::string>("topic");
}

//////////////////////////////////////////////////
void AltimeterSensor::Publish()
{
  if (this->topic.empty())
    return;

  if (!this->pub)
  {
    this->pub = this->node.Advertise<ignition::msgs::Altimeter>(this->topic);
    ignmsg << "Altimeter publishing messages on [" << this->topic << "]"
           << std::endl;
  }

  msgs::Altimeter msg;
  msg.mutable_header()->mutable_stamp()->set_sec(
      this->lastMeasurementTime.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(
      this->lastMeasurementTime.nsec);
  msg.set_vertical_position(this->verticalPosition);
  msg.set_vertical_velocity(this->verticalVelocity);
  msg.set_vertical_reference(this->verticalReference);
  this->pub.Publish(msg);
}

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
  this->dataPtr->CreateAltimeterEntities(_ecm);
}

//////////////////////////////////////////////////
void Altimeter::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  if (!_info.paused)
  {
    this->dataPtr->UpdateAltimeters(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      auto time = math::durationToSecNsec(_info.simTime);
      it.second->lastMeasurementTime = common::Time(time.first, time.second);

      it.second->Publish();
    }
  }

  this->dataPtr->RemoveAltimeterEntities(_ecm);
}

//////////////////////////////////////////////////
void AltimeterPrivate::CreateAltimeterEntities(EntityComponentManager &_ecm)
{
  // Create altimeters
  _ecm.EachNew<components::Altimeter>(
    [&](const Entity &_entity,
        const components::Altimeter *_altimeter)->bool
      {
        // Get initial pose of parent link and set the reference z pos
        // The WorldPose component was just created and so it's empty
        // We'll compute the world pose manually here
        double verticalReference = worldPose(_entity, _ecm).Pos().Z();
        auto sensor = std::make_unique<AltimeterSensor>();
        sensor->Load(_altimeter->Data());
        sensor->verticalReference = verticalReference;

        // create default topic for sensor if not specified
        if (sensor->topic.empty())
        {
          sensor->topic = scopedName(_entity, _ecm, "/") + "/altimeter";
        }

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void AltimeterPrivate::UpdateAltimeters(const EntityComponentManager &_ecm)
{
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
          double pos = worldPose.Pos().Z() - it->second->verticalReference;
          double vel = _worldLinearVel->Data().Z();
          it->second->verticalPosition = pos;
          it->second->verticalVelocity = vel;
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
