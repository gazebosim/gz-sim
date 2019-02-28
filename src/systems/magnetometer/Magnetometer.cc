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

#include <ignition/msgs/magnetometer.pb.h>

#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/Magnetometer.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

#include "Magnetometer.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Magnetometer sensor class
class ignition::gazebo::systems::MagnetometerSensor
{
  /// \brief Constructor
  public: MagnetometerSensor();

  /// \brief Destructor
  public: ~MagnetometerSensor();

  /// \brief Load the magnetometer from an sdf element
  /// \param[in] _sdf SDF element describing the magnetometer
  /// \param[in] _worldField Current magnetic field in the world frame.
  /// \param[in] _topic Topic to publish messages at
  public: void Load(const sdf::ElementPtr &_sdf,
    const math::Vector3d &_worldField, const std::string &_topic);

  /// \brief Publish magnetometer data over ign transport
  public: void Publish();

  /// \brief Topic to publish data to
  public: std::string topic;

  /// \brief The latest field reading from the sensor, based on the world
  /// field and the sensor's current position.
  public: ignition::math::Vector3d localField;

  /// \brief Store world magnetic field vector. We assume it is uniform
  /// everywhere in the world, and that it doesn't change during the simulation.
  public: ignition::math::Vector3d worldField;

  /// \brief Ign transport node
  public: transport::Node node;

  /// \brief publisher for magnetometer data
  public: transport::Node::Publisher pub;

  /// \brief common::Time from when the sensor was updated
  public: common::Time lastMeasurementTime;
};

/// \brief Private Magnetometer data class.
class ignition::gazebo::systems::MagnetometerPrivate
{
  /// \brief A map of magnetometer entity to its sensor.
  public: std::unordered_map<Entity, std::unique_ptr<MagnetometerSensor>>
      entitySensorMap;

  /// \brief Create magnetometer sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateMagnetometerEntities(EntityComponentManager &_ecm);

  /// \brief Update magnetometer sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Remove magnetometer sensors if their entities have been removed
  /// from simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveMagnetometerEntities(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
MagnetometerSensor::MagnetometerSensor() = default;

//////////////////////////////////////////////////
MagnetometerSensor::~MagnetometerSensor() = default;

//////////////////////////////////////////////////
void MagnetometerSensor::Load(const sdf::ElementPtr &_sdf,
    const math::Vector3d &_worldField, const std::string &_topic)
{
  if (_sdf->HasElement("topic"))
    this->topic = _sdf->Get<std::string>("topic");
  else
    // create default topic for sensor
    this->topic = _topic;

  this->pub = this->node.Advertise<ignition::msgs::Magnetometer>(this->topic);

  // Store the world's magnetic field
  this->worldField = _worldField;
}

//////////////////////////////////////////////////
void MagnetometerSensor::Publish()
{
  msgs::Magnetometer msg;
  msg.mutable_header()->mutable_stamp()->set_sec(
      this->lastMeasurementTime.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(
      this->lastMeasurementTime.nsec);

  msgs::Set(msg.mutable_field_tesla(), this->localField);
  this->pub.Publish(msg);
}

//////////////////////////////////////////////////
Magnetometer::Magnetometer() : System(), dataPtr(
    std::make_unique<MagnetometerPrivate>())
{
}

//////////////////////////////////////////////////
Magnetometer::~Magnetometer() = default;

//////////////////////////////////////////////////
void Magnetometer::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  this->dataPtr->CreateMagnetometerEntities(_ecm);
}

//////////////////////////////////////////////////
void Magnetometer::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  if (!_info.paused)
  {
    this->dataPtr->Update(_ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Update measurement time
      auto time = math::durationToSecNsec(_info.simTime);
      it.second->lastMeasurementTime = common::Time(time.first, time.second);

      it.second->Publish();
    }
  }

  this->dataPtr->RemoveMagnetometerEntities(_ecm);
}

//////////////////////////////////////////////////
void MagnetometerPrivate::CreateMagnetometerEntities(
    EntityComponentManager &_ecm)
{
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == worldEntity)
  {
    ignerr << "Missing world entity." << std::endl;
    return;
  }

  // Get the world magnetic field (defined in world frame)
  auto worldField = _ecm.Component<components::MagneticField>(worldEntity);
  if (nullptr == worldField)
  {
    ignerr << "World missing magnetic field." << std::endl;
    return;
  }

  // Create magnetometers
  _ecm.EachNew<components::Magnetometer>(
    [&](const Entity &_entity,
        const components::Magnetometer *_magnetometer)->bool
      {
        std::string defaultTopic = scopedName(_entity, _ecm, "/") +
          "/magnetometer";

        auto sensor = std::make_unique<MagnetometerSensor>();
        sensor->Load(_magnetometer->Data(), worldField->Data(),
            defaultTopic);

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void MagnetometerPrivate::Update(
    const EntityComponentManager &_ecm)
{
  _ecm.Each<components::Magnetometer,
            components::WorldPose>(
    [&](const Entity &_entity,
        const components::Magnetometer * /*_magnetometer*/,
        const components::WorldPose *_worldPose)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          // Get the magnetometer physical position
          math::Pose3d magnetometerWorldPose = _worldPose->Data();

          // Rotate the magnetic field into the body frame
          it->second->localField =
            magnetometerWorldPose.Rot().Inverse().RotateVector(
                it->second->worldField);
        }
        else
        {
          ignerr << "Failed to update magnetometer: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
void MagnetometerPrivate::RemoveMagnetometerEntities(
    const EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<components::Magnetometer>(
    [&](const Entity &_entity,
        const components::Magnetometer *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing magnetometer sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}

IGNITION_ADD_PLUGIN(Magnetometer, System,
  Magnetometer::ISystemPreUpdate,
  Magnetometer::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(Magnetometer,
                          "ignition::gazebo::systems::Magnetometer")
