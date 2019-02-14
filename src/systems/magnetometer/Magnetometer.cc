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
#include "ignition/gazebo/components/ParentEntity.hh"
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
  public: void Load(const sdf::ElementPtr &_sdf,
    const math::Vector3d &_magneticField, const std::string &_topic,
    const std::string &_sensorName);

  /// \brief Publish magnetometer data over ign transport
  public: void Publish();

  /// \brief Sensors name
  public: std::string sensorName;

  /// \brief Topic to publish data to
  public: std::string topic;

  /// \brief Message used to publish data
  public: msgs::Magnetometer magnetometerMsg;

  /// \brief Noise free linear acceleration
  public: ignition::math::Vector3d field;

  /// \brief store world magnetic field vector.
  public: ignition::math::Vector3d magneticField;

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
  public: Entity worldEntity = kNullEntity;

  /// \brief A map of magnetometer entity to its sensor.
  public: std::unordered_map<Entity, std::unique_ptr<MagnetometerSensor>>
      entitySensorMap;

  /// \brief Create magnetometer sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateMagnetometerEntities(EntityComponentManager &_ecm);

  /// \brief Update magnetometer sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
MagnetometerSensor::MagnetometerSensor() = default;

//////////////////////////////////////////////////
MagnetometerSensor::~MagnetometerSensor() = default;

//////////////////////////////////////////////////
void MagnetometerSensor::Load(const sdf::ElementPtr &_sdf,
    const math::Vector3d &_magneticField, const std::string &_topic,
    const std::string &_sensorName)
{
  if (_sdf->HasElement("topic"))
    this->topic = _sdf->Get<std::string>("topic");
  else
    // create default topic for sensor
    this->topic = _topic;

  this->pub = this->node.Advertise<ignition::msgs::Magnetometer>(this->topic);

  // Set sensors world magnetic field
  this->magneticField = _magneticField;

  // Set sensor name
  this->sensorName = _sensorName;
}

//////////////////////////////////////////////////
void MagnetometerSensor::Publish()
{
  this->magnetometerMsg.mutable_header()->mutable_stamp()->set_sec(
      this->lastMeasurementTime.sec);
  this->magnetometerMsg.mutable_header()->mutable_stamp()->set_nsec(
      this->lastMeasurementTime.nsec);

  msgs::Set(magnetometerMsg.mutable_field_tesla(),
      this->field);
  this->pub.Publish(magnetometerMsg);
}

//////////////////////////////////////////////////
Magnetometer::Magnetometer() : System(), dataPtr(
    std::make_unique<MagnetometerPrivate>())
{
}

//////////////////////////////////////////////////
Magnetometer::~Magnetometer()
{
}

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
  if (_info.paused)
    return;

  this->dataPtr->Update(_ecm);

  for (auto &it : this->dataPtr->entitySensorMap)
  {
    // Update measurement time
    auto time = math::durationToSecNsec(_info.simTime);
    it.second->lastMeasurementTime = common::Time(time.first, time.second);

    it.second->Publish();
  }
}

//////////////////////////////////////////////////
void MagnetometerPrivate::CreateMagnetometerEntities(
    EntityComponentManager &_ecm)
{
  // Get World Entity
  if (kNullEntity == this->worldEntity)
    this->worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == this->worldEntity)
  {
    ignerr << "Missing world entity." << std::endl;
    return;
  }

  // Get the world magnetic field (defined in world frame)
  auto magneticField = _ecm.Component<components::MagneticField>(worldEntity);
  if (nullptr == magneticField)
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
        std::string sensorName = _ecm.Component<components::Name>(
            _entity)->Data();

        auto sensor = std::make_unique<MagnetometerSensor>();
        sensor->Load(_magnetometer->Data(), magneticField->Data(),
            defaultTopic, sensorName);

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
          it->second->field =
            magnetometerWorldPose.Rot().Inverse().RotateVector(
                it->second->magneticField);
        }
        else
        {
          ignerr << "Failed to update magnetometer: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

IGNITION_ADD_PLUGIN(Magnetometer, System,
  Magnetometer::ISystemPreUpdate,
  Magnetometer::ISystemPostUpdate
)
