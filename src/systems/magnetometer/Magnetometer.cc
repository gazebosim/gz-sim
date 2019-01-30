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
  public: void Load(const sdf::ElementPtr &_sdf);

  /// \brief Publish magnetometer data over ign transport
  public: void Publish();

  /// \brief Topic to publish data to
  public: std::string topic;

  /// \brief Noise free linear acceleration
  public: ignition::math::Vector3d field;

  /// \brief store world magnetic field vector.
  public: ignition::math::Vector3d magneticField;

  /// \brief Ign transport node
  public: transport::Node node;

  /// \brief publisher for magnetometer data
  public: transport::Node::Publisher pub;
};

/// \brief Private Magnetometer data class.
class ignition::gazebo::systems::MagnetometerPrivate
{
  /// \brief Used to store whether objects have been created.
  public: bool initialized = false;

  /// \brief A map of magnetometer entity to its vertical reference
  public: std::unordered_map<Entity, std::unique_ptr<MagnetometerSensor>>
      entitySensorMap;

  /// \brief Create magnetometer sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateMagnetometerEntities(EntityComponentManager &_ecm);

  /// \brief Update magnetometer sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateMagnetometers(const EntityComponentManager &_ecm);

  /// \brief Helper function to generate default topic name for the sensor
  /// \param[in] _entity Entity to get the world pose for
  /// \param[in] _ecm Immutable reference to ECM.
  public: std::string DefaultTopic(const Entity &_entity,
    const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
MagnetometerSensor::MagnetometerSensor()
{
}

//////////////////////////////////////////////////
MagnetometerSensor::~MagnetometerSensor()
{
}

//////////////////////////////////////////////////
void MagnetometerSensor::Load(const sdf::ElementPtr &_sdf)
{
  if (_sdf->HasElement("topic"))
    this->topic = _sdf->Get<std::string>("topic");
}

//////////////////////////////////////////////////
void MagnetometerSensor::Publish()
{
  if (this->topic.empty())
    return;

  if (!this->pub)
    this->pub = this->node.Advertise<ignition::msgs::Magnetometer>(this->topic);

  msgs::Magnetometer msg;
  msgs::Set(msg.mutable_field_tesla(),
      this->field);
  this->pub.Publish(msg);
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
  if (!this->dataPtr->initialized)
  {
    this->dataPtr->CreateMagnetometerEntities(_ecm);
    this->dataPtr->initialized = true;
  }
}

//////////////////////////////////////////////////
void Magnetometer::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateMagnetometers(_ecm);

  for (auto &it : this->dataPtr->entitySensorMap)
  {
    it.second->Publish();
  }
}

//////////////////////////////////////////////////
void MagnetometerPrivate::CreateMagnetometerEntities(
    EntityComponentManager &_ecm)
{
  // Create magnetometers
  _ecm.Each<components::Magnetometer>(
    [&](const Entity &_entity,
        const components::Magnetometer *_magnetometer)->bool
      {
        // Get initial pose of parent link and set the reference z pos
        // The WorldPose component was just created and so it's empty
        // We'll compute the world pose manually here
        auto sensor = std::make_unique<MagnetometerSensor>();
        sensor->Load(_magnetometer->Data());

        // create default topic for sensor if not specified
        if (sensor->topic.empty())
          sensor->topic = this->DefaultTopic(_entity, _ecm);

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void MagnetometerPrivate::UpdateMagnetometers(
    const EntityComponentManager &_ecm)
{
  _ecm.Each<components::Magnetometer,
            components::WorldPose,
            components::MagneticField>(
    [&](const Entity &_entity,
        const components::Magnetometer * /*_magnetometer*/,
        const components::WorldPose *_worldPose,
        const components::MagneticField *_worldMagneticField)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          // Get the world magnetic field
          it->second->magneticField = _worldMagneticField->Data();

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

//////////////////////////////////////////////////
std::string MagnetometerPrivate::DefaultTopic(const Entity &_entity,
    const EntityComponentManager &_ecm)
{
  // default topic name:
  // /model/model_name/link/link_name/sensor/sensor_name/magnetometer
  std::string sensorName = _ecm.Component<components::Name>(_entity)->Data();
  auto p = _ecm.Component<components::ParentEntity>(_entity);
  std::string linkName = _ecm.Component<components::Name>(p->Data())->Data();
  std::string topic =
      "/link/" + linkName + "/sensor/" + sensorName + "/magnetometer";
  p = _ecm.Component<components::ParentEntity>(p->Data());
  // also handle nested models
  while (p)
  {
    std::string modelName = _ecm.Component<components::Name>(p->Data())->Data();
    topic = "/model/" + modelName + topic;

    // keep going up the tree
    p = _ecm.Component<components::ParentEntity>(p->Data());
  }
  return topic;
}

IGNITION_ADD_PLUGIN(Magnetometer, System,
  Magnetometer::ISystemPreUpdate,
  Magnetometer::ISystemPostUpdate
)
