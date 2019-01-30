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

#include <ignition/msgs/imu.pb.h>

#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Imu.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

#include "Imu.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Imu sensor class
class ignition::gazebo::systems::ImuSensor
{
  /// \brief Constructor
  public: ImuSensor();

  /// \brief Destructor
  public: ~ImuSensor();

  /// \brief Load the imu from an sdf element
  /// \param[in] _sdf SDF element describing the imu
  public: void Load(const sdf::ElementPtr &_sdf);

  /// \brief Publish imu data over ign transport
  public: void Publish();

  /// \brief Topic to publish data to
  public: std::string topic;

  /// \brief Noise free linear acceleration
  public: ignition::math::Vector3d linearAcc;

  /// \brief Noise free angular velocity.
  public: ignition::math::Vector3d angularVel;

  /// \brief transform from world frame to Imu reference frame.
  public: ignition::math::Quaterniond worldToReference;

  /// \brief transform from Imu frame to Imu reference frame.
  ignition::math::Quaterniond imuReferenceOrientation;

  /// \brief Save previous imu linear velocity in the specified frame
  /// for computing acceleration in the specified frame.
  /// \sa worldToReference
  public: ignition::math::Vector3d lastImuWorldLinearVel;

  /// \brief store gravity vector to be added to the imu output.
  public: ignition::math::Vector3d gravity;

  /// \brief Ign transport node
  public: transport::Node node;

  /// \brief publisher for imu data
  public: transport::Node::Publisher pub;
};

/// \brief Private Imu data class.
class ignition::gazebo::systems::ImuPrivate
{
  /// \brief Used to store whether objects have been created.
  public: bool initialized = false;

  /// \brief A map of imu entity to its vertical reference
  public: std::unordered_map<Entity, std::unique_ptr<ImuSensor>>
      entitySensorMap;

  /// \brief Create imu sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateImuEntities(EntityComponentManager &_ecm);

  /// \brief Update imu sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateImus(const EntityComponentManager &_ecm);

  /// \brief Helper function to generate default topic name for the sensor
  /// \param[in] _entity Entity to get the world pose for
  /// \param[in] _ecm Immutable reference to ECM.
  public: std::string DefaultTopic(const Entity &_entity,
    const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
ImuSensor::ImuSensor()
{
}

//////////////////////////////////////////////////
ImuSensor::~ImuSensor()
{
}

//////////////////////////////////////////////////
void ImuSensor::Load(const sdf::ElementPtr &_sdf)
{
  if (_sdf->HasElement("topic"))
    this->topic = _sdf->Get<std::string>("topic");
}

//////////////////////////////////////////////////
void ImuSensor::Publish()
{
  if (this->topic.empty())
    return;

  if (!this->pub)
    this->pub = this->node.Advertise<ignition::msgs::IMU>(this->topic);

  msgs::IMU msg;
  msgs::Set(msg.mutable_orientation(),
      this->imuReferenceOrientation);
  msgs::Set(msg.mutable_angular_velocity(),
      this->angularVel);
  msgs::Set(msg.mutable_linear_acceleration(),
      this->linearAcc);
  this->pub.Publish(msg);
}

//////////////////////////////////////////////////
Imu::Imu() : System(), dataPtr(std::make_unique<ImuPrivate>())
{
}

//////////////////////////////////////////////////
Imu::~Imu()
{
}

//////////////////////////////////////////////////
void Imu::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized)
  {
    this->dataPtr->CreateImuEntities(_ecm);
    this->dataPtr->initialized = true;
  }
}

//////////////////////////////////////////////////
void Imu::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateImus(_ecm);

  for (auto &it : this->dataPtr->entitySensorMap)
  {
    it.second->Publish();
  }
}

//////////////////////////////////////////////////
void ImuPrivate::CreateImuEntities(EntityComponentManager &_ecm)
{
  // Create imus
  _ecm.Each<components::Imu>(
    [&](const Entity &_entity,
        const components::Imu *_imu)->bool
      {
        // Get initial pose of parent link and set the reference z pos
        // The WorldPose component was just created and so it's empty
        // We'll compute the world pose manually here
        auto sensor = std::make_unique<ImuSensor>();
        sensor->Load(_imu->Data());

        // create default topic for sensor if not specified
        if (sensor->topic.empty())
          sensor->topic = this->DefaultTopic(_entity, _ecm);

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void ImuPrivate::UpdateImus(const EntityComponentManager &_ecm)
{
  _ecm.Each<components::Imu,
            components::WorldPose,
            components::Gravity,
            components::AngularVelocity,
            components::LinearAcceleration>(
    [&](const Entity &_entity,
        const components::Imu * /*_imu*/,
        const components::WorldPose *_worldPose,
        const components::Gravity *_worldGravity,
        const components::AngularVelocity *_angularVel,
        const components::LinearAcceleration *_linearAccel)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          math::Pose3d imuWorldPose = _worldPose->Data();
          math::Vector3d imuLinearAccel = _linearAccel->Data();
          math::Vector3d imuAngularVel = _angularVel->Data();

          // Get the IMU angular velocity (defined in imu's local frame)
          it->second->gravity = _worldGravity->Data();

          // Get the IMU angular velocity (defined in imu's local frame)
          it->second->angularVel = _angularVel->Data();

          // Get the IMU linear acceleration in the imu local frame
          it->second->linearAcc = _linearAccel->Data();

          // Add contribution from gravity
          // Do we want to skip if link does not have gravity enabled?
          //   e.g. if (this->dataPtr->parentEntity->GetGravityMode())
          it->second->linearAcc -= imuWorldPose.Rot().Inverse().RotateVector(
              it->second->gravity);

          // Set the IMU orientation
          // imu orientation with respect to reference frame
          it->second->imuReferenceOrientation =
              it->second->worldToReference.Inverse() * imuWorldPose.Rot();
        }
        else
        {
          std::cout << "Failed to update imu: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
std::string ImuPrivate::DefaultTopic(const Entity &_entity,
    const EntityComponentManager &_ecm)
{
  // default topic name:
  // /model/model_name/link/link_name/sensor/sensor_name/imu
  std::string sensorName = _ecm.Component<components::Name>(_entity)->Data();
  auto p = _ecm.Component<components::ParentEntity>(_entity);
  std::string linkName = _ecm.Component<components::Name>(p->Data())->Data();
  std::string topic =
      "/link/" + linkName + "/sensor/" + sensorName + "/imu";
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

IGNITION_ADD_PLUGIN(Imu, System,
  Imu::ISystemPreUpdate,
  Imu::ISystemPostUpdate
)
