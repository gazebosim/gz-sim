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

#include <sdf/Element.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Imu.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
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

  /// \brief Load the IMU from an sdf element
  /// \param[in] _sdf SDF element describing the IMU
  public: void Load(const sdf::ElementPtr &_sdf);

  /// \brief Publish IMU data over ign transport
  public: void Publish();

  /// \brief Sensors name
  public: std::string sensorName;

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

  /// \brief store gravity vector to be added to the IMU output.
  public: ignition::math::Vector3d gravity;

  /// \brief Ign transport node
  public: transport::Node node;

  /// \brief publisher for IMU data
  public: transport::Node::Publisher pub;

  /// \brief common::Time from when the sensor was updated
  public: common::Time lastMeasurementTime;
};

/// \brief Private Imu data class.
class ignition::gazebo::systems::ImuPrivate
{
  /// \brief A map of IMU entity to its IMU sensor.
  public: std::unordered_map<Entity, std::unique_ptr<ImuSensor>>
      entitySensorMap;

  /// \brief Create IMU sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateImuEntities(EntityComponentManager &_ecm);

  /// \brief Update IMU sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void Update(const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
ImuSensor::ImuSensor() = default;

//////////////////////////////////////////////////
ImuSensor::~ImuSensor() = default;

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
  msg.set_entity_name(this->sensorName);
  msg.mutable_header()->mutable_stamp()->set_sec(
      this->lastMeasurementTime.sec);
  msg.mutable_header()->mutable_stamp()->set_nsec(
      this->lastMeasurementTime.nsec);
  msgs::Set(msg.mutable_orientation(), this->imuReferenceOrientation);
  msgs::Set(msg.mutable_angular_velocity(), this->angularVel);
  msgs::Set(msg.mutable_linear_acceleration(), this->linearAcc);
  this->pub.Publish(msg);
}

//////////////////////////////////////////////////
Imu::Imu() : System(), dataPtr(std::make_unique<ImuPrivate>())
{
}

//////////////////////////////////////////////////
Imu::~Imu() = default;

//////////////////////////////////////////////////
void Imu::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  this->dataPtr->CreateImuEntities(_ecm);
}

//////////////////////////////////////////////////
void Imu::PostUpdate(const UpdateInfo &_info,
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

    // Publish sensor data
    it.second->Publish();
  }
}

//////////////////////////////////////////////////
void ImuPrivate::CreateImuEntities(EntityComponentManager &_ecm)
{
  // Get World Entity
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == worldEntity)
  {
    ignerr << "Missing world entity." << std::endl;
    return;
  }

  // Get the world acceleration (defined in world frame)
  auto gravity = _ecm.Component<components::Gravity>(worldEntity);
  if (nullptr == gravity)
  {
    ignerr << "World missing gravity." << std::endl;
    return;
  }

  // Create IMUs
  _ecm.EachNew<components::Imu>(
    [&](const Entity &_entity,
        const components::Imu *_imu)->bool
      {
        auto sensor = std::make_unique<ImuSensor>();
        sensor->Load(_imu->Data());

        // Set sensors world gravity
        sensor->gravity = gravity->Data();

        // Set sensor name
        sensor->sensorName = _ecm.Component<components::Name>(
            _entity)->Data();

        // create default topic for sensor if not specified
        if (sensor->topic.empty())
        {
          sensor->topic = scopedName(_entity, _ecm, "/") + "/imu";
        }

        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void ImuPrivate::Update(const EntityComponentManager &_ecm)
{
  _ecm.Each<components::Imu,
            components::WorldPose,
            components::AngularVelocity,
            components::LinearAcceleration>(
    [&](const Entity &_entity,
        const components::Imu * /*_imu*/,
        const components::WorldPose *_worldPose,
        const components::AngularVelocity *_angularVel,
        const components::LinearAcceleration *_linearAccel)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          const auto &imuWorldPose = _worldPose->Data();

          // Get the IMU angular velocity (defined in imu's local frame)
          it->second->angularVel = _angularVel->Data();

          // Get the IMU linear acceleration in the imu local frame
          it->second->linearAcc = _linearAccel->Data();

          // Add contribution from gravity
          // Do we want to skip if link does not have gravity enabled?
          it->second->linearAcc -= imuWorldPose.Rot().Inverse().RotateVector(
              it->second->gravity);

          // Set the IMU orientation
          // imu orientation with respect to reference frame
          it->second->imuReferenceOrientation =
              it->second->worldToReference.Inverse() * imuWorldPose.Rot();
        }
        else
        {
          ignerr << "Failed to update IMU: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

IGNITION_ADD_PLUGIN(Imu, System,
  Imu::ISystemPreUpdate,
  Imu::ISystemPostUpdate
)
