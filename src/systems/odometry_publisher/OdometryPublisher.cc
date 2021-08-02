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

#include "OdometryPublisher.hh"

#include <ignition/msgs/odometry.pb.h>

#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <tuple>

#include <ignition/common/Profiler.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/RollingMean.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::OdometryPublisherPrivate
{
  /// \brief Calculates odometry and publishes an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Name of the world-fixed coordinate frame for the odometry message.
  public: std::string odomFrame;

  /// \brief Name of the coordinate frame rigidly attached to the mobile
  /// robot base.
  public: std::string robotBaseFrame;

  /// \brief Number of dimensions to represent odometry.
  public: int dimensions;

  /// \brief Update period calculated from <odom__publish_frequency>.
  public: std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
  public: std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Diff drive odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief Rolling mean accumulators for the linear velocity
  public: std::tuple<math::RollingMean, math::RollingMean, math::RollingMean>
    linearMean;

  /// \brief Rolling mean accumulators for the angular velocity
  public: std::tuple<math::RollingMean, math::RollingMean, math::RollingMean>
    angularMean;

  /// \brief Initialized flag.
  public: bool initialized{false};

  /// \brief Current pose of the model in the odom frame.
  public: math::Pose3d lastUpdatePose{0, 0, 0, 0, 0, 0};

  /// \brief Current timestamp.
  public: math::clock::time_point lastUpdateTime;
};

//////////////////////////////////////////////////
OdometryPublisher::OdometryPublisher()
  : dataPtr(std::make_unique<OdometryPublisherPrivate>())
{
  std::get<0>(this->dataPtr->linearMean).SetWindowSize(10);
  std::get<1>(this->dataPtr->linearMean).SetWindowSize(10);
  std::get<2>(this->dataPtr->angularMean).SetWindowSize(10);
  std::get<0>(this->dataPtr->linearMean).Clear();
  std::get<1>(this->dataPtr->linearMean).Clear();
  std::get<2>(this->dataPtr->angularMean).Clear();

  if (this->dataPtr->dimensions == 3)
  {
    std::get<2>(this->dataPtr->linearMean).SetWindowSize(10);
    std::get<0>(this->dataPtr->angularMean).SetWindowSize(10);
    std::get<1>(this->dataPtr->angularMean).SetWindowSize(10);
    std::get<2>(this->dataPtr->linearMean).Clear();
    std::get<0>(this->dataPtr->angularMean).Clear();
    std::get<1>(this->dataPtr->angularMean).Clear();
  }
}

//////////////////////////////////////////////////
void OdometryPublisher::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "OdometryPublisher system plugin should be attached to a model"
           << " entity. Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->odomFrame = this->dataPtr->model.Name(_ecm) + "/" + "odom";
  if (!_sdf->HasElement("odom_frame"))
  {
    ignwarn << "OdometryPublisher system plugin missing <odom_frame>, "
      << "defaults to \"" << this->dataPtr->odomFrame << "\"" << std::endl;
  }
  else
  {
    this->dataPtr->odomFrame = _sdf->Get<std::string>("odom_frame");
  }

  this->dataPtr->robotBaseFrame = this->dataPtr->model.Name(_ecm)
    + "/" + "base_footprint";
  if (!_sdf->HasElement("robot_base_frame"))
  {
    ignwarn << "OdometryPublisher system plugin missing <robot_base_frame>, "
      << "defaults to \"" << this->dataPtr->robotBaseFrame << "\"" << std::endl;
  }
  else
  {
    this->dataPtr->robotBaseFrame = _sdf->Get<std::string>("robot_base_frame");
  }

  this->dataPtr->dimensions = 2;
  if (!_sdf->HasElement("dimensions"))
  {
    ignwarn << "OdometryPublisher system plugin missing <dimensions>, "
      << "defaults to \"" << this->dataPtr->dimensions << "\"" << std::endl;
  }
  else
  {
    this->dataPtr->dimensions = _sdf->Get<int>("dimensions");
    if (this->dataPtr->dimensions != 2 && this->dataPtr->dimensions != 3)
    {
      ignerr << "OdometryPublisher system plugin <dimensions> must be 2D or 3D "
             << "not " << this->dataPtr->dimensions
             << "D. Failed to initialize." << std::endl;
      return;
    }
  }

  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  if (odomFreq > 0)
  {
    std::chrono::duration<double> period{1 / odomFreq};
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }

  // Setup odometry
  std::string odomTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/odometry"};
  if (_sdf->HasElement("odom_topic"))
    odomTopic = _sdf->Get<std::string>("odom_topic");
  std::string odomTopicValid {transport::TopicUtils::AsValidTopic(odomTopic)};
  if (odomTopicValid.empty())
  {
    ignerr << "Failed to generate odom topic ["
           << odomTopic << "]" << std::endl;
    return;
  }
  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopicValid);
}

//////////////////////////////////////////////////
void OdometryPublisher::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("OdometryPublisher::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Create the pose component if it does not exist.
  auto pos = _ecm.Component<components::Pose>(
      this->dataPtr->model.Entity());
  if (!pos)
  {
    _ecm.CreateComponent(this->dataPtr->model.Entity(),
        components::Pose());
  }
}

//////////////////////////////////////////////////
void OdometryPublisher::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("OdometryPublisher::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void OdometryPublisherPrivate::UpdateOdometry(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("OdometryPublisher::UpdateOdometry");
  // Record start time.
  if (!this->initialized)
  {
    this->lastUpdateTime = std::chrono::steady_clock::time_point(_info.simTime);
    this->initialized = true;
    return;
  }

  // Construct the odometry message and publish it.
  msgs::Odometry msg;

  const std::chrono::duration<double> dt =
    std::chrono::steady_clock::time_point(_info.simTime) - lastUpdateTime;
  // We cannot estimate the speed if the time interval is zero (or near
  // zero).
  if (math::equal(0.0, dt.count()))
    return;

  // Get and set robotBaseFrame to odom transformation.
  const math::Pose3d pose = worldPose(this->model.Entity(), _ecm);
  msg.mutable_pose()->mutable_position()->set_x(pose.Pos().X());
  msg.mutable_pose()->mutable_position()->set_y(pose.Pos().Y());
  msgs::Set(msg.mutable_pose()->mutable_orientation(), pose.Rot());
  if (this->dimensions == 3)
  {
    msg.mutable_pose()->mutable_position()->set_z(pose.Pos().Z());
  }

  // Get linear and angular displacements from last updated pose.
  double linearDisplacementX = pose.Pos().X() - this->lastUpdatePose.Pos().X();
  double linearDisplacementY = pose.Pos().Y() - this->lastUpdatePose.Pos().Y();

  double currentYaw = pose.Rot().Yaw();
  const double lastYaw = this->lastUpdatePose.Rot().Yaw();
  while (currentYaw < lastYaw - IGN_PI) currentYaw += 2 * IGN_PI;
  while (currentYaw > lastYaw + IGN_PI) currentYaw -= 2 * IGN_PI;
  const float yawDiff = currentYaw - lastYaw;

  // Get velocities assuming 2D
  if (this->dimensions == 2)
  {
    double linearVelocityX = (cosf(currentYaw) * linearDisplacementX
      + sinf(currentYaw) * linearDisplacementY) / dt.count();
    double linearVelocityY = (cosf(currentYaw) * linearDisplacementY
      - sinf(currentYaw) * linearDisplacementX) / dt.count();
    std::get<0>(this->linearMean).Push(linearVelocityX);
    std::get<1>(this->linearMean).Push(linearVelocityY);
    msg.mutable_twist()->mutable_linear()->set_x(
      std::get<0>(this->linearMean).Mean());
    msg.mutable_twist()->mutable_linear()->set_y(
      std::get<1>(this->linearMean).Mean());
  }
  // Get velocities and roll/pitch rates assuming 3D
  else if (this->dimensions == 3)
  {
    double currentRoll = pose.Rot().Roll();
    const double lastRoll = this->lastUpdatePose.Rot().Roll();
    while (currentRoll < lastRoll - IGN_PI) currentRoll += 2 * IGN_PI;
    while (currentRoll > lastRoll + IGN_PI) currentRoll -= 2 * IGN_PI;
    const float rollDiff = currentRoll - lastRoll;

    double currentPitch = pose.Rot().Pitch();
    const double lastPitch = this->lastUpdatePose.Rot().Pitch();
    while (currentPitch < lastPitch - IGN_PI) currentPitch += 2 * IGN_PI;
    while (currentPitch > lastPitch + IGN_PI) currentPitch -= 2 * IGN_PI;
    const float pitchDiff = currentPitch - lastPitch;

    double linearDisplacementZ =
      pose.Pos().Z() - this->lastUpdatePose.Pos().Z();
    math::Vector3 linearDisplacement(linearDisplacementX, linearDisplacementY,
      linearDisplacementZ);
    math::Vector3 linearVelocity =
      pose.Rot().RotateVectorReverse(linearDisplacement) / dt.count();
    std::get<0>(this->linearMean).Push(linearVelocity.X());
    std::get<1>(this->linearMean).Push(linearVelocity.Y());
    std::get<2>(this->linearMean).Push(linearVelocity.Z());
    std::get<0>(this->angularMean).Push(rollDiff / dt.count());
    std::get<1>(this->angularMean).Push(pitchDiff / dt.count());
    msg.mutable_twist()->mutable_linear()->set_x(
      std::get<0>(this->linearMean).Mean());
    msg.mutable_twist()->mutable_linear()->set_y(
      std::get<1>(this->linearMean).Mean());
    msg.mutable_twist()->mutable_linear()->set_z(
      std::get<2>(this->linearMean).Mean());
    msg.mutable_twist()->mutable_angular()->set_x(
      std::get<0>(this->angularMean).Mean());
    msg.mutable_twist()->mutable_angular()->set_y(
      std::get<1>(this->angularMean).Mean());
  }

  // Set yaw rate
  std::get<2>(this->angularMean).Push(yawDiff / dt.count());
  msg.mutable_twist()->mutable_angular()->set_z(
    std::get<2>(this->angularMean).Mean());

  // Set the time stamp in the header.
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Set the frame ids.
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(odomFrame);
  auto childFrame = msg.mutable_header()->add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(robotBaseFrame);

  this->lastUpdatePose = pose;
  this->lastUpdateTime = std::chrono::steady_clock::time_point(_info.simTime);

  // Throttle publishing.
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;
  this->odomPub.Publish(msg);
}

IGNITION_ADD_PLUGIN(OdometryPublisher,
                    ignition::gazebo::System,
                    OdometryPublisher::ISystemConfigure,
                    OdometryPublisher::ISystemPreUpdate,
                    OdometryPublisher::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OdometryPublisher,
                          "ignition::gazebo::systems::OdometryPublisher")
