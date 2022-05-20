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

#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>

#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/RollingMean.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::OdometryPublisherPrivate
{
  /// \brief Calculates odometry and publishes an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateOdometry(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm);

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

  /// \brief Diff drive odometry with covariance message publisher.
  public: transport::Node::Publisher odomCovPub;

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

  /// \brief Allow specifying constant xyz and rpy offsets
  public: gz::math::Pose3d offset = {0, 0, 0, 0, 0, 0};

  /// \brief Gaussian noise
  public: double gaussianNoise = 0.0;
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
    gzerr << "OdometryPublisher system plugin should be attached to a model"
           << " entity. Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->odomFrame = this->dataPtr->model.Name(_ecm) + "/" + "odom";
  if (!_sdf->HasElement("odom_frame"))
  {
    gzwarn << "OdometryPublisher system plugin missing <odom_frame>, "
      << "defaults to \"" << this->dataPtr->odomFrame << "\"" << std::endl;
  }
  else
  {
    this->dataPtr->odomFrame = _sdf->Get<std::string>("odom_frame");
  }

  if (_sdf->HasElement("xyz_offset"))
  {
    this->dataPtr->offset.Pos() = _sdf->Get<gz::math::Vector3d>(
      "xyz_offset");
  }

  if (_sdf->HasElement("rpy_offset"))
  {
    this->dataPtr->offset.Rot() =
      gz::math::Quaterniond(_sdf->Get<gz::math::Vector3d>(
        "rpy_offset"));
  }

  if (_sdf->HasElement("gaussian_noise"))
  {
    this->dataPtr->gaussianNoise = _sdf->Get<double>("gaussian_noise");
  }

  this->dataPtr->robotBaseFrame = this->dataPtr->model.Name(_ecm)
    + "/" + "base_footprint";
  if (!_sdf->HasElement("robot_base_frame"))
  {
    gzwarn << "OdometryPublisher system plugin missing <robot_base_frame>, "
      << "defaults to \"" << this->dataPtr->robotBaseFrame << "\"" << std::endl;
  }
  else
  {
    this->dataPtr->robotBaseFrame = _sdf->Get<std::string>("robot_base_frame");
  }

  this->dataPtr->dimensions = 2;
  if (!_sdf->HasElement("dimensions"))
  {
    igndbg << "OdometryPublisher system plugin missing <dimensions>, "
      << "defaults to \"" << this->dataPtr->dimensions << "\"" << std::endl;
  }
  else
  {
    this->dataPtr->dimensions = _sdf->Get<int>("dimensions");
    if (this->dataPtr->dimensions != 2 && this->dataPtr->dimensions != 3)
    {
      gzerr << "OdometryPublisher system plugin <dimensions> must be 2D or 3D "
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
  std::string odomCovTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/odometry_with_covariance"};

  if (_sdf->HasElement("odom_topic"))
    odomTopic = _sdf->Get<std::string>("odom_topic");
  if (_sdf->HasElement("odom_covariance_topic"))
    odomCovTopic = _sdf->Get<std::string>("odom_covariance_topic");

  std::string odomTopicValid {transport::TopicUtils::AsValidTopic(odomTopic)};
  if (odomTopicValid.empty())
  {
    gzerr << "Failed to generate odom topic ["
           << odomTopic << "]" << std::endl;
  }
  else
  {
    this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
        odomTopicValid);
  }

  std::string odomCovTopicValid {
    transport::TopicUtils::AsValidTopic(odomCovTopic)};
  if (odomCovTopicValid.empty())
  {
    gzerr << "Failed to generate odom topic ["
           << odomCovTopic << "]" << std::endl;
  }
  else
  {
    this->dataPtr->odomCovPub = this->dataPtr->node.Advertise<
        msgs::OdometryWithCovariance>(odomCovTopicValid);
  }
}

//////////////////////////////////////////////////
void OdometryPublisher::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  IGN_PROFILE("OdometryPublisher::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
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
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
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
  const math::Pose3d rawPose = worldPose(this->model.Entity(), _ecm);
  math::Pose3d pose = rawPose * this->offset;
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
      std::get<0>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_linear()->set_y(
      std::get<1>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_linear()->set_z(
      gz::math::Rand::DblNormal(0, this->gaussianNoise));

    msg.mutable_twist()->mutable_angular()->set_x(
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_angular()->set_y(
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
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
      std::get<0>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_linear()->set_y(
      std::get<1>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_linear()->set_z(
      std::get<2>(this->linearMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_angular()->set_x(
      std::get<0>(this->angularMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
    msg.mutable_twist()->mutable_angular()->set_y(
      std::get<1>(this->angularMean).Mean() +
      gz::math::Rand::DblNormal(0, this->gaussianNoise));
  }

  // Set yaw rate
  std::get<2>(this->angularMean).Push(yawDiff / dt.count());
  msg.mutable_twist()->mutable_angular()->set_z(
    std::get<2>(this->angularMean).Mean() +
    gz::math::Rand::DblNormal(0, this->gaussianNoise));

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
  if (this->odomPub.Valid())
  {
    this->odomPub.Publish(msg);
  }

  // Generate odometry with covariance message and publish it.
  msgs::OdometryWithCovariance msgCovariance;

  // Set the time stamp in the header.
  msgCovariance.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Set the frame ids.
  frame = msgCovariance.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(odomFrame);
  childFrame = msg.mutable_header()->add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(robotBaseFrame);

  // Copy position from odometry msg.
  msgCovariance.mutable_pose_with_covariance()->
    mutable_pose()->mutable_position()->set_x(msg.pose().position().x());
  msgCovariance.mutable_pose_with_covariance()->
    mutable_pose()->mutable_position()->set_y(msg.pose().position().y());
  msgCovariance.mutable_pose_with_covariance()->
    mutable_pose()->mutable_position()->set_z(msg.pose().position().z());

  // Copy twist from odometry msg.
  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_angular()->set_x(msg.twist().angular().x());
  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_angular()->set_y(msg.twist().angular().y());
  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_angular()->set_z(msg.twist().angular().z());

  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_linear()->set_x(msg.twist().linear().x());
  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_linear()->set_y(msg.twist().linear().y());
  msgCovariance.mutable_twist_with_covariance()->
    mutable_twist()->mutable_linear()->set_z(msg.twist().linear().z());

  // Populate the covariance matrix.
  // Should the matrix me populated for pose as well ?
  auto gn2 = this->gaussianNoise * this->gaussianNoise;
  for (int i = 0; i < 36; i++)
  {
    if (i % 7 == 0)
    {
      msgCovariance.mutable_pose_with_covariance()->
        mutable_covariance()->add_data(gn2);
      msgCovariance.mutable_twist_with_covariance()->
        mutable_covariance()->add_data(gn2);
    }
    else
    {
      msgCovariance.mutable_pose_with_covariance()->
        mutable_covariance()->add_data(0);
      msgCovariance.mutable_twist_with_covariance()->
        mutable_covariance()->add_data(0);
    }
  }
  if (this->odomCovPub.Valid())
  {
    this->odomCovPub.Publish(msgCovariance);
  }
}

IGNITION_ADD_PLUGIN(OdometryPublisher,
                    gz::sim::System,
                    OdometryPublisher::ISystemConfigure,
                    OdometryPublisher::ISystemPreUpdate,
                    OdometryPublisher::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OdometryPublisher,
                          "gz::sim::systems::OdometryPublisher")
