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

#include <gz/msgs/header.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/pose_v.pb.h>

#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Rand.hh>
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

  /// \brief Calculates angular velocity in body frame from world frame poses.
  /// \param[in] _lastPose Pose at last timestep in world frame.
  /// \param[in] _currentPose Pose at current timestep in world frame.
  /// \param[in] _dt Time elapsed from last to current timestep.
  /// \return Angular velocity computed in body frame at current timestep.
  public: static math::Vector3d CalculateAngularVelocity(
    const math::Pose3d &_lastPose, const math::Pose3d &_currentPose,
    std::chrono::duration<double> _dt);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Model interface
  //! [modelDeclaration]
  public: Model model{kNullEntity};
  //! [modelDeclaration]

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

  /// \brief Odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief Odometry with covariance message publisher.
  public: transport::Node::Publisher odomCovPub;

  /// \brief Pose vector (TF) message publisher.
  public: transport::Node::Publisher tfPub;

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
  public: std::chrono::steady_clock::time_point lastUpdateTime;

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

  if (this->dataPtr->dimensions == 3)
  {
    std::get<2>(this->dataPtr->linearMean).SetWindowSize(10);
    std::get<0>(this->dataPtr->angularMean).SetWindowSize(10);
    std::get<1>(this->dataPtr->angularMean).SetWindowSize(10);
  }
}

//////////////////////////////////////////////////
//! [Configure]
void OdometryPublisher::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  //! [Configure]

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "OdometryPublisher system plugin should be attached to a model"
           << " entity. Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->odomFrame = this->dataPtr->model.Name(_ecm) + "/" + "odom";
  if (!_sdf->HasElement("odom_frame"))
  {
    gzdbg << "OdometryPublisher system plugin missing <odom_frame>, "
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
    gzdbg << "OdometryPublisher system plugin missing <robot_base_frame>, "
      << "defaults to \"" << this->dataPtr->robotBaseFrame << "\"" << std::endl;
  }
  else
  {
    this->dataPtr->robotBaseFrame = _sdf->Get<std::string>("robot_base_frame");
  }

  this->dataPtr->dimensions = 2;
  if (!_sdf->HasElement("dimensions"))
  {
    gzdbg << "OdometryPublisher system plugin missing <dimensions>, "
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
    //! [definePub]
    this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
        odomTopicValid);
    //! [definePub]
    gzmsg << "OdometryPublisher publishing odometry on [" << odomTopicValid
           << "]" << std::endl;
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
    gzmsg << "OdometryPublisher publishing odometry with covariance on ["
           << odomCovTopicValid << "]" << std::endl;
  }

  std::string tfTopic{"/model/" + this->dataPtr->model.Name(_ecm) + "/pose"};
  if (_sdf->HasElement("tf_topic"))
    tfTopic = _sdf->Get<std::string>("tf_topic");
  std::string tfTopicValid {transport::TopicUtils::AsValidTopic(tfTopic)};
  if (tfTopicValid.empty())
  {
    gzerr << "Failed to generate valid TF topic from [" << tfTopic << "]"
           << std::endl;
  }
  else
  {
    this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
        tfTopicValid);
    gzmsg << "OdometryPublisher publishing Pose_V (TF) on ["
           << tfTopicValid << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void OdometryPublisher::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("OdometryPublisher::PreUpdate");

  // \TODO(anyone) This is a temporary fix for
  // gazebosim/gz-sim#2165 until gazebosim/gz-sim#2217 is resolved.
  if (kNullEntity == this->dataPtr->model.Entity())
  {
    return;
  }

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzwarn << "OdometryPublisher model no longer valid. "
           << "Disabling plugin." << std::endl;
    this->dataPtr->model = Model(kNullEntity);
    return;
  }

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }
}

//////////////////////////////////////////////////
void OdometryPublisher::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("OdometryPublisher::PostUpdate");

  // \TODO(anyone) This is a temporary fix for
  // gazebosim/gz-sim#2165 until gazebosim/gz-sim#2217 is resolved.
  if (kNullEntity == this->dataPtr->model.Entity())
  {
    return;
  }


  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
math::Vector3d OdometryPublisherPrivate::CalculateAngularVelocity(
    const math::Pose3d &_lastPose, const math::Pose3d &_currentPose,
    std::chrono::duration<double> _dt)
{
  // Compute the first order finite difference between current and previous
  // rotation as quaternion.
  const math::Quaterniond rotationDiff =
    _currentPose.Rot() * _lastPose.Rot().Inverse();

  math::Vector3d rotationAxis;
  double rotationAngle;
  rotationDiff.AxisAngle(rotationAxis, rotationAngle);

  const math::Vector3d angularVelocity =
    (rotationAngle / _dt.count()) * rotationAxis;

  return _currentPose.Rot().RotateVectorReverse(angularVelocity);
}

//////////////////////////////////////////////////
void OdometryPublisherPrivate::UpdateOdometry(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("OdometryPublisher::UpdateOdometry");
  // Record start time.
  if (!this->initialized)
  {
    this->lastUpdateTime = std::chrono::steady_clock::time_point(_info.simTime);
    this->initialized = true;
    return;
  }

  // Construct the odometry message and publish it.
  //! [declarePoseMsg]
  msgs::Odometry msg;
  //! [declarePoseMsg]

  const std::chrono::duration<double> dt =
    std::chrono::steady_clock::time_point(_info.simTime) - lastUpdateTime;
  // We cannot estimate the speed if the time interval is zero (or near
  // zero).
  if (math::equal(0.0, dt.count()))
    return;

  // Get and set robotBaseFrame to odom transformation.
  //! [worldPose]
  const math::Pose3d rawPose = worldPose(this->model.Entity(), _ecm);
  //! [worldPose]
  //! [setPoseMsg]
  math::Pose3d pose = rawPose * this->offset;
  msg.mutable_pose()->mutable_position()->set_x(pose.Pos().X());
  msg.mutable_pose()->mutable_position()->set_y(pose.Pos().Y());
  msgs::Set(msg.mutable_pose()->mutable_orientation(), pose.Rot());
  if (this->dimensions == 3)
  {
    msg.mutable_pose()->mutable_position()->set_z(pose.Pos().Z());
  }
  //! [setPoseMsg]

  // Get linear and angular displacements from last updated pose.
  double linearDisplacementX = pose.Pos().X() - this->lastUpdatePose.Pos().X();
  double linearDisplacementY = pose.Pos().Y() - this->lastUpdatePose.Pos().Y();

  double currentYaw = pose.Rot().Yaw();
  const math::Vector3d angularVelocityBody = CalculateAngularVelocity(
    this->lastUpdatePose, pose, dt);

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
    double linearDisplacementZ =
      pose.Pos().Z() - this->lastUpdatePose.Pos().Z();
    math::Vector3 linearDisplacement(linearDisplacementX, linearDisplacementY,
      linearDisplacementZ);
    math::Vector3 linearVelocity =
      pose.Rot().RotateVectorReverse(linearDisplacement) / dt.count();
    std::get<0>(this->linearMean).Push(linearVelocity.X());
    std::get<1>(this->linearMean).Push(linearVelocity.Y());
    std::get<2>(this->linearMean).Push(linearVelocity.Z());
    std::get<0>(this->angularMean).Push(angularVelocityBody.X());
    std::get<1>(this->angularMean).Push(angularVelocityBody.Y());
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
  std::get<2>(this->angularMean).Push(angularVelocityBody.Z());
  msg.mutable_twist()->mutable_angular()->set_z(
    std::get<2>(this->angularMean).Mean() +
    gz::math::Rand::DblNormal(0, this->gaussianNoise));

  // Set the time stamp in the header.
  msgs::Header header;
  header.mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));

  // Set the frame ids.
  auto frame = header.add_data();
  frame->set_key("frame_id");
  frame->add_value(odomFrame);
  auto childFrame = header.add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(robotBaseFrame);

  msg.mutable_header()->CopyFrom(header);

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
    //! [publishMsg]
    this->odomPub.Publish(msg);
    //! [publishMsg]
  }

  // Generate odometry with covariance message and publish it.
  msgs::OdometryWithCovariance msgCovariance;

  // Set the time stamp in the header.
  msgCovariance.mutable_header()->CopyFrom(header);

  // Copy position from odometry msg.
  msgCovariance.mutable_pose_with_covariance()->
    mutable_pose()->mutable_position()->set_x(msg.pose().position().x());
  msgCovariance.mutable_pose_with_covariance()->
    mutable_pose()->mutable_position()->set_y(msg.pose().position().y());
  msgCovariance.mutable_pose_with_covariance()->
    mutable_pose()->mutable_position()->set_z(msg.pose().position().z());

  // Copy orientation from odometry msg.
  msgs::Set(msgCovariance.mutable_pose_with_covariance()->mutable_pose()->
    mutable_orientation(), pose.Rot());

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

  if (this->tfPub.Valid())
  {
    msgs::Pose_V tfMsg;
    auto tfMsgPose = tfMsg.add_pose();
    tfMsgPose->CopyFrom(msg.pose());
    tfMsgPose->mutable_header()->CopyFrom(header);

    this->tfPub.Publish(tfMsg);
  }
}

GZ_ADD_PLUGIN(OdometryPublisher,
                    gz::sim::System,
                    OdometryPublisher::ISystemConfigure,
                    OdometryPublisher::ISystemPreUpdate,
                    OdometryPublisher::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(OdometryPublisher,
                          "gz::sim::systems::OdometryPublisher")
