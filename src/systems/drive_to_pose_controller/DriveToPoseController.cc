/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include "DriveToPoseController.hh"

#include <gz/common/Profiler.hh>

#include <gz/math/Pose3.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/convert/Pose.hh>

#include <gz/plugin/Register.hh>

#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
class gz::sim::systems::DriveToPoseControllerPrivate
{
  /// \brief Callback for pose command
  /// \param[in] _msg Pose message
  public: void OnCmdPose(const msgs::Pose_V &_msg);

  /// \brief Callback for odometry message
  /// \param[in] _msg Current odometry message
  public: void OnCurrentPose(const msgs::Pose_V &_msg);

  /// \brief Calculate velocity and publish a twist message
  public: void CalculateVelocity();

  /// \brief Normalize the angles between -pi to pi
  /// \param[in] _angle Angle to normalize
  public: void NormalizeAngle(double &_angle);

  /// \brief Gazebo communication node
  public: transport::Node node;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Velocity message publisher.
  public: transport::Node::Publisher velocityPublisher;

  /// \brief Pose reached publisher
  public: transport::Node::Publisher poseReachedPublisher;

  /// \brief Current pose of the model
  public: std::optional<math::Pose3d> currentPose;

  /// \brief Target pose of the model
  public: std::optional<math::Pose3d> targetPose;

  /// \brief Proportional gain for angular velocity
  public: double angularPGain{2.0};

  /// \brief Proportional gain for linear velocity
  public: double linearPGain{1.0};

  /// \brief Allowable angular deviation
  public: double angularDeviation{0.05};

  /// \brief Allowable linear deviation
  public: double linearDeviation{0.1};

  /// \brief Mutex to protect current and target poses
  public: std::mutex mutex;
};

//////////////////////////////////////////////////
DriveToPoseController::DriveToPoseController()
    : dataPtr(std::make_unique<DriveToPoseControllerPrivate>())
{
  // Do nothing
}

//////////////////////////////////////////////////
void DriveToPoseController::Configure(
    const Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    EntityComponentManager& _ecm,
    EventManager& /*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "This plugin should be attached to a model entity. "
          << "Initialization failed." << std::endl;
    return;
  }

  // Get linear P gain if available
  if (_sdf->HasElement("linear_p_gain"))
  {
    this->dataPtr->linearPGain = _sdf->Get<double>("linear_p_gain");
  }

  // Get angular P gain if available
  if (_sdf->HasElement("angular_p_gain"))
  {
    this->dataPtr->angularPGain = _sdf->Get<double>("angular_p_gain");
  }

  // Get allowable linear deviation if available
  if (_sdf->HasElement("linear_deviation"))
  {
    this->dataPtr->linearDeviation = _sdf->Get<double>("linear_deviation");
  }

  // Get allowable angular deviation if available
  if (_sdf->HasElement("angular_deviation"))
  {
    this->dataPtr->angularDeviation = _sdf->Get<double>("angular_deviation");
  }

  // Topic namespace for publish and subscribe
  std::string topicNamespace = "/model/" + this->dataPtr->model.Name(_ecm);

  // Subscribe to odometry pose publisher
  this->dataPtr->node.Subscribe(
    topicNamespace + "/pose", &DriveToPoseControllerPrivate::OnCurrentPose,
    this->dataPtr.get());

  std::vector<transport::MessagePublisher> publishers;
  std::vector<transport::MessagePublisher> subscribers;
  this->dataPtr->node.TopicInfo(
    topicNamespace + "/pose", publishers, subscribers);
  if (publishers.size() < 1)
  {
    gzwarn << "Unable to find publisher on /pose topic!" << std::endl;
    gzwarn << "Make sure OdometryPublisher plugin "
           << "is loaded through the SDF." << std::endl;
  }

  // Subscribe to command pose topic
  this->dataPtr->node.Subscribe(
    topicNamespace + "/cmd_pose", &DriveToPoseControllerPrivate::OnCmdPose,
    this->dataPtr.get());


  // Create velocity publisher
  this->dataPtr->velocityPublisher =
    this->dataPtr->node.Advertise<msgs::Twist>(topicNamespace + "/cmd_vel");
  if (!this->dataPtr->velocityPublisher.HasConnections())
  {
    gzwarn << "Unable to find a subscriber on /cmd_vel topic!" << std::endl;
    gzwarn << "Make sure DiffDrive plugin "
           << "is loaded through the SDF." << std::endl;
  }

  // Create pose reached publisher
  this->dataPtr->poseReachedPublisher =
    this->dataPtr->node.Advertise<msgs::Pose>(topicNamespace + "/reached_pose");

  gzdbg << "DriveToPoseController initialized with "
        << "the following parameters:" << std::endl;
  gzdbg << "linear_p_gain: "
        << this->dataPtr->linearPGain << std::endl;
  gzdbg << "angular_p_gain: "
        << this->dataPtr->angularPGain << std::endl;
  gzdbg << "linear_deviation: "
        << this->dataPtr->linearDeviation << std::endl;
  gzdbg << "angular_deviation: "
        << this->dataPtr->angularDeviation << std::endl;
}

//////////////////////////////////////////////////
void DriveToPoseController::PostUpdate(
    const UpdateInfo& _info,
    const EntityComponentManager& /*_ecm*/)
{
  GZ_PROFILE("DriveToPoseController::PostUpdate");

  if (_info.paused) return;

  // Calculate velocity and publish
  this->dataPtr->CalculateVelocity();
}

//////////////////////////////////////////////////
void DriveToPoseControllerPrivate::CalculateVelocity()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (!this->targetPose || !this->currentPose) return;

  math::Pose3d target = *this->targetPose;
  math::Pose3d current = *this->currentPose;

  double linearError =
    sqrt(pow(target.X() - current.X(), 2) + pow(target.Y() - current.Y(), 2));

  double angularError = target.Yaw() - current.Yaw();
  NormalizeAngle(angularError);

  if (linearError <= this->linearDeviation)
  {
    linearError = 0.0;
    if (abs(angularError) <= this->angularDeviation)
    {
      angularError = 0.0;
      this->targetPose.reset();
    }
  }
  else
  {
    double headingError =
      atan2(target.Y() - current.Y(), target.X() - current.X()) - current.Yaw();
    NormalizeAngle(headingError);
    if (abs(headingError) <= this->angularDeviation)
    {
      angularError = 0.0;
    }
    else
    {
      angularError = headingError;
      linearError = 0.0;
    }
  }

  msgs::Twist cmdVelMsg;
  cmdVelMsg.mutable_linear()->set_x(this->linearPGain * linearError);
  cmdVelMsg.mutable_angular()->set_z(this->angularPGain * angularError);
  this->velocityPublisher.Publish(cmdVelMsg);

  if (!this->targetPose)
    this->poseReachedPublisher.Publish(msgs::Convert(*this->currentPose));
}

//////////////////////////////////////////////////
void DriveToPoseControllerPrivate::NormalizeAngle(double &_angle)
{
  if (_angle > GZ_PI)
  {
    _angle -= 2 * GZ_PI;
  }
  else if (_angle < -GZ_PI)
  {
    _angle += 2 * GZ_PI;
  }
}

//////////////////////////////////////////////////
void DriveToPoseControllerPrivate::OnCmdPose(const msgs::Pose_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetPose = msgs::Convert(_msg.pose(0));
}

//////////////////////////////////////////////////
void DriveToPoseControllerPrivate::OnCurrentPose(const msgs::Pose_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->currentPose = msgs::Convert(_msg.pose(0));
}

GZ_ADD_PLUGIN(DriveToPoseController,
              System,
              ISystemConfigure,
              ISystemPostUpdate);

GZ_ADD_PLUGIN_ALIAS(DriveToPoseController,
                    "gz::sim::systems::DriveToPoseController");
