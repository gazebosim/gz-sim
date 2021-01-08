/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/math/DiffDriveOdometry.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/RollingMean.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Velocity command.
struct Commands
{
  /// \brief Linear velocity.
  double lin;

  /// \brief Angular velocity.
  double ang;

  Commands() : lin(0.0), ang(0.0) {}
};

class ignition::gazebo::systems::OdometryPublisherPrivate
{
  /// \brief Update odometry and publish an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Model interface
  public: Model model{kNullEntity};

  public: std::string odomFrame;

  public: std::string robotBaseFrame;

  /// \brief The model's canonical link.
  //public: Link canonicalLink{kNullEntity};

  /// \brief Update period calculated from <odom__publish_frequency>.
  public: std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
  public: std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Diff drive odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief A mutex to protect the target velocity command.
  //public: std::mutex mutex;

  /// \brief Integrates the velocities (linear and angular) using 2nd order
  /// Runge-Kutta.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _angular Angular velocity.
  //public: void IntegrateRungeKutta2(double _linear, double _angular);

  /// \brief Integrates the velocities (linear and angular) using exact
  /// method.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _angular Angular velocity.
  //public: void IntegrateExact(double _linear, double _angular);

  /// \brief Rolling mean accumulators for the linear velocity
  public: std::pair<math::RollingMean, math::RollingMean> linearMean;

  /// \brief Rolling mean accumulators for the angular velocity
  public: math::RollingMean angularMean;

  public: bool initialized{false};

  public: math::Pose3d lastUpdatePose{0, 0, 0, 0, 0, 0};

  /// \brief Current timestamp.
  public: math::clock::time_point lastUpdateTime;
};

//////////////////////////////////////////////////
OdometryPublisher::OdometryPublisher()
  : dataPtr(std::make_unique<OdometryPublisherPrivate>())
{
  this->dataPtr->linearMean.first.SetWindowSize(10);
  this->dataPtr->linearMean.second.SetWindowSize(10);
  this->dataPtr->angularMean.SetWindowSize(10);
  this->dataPtr->linearMean.first.Clear();
  this->dataPtr->linearMean.second.Clear();
  this->dataPtr->angularMean.Clear();
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
    ignerr << "DiffDrive plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->odomFrame = "odom";
  if (!_sdf->HasElement("odom_frame"))
  {
    ignwarn << "PlanarMovePlugin missing <odom_frame>, "
      << "defaults to \"" << this->dataPtr->odomFrame << "\"" << std::endl;
  }
  else
  {
    this->dataPtr->odomFrame = _sdf->Get<std::string>("odom_frame");
  }

  this->dataPtr->robotBaseFrame = "robotBaseFrame";
  if (!_sdf->HasElement("robot_base_frame"))
  {
    ignwarn << "PlanarMovePlugin missing <robot_base_frame>, "
      << "defaults to \"" << this->dataPtr->robotBaseFrame << "\"" << std::endl;
  }
  else
  {
    this->dataPtr->robotBaseFrame = _sdf->Get<std::string>("robot_base_frame");
  }

  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  if (odomFreq > 0)
  {
    std::chrono::duration<double> odomPer{1 / odomFreq};
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
  }

  // Setup odometry.
  std::string odomTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/odometryFoo"};
  if (_sdf->HasElement("odom_topic"))
    odomTopic = _sdf->Get<std::string>("odom_topic");
  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopic);
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

  auto modelName = this->dataPtr->model.Name(_ecm);

  // Create the pose component it does not exist.
  auto pos = _ecm.Component<components::Pose>(
      this->dataPtr->model.Entity());
  if (!pos)
  {
    _ecm.CreateComponent(this->dataPtr->model.Entity(),
        components::Pose());
  }
  auto linearVelocity = _ecm.Component<components::LinearVelocity>(
      this->dataPtr->model.Entity());
  if (!linearVelocity)
  {
    _ecm.CreateComponent(this->dataPtr->model.Entity(),
        components::LinearVelocity());
  }
  auto angularVelocity = _ecm.Component<components::AngularVelocity>(
      this->dataPtr->model.Entity());
  if (!angularVelocity)
  {
    _ecm.CreateComponent(this->dataPtr->model.Entity(),
        components::AngularVelocity());
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
void OdometryPublisherPrivate::UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("DiffDrive::UpdateOdometry");
  // Initialize, if not already initialized.
  if (!this->initialized)
  {
    this->lastUpdateTime = std::chrono::steady_clock::time_point(_info.simTime);
    this->initialized = true;
    return; //to check why
  }
  // Construct the odometry message and publish it.
  msgs::Odometry msg;

  auto pos = _ecm.Component<components::Pose>(this->model.Entity());
  auto linearVelocity = _ecm.Component<components::LinearVelocity>(this->model.Entity());
  auto angularVelocity = _ecm.Component<components::AngularVelocity>(this->model.Entity());
  if (!pos || !linearVelocity || !angularVelocity)
  {
    return;
  }

  // Compute x, y and heading using velocity
  const std::chrono::duration<double> dt =
    std::chrono::steady_clock::time_point(_info.simTime) - lastUpdateTime;

  // We cannot estimate the speed if the time interval is zero (or near
  // zero).
  if (math::equal(0.0, dt.count()))
    return;

  // World to odom transformation
  msgs::Set(msg.mutable_pose()->mutable_position(), pos->Data().Pos()); //to only set x,y
  msgs::Set(msg.mutable_pose()->mutable_orientation(), pos->Data().Rot()); //to only set yaw

  // Get linear and angular displacements from last updated pose
  double linearDisplacementX = pos->Data().Pos().X() - this->lastUpdatePose.Pos().X();
  double linearDisplacementY = pos->Data().Pos().Y() - this->lastUpdatePose.Pos().Y();

  double curr_yaw = pos->Data().Rot().Yaw();
  const double last_yaw = this->lastUpdatePose.Rot().Yaw();
  while (curr_yaw < last_yaw - M_PI) curr_yaw += 2 * M_PI;
  while (curr_yaw > last_yaw + M_PI) curr_yaw -= 2 * M_PI;
  const float angularDiff = curr_yaw - last_yaw;

  // auto odom_lin_vel_comp = linearVelocity->Data();
  // Get velocities in child frame (i.e. base_link frame) and add to message
  double linearVelocityX = (cosf(curr_yaw) * linearDisplacementX
    + sinf(curr_yaw) * linearDisplacementX) / dt.count();
  double linearVelocityY = (cosf(curr_yaw) * linearDisplacementY
    - sinf(curr_yaw) * linearDisplacementY) / dt.count();
  this->linearMean.first.Push(linearVelocityX);
  this->linearMean.second.Push(linearVelocityY);
  this->angularMean.Push(angularDiff / dt.count());
  msg.mutable_twist()->mutable_linear()->set_x(this->linearMean.first.Mean());
  msg.mutable_twist()->mutable_linear()->set_y(this->linearMean.second.Mean());
  msg.mutable_twist()->mutable_angular()->set_z(this->angularMean.Mean());//(angularVelocity->Data()[2]);

  // Set the time stamp in the header
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Set the frame ids.
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(odomFrame);//this->model.Name(_ecm) + "/odom");
  auto childFrame = msg.mutable_header()->add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(robotBaseFrame);//this->model.Name(_ecm) + "/" + "...");

  this->lastUpdatePose = pos->Data();
  this->lastUpdateTime = std::chrono::steady_clock::time_point(_info.simTime);

  // Throttle publishing
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;
  // Publish the message
  this->odomPub.Publish(msg);
}

//////////////////////////////////////////////////
/*void OdometryPublisherPrivate::IntegrateRungeKutta2(
    double _linear, double _angular)
{
  const double direction = *this->heading + _angular * 0.5;

  // Runge-Kutta 2nd order integration:
  this->x += _linear * std::cos(direction);
  this->y += _linear * std::sin(direction);
  this->heading += _angular;
}

//////////////////////////////////////////////////
void OdometryPublisherPrivate::IntegrateExact(double _linear, double _angular)
{
  if (std::fabs(_angular) < 1e-6)
  {
    this->IntegrateRungeKutta2(_linear, _angular);
  }
  else
  {
    // Exact integration (should solve problems when angular is zero):
    const double headingOld = *this->heading;
    const double ratio = _linear / _angular;
    this->heading += _angular;
    this->x += ratio * (std::sin(*this->heading) - std::sin(headingOld));
    this->y += -ratio * (std::cos(*this->heading) - std::cos(headingOld));
  }
}*/

IGNITION_ADD_PLUGIN(OdometryPublisher,
                    ignition::gazebo::System,
                    OdometryPublisher::ISystemConfigure,
                    OdometryPublisher::ISystemPreUpdate,
                    OdometryPublisher::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OdometryPublisher, "ignition::gazebo::systems::OdometryPublisher")
