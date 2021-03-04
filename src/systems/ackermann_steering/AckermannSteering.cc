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

#include "AckermannSteering.hh"

#include <ignition/msgs/odometry.pb.h>

#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Angle.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "SpeedLimiter.hh"

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

class ignition::gazebo::systems::AckermannSteeringPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const ignition::msgs::Twist &_msg);

  /// \brief Update odometry and publish an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Entity of the left joint
  public: std::vector<Entity> leftJoints;

  /// \brief Entity of the right joint
  public: std::vector<Entity> rightJoints;

  /// \brief Entity of the left steering joint
  public: std::vector<Entity> leftSteeringJoints;

  /// \brief Entity of the right steering joint
  public: std::vector<Entity> rightSteeringJoints;

  /// \brief Name of left joint
  public: std::vector<std::string> leftJointNames;

  /// \brief Name of right joint
  public: std::vector<std::string> rightJointNames;

  /// \brief Name of left steering joint
  public: std::vector<std::string> leftSteeringJointNames;

  /// \brief Name of right steering joint
  public: std::vector<std::string> rightSteeringJointNames;

  /// \brief Calculated speed of left wheel joint(s)
  public: double leftJointSpeed{0};

  /// \brief Calculated speed of right wheel joint(s)
  public: double rightJointSpeed{0};

  /// \brief Calculated speed of left joint
  public: double leftSteeringJointSpeed{0};

  /// \brief Calculated speed of right joint
  public: double rightSteeringJointSpeed{0};

  /// \brief Distance between left and right wheels
  public: double wheelSeparation{1.0};

  /// \brief Distance between left and right wheel kingpins
  public: double kingpinWidth{0.8};

  /// \brief Distance between front and back wheels
  public: double wheelBase{1.0};

  /// \brief Maximum turning angle to limit steering to
  public: double steeringLimit{0.5};

  /// \brief Wheel radius
  public: double wheelRadius{0.2};

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The model's canonical link.
  public: Link canonicalLink{kNullEntity};

  /// \brief Update period calculated from <odom__publish_frequency>.
  public: std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
  public: std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Ackermann steering odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief Odometry X value
  public: double odomX{0.0};

  /// \brief Odometry Y value
  public: double odomY{0.0};

  /// \brief Odometry yaw value
  public: double odomYaw{0.0};

  /// \brief Odometry old left value
  public: double odomOldLeft{0.0};

  /// \brief Odometry old right value
  public: double odomOldRight{0.0};

  /// \brief Odometry last time value
  public: std::chrono::steady_clock::duration lastOdomTime{0};

  /// \brief Linear velocity limiter.
  public: std::unique_ptr<SpeedLimiter> limiterLin;

  /// \brief Angular velocity limiter.
  public: std::unique_ptr<SpeedLimiter> limiterAng;

  /// \brief Previous control command.
  public: Commands last0Cmd;

  /// \brief Previous control command to last0Cmd.
  public: Commands last1Cmd;

  /// \brief Last target velocity requested.
  public: msgs::Twist targetVel;

  /// \brief A mutex to protect the target velocity command.
  public: std::mutex mutex;

  /// \brief frame_id from sdf.
  public: std::string sdfFrameId;

  /// \brief child_frame_id from sdf.
  public: std::string sdfChildFrameId;
};

//////////////////////////////////////////////////
AckermannSteering::AckermannSteering()
  : dataPtr(std::make_unique<AckermannSteeringPrivate>())
{
}

//////////////////////////////////////////////////
void AckermannSteering::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "AckermannSteering plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = Link(links[0]);

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto ptr = const_cast<sdf::Element *>(_sdf.get());

  // Get params from SDF
  sdf::ElementPtr sdfElem = ptr->GetElement("left_joint");
  while (sdfElem)
  {
    this->dataPtr->leftJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("left_joint");
  }
  sdfElem = ptr->GetElement("right_joint");
  while (sdfElem)
  {
    this->dataPtr->rightJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("right_joint");
  }
  sdfElem = ptr->GetElement("left_steering_joint");
  while (sdfElem)
  {
    this->dataPtr->leftSteeringJointNames.push_back(
                          sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("left_steering_joint");
  }
  sdfElem = ptr->GetElement("right_steering_joint");
  while (sdfElem)
  {
    this->dataPtr->rightSteeringJointNames.push_back(
                           sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("right_steering_joint");
  }

  this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
      this->dataPtr->wheelSeparation).first;
  this->dataPtr->kingpinWidth = _sdf->Get<double>("kingpin_width",
      this->dataPtr->kingpinWidth).first;
  this->dataPtr->wheelBase = _sdf->Get<double>("wheel_base",
      this->dataPtr->wheelBase).first;
  this->dataPtr->steeringLimit = _sdf->Get<double>("steering_limit",
      this->dataPtr->steeringLimit).first;
  this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
      this->dataPtr->wheelRadius).first;

  // Parse speed limiter parameters.
  bool hasVelocityLimits     = false;
  bool hasAccelerationLimits = false;
  bool hasJerkLimits         = false;
  double minVel              = std::numeric_limits<double>::lowest();
  double maxVel              = std::numeric_limits<double>::max();
  double minAccel            = std::numeric_limits<double>::lowest();
  double maxAccel            = std::numeric_limits<double>::max();
  double minJerk             = std::numeric_limits<double>::lowest();
  double maxJerk             = std::numeric_limits<double>::max();

  if (_sdf->HasElement("min_velocity"))
  {
    minVel = _sdf->Get<double>("min_velocity");
    hasVelocityLimits = true;
  }
  if (_sdf->HasElement("max_velocity"))
  {
    maxVel = _sdf->Get<double>("max_velocity");
    hasVelocityLimits = true;
  }
  if (_sdf->HasElement("min_acceleration"))
  {
    minAccel = _sdf->Get<double>("min_acceleration");
    hasAccelerationLimits = true;
  }
  if (_sdf->HasElement("max_acceleration"))
  {
    maxAccel = _sdf->Get<double>("max_acceleration");
    hasAccelerationLimits = true;
  }
  if (_sdf->HasElement("min_jerk"))
  {
    minJerk = _sdf->Get<double>("min_jerk");
    hasJerkLimits = true;
  }
  if (_sdf->HasElement("max_jerk"))
  {
    maxJerk = _sdf->Get<double>("max_jerk");
    hasJerkLimits = true;
  }

  // Instantiate the speed limiters.
  this->dataPtr->limiterLin = std::make_unique<SpeedLimiter>(
    hasVelocityLimits, hasAccelerationLimits, hasJerkLimits,
    minVel, maxVel, minAccel, maxAccel, minJerk, maxJerk);

  this->dataPtr->limiterAng = std::make_unique<SpeedLimiter>(
    hasVelocityLimits, hasAccelerationLimits, hasJerkLimits,
    minVel, maxVel, minAccel, maxAccel, minJerk, maxJerk);

  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  if (odomFreq > 0)
  {
    std::chrono::duration<double> odomPer{1 / odomFreq};
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
  }

  // Subscribe to commands
  std::vector<std::string> topics;
  if (_sdf->HasElement("topic"))
  {
    topics.push_back(_sdf->Get<std::string>("topic"));
  }
  topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel");
  auto topic = validTopic(topics);
  if (topic.empty())
  {
    ignerr << "AckermannSteering plugin received invalid model name "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->node.Subscribe(topic, &AckermannSteeringPrivate::OnCmdVel,
      this->dataPtr.get());

  std::vector<std::string> odomTopics;
  if (_sdf->HasElement("odom_topic"))
  {
    odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
  }
  odomTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
      "/odometry");
  auto odomTopic = validTopic(odomTopics);
  if (topic.empty())
  {
    ignerr << "AckermannSteering plugin received invalid model name "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopic);

  if (_sdf->HasElement("frame_id"))
    this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

  if (_sdf->HasElement("child_frame_id"))
    this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");

  ignmsg << "AckermannSteering subscribing to twist messages on [" <<
      topic << "]" << std::endl;
}

//////////////////////////////////////////////////
void AckermannSteering::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("AckermannSteering::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  static std::set<std::string> warnedModels;
  auto modelName = this->dataPtr->model.Name(_ecm);
  if (this->dataPtr->leftJoints.empty() ||
      this->dataPtr->rightJoints.empty() ||
      this->dataPtr->leftSteeringJoints.empty() ||
      this->dataPtr->rightSteeringJoints.empty())
  {
    bool warned{false};
    for (const std::string &name : this->dataPtr->leftJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->leftJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        ignwarn << "Failed to find left joint [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    for (const std::string &name : this->dataPtr->rightJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->rightJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        ignwarn << "Failed to find right joint [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }
    for (const std::string &name : this->dataPtr->leftSteeringJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->leftSteeringJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        ignwarn << "Failed to find left steering joint ["
                << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    for (const std::string &name : this->dataPtr->rightSteeringJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->rightSteeringJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        ignwarn << "Failed to find right steering joint [" <<
            name << "] for model [" << modelName << "]" << std::endl;
        warned = true;
      }
    }
    if (warned)
    {
      warnedModels.insert(modelName);
    }
  }

  if (this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty() ||
      this->dataPtr->leftSteeringJoints.empty() ||
      this->dataPtr->rightSteeringJoints.empty())
    return;

  if (warnedModels.find(modelName) != warnedModels.end())
  {
    ignmsg << "Found joints for model [" << modelName
           << "], plugin will start working." << std::endl;
    warnedModels.erase(modelName);
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  for (Entity joint : this->dataPtr->leftJoints)
  {
    // Update wheel velocity
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);

    if (vel == nullptr)
    {
      _ecm.CreateComponent(
          joint, components::JointVelocityCmd({this->dataPtr->leftJointSpeed}));
    }
    else
    {
      *vel = components::JointVelocityCmd({this->dataPtr->leftJointSpeed});
    }
  }

  for (Entity joint : this->dataPtr->rightJoints)
  {
    // Update wheel velocity
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);

    if (vel == nullptr)
    {
      _ecm.CreateComponent(joint,
          components::JointVelocityCmd({this->dataPtr->rightJointSpeed}));
    }
    else
    {
      *vel = components::JointVelocityCmd({this->dataPtr->rightJointSpeed});
    }
  }

  // Update steering
  for (Entity joint : this->dataPtr->leftSteeringJoints)
  {
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);

    if (vel == nullptr)
    {
      _ecm.CreateComponent(
          joint, components::JointVelocityCmd(
                             {this->dataPtr->leftSteeringJointSpeed}));
    }
    else
    {
      *vel = components::JointVelocityCmd(
                         {this->dataPtr->leftSteeringJointSpeed});
    }
  }

  for (Entity joint : this->dataPtr->rightSteeringJoints)
  {
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);

    if (vel == nullptr)
    {
      _ecm.CreateComponent(joint,
          components::JointVelocityCmd(
                  {this->dataPtr->rightSteeringJointSpeed}));
    }
    else
    {
      *vel = components::JointVelocityCmd(
                     {this->dataPtr->rightSteeringJointSpeed});
    }
  }

  // Create the left and right side joint position components if they
  // don't exist.
  auto leftPos = _ecm.Component<components::JointPosition>(
      this->dataPtr->leftJoints[0]);
  if (!leftPos)
  {
    _ecm.CreateComponent(this->dataPtr->leftJoints[0],
        components::JointPosition());
  }

  auto rightPos = _ecm.Component<components::JointPosition>(
      this->dataPtr->rightJoints[0]);
  if (!rightPos)
  {
    _ecm.CreateComponent(this->dataPtr->rightJoints[0],
        components::JointPosition());
  }

  auto leftSteeringPos = _ecm.Component<components::JointPosition>(
      this->dataPtr->leftSteeringJoints[0]);
  if (!leftSteeringPos)
  {
    _ecm.CreateComponent(this->dataPtr->leftSteeringJoints[0],
        components::JointPosition());
  }

  auto rightSteeringPos = _ecm.Component<components::JointPosition>(
      this->dataPtr->rightSteeringJoints[0]);
  if (!rightSteeringPos)
  {
    _ecm.CreateComponent(this->dataPtr->rightSteeringJoints[0],
        components::JointPosition());
  }
}

//////////////////////////////////////////////////
void AckermannSteering::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("AckermannSteering::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void AckermannSteeringPrivate::UpdateOdometry(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("AckermannSteering::UpdateOdometry");
  // Initialize, if not already initialized.

  if (this->leftJoints.empty() || this->rightJoints.empty() ||
      this->leftSteeringJoints.empty() || this->rightSteeringJoints.empty())
    return;

  // Get the first joint positions for the left and right side.
  auto leftPos = _ecm.Component<components::JointPosition>(this->leftJoints[0]);
  auto rightPos = _ecm.Component<components::JointPosition>(
      this->rightJoints[0]);
  auto leftSteeringPos = _ecm.Component<components::JointPosition>(
      this->leftSteeringJoints[0]);
  auto rightSteeringPos = _ecm.Component<components::JointPosition>(
      this->rightSteeringJoints[0]);

  // Abort if the joints were not found or just created.
  if (!leftPos || !rightPos || leftPos->Data().empty() ||
      rightPos->Data().empty() ||
      !leftSteeringPos || !rightSteeringPos ||
      leftSteeringPos->Data().empty() ||
      rightSteeringPos->Data().empty())
  {
    return;
  }

  // Calculate the odometry
  double phi = 0.5 * (leftSteeringPos->Data()[0] + rightSteeringPos->Data()[0]);
  double radius = this->wheelBase / tan(phi);
  double dist = 0.5 * this->wheelRadius *
      ((leftPos->Data()[0] - this->odomOldLeft) +
       (rightPos->Data()[0] - this->odomOldRight));
  double deltaAngle = dist / radius;
  this->odomYaw += deltaAngle;
  this->odomYaw = math::Angle(this->odomYaw).Normalized().Radian();
  this->odomX += dist * cos(this->odomYaw);
  this->odomY += dist * sin(this->odomYaw);
  auto odomTimeDiff = _info.simTime - this->lastOdomTime;
  double tdiff = std::chrono::duration<double>(odomTimeDiff).count();
  double odomLinearVelocity = dist / tdiff;
  double odomAngularVelocity = deltaAngle / tdiff;
  this->lastOdomTime = _info.simTime;
  this->odomOldLeft = leftPos->Data()[0];
  this->odomOldRight = rightPos->Data()[0];

  // Throttle odometry publishing
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;

  // Construct the odometry message and publish it.
  msgs::Odometry msg;
  msg.mutable_pose()->mutable_position()->set_x(this->odomX);
  msg.mutable_pose()->mutable_position()->set_y(this->odomY);

  math::Quaterniond orientation(0, 0, this->odomYaw);
  msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

  msg.mutable_twist()->mutable_linear()->set_x(odomLinearVelocity);
  msg.mutable_twist()->mutable_angular()->set_z(odomAngularVelocity);

  // Set the time stamp in the header
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Set the frame id.
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  if (this->sdfFrameId.empty())
  {
    frame->add_value(this->model.Name(_ecm) + "/odom");
  }
  else
  {
    frame->add_value(this->sdfFrameId);
  }

  std::optional<std::string> linkName = this->canonicalLink.Name(_ecm);
  if (this->sdfChildFrameId.empty())
  {
    if (linkName)
    {
      auto childFrame = msg.mutable_header()->add_data();
      childFrame->set_key("child_frame_id");
      childFrame->add_value(this->model.Name(_ecm) + "/" + *linkName);
    }
  }
  else
  {
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->sdfChildFrameId);
  }

  // Publish the message
  this->odomPub.Publish(msg);
}

//////////////////////////////////////////////////
void AckermannSteeringPrivate::UpdateVelocity(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("AckermannSteering::UpdateVelocity");

  double linVel;
  double angVel;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    linVel = this->targetVel.linear().x();
    angVel = this->targetVel.angular().z();
  }

  const double dt = std::chrono::duration<double>(_info.dt).count();

  // Limit the target velocity if needed.
  this->limiterLin->Limit(linVel, this->last0Cmd.lin, this->last1Cmd.lin, dt);
  this->limiterAng->Limit(angVel, this->last0Cmd.ang, this->last1Cmd.ang, dt);

  // Update history of commands.
  this->last1Cmd = last0Cmd;
  this->last0Cmd.lin = linVel;
  this->last0Cmd.ang = angVel;

  // Convert the target velocities to joint velocities and angles
  double turningRadius = linVel / angVel;
  double minimumTurningRadius = this->wheelBase / sin(this->steeringLimit);
  if ((turningRadius >= 0.0) && (turningRadius < minimumTurningRadius))
  {
    turningRadius = minimumTurningRadius;
  }
  if ((turningRadius <= 0.0) && (turningRadius > -minimumTurningRadius))
  {
    turningRadius = -minimumTurningRadius;
  }
  // special case for angVel of zero
  if (fabs(angVel) < 0.001)
  {
    turningRadius = 1000000000.0;
  }

  double leftSteeringJointAngle =
      atan(this->wheelBase / (turningRadius - (this->kingpinWidth / 2.0)));
  double rightSteeringJointAngle =
      atan(this->wheelBase / (turningRadius + (this->kingpinWidth / 2.0)));
  double phi = atan(this->wheelBase / turningRadius);

  // Partially simulate a simple differential
  this->rightJointSpeed =
      (linVel * (1.0 + (this->wheelSeparation * tan(phi)) /
                 (2.0 * this->wheelBase))) / this->wheelRadius;
  this->leftJointSpeed =
      (linVel * (1.0 - (this->wheelSeparation * tan(phi)) /
                 (2.0 * this->wheelBase))) / this->wheelRadius;

  auto leftSteeringPos = _ecm.Component<components::JointPosition>(
      this->leftSteeringJoints[0]);
  auto rightSteeringPos = _ecm.Component<components::JointPosition>(
      this->rightSteeringJoints[0]);

  // Abort if the joints were not found or just created.
  if (!leftSteeringPos || !rightSteeringPos ||
      leftSteeringPos->Data().empty() ||
      rightSteeringPos->Data().empty())
  {
    return;
  }

  double leftDelta = leftSteeringJointAngle - leftSteeringPos->Data()[0];
  double rightDelta = rightSteeringJointAngle - rightSteeringPos->Data()[0];

  // Simple proportional control with a gain of 1
  // Adding programmable PID values might be a future feature.
  // Works as is for tested cases
  this->leftSteeringJointSpeed = leftDelta;
  this->rightSteeringJointSpeed = rightDelta;
}

//////////////////////////////////////////////////
void AckermannSteeringPrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}

IGNITION_ADD_PLUGIN(AckermannSteering,
                    ignition::gazebo::System,
                    AckermannSteering::ISystemConfigure,
                    AckermannSteering::ISystemPreUpdate,
                    AckermannSteering::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(AckermannSteering,
                          "ignition::gazebo::systems::AckermannSteering")
