/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "DiffDrive.hh"

#include <ignition/msgs/odometry.pb.h>

#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/math/DiffDriveOdometry.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

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

class ignition::gazebo::systems::DiffDrivePrivate
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

  /// \brief Name of left joint
  public: std::vector<std::string> leftJointNames;

  /// \brief Name of right joint
  public: std::vector<std::string> rightJointNames;

  /// \brief Calculated speed of left joint
  public: double leftJointSpeed{0};

  /// \brief Calculated speed of right joint
  public: double rightJointSpeed{0};

  /// \brief Distance between wheels
  public: double wheelSeparation{1.0};

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

  /// \brief Diff drive odometry.
  public: math::DiffDriveOdometry odom;

  /// \brief Diff drive odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief Diff drive tf message publisher.
  public: transport::Node::Publisher tfPub;

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
DiffDrive::DiffDrive()
  : dataPtr(std::make_unique<DiffDrivePrivate>())
{
}

//////////////////////////////////////////////////
void DiffDrive::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = Link(links[0]);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "DiffDrive plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

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

  this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
      this->dataPtr->wheelSeparation).first;
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

  // Setup odometry.
  this->dataPtr->odom.SetWheelParams(this->dataPtr->wheelSeparation,
      this->dataPtr->wheelRadius, this->dataPtr->wheelRadius);

  // Subscribe to commands
  std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");
  this->dataPtr->node.Subscribe(topic, &DiffDrivePrivate::OnCmdVel,
      this->dataPtr.get());

  std::string odomTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/odometry"};
  if (_sdf->HasElement("odom_topic"))
    odomTopic = _sdf->Get<std::string>("odom_topic");
  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopic);

  this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
      "tf");

  if (_sdf->HasElement("frame_id"))
    this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

  if (_sdf->HasElement("child_frame_id"))
    this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");

  ignmsg << "DiffDrive subscribing to twist messages on [" << topic << "]"
         << std::endl;
}

//////////////////////////////////////////////////
void DiffDrive::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("DiffDrive::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  if (this->dataPtr->leftJoints.empty() ||
      this->dataPtr->rightJoints.empty())
  {
    for (const std::string &name : this->dataPtr->leftJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->leftJoints.push_back(joint);
    }

    for (const std::string &name : this->dataPtr->rightJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->rightJoints.push_back(joint);
    }
  }

  if (this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty())
    return;

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
}

//////////////////////////////////////////////////
void DiffDrive::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("DiffDrive::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void DiffDrivePrivate::UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("DiffDrive::UpdateOdometry");
  // Initialize, if not already initialized.
  if (!this->odom.Initialized())
  {
    this->odom.Init(std::chrono::steady_clock::time_point(_info.simTime));
    return;
  }

  // Get the first joint positions for the left and right side.
  auto leftPos = _ecm.Component<components::JointPosition>(this->leftJoints[0]);
  auto rightPos = _ecm.Component<components::JointPosition>(
      this->rightJoints[0]);

  // Abort if the joints were not found or just created.
  if (!leftPos || !rightPos || leftPos->Data().empty() ||
      rightPos->Data().empty())
  {
    return;
  }

  this->odom.Update(leftPos->Data()[0], rightPos->Data()[0],
      std::chrono::steady_clock::time_point(_info.simTime));

  // Throttle publishing
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;

  // Construct the odometry message and publish it.
  msgs::Odometry msg;
  msg.mutable_pose()->mutable_position()->set_x(this->odom.X());
  msg.mutable_pose()->mutable_position()->set_y(this->odom.Y());

  math::Quaterniond orientation(0, 0, *this->odom.Heading());
  msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

  msg.mutable_twist()->mutable_linear()->set_x(this->odom.LinearVelocity());
  msg.mutable_twist()->mutable_angular()->set_z(*this->odom.AngularVelocity());

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

  // Construct the Pose_V/tf message and publish it.
  msgs::Pose_V tf_msg;
  ignition::msgs::Pose *tf_msg_pose = nullptr;
  tf_msg_pose = tf_msg.add_pose();
  tf_msg_pose->mutable_header()->CopyFrom(*msg.mutable_header());
  tf_msg_pose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
  tf_msg_pose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());

  // Publish the messages
  this->odomPub.Publish(msg);
  this->tfPub.Publish(tf_msg);
}

//////////////////////////////////////////////////
void DiffDrivePrivate::UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  IGN_PROFILE("DiffDrive::UpdateVelocity");

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

  // Convert the target velocities to joint velocities.
  this->rightJointSpeed =
    (linVel + angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
  this->leftJointSpeed =
    (linVel - angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
}

//////////////////////////////////////////////////
void DiffDrivePrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}

IGNITION_ADD_PLUGIN(DiffDrive,
                    ignition::gazebo::System,
                    DiffDrive::ISystemConfigure,
                    DiffDrive::ISystemPreUpdate,
                    DiffDrive::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(DiffDrive, "ignition::gazebo::systems::DiffDrive")
