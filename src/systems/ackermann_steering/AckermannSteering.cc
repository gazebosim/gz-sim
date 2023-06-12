/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 * Copyright (C) 2023 Benjamin Perseghetti, Rudis Laboratories
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

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/odometry.pb.h>

#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Angle.hh>
#include <gz/math/SpeedLimiter.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
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

class gz::sim::systems::AckermannSteeringPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const msgs::Twist &_msg);

  /// \brief Callback for angle subscription
  /// \param[in] _msg angle message
  public: void OnCmdAng(const msgs::Double &_msg);

  /// \brief Callback for actuator angle subscription
  /// \param[in] _msg angle message
  public: void OnActuatorAng(const msgs::Actuators &_msg);

  /// \brief Update odometry and publish an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateOdometry(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Update the angle.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateAngle(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Use angle steer only mode.
  public: bool steeringOnly{false};

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

  /// \brief Index of angle actuator.
  public: int actuatorNumber = 0;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The model's canonical link.
  public: Link canonicalLink{kNullEntity};

  /// \brief True if using Actuator msg to control steering angle.
  public: bool useActuatorMsg{false};

  /// \brief Update period calculated from <odom__publish_frequency>.
  public: std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
  public: std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Ackermann steering odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief Ackermann tf message publisher.
  public: transport::Node::Publisher tfPub;

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
  public: std::unique_ptr<math::SpeedLimiter> limiterLin;

  /// \brief Angular velocity limiter.
  public: std::unique_ptr<math::SpeedLimiter> limiterAng;

  /// \brief Previous control command.
  public: Commands last0Cmd;

  /// \brief Previous control command to last0Cmd.
  public: Commands last1Cmd;

  /// \brief Last target velocity requested.
  public: msgs::Twist targetVel;

  /// \brief Last target angle requested.
  public: double targetAng{0.0};

  /// \brief P gain for angular position.
  public: double gainPAng{1.0};

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
    gzerr << "AckermannSteering plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = Link(links[0]);

  if (_sdf->HasElement("steering_only"))
  {
    this->dataPtr->steeringOnly = _sdf->Get<bool>("steering_only");
    gzmsg << "Using steering only mode: " << this->dataPtr->steeringOnly
            << std::endl;
  }

  if (_sdf->HasElement("use_actuator_msg") &&
    _sdf->Get<bool>("use_actuator_msg"))
  {
    if (_sdf->HasElement("actuator_number"))
    {
      this->dataPtr->actuatorNumber =
        _sdf->Get<int>("actuator_number");
      this->dataPtr->useActuatorMsg = true;
      if (!this->dataPtr->steeringOnly)
      {
        this->dataPtr->steeringOnly = true;
      }
    }
    else
    {
      gzerr << "Please specify an actuator_number" <<
        "to use Actuator position message control." << std::endl;
    }
  }

  // Get params from SDF
  auto sdfLeftSteerElem = _sdf->FindElement("left_steering_joint");
  while (sdfLeftSteerElem)
  {
    this->dataPtr->leftSteeringJointNames.push_back(
                          sdfLeftSteerElem->Get<std::string>());
    sdfLeftSteerElem = sdfLeftSteerElem->GetNextElement(
      "left_steering_joint");
  }
  auto sdfRightSteerElem = _sdf->FindElement("right_steering_joint");
  while (sdfRightSteerElem)
  {
    this->dataPtr->rightSteeringJointNames.push_back(
                           sdfRightSteerElem->Get<std::string>());
    sdfRightSteerElem = sdfRightSteerElem->GetNextElement(
      "right_steering_joint");
  }
  if (!this->dataPtr->steeringOnly)
  {
    auto sdfLeftElem = _sdf->FindElement("left_joint");
    while (sdfLeftElem)
    {
      this->dataPtr->leftJointNames.push_back(
        sdfLeftElem->Get<std::string>());
      sdfLeftElem = sdfLeftElem->GetNextElement("left_joint");
    }
    auto sdfRightElem = _sdf->FindElement("right_joint");
    while (sdfRightElem)
    {
      this->dataPtr->rightJointNames.push_back(
        sdfRightElem->Get<std::string>());
      sdfRightElem = sdfRightElem->GetNextElement("right_joint");
    }
  }
  if (!this->dataPtr->steeringOnly)
  {
    this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
      this->dataPtr->wheelRadius).first;
    this->dataPtr->kingpinWidth = _sdf->Get<double>("kingpin_width",
      this->dataPtr->kingpinWidth).first;
  }
  this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
      this->dataPtr->wheelSeparation).first;

  this->dataPtr->wheelBase = _sdf->Get<double>("wheel_base",
      this->dataPtr->wheelBase).first;
  this->dataPtr->steeringLimit = _sdf->Get<double>("steering_limit",
      this->dataPtr->steeringLimit).first;

  // Get proportional gain for steering angle.
  if (_sdf->HasElement("steer_p_gain"))
  {
    this->dataPtr->gainPAng = _sdf->Get<double>("steer_p_gain");
  }

  // Instantiate the linear speed limiters.
  this->dataPtr->limiterLin = std::make_unique<math::SpeedLimiter>();

  // Parse linear speed limiter parameters.
  if (_sdf->HasElement("min_velocity"))
  {
    const double minVel = _sdf->Get<double>("min_velocity");
    this->dataPtr->limiterLin->SetMinVelocity(minVel);
  }
  if (_sdf->HasElement("max_velocity"))
  {
    const double maxVel = _sdf->Get<double>("max_velocity");
    this->dataPtr->limiterLin->SetMaxVelocity(maxVel);
  }
  if (_sdf->HasElement("min_acceleration"))
  {
    const double minAccel = _sdf->Get<double>("min_acceleration");
    this->dataPtr->limiterLin->SetMinAcceleration(minAccel);
  }
  if (_sdf->HasElement("max_acceleration"))
  {
    const double maxAccel = _sdf->Get<double>("max_acceleration");
    this->dataPtr->limiterLin->SetMaxAcceleration(maxAccel);
  }
  if (_sdf->HasElement("min_jerk"))
  {
    const double minJerk = _sdf->Get<double>("min_jerk");
    this->dataPtr->limiterLin->SetMinJerk(minJerk);
  }
  if (_sdf->HasElement("max_jerk"))
  {
    const double maxJerk = _sdf->Get<double>("max_jerk");
    this->dataPtr->limiterLin->SetMaxJerk(maxJerk);
  }

  //Instantiate the angular speed limiters.
  this->dataPtr->limiterAng = std::make_unique<math::SpeedLimiter>();

  // Parse angular speed limiter parameters.
  if (_sdf->HasElement("min_angular_velocity"))
  {
    const double minAngVel = _sdf->Get<double>("min_angular_velocity");
    this->dataPtr->limiterAng->SetMinVelocity(minAngVel);
  }
  if (_sdf->HasElement("max_angular_velocity"))
  {
    const double maxAngVel = _sdf->Get<double>("max_angular_velocity");
    this->dataPtr->limiterAng->SetMaxVelocity(maxAngVel);
  }
  if (_sdf->HasElement("min_angular_acceleration"))
  {
    const double minAngAccel = _sdf->Get<double>("min_angular_acceleration");
    this->dataPtr->limiterAng->SetMinAcceleration(minAngAccel);
  }
  if (_sdf->HasElement("max_angular_acceleration"))
  {
    const double maxAngAccel = _sdf->Get<double>("max_angular_acceleration");
    this->dataPtr->limiterAng->SetMaxAcceleration(maxAngAccel);
  }
  if (_sdf->HasElement("min_angular_jerk"))
  {
    const double minAngJerk = _sdf->Get<double>("min_angular_jerk");
    this->dataPtr->limiterAng->SetMinJerk(minAngJerk);
  }
  if (_sdf->HasElement("max_angular_jerk"))
  {
    const double maxAngJerk = _sdf->Get<double>("max_angular_jerk");
    this->dataPtr->limiterAng->SetMaxJerk(maxAngJerk);
  }




  if (!this->dataPtr->steeringOnly)
  {
    double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
    if (odomFreq > 0)
    {
      std::chrono::duration<double> odomPer{1 / odomFreq};
      this->dataPtr->odomPubPeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          odomPer);
    }
  }
  // Subscribe to commands
  std::vector<std::string> topics;


  if (_sdf->HasElement("topic"))
  {
    topics.push_back(_sdf->Get<std::string>("topic"));
  }
  else if (_sdf->HasElement("sub_topic"))
  {
    topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
      "/" + _sdf->Get<std::string>("sub_topic"));
  }
  else if ((this->dataPtr->steeringOnly) &&
    (!this->dataPtr->useActuatorMsg))
  {
    topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
      "/steer_angle");
  }
  else if ((this->dataPtr->steeringOnly) &&
    (this->dataPtr->useActuatorMsg))
  {
    topics.push_back("/actuators");
  }
  else if (!this->dataPtr->steeringOnly)
  {
    topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel");
  }

  auto topic = validTopic(topics);
  if (topic.empty())
  {
    gzerr << "AckermannSteering plugin received invalid topic name "
           << "Failed to initialize." << std::endl;
    return;
  }
  if (this->dataPtr->steeringOnly)
  {
    if (this->dataPtr->useActuatorMsg)
    {
      this->dataPtr->node.Subscribe(topic,
        &AckermannSteeringPrivate::OnActuatorAng,
        this->dataPtr.get());
      gzmsg << "AckermannSteering subscribing to Actuator messages on ["
            << topic << "]" << std::endl;
    }
    else
    {
      this->dataPtr->node.Subscribe(topic,
        &AckermannSteeringPrivate::OnCmdAng,
        this->dataPtr.get());
      gzmsg << "AckermannSteering subscribing to float messages on ["
            << topic << "]" << std::endl;
    }
  }
  else
  {
    this->dataPtr->node.Subscribe(topic, &AckermannSteeringPrivate::OnCmdVel,
      this->dataPtr.get());
    gzmsg << "AckermannSteering subscribing to twist messages on ["
          << topic << "]" << std::endl;
  }
  if (!this->dataPtr->steeringOnly)
  {
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
      gzerr << "AckermannSteering plugin received invalid model name "
            << "Failed to initialize." << std::endl;
      return;
    }

    this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
        odomTopic);

    std::vector<std::string> tfTopics;
    if (_sdf->HasElement("tf_topic"))
    {
      tfTopics.push_back(_sdf->Get<std::string>("tf_topic"));
    }
    tfTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
      "/tf");
    auto tfTopic = validTopic(tfTopics);
    if (tfTopic.empty())
    {
      gzerr << "AckermannSteering plugin invalid tf topic name "
            << "Failed to initialize." << std::endl;
      return;
    }

    this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
        tfTopic);

    if (_sdf->HasElement("frame_id"))
      this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

    if (_sdf->HasElement("child_frame_id"))
      this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");
  }
}

//////////////////////////////////////////////////
void AckermannSteering::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("AckermannSteering::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  static std::set<std::string> warnedModels;
  auto modelName = this->dataPtr->model.Name(_ecm);
  if (!this->dataPtr->steeringOnly &&
      (this->dataPtr->leftJoints.empty() ||
      this->dataPtr->rightJoints.empty()))
  {
    bool warned{false};
    for (const std::string &name : this->dataPtr->leftJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->leftJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find left joint [" << name << "] for model ["
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
        gzwarn << "Failed to find right joint [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }
    if (warned)
    {
      warnedModels.insert(modelName);
    }
  }
  if (this->dataPtr->leftSteeringJoints.empty() ||
      this->dataPtr->rightSteeringJoints.empty())
  {
    bool warned{false};
    for (const std::string &name : this->dataPtr->leftSteeringJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->leftSteeringJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find left steering joint ["
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
        gzwarn << "Failed to find right steering joint [" <<
            name << "] for model [" << modelName << "]" << std::endl;
        warned = true;
      }
    }
    if (warned)
    {
      warnedModels.insert(modelName);
    }
  }
  if (!this->dataPtr->steeringOnly &&
    (this->dataPtr->leftJoints.empty() ||
    this->dataPtr->rightJoints.empty()))
    return;
  else if (this->dataPtr->leftSteeringJoints.empty() ||
      this->dataPtr->rightSteeringJoints.empty())
    return;

  if (warnedModels.find(modelName) != warnedModels.end())
  {
    gzmsg << "Found joints for model [" << modelName
           << "], plugin will start working." << std::endl;
    warnedModels.erase(modelName);
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;
  if (!this->dataPtr->steeringOnly)
  {
    for (Entity joint : this->dataPtr->leftJoints)
    {
      // Update wheel velocity
      auto vel = _ecm.Component<components::JointVelocityCmd>(joint);

      if (vel == nullptr)
      {
        _ecm.CreateComponent(
            joint, components::JointVelocityCmd(
              {this->dataPtr->leftJointSpeed}));
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
  if (!this->dataPtr->steeringOnly)
  {
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
  GZ_PROFILE("AckermannSteering::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;
  if (this->dataPtr->steeringOnly)
  {
    this->dataPtr->UpdateAngle(_info, _ecm);
  }
  else
  {
    this->dataPtr->UpdateVelocity(_info, _ecm);
    this->dataPtr->UpdateOdometry(_info, _ecm);
  }
}

//////////////////////////////////////////////////
void AckermannSteeringPrivate::UpdateOdometry(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("AckermannSteering::UpdateOdometry");
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

  // Construct the Pose_V/tf message and publish it.
  msgs::Pose_V tfMsg;
  msgs::Pose *tfMsgPose = tfMsg.add_pose();
  tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
  tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
  tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());

  // Publish the message
  this->odomPub.Publish(msg);
  this->tfPub.Publish(tfMsg);
}

//////////////////////////////////////////////////
void AckermannSteeringPrivate::UpdateVelocity(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("AckermannSteering::UpdateVelocity");

  double linVel;
  double angVel;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    linVel = this->targetVel.linear().x();
    angVel = this->targetVel.angular().z();
  }

  // Limit the target velocity if needed.
  this->limiterLin->Limit(
      linVel, this->last0Cmd.lin, this->last1Cmd.lin, _info.dt);
  this->limiterAng->Limit(
      angVel, this->last0Cmd.ang, this->last1Cmd.ang, _info.dt);

  // Update history of commands.
  this->last1Cmd = last0Cmd;
  this->last0Cmd.lin = linVel;
  this->last0Cmd.ang = angVel;

  // Convert the target velocities to joint velocities and angles
  double turningRadius = linVel / angVel;
  double minimumTurningRadius = this->wheelBase / sin(this->steeringLimit);
  if (fabs(linVel) > 0.0)
  {
    if ((turningRadius >= 0.0) && (turningRadius < minimumTurningRadius))
    {
      turningRadius = minimumTurningRadius;
    }
    if ((turningRadius <= 0.0) && (turningRadius > -minimumTurningRadius))
    {
      turningRadius = -minimumTurningRadius;
    }
  }
  // special case for angVel not zero and linVel zero
  else if (fabs(angVel) >= 0.001)
  {
    turningRadius = (angVel / fabs(angVel)) * minimumTurningRadius;
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

  // simple proportional controller
  this->leftSteeringJointSpeed = leftDelta * this->gainPAng;
  this->rightSteeringJointSpeed = rightDelta * this->gainPAng;
}

//////////////////////////////////////////////////
void AckermannSteeringPrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}

//////////////////////////////////////////////////
void AckermannSteeringPrivate::UpdateAngle(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("AckermannSteering::UpdateAngle");

  double ang;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    ang = this->targetAng;
  }

  // Limit the target angle if needed.
  this->limiterAng->Limit(
      ang, this->last0Cmd.ang, this->last1Cmd.ang, _info.dt);

  if (fabs(ang) > this->steeringLimit)
  {
    ang = (ang / fabs(ang)) * this->steeringLimit;
  }

  // Update history of commands.
  this->last1Cmd = last0Cmd;
  this->last0Cmd.ang = ang;

  double leftSteeringJointAngle =
      atan((2.0 * this->wheelBase * sin(ang)) / \
      ((2.0 * this->wheelBase * cos(ang)) + \
      (1.0 * this->wheelSeparation * sin(ang))));
  double rightSteeringJointAngle =
      atan((2.0 * this->wheelBase * sin(ang)) / \
      ((2.0 * this->wheelBase * cos(ang)) - \
      (1.0 * this->wheelSeparation * sin(ang))));

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

  // Simple proportional control with settable gain.
  // Adding programmable PID values might be a future feature.
  this->leftSteeringJointSpeed = this->gainPAng * leftDelta;
  this->rightSteeringJointSpeed = this->gainPAng * rightDelta;
}

//////////////////////////////////////////////////
void AckermannSteeringPrivate::OnCmdAng(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetAng = _msg.data();
}

void AckermannSteeringPrivate::OnActuatorAng(const msgs::Actuators &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->actuatorNumber > _msg.position_size() - 1)
  {
    gzerr << "You tried to access index " << this->actuatorNumber
      << " of the Actuator position array which is of size "
      << _msg.position_size() << std::endl;
    return;
  }

  this->targetAng = static_cast<double>(_msg.position(this->actuatorNumber));
}

GZ_ADD_PLUGIN(AckermannSteering,
                    System,
                    AckermannSteering::ISystemConfigure,
                    AckermannSteering::ISystemPreUpdate,
                    AckermannSteering::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(AckermannSteering,
                    "gz::sim::systems::AckermannSteering")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(AckermannSteering,
                          "ignition::gazebo::systems::AckermannSteering")
