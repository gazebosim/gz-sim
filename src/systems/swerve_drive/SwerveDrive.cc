/*
 * Copyright (C) Jiayi Cai
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
#include "SwerveDrive.hh"

#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>

#include <chrono>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/SwerveDriveOdometry.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Angle.hh>
#include <gz/math/SpeedLimiter.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/JointVelocity.hh"
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

  /// \brief Lateral velocity.
  double lat;

  /// \brief Angular velocity.
  double ang;

  Commands() : lin(0.0), lat(0.0), ang(0.0) {}
};

class gz::sim::systems::SwerveDrivePrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const gz::msgs::Twist &_msg);

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

  /// \brief Optimize the wheel command to minimize the steering angle change
  /// \param[in,out] _steeringDeltaAngle The steering angle change to optimize
  /// \param[in,out] _wheelSpeed The wheel speed to optimize
  private: void OptimizeWheelCmd(double &_steeringDeltaAngle,
    double &_wheelSpeed);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Entity of the front left wheel joint
  public: Entity frontLeftWheelJoint;

  /// \brief Entity of the front right wheel joint
  public: Entity frontRightWheelJoint;

  /// \brief Entity of the back left wheel joint
  public: Entity backLeftWheelJoint;

  /// \brief Entity of the back right wheel joint
  public: Entity backRightWheelJoint;

  /// \brief Entity of the front left steering joint
  public: Entity frontLeftSteeringJoint;

  /// \brief Entity of the front right steering joint
  public: Entity frontRightSteeringJoint;

  /// \brief Entity of the back left steering joint
  public: Entity backLeftSteeringJoint;

  /// \brief Entity of the back right steering joint
  public: Entity backRightSteeringJoint;

  /// \brief Name of front left wheel joint
  public: std::string frontLeftWheelJointName;

  /// \brief Name of front right wheel joint
  public: std::string frontRightWheelJointName;

  /// \brief Name of back left wheel joint
  public: std::string backLeftWheelJointName;

  /// \brief Name of back right wheel joint
  public: std::string backRightWheelJointName;

  /// \brief Name of front left steering joint
  public: std::string frontLeftSteeringJointName;

  /// \brief Name of front right steering joint
  public: std::string frontRightSteeringJointName;

  /// \brief Name of back left steering joint
  public: std::string backLeftSteeringJointName;

  /// \brief Name of back right steering joint
  public: std::string backRightSteeringJointName;

  /// \brief Calculated speed of front left wheel joint
  public: double frontLeftWheelJointSpeed{0};

  /// \brief Calculated speed of front right wheel joint
  public: double frontRightWheelJointSpeed{0};

  /// \brief Calculated speed of back left wheel joint
  public: double backLeftWheelJointSpeed{0};

  /// \brief Calculated speed of back right wheel joint
  public: double backRightWheelJointSpeed{0};

  /// \brief Calculated speed of front left steering joint
  public: double frontLeftSteeringJointSpeed{0};

  /// \brief Calculated speed of front right steering joint
  public: double frontRightSteeringJointSpeed{0};

  /// \brief Calculated speed of back left steering joint
  public: double backLeftSteeringJointSpeed{0};

  /// \brief Calculated speed of back right steering joint
  public: double backRightSteeringJointSpeed{0};

  /// \brief Lateral distance between left and right wheels
  public: double wheelSeparation{0.5};

  /// \brief Longitudinal distance between front and back wheels
  public: double wheelbase{0.5};

  /// \brief Wheel radius
  public: double wheelRadius{0.15};

  /// \brief Enable wheel speed limiting when the steering angle is not
  /// aligned with the commanded direction.
  public: bool limitSpeedOnSteeringErr{true};

  /// \brief Steering angle error (in radians) above which wheel speed
  /// scaling is applied.
  public: double steeringErrThreshold{GZ_PI / 10.0};

  /// \brief Exponent applied to the cosine-based speed scaling factor.
  public: int speedLimitPower{2};

  /// \brief Model interfaces
  public: Model model{kNullEntity};

  /// \brief The model's canonical link.
  public: Link canonicalLink{kNullEntity};

  /// \brief Update period calculated from <odom__publish_frequency>.
  public: std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
  public: std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Swerve drive odometry.
  public: math::SwerveDriveOdometry odom;

  /// \brief Swerve drive odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief Swerve drive tf message publisher.
  public: transport::Node::Publisher tfPub;

  /// \brief Linear velocity limiter.
  public: std::unique_ptr<gz::math::SpeedLimiter> limiterLin;

  /// \brief Angular velocity limiter.
  public: std::unique_ptr<gz::math::SpeedLimiter> limiterAng;

  /// \brief Previous control command.
  public: Commands last0Cmd;

  /// \brief Previous control command to last0Cmd.
  public: Commands last1Cmd;

  /// \brief Last target velocity requested.
  public: msgs::Twist targetVel;

  /// \brief A mutex to protect the target velocity command.
  public: std::mutex mutex;

  /// \brief P gain for angular position.
  public: double gainPAng{1.0};

  /// \brief frame_id from sdf.
  public: std::string sdfFrameId;

  /// \brief child_frame_id from sdf.
  public: std::string sdfChildFrameId;
};

//////////////////////////////////////////////////
SwerveDrive::SwerveDrive()
  : dataPtr(std::make_unique<SwerveDrivePrivate>())
{
}

//////////////////////////////////////////////////
void SwerveDrive::Configure(const Entity &_entity,
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
    gzerr << "SwerveDrive plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  if (_sdf->HasElement("front_left_wheel_joint"))
  {
    this->dataPtr->frontLeftWheelJointName =
      _sdf->Get<std::string>("front_left_wheel_joint");
  }
  if (_sdf->HasElement("front_right_wheel_joint"))
  {
    this->dataPtr->frontRightWheelJointName =
      _sdf->Get<std::string>("front_right_wheel_joint");
  }
  if (_sdf->HasElement("back_left_wheel_joint"))
  {
    this->dataPtr->backLeftWheelJointName =
      _sdf->Get<std::string>("back_left_wheel_joint");
  }
  if (_sdf->HasElement("back_right_wheel_joint"))
  {
    this->dataPtr->backRightWheelJointName =
      _sdf->Get<std::string>("back_right_wheel_joint");
  }
  if (_sdf->HasElement("front_left_steering_joint"))
  {
    this->dataPtr->frontLeftSteeringJointName =
      _sdf->Get<std::string>("front_left_steering_joint");
  }
  if (_sdf->HasElement("front_right_steering_joint"))
  {
    this->dataPtr->frontRightSteeringJointName =
      _sdf->Get<std::string>("front_right_steering_joint");
  }
  if (_sdf->HasElement("back_left_steering_joint"))
  {
    this->dataPtr->backLeftSteeringJointName =
      _sdf->Get<std::string>("back_left_steering_joint");
  }
  if (_sdf->HasElement("back_right_steering_joint"))
  {
    this->dataPtr->backRightSteeringJointName =
      _sdf->Get<std::string>("back_right_steering_joint");
  }

  this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
      this->dataPtr->wheelSeparation).first;
  this->dataPtr->wheelbase = _sdf->Get<double>("wheelbase",
      this->dataPtr->wheelbase).first;
  this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
      this->dataPtr->wheelRadius).first;

  // Get proportional gain for steering angle.
  if (_sdf->HasElement("steer_p_gain"))
  {
    this->dataPtr->gainPAng = _sdf->Get<double>("steer_p_gain");
  }

  // Parse limit speed on steering error parameters.
  if (_sdf->HasElement("limit_speed_on_steering_err"))
  {
    this->dataPtr->limitSpeedOnSteeringErr =
        _sdf->Get<bool>("limit_speed_on_steering_err");
  }
  if (_sdf->HasElement("steering_err_threshold"))
  {
    this->dataPtr->steeringErrThreshold =
        _sdf->Get<double>("steering_err_threshold");
  }
  if (_sdf->HasElement("speed_limit_power"))
  {
    this->dataPtr->speedLimitPower =
        _sdf->Get<int>("speed_limit_power");
  }

  // Instantiate the speed limiters.
  this->dataPtr->limiterLin = std::make_unique<gz::math::SpeedLimiter>();
  this->dataPtr->limiterAng = std::make_unique<gz::math::SpeedLimiter>();

  // Parse speed limiter parameters.

  // Min Velocity
  if (_sdf->HasElement("min_linear_velocity"))
  {
    const double minLinVel = _sdf->Get<double>("min_linear_velocity");
    this->dataPtr->limiterLin->SetMinVelocity(minLinVel);
  }
  if (_sdf->HasElement("min_angular_velocity"))
  {
    const double minAngVel = _sdf->Get<double>("min_angular_velocity");
    this->dataPtr->limiterAng->SetMinVelocity(minAngVel);
  }

  // Max Velocity
  if (_sdf->HasElement("max_linear_velocity"))
  {
    const double maxLinVel = _sdf->Get<double>("max_linear_velocity");
    this->dataPtr->limiterLin->SetMaxVelocity(maxLinVel);
  }
  if (_sdf->HasElement("max_angular_velocity"))
  {
    const double maxAngVel = _sdf->Get<double>("max_angular_velocity");
    this->dataPtr->limiterAng->SetMaxVelocity(maxAngVel);
  }

  // Min Acceleration
  if (_sdf->HasElement("min_linear_acceleration"))
  {
    const double minLinAccel = _sdf->Get<double>("min_linear_acceleration");
    this->dataPtr->limiterLin->SetMinAcceleration(minLinAccel);
  }
  if (_sdf->HasElement("min_angular_acceleration"))
  {
    const double minAngAccel = _sdf->Get<double>("min_angular_acceleration");
    this->dataPtr->limiterAng->SetMinAcceleration(minAngAccel);
  }

  // Max Acceleration
  if (_sdf->HasElement("max_linear_acceleration"))
  {
    const double maxLinAccel = _sdf->Get<double>("max_linear_acceleration");
    this->dataPtr->limiterLin->SetMaxAcceleration(maxLinAccel);
  }
  if (_sdf->HasElement("max_angular_acceleration"))
  {
    const double maxAngAccel = _sdf->Get<double>("max_angular_acceleration");
    this->dataPtr->limiterAng->SetMaxAcceleration(maxAngAccel);
  }

  // Min Jerk
  if (_sdf->HasElement("min_linear_jerk"))
  {
    const double minLinJerk = _sdf->Get<double>("min_linear_jerk");
    this->dataPtr->limiterLin->SetMinJerk(minLinJerk);
  }
  if (_sdf->HasElement("min_angular_jerk"))
  {
    const double minAngJerk = _sdf->Get<double>("min_angular_jerk");
    this->dataPtr->limiterAng->SetMinJerk(minAngJerk);
  }

  // Max Jerk
  if (_sdf->HasElement("max_linear_jerk"))
  {
    const double maxLinJerk = _sdf->Get<double>("max_linear_jerk");
    this->dataPtr->limiterLin->SetMaxJerk(maxLinJerk);
  }
  if (_sdf->HasElement("max_angular_jerk"))
  {
    const double maxAngJerk = _sdf->Get<double>("max_angular_jerk");
    this->dataPtr->limiterAng->SetMaxJerk(maxAngJerk);
  }

  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  if (odomFreq > 0)
  {
    std::chrono::duration<double> odomPer{1 / odomFreq};
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
  }

  // Setup odometry.
  this->dataPtr->odom.SetWheelParams(this->dataPtr->wheelSeparation,
      this->dataPtr->wheelbase,
      this->dataPtr->wheelRadius);

  // Subscribe to commands
  std::vector<std::string> topics;
  if (_sdf->HasElement("topic"))
  {
    topics.push_back(_sdf->Get<std::string>("topic"));
  }
  topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel");
  auto topic = validTopic(topics);

  this->dataPtr->node.Subscribe(topic, &SwerveDrivePrivate::OnCmdVel,
      this->dataPtr.get());

  std::vector<std::string> odomTopics;
  if (_sdf->HasElement("odom_topic"))
  {
    odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
  }
  odomTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
      "/odometry");
  auto odomTopic = validTopic(odomTopics);

  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopic);

  std::vector<std::string> tfTopics;
  if (_sdf->HasElement("tf_topic"))
  {
    tfTopics.push_back(_sdf->Get<std::string>("tf_topic"));
  }
  tfTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/tf");
  auto tfTopic = validTopic(tfTopics);

  this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
      tfTopic);

  if (_sdf->HasElement("frame_id"))
    this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

  if (_sdf->HasElement("child_frame_id"))
    this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");

  gzmsg << "SwerveDrive publishing odom messages on [" << odomTopic << "]"
         << std::endl;

  gzmsg << "SwerveDrive publishing tf messages on [" << tfTopic << "]"
         << std::endl;

  gzmsg << "SwerveDrive subscribing to twist messages on [" << topic << "]"
         << std::endl;
}

//////////////////////////////////////////////////
void SwerveDrive::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("SwerveDrive::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  static std::set<std::string> warnedModels;
  auto modelName = this->dataPtr->model.Name(_ecm);
  if (this->dataPtr->frontLeftWheelJoint == kNullEntity ||
      this->dataPtr->frontRightWheelJoint == kNullEntity ||
      this->dataPtr->backLeftWheelJoint == kNullEntity ||
      this->dataPtr->backRightWheelJoint == kNullEntity ||
      this->dataPtr->frontLeftSteeringJoint == kNullEntity ||
      this->dataPtr->frontRightSteeringJoint == kNullEntity ||
      this->dataPtr->backLeftSteeringJoint == kNullEntity ||
      this->dataPtr->backRightSteeringJoint == kNullEntity)
  {
    bool warned{false};
    if (this->dataPtr->frontLeftWheelJoint == kNullEntity)
    {
      std::string name = this->dataPtr->frontLeftWheelJointName;
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->frontLeftWheelJoint = joint;
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find front left wheel joint [" << name
                << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    if (this->dataPtr->frontRightWheelJoint == kNullEntity)
    {
      std::string name = this->dataPtr->frontRightWheelJointName;
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->frontRightWheelJoint = joint;
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find front right wheel joint [" << name
                << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    if (this->dataPtr->backLeftWheelJoint == kNullEntity)
    {
      std::string name = this->dataPtr->backLeftWheelJointName;
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->backLeftWheelJoint = joint;
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find back left wheel joint [" << name
                << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    if (this->dataPtr->backRightWheelJoint == kNullEntity)
    {
      std::string name = this->dataPtr->backRightWheelJointName;
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->backRightWheelJoint = joint;
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find back right wheel joint [" << name
                << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    if (this->dataPtr->frontLeftSteeringJoint == kNullEntity)
    {
      std::string name = this->dataPtr->frontLeftSteeringJointName;
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->frontLeftSteeringJoint = joint;
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find front left steering joint [" << name
                << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    if (this->dataPtr->frontRightSteeringJoint == kNullEntity)
    {
      std::string name = this->dataPtr->frontRightSteeringJointName;
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->frontRightSteeringJoint = joint;
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find front right steering joint [" << name
                << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    if (this->dataPtr->backLeftSteeringJoint == kNullEntity)
    {
      std::string name = this->dataPtr->backLeftSteeringJointName;
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->backLeftSteeringJoint = joint;
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find back left steering joint [" << name
                << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    if (this->dataPtr->backRightSteeringJoint == kNullEntity)
    {
      std::string name = this->dataPtr->backRightSteeringJointName;
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->backRightSteeringJoint = joint;
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find back right steering joint [" << name
                << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    if (warned)
    {
      warnedModels.insert(modelName);
    }
  }

  if (this->dataPtr->frontLeftWheelJoint == kNullEntity ||
      this->dataPtr->frontRightWheelJoint == kNullEntity ||
      this->dataPtr->backLeftWheelJoint == kNullEntity ||
      this->dataPtr->backRightWheelJoint == kNullEntity ||
      this->dataPtr->frontLeftSteeringJoint == kNullEntity ||
      this->dataPtr->frontRightSteeringJoint == kNullEntity ||
      this->dataPtr->backLeftSteeringJoint == kNullEntity ||
      this->dataPtr->backRightSteeringJoint == kNullEntity)
  {
    return;
  }

  if (warnedModels.find(modelName) != warnedModels.end())
  {
    gzmsg << "Found joints for model [" << modelName
           << "], plugin will start working." << std::endl;
    warnedModels.erase(modelName);
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Update wheel velocity
  _ecm.SetComponentData<components::JointVelocityCmd>(
    this->dataPtr->frontLeftWheelJoint,
    {this->dataPtr->frontLeftWheelJointSpeed});

  _ecm.SetComponentData<components::JointVelocityCmd>(
    this->dataPtr->frontRightWheelJoint,
    {this->dataPtr->frontRightWheelJointSpeed});

  _ecm.SetComponentData<components::JointVelocityCmd>(
    this->dataPtr->backLeftWheelJoint,
    {this->dataPtr->backLeftWheelJointSpeed});

  _ecm.SetComponentData<components::JointVelocityCmd>(
    this->dataPtr->backRightWheelJoint,
    {this->dataPtr->backRightWheelJointSpeed});

  _ecm.SetComponentData<components::JointVelocityCmd>(
    this->dataPtr->frontLeftSteeringJoint,
    {this->dataPtr->frontLeftSteeringJointSpeed});

  _ecm.SetComponentData<components::JointVelocityCmd>(
    this->dataPtr->frontRightSteeringJoint,
    {this->dataPtr->frontRightSteeringJointSpeed});

  _ecm.SetComponentData<components::JointVelocityCmd>(
    this->dataPtr->backLeftSteeringJoint,
    {this->dataPtr->backLeftSteeringJointSpeed});

  _ecm.SetComponentData<components::JointVelocityCmd>(
    this->dataPtr->backRightSteeringJoint,
    {this->dataPtr->backRightSteeringJointSpeed});

  // Create the joint position components if they don't exist.
  enableComponent<components::JointPosition>(
      _ecm, this->dataPtr->frontLeftWheelJoint);

  enableComponent<components::JointPosition>(
      _ecm, this->dataPtr->frontRightWheelJoint);

  enableComponent<components::JointPosition>(
      _ecm, this->dataPtr->backLeftWheelJoint);

  enableComponent<components::JointPosition>(
      _ecm, this->dataPtr->backRightWheelJoint);

  enableComponent<components::JointPosition>(
      _ecm, this->dataPtr->frontLeftSteeringJoint);

  enableComponent<components::JointPosition>(
      _ecm, this->dataPtr->frontRightSteeringJoint);

  enableComponent<components::JointPosition>(
      _ecm, this->dataPtr->backLeftSteeringJoint);

  enableComponent<components::JointPosition>(
      _ecm, this->dataPtr->backRightSteeringJoint);

  // Create the angular velocity components if they don't exist.
  enableComponent<components::JointVelocity>(
      _ecm, this->dataPtr->frontLeftWheelJoint);

  enableComponent<components::JointVelocity>(
      _ecm, this->dataPtr->frontRightWheelJoint);

  enableComponent<components::JointVelocity>(
      _ecm, this->dataPtr->backLeftWheelJoint);

  enableComponent<components::JointVelocity>(
      _ecm, this->dataPtr->backRightWheelJoint);

}

//////////////////////////////////////////////////
void SwerveDrive::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("SwerveDrive::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void SwerveDrivePrivate::UpdateOdometry(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("SwerveDrive::UpdateOdometry");

  if (this->frontLeftWheelJoint == kNullEntity ||
      this->frontRightWheelJoint == kNullEntity ||
      this->backLeftWheelJoint == kNullEntity ||
      this->backRightWheelJoint == kNullEntity ||
      this->frontLeftSteeringJoint == kNullEntity ||
      this->frontRightSteeringJoint == kNullEntity ||
      this->backLeftSteeringJoint == kNullEntity ||
      this->backRightSteeringJoint == kNullEntity)
  {
    return;
  }

  // Get the angular velocities for each wheel joint.
  auto frontLeftWheelAngVel = _ecm.Component<components::JointVelocity>(
    this->frontLeftWheelJoint);
  auto frontRightWheelAngVel = _ecm.Component<components::JointVelocity>(
    this->frontRightWheelJoint);
  auto backLeftWheelAngVel = _ecm.Component<components::JointVelocity>(
    this->backLeftWheelJoint);
  auto backRightWheelAngVel = _ecm.Component<components::JointVelocity>(
    this->backRightWheelJoint);

  // Get the positions for each steering joint.
  auto frontLeftSteeringPos = _ecm.Component<components::JointPosition>(
    this->frontLeftSteeringJoint);
  auto frontRightSteeringPos = _ecm.Component<components::JointPosition>(
    this->frontRightSteeringJoint);
  auto backLeftSteeringPos = _ecm.Component<components::JointPosition>(
    this->backLeftSteeringJoint);
  auto backRightSteeringPos = _ecm.Component<components::JointPosition>(
    this->backRightSteeringJoint);

  // Initialize, if not already initialized.
  if (!this->odom.Initialized())
  {
    this->odom.Init(std::chrono::steady_clock::time_point(_info.simTime));
    return;
  }

  // Abort if the joints were not found or just created.
  if (!frontLeftWheelAngVel || frontLeftWheelAngVel->Data().empty() ||
      !frontRightWheelAngVel || frontRightWheelAngVel->Data().empty() ||
      !backLeftWheelAngVel || backLeftWheelAngVel->Data().empty() ||
      !backRightWheelAngVel || backRightWheelAngVel->Data().empty() ||
      !frontLeftSteeringPos || frontLeftSteeringPos->Data().empty() ||
      !frontRightSteeringPos || frontRightSteeringPos->Data().empty() ||
      !backLeftSteeringPos || backLeftSteeringPos->Data().empty() ||
      !backRightSteeringPos || backRightSteeringPos->Data().empty())
  {
    return;
  }

  this->odom.Update(
      frontLeftWheelAngVel->Data()[0],
      frontRightWheelAngVel->Data()[0],
      backLeftWheelAngVel->Data()[0],
      backRightWheelAngVel->Data()[0],
      frontLeftSteeringPos->Data()[0],
      frontRightSteeringPos->Data()[0],
      backLeftSteeringPos->Data()[0],
      backRightSteeringPos->Data()[0],
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
  msg.mutable_twist()->mutable_linear()->set_y(this->odom.LateralVelocity());
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
  msgs::Pose_V tfMsg;
  gz::msgs::Pose *tfMsgPose = tfMsg.add_pose();
  tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
  tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
  tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());

  // Publish the messages
  this->odomPub.Publish(msg);
  this->tfPub.Publish(tfMsg);
}

//////////////////////////////////////////////////
void SwerveDrivePrivate::UpdateVelocity(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("SwerveDrive::UpdateVelocity");

  double linVel;
  double latVel;
  double angVel;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    linVel = this->targetVel.linear().x();
    latVel = this->targetVel.linear().y();
    angVel = this->targetVel.angular().z();
  }

  // Limit the target velocity if needed.
  this->limiterLin->Limit(
      linVel, this->last0Cmd.lin, this->last1Cmd.lin, _info.dt);
  this->limiterLin->Limit(
      latVel, this->last0Cmd.lat, this->last1Cmd.lat, _info.dt);
  this->limiterAng->Limit(
      angVel, this->last0Cmd.ang, this->last1Cmd.ang, _info.dt);

  // Update history of commands.
  this->last1Cmd = last0Cmd;
  this->last0Cmd.lin = linVel;
  this->last0Cmd.lat = latVel;
  this->last0Cmd.ang = angVel;

  // Convert the target velocities to joint velocities and angles
  // These calculations are based on the following references:
  // https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#swerve-drive-robots:~:text=Swerve%20Drive%20Robots%EF%83%81
  const double halfWheelbase = wheelbase / 2.0;
  const double halfWheelSeparation = wheelSeparation / 2.0;
  double frontLeftWheelLinVel = linVel - angVel * halfWheelSeparation;
  double frontLeftWheelLatVel = latVel + angVel * halfWheelbase;
  double frontRightWheelLinVel = linVel + angVel * halfWheelSeparation;
  double frontRightWheelLatVel = latVel + angVel * halfWheelbase;
  double backLeftWheelLinVel = linVel - angVel * halfWheelSeparation;
  double backLeftWheelLatVel = latVel - angVel * halfWheelbase;
  double backRightWheelLinVel = linVel + angVel * halfWheelSeparation;
  double backRightWheelLatVel = latVel - angVel * halfWheelbase;

  this->frontLeftWheelJointSpeed = sqrt(pow(frontLeftWheelLinVel, 2)
      + pow(frontLeftWheelLatVel, 2)) / wheelRadius;
  this->frontRightWheelJointSpeed = sqrt(pow(frontRightWheelLinVel, 2)
      + pow(frontRightWheelLatVel, 2)) / wheelRadius;
  this->backLeftWheelJointSpeed = sqrt(pow(backLeftWheelLinVel, 2)
      + pow(backLeftWheelLatVel, 2)) / wheelRadius;
  this->backRightWheelJointSpeed = sqrt(pow(backRightWheelLinVel, 2)
      + pow(backRightWheelLatVel, 2)) / wheelRadius;

  double frontLeftSteeringTarget = atan2(
      frontLeftWheelLatVel,
      frontLeftWheelLinVel);
  double frontRightSteeringTarget = atan2(
      frontRightWheelLatVel,
      frontRightWheelLinVel);
  double backLeftSteeringTarget = atan2(
      backLeftWheelLatVel,
      backLeftWheelLinVel);
  double backRightSteeringTarget = atan2(
      backRightWheelLatVel,
      backRightWheelLinVel);

  auto frontLeftSteeringPos = _ecm.Component<components::JointPosition>(
      this->frontLeftSteeringJoint);
  auto frontRightSteeringPos = _ecm.Component<components::JointPosition>(
      this->frontRightSteeringJoint);
  auto backLeftSteeringPos = _ecm.Component<components::JointPosition>(
      this->backLeftSteeringJoint);
  auto backRightSteeringPos = _ecm.Component<components::JointPosition>(
      this->backRightSteeringJoint);

  if (!frontLeftSteeringPos || frontLeftSteeringPos->Data().empty() ||
      !frontRightSteeringPos || frontRightSteeringPos->Data().empty() ||
      !backLeftSteeringPos || backLeftSteeringPos->Data().empty() ||
      !backRightSteeringPos || backRightSteeringPos->Data().empty())
  {
    // Cannot proceed without steering position data.
    return;
  }

  double frontLeftSteeringDelta =
    frontLeftSteeringTarget - frontLeftSteeringPos->Data()[0];
  double frontRightSteeringDelta =
    frontRightSteeringTarget - frontRightSteeringPos->Data()[0];
  double backLeftSteeringDelta =
    backLeftSteeringTarget - backLeftSteeringPos->Data()[0];
  double backRightSteeringDelta =
    backRightSteeringTarget - backRightSteeringPos->Data()[0];

  OptimizeWheelCmd(frontLeftSteeringDelta, this->frontLeftWheelJointSpeed);
  OptimizeWheelCmd(frontRightSteeringDelta, this->frontRightWheelJointSpeed);
  OptimizeWheelCmd(backLeftSteeringDelta, this->backLeftWheelJointSpeed);
  OptimizeWheelCmd(backRightSteeringDelta, this->backRightWheelJointSpeed);

  // Simple proportional control with settable gain.
  // Adding programmable PID values might be a future feature.
  this->frontLeftSteeringJointSpeed = this->gainPAng * frontLeftSteeringDelta;
  this->frontRightSteeringJointSpeed = this->gainPAng * frontRightSteeringDelta;
  this->backLeftSteeringJointSpeed = this->gainPAng * backLeftSteeringDelta;
  this->backRightSteeringJointSpeed = this->gainPAng * backRightSteeringDelta;

}

//////////////////////////////////////////////////
void SwerveDrivePrivate::OptimizeWheelCmd(
  double &_steeringDeltaAngle,
  double &_wheelSpeed)
{
  // If the steering angle is greater than 90 degrees,it's more efficient
  // to reverse the wheel direction and steer in the opposite direction.
  if (std::abs(_steeringDeltaAngle) > GZ_PI / 2)
  {
    _steeringDeltaAngle += (_steeringDeltaAngle > 0 ? -GZ_PI : GZ_PI);
    _wheelSpeed = -_wheelSpeed;
  }

  // Apply speed scaling during large steering error (if enabled).
  if (this->limitSpeedOnSteeringErr &&
      std::abs(_steeringDeltaAngle) > this->steeringErrThreshold)
  {
    // If the steering angle is greater than steeringErrThreshold, reduce the
    // wheel speed to prevent slipping.
    const double scale =
      std::pow(std::cos(_steeringDeltaAngle), this->speedLimitPower);
    _wheelSpeed *= scale;
  }
}

//////////////////////////////////////////////////
void SwerveDrivePrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}

GZ_ADD_PLUGIN(SwerveDrive,
              System,
              SwerveDrive::ISystemConfigure,
              SwerveDrive::ISystemPreUpdate,
              SwerveDrive::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(SwerveDrive,
                    "gz::sim::systems::SwerveDrive")
