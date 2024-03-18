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

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/Model.hh>

#include <gz/math/PID.hh>

#include <gz/msgs/float.pb.h>
#include <gz/msgs/joint_trajectory.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>
#include <gz/transport/TopicUtils.hh>

#include <map>
#include <string>
#include <vector>

#include "JointTrajectoryController.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Helper class that contains all parameters required to create and
/// configure an instance of ActuatedJoint
class JointParameters
{
  /// \brief Parse all parameters required for creation of ActuatedJoint and
  /// return them in a map
  /// \param[in] _sdf SDF reference used to obtain the parameters
  /// \param[in] _ecm Gazebo Entity Component Manager
  /// \param[in] _enabledJoints List of joint entities that are enabled and
  /// need to be created
  /// \return Map of parameters for each joint, the first entry of pair
  /// indicates the joint name
  public: static std::map<std::string, JointParameters> ParseAll(
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              std::vector<Entity> _enabledJoints);

  /// \brief Parse all values of a single parameter that is specified multiple
  /// times in SDF
  /// \param[in] _sdf SDF reference used to obtain the parameters
  /// \param[in] _parameterName Name of the repeated parameter to parse all
  /// values for
  /// \return Ordered list of all values for a given repeated parameter
  public: template <typename T> static std::vector<T> Parse(
              const std::shared_ptr<const sdf::Element> &_sdf,
              const std::string &_parameterName);

  /// \brief Return value from `_vec` at `_index`, or `_alternative_value` if
  /// `_vec` is not large enough
  /// \param[in] _vec Vector that contains desired value of type T
  /// \param[in] _index Index at which the value is stored
  /// \param[in] _alternative_value Alternative or default value of type T
  /// \return Value from `_vec` at `_index`, or `_alternative_value` if `_vec`
  /// is not large enough
  public: template <typename T> static T NthElementOr(
              const std::vector<T> &_vec,
              const size_t &_index,
              const T &_alternative_value);

  /// \brief Initial position of the joint
  public: double initialPosition;
  /// \brief Default value for initial position of the joint
  public: static constexpr double initialPositionDefault = 0.0;

  /// \brief Parameters required for creation of new PID controller
  public: struct PID
  {
    /// \brief Proportional gain
    double pGain;
    /// \brief Default value for proportional gain
    static constexpr double pGainDefault = 0.0;

    /// \brief Integral gain
    double iGain;
    /// \brief Default value for integral gain
    static constexpr double iGainDefault = 0.0;

    /// \brief Derivative gain
    double dGain;
    /// \brief Default value for derivative gain
    static constexpr double dGainDefault = 0.0;

    /// \brief Integral lower limit
    double iMin;
    /// \brief Default value for integral lower limit
    static constexpr double iMinDefault = 0.0;

    /// \brief Integral upper limit
    double iMax;
    /// \brief Default value for integral upper limit
    static constexpr double iMaxDefault = -1.0;

    /// \brief Output min value
    double cmdMin;
    /// \brief Default value for output min value
    static constexpr double cmdMinDefault = 0.0;

    /// \brief Output max value
    double cmdMax;
    /// \brief Default value for output max value
    static constexpr double cmdMaxDefault = -1.0;

    /// \brief Output offset
    double cmdOffset;
    /// \brief Default value for output offset
    static constexpr double cmdOffsetDefault = 0.0;
  } positionPID, velocityPID;
};

/// \brief A single 1-axis joint that is controlled by JointTrajectoryController
/// plugin
class ActuatedJoint
{
  /// \brief Default contructor
  public: ActuatedJoint() = default;

  /// \brief Constructor that properly configures the actuated joint
  /// \param[in] _entity Entity of the joint
  /// \param[in] _params All parameters of the joint required for its
  /// configuration
  public: ActuatedJoint(const Entity &_entity,
                        const JointParameters &_params);

  /// \brief Setup components required for control of this joint
  /// \param[in,out] _ecm Gazebo Entity Component Manager
  public: void SetupComponents(
              gz::sim::EntityComponentManager &_ecm) const;

  /// \brief Set target of the joint that the controller will attempt to reach
  /// \param[in] _targetPoint Targets of all controlled joint
  /// \param[in] _jointIndex Index of the joint, used to determine what index
  /// of `_targetPoint` to use
  public: void SetTarget(
              const gz::msgs::JointTrajectoryPoint &_targetPoint,
              const size_t &_jointIndex);

  /// \brief Update command force that is applied on the joint
  /// \param[in,out] _ecm Gazebo Entity Component Manager
  /// \param[in] _dt Time difference to update for
  public: void Update(gz::sim::EntityComponentManager &_ecm,
                      const std::chrono::steady_clock::duration &_dt);

  /// \brief Reset the target of the joint
  public: void ResetTarget();

  /// \brief Reset the position and velocity PID error on the joint
  public: void ResetPIDs();

  /// \brief Entity of the joint
  public: Entity entity;

  /// \brief Target state that the joint controller should reach
  public: struct TargetState
  {
    /// \brief Target position of the joint
    double position;
    /// \brief Target position of the joint
    double velocity;
    /// \brief Target acceleration of the joint
    /// \attention Acceleration control is NOT implemented at the moment
    double acceleration;
    /// \brief Target force or torque of the joint
    double effort;
  } target;

  /// \brief Initial position of the joint
  public: double initialPosition;

  /// \brief List of PID controllers used for the control of this actuated joint
  public: struct PIDs
  {
    /// \brief Position PID controller
    gz::math::PID position;
    /// \brief Velocity PID controller
    gz::math::PID velocity;
  } pids;
};

/// \brief Information about trajectory that is followed by
/// JointTrajectoryController plugin
class Trajectory
{
  /// \brief Update index of trajectory points, such that it directs to a point
  /// that needs to be currently followed
  /// \param[in] _simTime Current simulation time
  /// \return True if index of the trajectory point was updated, False otherwise
  public: bool UpdateCurrentPoint(
              const std::chrono::steady_clock::duration &_simTime);

  /// \brief Determine if the trajectory goal was reached
  /// \return True if trajectory goal was reached, False otherwise
  public: bool IsGoalReached() const;

  /// \brief Compute progress of the current trajectory
  /// \return Fraction of the completed points in range (0.0, 1.0]
  public: float ComputeProgress() const;

  /// \brief Reset trajectory internals, i.e. clean list of joint names, points
  /// and reset index of the current point
  public: void Reset();

  /// \brief Status of the trajectory
  public: enum TrajectoryStatus
  {
    /// \brief Trajectory is new and needs to be configure on the next update
    /// loop
    New,
    /// \brief Trajectory is currently being followed
    Active,
    /// \brief Trajectory goal is reached
    Reached,
  } status;

  /// \brief Start time of trajectory
  public: std::chrono::steady_clock::duration startTime;

  /// \brief Index of the current trajectory point
  public: unsigned int pointIndex;

  /// \brief Ordered joints that need to be actuated to follow the current
  /// trajectory
  public: std::vector<std::string> jointNames;

  /// \brief Trajectory defined in terms of temporal points, whose members are
  /// ordered according to `jointNames`
  public: std::vector<gz::msgs::JointTrajectoryPoint> points;
};

/// \brief Private data of the JointTrajectoryController plugin
class gz::sim::systems::JointTrajectoryControllerPrivate
{
  /// \brief Get a list of enabled, unique, 1-axis joints of the model. If no
  /// joint names are specified in the plugin configuration, all valid 1-axis
  /// joints are returned
  /// \param[in] _entity Entity of the model that the plugin is being
  /// configured for
  /// \param[in] _sdf SDF reference used to determine enabled joints
  /// \param[in] _ecm Gazebo Entity Component Manager
  /// \return List of entities containinig all enabled joints
  public: std::vector<Entity> GetEnabledJoints(
              const Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              EntityComponentManager &_ecm) const;

  /// \brief Callback for joint trajectory subscription
  /// \param[in] _msg A new message describing a joint trajectory that needs
  /// to be followed
  public: void JointTrajectoryCallback(
              const gz::msgs::JointTrajectory &_msg);

  /// \brief Reset internals of the plugin, without affecting already created
  /// components
  public: void Reset();

  /// \brief Gazebo communication node
  public: transport::Node node;

  /// \brief Publisher of the progress for currently followed trajectory
  public: transport::Node::Publisher progressPub;

  /// \brief Map of actuated joints, where the first entry of pair is the name
  /// of the joint
  public: std::map<std::string, ActuatedJoint> actuatedJoints;

  /// \brief Mutex projecting trajectory
  public: std::mutex trajectoryMutex;

  /// \brief Information about trajectory that should be followed
  public: Trajectory trajectory;

  /// \brief Flag that determines whether to use message header timestamp as
  /// the trajectory start, where simulation time at the beginning of execution
  /// is used otherwise
  public: bool useHeaderStartTime;

  /// \brief Flag that determines if all components required for control are
  /// already setup
  public: bool componentSetupFinished;
};

/////////////////////////////////
/// JointTrajectoryController ///
/////////////////////////////////

//////////////////////////////////////////////////
JointTrajectoryController::JointTrajectoryController()
    : dataPtr(std::make_unique<JointTrajectoryControllerPrivate>())
{
}

//////////////////////////////////////////////////
void JointTrajectoryController::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager & /*_eventManager*/)
{
  // Make sure the controller is attached to a valid model
  const auto model = Model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "[JointTrajectoryController] Failed to initialize because ["
           << model.Name(_ecm) << "(Entity=" << _entity
           << ")] is not a model. Please make sure that"
              " JointTrajectoryController is attached to a valid model.\n";
    return;
  }
  gzmsg << "[JointTrajectoryController] Setting up controller for ["
         << model.Name(_ecm) << "(Entity=" << _entity << ")].\n";

  // Get list of enabled joints
  const auto enabledJoints = this->dataPtr->GetEnabledJoints(_entity,
                                                             _sdf,
                                                             _ecm);

  // For each enabled joint, parse all of its parameters from SDF
  auto jointParameters = JointParameters::ParseAll(_sdf, _ecm, enabledJoints);

  // Iterate over all enabled joints and create/configure them
  for (const auto &jointEntity : enabledJoints)
  {
    const auto jointName =
        _ecm.Component<components::Name>(jointEntity)->Data();
    this->dataPtr->actuatedJoints[jointName] =
        ActuatedJoint(jointEntity, jointParameters[jointName]);
    gzmsg << "[JointTrajectoryController] Configured joint ["
           << jointName << "(Entity=" << jointEntity << ")].\n";
  }

  // Make sure at least one joint is configured
  if (this->dataPtr->actuatedJoints.empty())
  {
    gzerr << "[JointTrajectoryController] Failed to initialize because ["
           << model.Name(_ecm) << "(Entity=" << _entity
           << ")] has no supported joints.\n";
    return;
  }

  // Get additional parameters from SDF
  if (_sdf->HasAttribute("use_header_start_time"))
  {
    this->dataPtr->useHeaderStartTime =
        _sdf->Get<bool>("use_header_start_time");
  }
  else
  {
    this->dataPtr->useHeaderStartTime = false;
  }

  // Subscribe to joint trajectory commands
  auto trajectoryTopic = _sdf->Get<std::string>("topic");
  if (trajectoryTopic.empty())
  {
    // If not specified, use the default topic based on model name
    trajectoryTopic = "/model/" + model.Name(_ecm) + "/joint_trajectory";
  }
  // Make sure the topic is valid
  const auto validTrajectoryTopic = transport::TopicUtils::AsValidTopic(
      trajectoryTopic);
  if (validTrajectoryTopic.empty())
  {
    gzerr << "[JointTrajectoryController] Cannot subscribe to invalid topic ["
           << trajectoryTopic << "].\n";
    return;
  }
  // Subscribe
  gzmsg << "[JointTrajectoryController] Subscribing to joint trajectory"
            " commands on topic [" << validTrajectoryTopic << "].\n";
  this->dataPtr->node.Subscribe(
      validTrajectoryTopic,
      &JointTrajectoryControllerPrivate::JointTrajectoryCallback,
      this->dataPtr.get());

  // Advertise progress
  const auto progressTopic = validTrajectoryTopic + "_progress";
  gzmsg << "[JointTrajectoryController] Advertising joint trajectory progress"
            " on topic [" << progressTopic << "].\n";
  this->dataPtr->progressPub =
      this->dataPtr->node.Advertise<gz::msgs::Float>(progressTopic);
}

//////////////////////////////////////////////////
void JointTrajectoryController::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("JointTrajectoryController::PreUpdate");

  // Create required components for each joint (only once)
  if (!this->dataPtr->componentSetupFinished)
  {
    for (auto &actuatedJoint : this->dataPtr->actuatedJoints)
    {
      ActuatedJoint *joint = &actuatedJoint.second;
      joint->SetupComponents(_ecm);
    }
    this->dataPtr->componentSetupFinished = true;
  }

  // Reset plugin if jump back in time is detected
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzmsg << "[JointTrajectoryController] Resetting plugin because jump back"
              " in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << " s] was detected.\n";
    this->dataPtr->Reset();
  }

  // Nothing else to do if paused
  if (_info.paused)
  {
    return;
  }

  // Update joint targets based on the current trajectory
  {
    auto isTargetUpdateRequired = false;

    // Lock mutex before accessing trajectory
    std::lock_guard<std::mutex> lock(this->dataPtr->trajectoryMutex);

    if (this->dataPtr->trajectory.status == Trajectory::New)
    {
      // Set trajectory start time if not set before
      if (this->dataPtr->trajectory.startTime.count() == 0)
      {
        this->dataPtr->trajectory.startTime = _info.simTime;
        this->dataPtr->trajectory.status = Trajectory::Active;
      }

      // If the new trajectory has no points, consider it reached
      if (this->dataPtr->trajectory.points.empty())
      {
        this->dataPtr->trajectory.status = Trajectory::Reached;
      }

      // Update is always needed for a new trajectory
      isTargetUpdateRequired = true;
    }

    if (this->dataPtr->trajectory.status == Trajectory::Active)
    {
      // Determine what point needs to be reached at the current time
      if (this->dataPtr->trajectory.UpdateCurrentPoint(_info.simTime))
      {
        // Update is needed if point was updated
        isTargetUpdateRequired = true;
      }
    }

    // Update the target for each joint that is defined in the
    // trajectory, if needed
    if (isTargetUpdateRequired &&
        this->dataPtr->trajectory.status != Trajectory::Reached)
    {
      const auto targetPoint =
          this->dataPtr->trajectory.points[this->dataPtr->trajectory
                                               .pointIndex];
      for (auto jointIndex = 0u;
           jointIndex < this->dataPtr->trajectory.jointNames.size();
           ++jointIndex)
      {
        const auto jointName = this->dataPtr->trajectory.jointNames[jointIndex];
        if (this->dataPtr->actuatedJoints.count(jointName) == 0)
        {
          // Warning about unconfigured joint is already logged above
          continue;
        }
        auto *joint = &this->dataPtr->actuatedJoints[jointName];
        joint->SetTarget(targetPoint, jointIndex);
      }

      // If there are no more points after the current one, set the trajectory
      // to Reached
      if (this->dataPtr->trajectory.IsGoalReached())
      {
        this->dataPtr->trajectory.status = Trajectory::Reached;
      }

      // Publish current progress of the trajectory
      gz::msgs::Float progressMsg;
      progressMsg.set_data(this->dataPtr->trajectory.ComputeProgress());
      this->dataPtr->progressPub.Publish(progressMsg);
    }
  }

  // Control loop
  for (auto &actuatedJoint : this->dataPtr->actuatedJoints)
  {
    auto *joint = &actuatedJoint.second;
    joint->Update(_ecm, _info.dt);
  }
}

////////////////////////////////////////
/// JointTrajectoryControllerPrivate ///
////////////////////////////////////////

//////////////////////////////////////////////////
std::vector<Entity> JointTrajectoryControllerPrivate::GetEnabledJoints(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm) const
{
  std::vector<Entity> output;

  // Get list of user-enabled joint names. If empty, enable all 1-axis joints
  const auto enabledJoints = JointParameters::Parse<std::string>(_sdf,
                                                                 "joint_name");

  // Get list of joint entities of the model
  // If there are joints explicitely enabled by the user, get only those
  std::vector<Entity> jointEntities;
  if (!enabledJoints.empty())
  {
    for (const auto &enabledJointName : enabledJoints)
    {
      auto enabledJointEntity = _ecm.ChildrenByComponents(
          _entity, components::Joint(), components::Name(enabledJointName));
      // Check that model has exactly one joint that matches the name
      if (enabledJointEntity.empty())
      {
        gzerr << "[JointTrajectoryController] Model does not contain joint ["
               << enabledJointName << "], which was explicitly enabled.\n";
        continue;
      }
      else if (enabledJointEntity.size() > 1)
      {
        gzwarn << "[JointTrajectoryController] Model has "
                << enabledJointEntity.size() << " duplicate joints named ["
                << enabledJointName << "]. Only the first (Entity="
                << enabledJointEntity[0] << ") will be configured.\n";
      }
      // Add entity to the list of enabled joints
      jointEntities.push_back(enabledJointEntity[0]);
    }
  }
  else
  {
    jointEntities = _ecm.ChildrenByComponents(_entity, components::Joint());
  }

  // Iterate over all joints and verify whether they can be enabled or not
  for (const auto &jointEntity : jointEntities)
  {
    const auto jointName = _ecm.Component<components::Name>(
                                   jointEntity)->Data();

    // Ignore duplicate joints
    for (const auto &actuatedJoint : this->actuatedJoints)
    {
      if (actuatedJoint.second.entity == jointEntity)
      {
        gzwarn << "[JointTrajectoryController] Ignoring duplicate joint ["
                << jointName << "(Entity=" << jointEntity << ")].\n";
        continue;
      }
    }

    // Make sure the joint type is supported, i.e. it has exactly one
    // actuated axis
    const auto *jointType = _ecm.Component<components::JointType>(jointEntity);
    switch (jointType->Data())
    {
      case sdf::JointType::PRISMATIC:
      case sdf::JointType::REVOLUTE:
      case sdf::JointType::CONTINUOUS:
      case sdf::JointType::GEARBOX:
      {
        // Supported joint type
        break;
      }
      case sdf::JointType::FIXED:
      {
        gzdbg << "[JointTrajectoryController] Fixed joint [" << jointName
               << "(Entity=" << jointEntity << ")] is skipped.\n";
        continue;
      }
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::SCREW:
      case sdf::JointType::BALL:
      case sdf::JointType::UNIVERSAL:
      {
        gzwarn << "[JointTrajectoryController] Joint [" << jointName
                << "(Entity=" << jointEntity
                << ")] is of unsupported type. Only joints with a single axis"
                   " are supported.\n";
        continue;
      }
      default:
      {
        gzwarn << "[JointTrajectoryController] Joint [" << jointName
                << "(Entity=" << jointEntity << ")] is of unknown type.\n";
        continue;
      }
    }
    output.push_back(jointEntity);
  }

  return output;
}

//////////////////////////////////////////////////
void JointTrajectoryControllerPrivate::JointTrajectoryCallback(
    const gz::msgs::JointTrajectory &_msg)
{
  // Make sure the message is valid
  if (_msg.joint_names_size() == 0)
  {
    gzwarn << "[JointTrajectoryController] JointTrajectory message does not"
               " contain any joint names.\n";
    return;
  }

  // Warn user that accelerations are currently ignored if the first point
  // contains them
  if (_msg.points(0).accelerations_size() > 0)
  {
    gzwarn << "[JointTrajectoryController] JointTrajectory message contains"
               " acceleration commands, which are currently ignored.\n";
  }

  // Lock mutex guarding the trajectory
  std::lock_guard<std::mutex> lock(this->trajectoryMutex);

  if (this->trajectory.status != Trajectory::Reached)
  {
    gzwarn << "[JointTrajectoryController] A new JointTrajectory message was"
               " received while executing a previous trajectory.\n";
  }

  // Get start time of the trajectory from message header if desired
  // If not enabled or there is no header, set start time to 0 and determine
  // it later from simTime
  if (this->useHeaderStartTime && _msg.has_header())
  {
    if (_msg.header().has_stamp())
    {
      const auto stamp = _msg.header().stamp();
      this->trajectory.startTime = std::chrono::seconds(stamp.sec()) +
                                   std::chrono::nanoseconds(stamp.nsec());
    }
  }
  else
  {
    this->trajectory.startTime = std::chrono::nanoseconds(0);
  }

  // Reset for a new trajectory
  this->trajectory.Reset();

  // Extract joint names and points
  for (const auto &joint_name : _msg.joint_names())
  {
    this->trajectory.jointNames.push_back(joint_name);
  }
  for (const auto &point : _msg.points())
  {
    this->trajectory.points.push_back(point);
  }
}

//////////////////////////////////////////////////
void JointTrajectoryControllerPrivate::Reset()
{
  for (auto &actuatedJoint : this->actuatedJoints)
  {
    auto *joint = &actuatedJoint.second;
    // Reset joint target
    joint->ResetTarget();
    // Reset PIDs
    joint->ResetPIDs();
  }

  // Reset trajectory
  this->trajectory.Reset();
}

///////////////////////
/// JointParameters ///
///////////////////////

//////////////////////////////////////////////////
std::map<std::string, JointParameters> JointParameters::ParseAll(
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    std::vector<Entity> _enabledJoints)
{
  std::map<std::string, JointParameters> output;

  const auto initialPositionAll   = JointParameters::Parse<double>(_sdf,
                                                     "initial_position");

  const auto positionPGainAll     = JointParameters::Parse<double>(_sdf,
                                                     "position_p_gain");
  const auto positionIGainAll     = JointParameters::Parse<double>(_sdf,
                                                     "position_i_gain");
  const auto positionDGainAll     = JointParameters::Parse<double>(_sdf,
                                                     "position_d_gain");
  const auto positionIMinAll      = JointParameters::Parse<double>(_sdf,
                                                     "position_i_min");
  const auto positionIMaxAll      = JointParameters::Parse<double>(_sdf,
                                                     "position_i_max");
  const auto positionCmdMinAll    = JointParameters::Parse<double>(_sdf,
                                                     "position_cmd_min");
  const auto positionCmdMaxAll    = JointParameters::Parse<double>(_sdf,
                                                     "position_cmd_max");
  const auto positionCmdOffsetAll = JointParameters::Parse<double>(_sdf,
                                                     "position_cmd_offset");

  const auto velocityPGainAll     = JointParameters::Parse<double>(_sdf,
                                                     "velocity_p_gain");
  const auto velocityIGainAll     = JointParameters::Parse<double>(_sdf,
                                                     "velocity_i_gain");
  const auto velocityDGainAll     = JointParameters::Parse<double>(_sdf,
                                                     "velocity_d_gain");
  const auto velocityIMinAll      = JointParameters::Parse<double>(_sdf,
                                                     "velocity_i_min");
  const auto velocityIMaxAll      = JointParameters::Parse<double>(_sdf,
                                                     "velocity_i_max");
  const auto velocityCmdMinAll    = JointParameters::Parse<double>(_sdf,
                                                     "velocity_cmd_min");
  const auto velocityCmdMaxAll    = JointParameters::Parse<double>(_sdf,
                                                     "velocity_cmd_max");
  const auto velocityCmdOffsetAll = JointParameters::Parse<double>(_sdf,
                                                     "velocity_cmd_offset");

  for (std::size_t i = 0; i < _enabledJoints.size(); ++i)
  {
    JointParameters params;
    params.initialPosition       = params.NthElementOr(initialPositionAll, i,
                                          params.initialPositionDefault);

    params.positionPID.pGain     = params.NthElementOr(positionPGainAll, i,
                                          params.positionPID.pGainDefault);
    params.positionPID.iGain     = params.NthElementOr(positionIGainAll, i,
                                          params.positionPID.iGainDefault);
    params.positionPID.dGain     = params.NthElementOr(positionDGainAll, i,
                                          params.positionPID.dGainDefault);
    params.positionPID.iMin      = params.NthElementOr(positionIMinAll, i,
                                          params.positionPID.iMinDefault);
    params.positionPID.iMax      = params.NthElementOr(positionIMaxAll, i,
                                          params.positionPID.iMaxDefault);
    params.positionPID.cmdMin    = params.NthElementOr(positionCmdMinAll, i,
                                          params.positionPID.cmdMinDefault);
    params.positionPID.cmdMax    = params.NthElementOr(positionCmdMaxAll, i,
                                          params.positionPID.cmdMaxDefault);
    params.positionPID.cmdOffset = params.NthElementOr(positionCmdOffsetAll, i,
                                          params.positionPID.cmdOffsetDefault);

    params.velocityPID.pGain     = params.NthElementOr(velocityPGainAll, i,
                                          params.velocityPID.pGainDefault);
    params.velocityPID.iGain     = params.NthElementOr(velocityIGainAll, i,
                                          params.velocityPID.iGainDefault);
    params.velocityPID.dGain     = params.NthElementOr(velocityDGainAll, i,
                                          params.velocityPID.dGainDefault);
    params.velocityPID.iMin      = params.NthElementOr(velocityIMinAll, i,
                                          params.velocityPID.iMinDefault);
    params.velocityPID.iMax      = params.NthElementOr(velocityIMaxAll, i,
                                          params.velocityPID.iMaxDefault);
    params.velocityPID.cmdMin    = params.NthElementOr(velocityCmdMinAll, i,
                                          params.velocityPID.cmdMinDefault);
    params.velocityPID.cmdMax    = params.NthElementOr(velocityCmdMaxAll, i,
                                          params.velocityPID.cmdMaxDefault);
    params.velocityPID.cmdOffset = params.NthElementOr(velocityCmdOffsetAll, i,
                                          params.velocityPID.cmdOffsetDefault);

    const auto jointName = _ecm.Component<components::Name>(
                                   _enabledJoints[i])->Data();
    output[jointName] = params;
  }

  return output;
}

//////////////////////////////////////////////////
template <typename T>
std::vector<T> JointParameters::Parse(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_parameterName)
{
  std::vector<T> output;

  if (_sdf->HasElement(_parameterName))
  {
    auto param = _sdf->FindElement(_parameterName);
    while (param)
    {
      output.push_back(param->Get<T>());
      param = param->GetNextElement(_parameterName);
    }
  }

  return output;
}

////////////////////////////////////////////////
template <typename T>
T JointParameters::NthElementOr(const std::vector<T> &_vec,
                                const size_t &_index,
                                const T &_alternative_value)
{
  if (_index < _vec.size())
  {
    return _vec[_index];
  }
  else
  {
    return _alternative_value;
  }
}

/////////////////////
/// ActuatedJoint ///
/////////////////////

//////////////////////////////////////////////////
ActuatedJoint::ActuatedJoint(const Entity &_entity,
                             const JointParameters &_params)
{
  this->entity = _entity;

  this->initialPosition = _params.initialPosition;
  this->target.position = _params.initialPosition;
  this->target.velocity = 0.0;
  this->target.acceleration = 0.0;
  this->target.effort = 0.0;

  this->pids.position = gz::math::PID(_params.positionPID.pGain,
                                            _params.positionPID.iGain,
                                            _params.positionPID.dGain,
                                            _params.positionPID.iMax,
                                            _params.positionPID.iMin,
                                            _params.positionPID.cmdMax,
                                            _params.positionPID.cmdMin,
                                            _params.positionPID.cmdOffset);

  this->pids.velocity = gz::math::PID(_params.velocityPID.pGain,
                                            _params.velocityPID.iGain,
                                            _params.velocityPID.dGain,
                                            _params.velocityPID.iMax,
                                            _params.velocityPID.iMin,
                                            _params.velocityPID.cmdMax,
                                            _params.velocityPID.cmdMin,
                                            _params.velocityPID.cmdOffset);

  gzdbg << "[JointTrajectoryController] Parameters for joint (Entity="
         << _entity << "):\n"
         << "initial_position: ["    << _params.initialPosition       << "]\n"
         << "position_p_gain: ["     << _params.positionPID.pGain     << "]\n"
         << "position_i_gain: ["     << _params.positionPID.iGain     << "]\n"
         << "position_d_gain: ["     << _params.positionPID.dGain     << "]\n"
         << "position_i_min: ["      << _params.positionPID.iMax      << "]\n"
         << "position_i_max: ["      << _params.positionPID.iMax      << "]\n"
         << "position_cmd_min: ["    << _params.positionPID.cmdMin    << "]\n"
         << "position_cmd_max: ["    << _params.positionPID.cmdMax    << "]\n"
         << "position_cmd_offset: [" << _params.positionPID.cmdOffset << "]\n"
         << "velocity_p_gain: ["     << _params.velocityPID.pGain     << "]\n"
         << "velocity_i_gain: ["     << _params.velocityPID.iGain     << "]\n"
         << "velocity_d_gain: ["     << _params.velocityPID.dGain     << "]\n"
         << "velocity_i_min: ["      << _params.velocityPID.iMax      << "]\n"
         << "velocity_i_max: ["      << _params.velocityPID.iMax      << "]\n"
         << "velocity_cmd_min: ["    << _params.velocityPID.cmdMin    << "]\n"
         << "velocity_cmd_max: ["    << _params.velocityPID.cmdMax    << "]\n"
         << "velocity_cmd_offset: [" << _params.velocityPID.cmdOffset << "]\n";
}

//////////////////////////////////////////////////
void ActuatedJoint::SetupComponents(
    gz::sim::EntityComponentManager &_ecm) const
{
  // Create JointPosition component if one does not exist
  if (nullptr == _ecm.Component<components::JointPosition>(this->entity))
  {
    _ecm.CreateComponent(this->entity, components::JointPosition());
  }

  // Create JointVelocity component if one does not exist
  if (nullptr == _ecm.Component<components::JointVelocity>(this->entity))
  {
    _ecm.CreateComponent(this->entity, components::JointVelocity());
  }

  // Create JointForceCmd component if one does not exist
  if (nullptr == _ecm.Component<components::JointForceCmd>(this->entity))
  {
    _ecm.CreateComponent(this->entity, components::JointForceCmd({0.0}));
  }
}

//////////////////////////////////////////////////
void ActuatedJoint::SetTarget(
    const gz::msgs::JointTrajectoryPoint &_targetPoint,
    const size_t &_jointIndex)
{
  if ((signed)_jointIndex < _targetPoint.positions_size())
  {
    this->target.position = _targetPoint.positions(_jointIndex);
  }
  if ((signed)_jointIndex < _targetPoint.velocities_size())
  {
    this->target.velocity = _targetPoint.velocities(_jointIndex);
  }
  if ((signed)_jointIndex < _targetPoint.accelerations_size())
  {
    this->target.acceleration = _targetPoint.accelerations(_jointIndex);
  }
  if ((signed)_jointIndex < _targetPoint.effort_size())
  {
    this->target.effort = _targetPoint.effort(_jointIndex);
  }
}

//////////////////////////////////////////////////
void ActuatedJoint::Update(gz::sim::EntityComponentManager &_ecm,
                           const std::chrono::steady_clock::duration &_dt)
{
  // Get JointPosition and JointVelocity components
  const auto jointPositionComponent = _ecm.Component<components::JointPosition>(
      this->entity);
  const auto jointVelocityComponent = _ecm.Component<components::JointVelocity>(
      this->entity);

  // Compute control errors and force for each PID controller
  double forcePosition = 0.0, forceVelocity = 0.0;
  if (!jointPositionComponent->Data().empty())
  {
    double errorPosition = jointPositionComponent->Data()[0] -
                           this->target.position;
    forcePosition = this->pids.position.Update(errorPosition, _dt);
  }
  if (!jointVelocityComponent->Data().empty())
  {
    double errorVelocity = jointVelocityComponent->Data()[0] -
                           this->target.velocity;
    forceVelocity = this->pids.velocity.Update(errorVelocity, _dt);
  }

  // Sum all forces
  const double force = forcePosition + forceVelocity + this->target.effort;

  // Get JointForceCmd component and apply command force
  auto jointForceCmdComponent = _ecm.Component<components::JointForceCmd>(
      this->entity);
  jointForceCmdComponent->Data()[0] = force;
}

//////////////////////////////////////////////////
void ActuatedJoint::ResetTarget()
{
  this->target.position = this->initialPosition;
  this->target.velocity = 0.0;
  this->target.acceleration = 0.0;
  this->target.effort = 0.0;
}

//////////////////////////////////////////////////
void ActuatedJoint::ResetPIDs()
{
  this->pids.position.Reset();
  this->pids.velocity.Reset();
}

//////////////////
/// Trajectory ///
//////////////////

//////////////////////////////////////////////////
bool Trajectory::UpdateCurrentPoint(
    const std::chrono::steady_clock::duration &_simTime)
{
  bool isUpdated = false;

  const auto trajectoryTime = _simTime - this->startTime;
  while (true)
  {
    // Break if end of trajectory is reached (there are no more points after
    // the current one)
    if (this->IsGoalReached())
    {
      break;
    }

    // Break if point needs to be followed
    const auto pointTFS = this->points[this->pointIndex].time_from_start();
    const auto pointTime = std::chrono::seconds(pointTFS.sec()) +
                           std::chrono::nanoseconds(pointTFS.nsec());
    if (pointTime >= trajectoryTime)
    {
      break;
    }

    // Otherwise increment and try again (joint targets need to be updated)
    ++this->pointIndex;
    isUpdated = true;
  };

  // Return true if a point index was updated
  return isUpdated;
}

//////////////////////////////////////////////////
bool Trajectory::IsGoalReached() const
{
  return this->pointIndex + 1 >= this->points.size();
}

//////////////////////////////////////////////////
float Trajectory::ComputeProgress() const
{
  if (this->points.size() == 0)
  {
    return 1.0;
  }
  else
  {
    return static_cast<float>(this->pointIndex + 1) /
           static_cast<float>(this->points.size());
  }
}

//////////////////////////////////////////////////
void Trajectory::Reset()
{
  this->status = Trajectory::New;
  this->pointIndex = 0;
  this->jointNames.clear();
  this->points.clear();
}

// Register plugin
GZ_ADD_PLUGIN(JointTrajectoryController,
                    gz::sim::System,
                    JointTrajectoryController::ISystemConfigure,
                    JointTrajectoryController::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(
    JointTrajectoryController,
    "gz::sim::systems::JointTrajectoryController")
