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

/*
 * \author Nick Lamprianidis <nlamprian@gmail.com>
 * \date January 2021
 */

#include <algorithm>
#include <chrono>
#include <mutex>
#include <regex>
#include <string>
#include <vector>

#include "Elevator.hh"
#include "ElevatorCommonPrivate.hh"
#include "ElevatorStateMachine.hh"
#include "utils/DoorTimer.hh"
#include "utils/JointMonitor.hh"

#include <gz/msgs/double.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/laserscan.pb.h>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
class ElevatorPrivate : public ElevatorCommonPrivate
{
  /// \brief Destructor
  public: virtual ~ElevatorPrivate() override;

  /// \brief Initializes the cabin of the elevator
  /// \param[in] _cabinJointName Name of the cabin joint
  /// \param[in] _floorLinkPrefix Name prefix of the floor links
  /// \param[in] _topicPrefix Topic prefix for the command publisher
  /// \param[in] _ecm Entity component manager
  /// \return True on successful initialization, or false otherwise
  public: bool InitCabin(const std::string &_cabinJointName,
                         const std::string &_floorLinkPrefix,
                         const std::string &_topicPrefix,
                         EntityComponentManager &_ecm);

  /// \brief Initializes the doors of the elevator
  /// \param[in] _doorJointPrefix Name prefix of the door joints
  /// \param[in] _topicPrefix Topic prefix for the command publishers
  /// \param[in] _ecm Entity component manager
  /// \return True on successful initialization, or false otherwise
  public: bool InitDoors(const std::string &_doorJointPrefix,
                         const std::string &_topicPrefix,
                         EntityComponentManager &_ecm);

  // Documentation inherited
  public: virtual void StartDoorTimer(
      int32_t _floorTarget,
      const std::function<void()> &_timeoutCallback) override;

  // Documentation inherited
  public: virtual void SetDoorMonitor(
      int32_t _floorTarget, double _jointTarget, double _posEps, double _velEps,
      const std::function<void()> &_jointTargetReachedCallback) override;

  // Documentation inherited
  public: virtual void SetCabinMonitor(
      int32_t _floorTarget, double _jointTarget, double _posEps, double _velEps,
      const std::function<void()> &_jointTargetReachedCallback) override;

  /// \brief Updates the elevator state based on the current cabin position and
  /// then publishes the new state
  /// \param[in] _info Current simulation step info
  /// \param[in] _ecm Entity component manager
  public: void UpdateState(const gz::sim::UpdateInfo &_info,
                           const EntityComponentManager &_ecm);

  /// \brief Callback for the door lidar scans
  /// \param[in] _floorLevel Floor level
  /// \param[in] _msg Laserscan message
  public: void OnLidarMsg(size_t _floorLevel, const msgs::LaserScan &_msg);

  /// \brief Callback for the elevator commands
  /// \param[in] _msg Message that carries the floor level target
  public: void OnCmdMsg(const msgs::Int32 &_msg);

  /// \brief Gazebo communication node
  public: transport::Node node;

  /// \brief Model to which this system belongs
  public: Model model;

  /// \brief Joints of the doors of the elevator
  public: std::vector<Entity> doorJoints;

  /// \brief Joint of the cabin
  public: Entity cabinJoint;

  /// \brief State vector that identifies whether the doorway on each floor
  /// level is blocked
  public: std::vector<bool> isDoorwayBlockedStates;

  /// \brief Timer that keeps the door at the target floor level open
  public: std::unique_ptr<DoorTimer> doorTimer;

  /// \brief Monitor that checks whether the door at the target floor level has
  /// been opened or closed
  public: JointMonitor doorJointMonitor;

  /// \brief Monitor that checks whether the cabin has reached the target floor
  /// level
  public: JointMonitor cabinJointMonitor;

  /// \brief System update period calculated from <update_rate>
  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Last system update simulation time
  public: std::chrono::steady_clock::duration lastUpdateTime{0};

  /// \brief State publish period calculated from <state_publish_rate>
  public: std::chrono::steady_clock::duration statePubPeriod{0};

  /// \brief Last state publish simulation time
  public: std::chrono::steady_clock::duration lastStatePubTime{0};

  /// \brief Elevator state publisher
  public: transport::Node::Publisher statePub;

  /// \brief Elevator state message
  public: msgs::Int32 stateMsg;

  /// \brief Elevator state machine
  public: std::unique_ptr<ElevatorStateMachine> stateMachine;
};

//////////////////////////////////////////////////
Elevator::Elevator() : dataPtr(std::make_shared<ElevatorPrivate>()) {}

//////////////////////////////////////////////////
void Elevator::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  // Initialize system update period
  double rate = _sdf->Get<double>("update_rate", 10).first;
  std::chrono::duration<double> period{rate > 0 ? 1 / rate : 0};
  this->dataPtr->updatePeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  // Get floor link prefix
  std::string floorLinkPrefix =
      _sdf->Get<std::string>("floor_link_prefix", "floor_").first;

  // Get door joint prefix
  std::string doorJointPrefix =
      _sdf->Get<std::string>("door_joint_prefix", "door_").first;

  // Get cabin joint name
  std::string cabinJointName =
      _sdf->Get<std::string>("cabin_joint", "lift").first;

  std::string topicPrefix = "/model/" + this->dataPtr->model.Name(_ecm);

  if (!this->dataPtr->InitCabin(cabinJointName, floorLinkPrefix, topicPrefix,
                                _ecm))
    return;

  if (!this->dataPtr->InitDoors(doorJointPrefix, topicPrefix, _ecm))
    return;

  // Initialize door timer
  double duration = _sdf->Get<double>("open_door_wait_duration", 5.0).first;
  this->dataPtr->doorTimer = std::make_unique<DoorTimer>(
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(duration)));

  // Initialize state publisher
  std::string stateTopicName =
      _sdf->Get<std::string>("state_topic", topicPrefix + "/state").first;
  // NOTE Topic should be latched; no latch option so far
  this->dataPtr->statePub =
      this->dataPtr->node.Advertise<msgs::Int32>(stateTopicName);

  // Initialize state publish period
  double stateRate = _sdf->Get<double>("state_publish_rate", 5.0).first;
  std::chrono::duration<double> statePeriod{stateRate > 0 ? 1 / stateRate : 0};
  this->dataPtr->statePubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          statePeriod);

  // Initialize state machine
  this->dataPtr->stateMachine =
      std::make_unique<ElevatorStateMachine>(this->dataPtr);

  // Subscribe to command topic
  std::string cmdTopicName =
      _sdf->Get<std::string>("cmd_topic", topicPrefix + "/cmd").first;
  this->dataPtr->node.Subscribe(cmdTopicName, &ElevatorPrivate::OnCmdMsg,
                                this->dataPtr.get());
  gzmsg << "System " << this->dataPtr->model.Name(_ecm) << " subscribed to "
         << cmdTopicName << " for command messages" << std::endl;
}

//////////////////////////////////////////////////
void Elevator::PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Elevator::PostUpdate");
  if (_info.paused) return;

  // Throttle update rate
  auto elapsed = _info.simTime - this->dataPtr->lastUpdateTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->dataPtr->updatePeriod)
    return;
  this->dataPtr->lastUpdateTime = _info.simTime;

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  this->dataPtr->UpdateState(_info, _ecm);
  this->dataPtr->doorTimer->Update(
      _info, this->dataPtr->isDoorwayBlockedStates[this->dataPtr->state]);
  this->dataPtr->doorJointMonitor.Update(_ecm);
  this->dataPtr->cabinJointMonitor.Update(_ecm);
}

//////////////////////////////////////////////////
ElevatorPrivate::~ElevatorPrivate()
{
}

//////////////////////////////////////////////////
bool ElevatorPrivate::InitCabin(const std::string &_cabinJointName,
                                const std::string &_floorLinkPrefix,
                                const std::string &_topicPrefix,
                                EntityComponentManager &_ecm)
{
  // Validate and initialize cabin joint
  this->cabinJoint = this->model.JointByName(_ecm, _cabinJointName);
  if (this->cabinJoint == kNullEntity)
  {
    gzerr << "Failed to find cabin joint " << _cabinJointName << std::endl;
    return false;
  }
  if (!_ecm.EntityHasComponentType(this->cabinJoint,
                                   components::JointPosition().TypeId()))
    _ecm.CreateComponent(this->cabinJoint, components::JointPosition());
  if (!_ecm.EntityHasComponentType(this->cabinJoint,
                                   components::JointVelocity().TypeId()))
    _ecm.CreateComponent(this->cabinJoint, components::JointVelocity());

  // Initialize cabin floor targets
  size_t numFloorLinks = 0;
  std::regex floorLinkRegex(_floorLinkPrefix + "\\d+");
  std::vector<Entity> links =
      _ecm.ChildrenByComponents(this->model.Entity(), components::Link());
  for (const auto &link : links)
  {
    auto name = _ecm.Component<components::Name>(link)->Data();
    numFloorLinks += std::regex_match(name, floorLinkRegex);
  }
  for (size_t i = 0; i < numFloorLinks; ++i)
  {
    auto name = _floorLinkPrefix + std::to_string(i);
    auto link = this->model.LinkByName(_ecm, name);
    if (link == kNullEntity)
    {
      gzerr << "Failed to find floor link " << name << std::endl;
      return false;
    }
    auto z = _ecm.Component<components::Pose>(link)->Data().Z();
    this->cabinTargets.push_back(z);
  }

  // Initialize cabin joint command publisher
  std::string cabinJointCmdTopicName =
      _topicPrefix + "/joint/" + _cabinJointName + "/0/cmd_pos";
  this->cabinJointCmdPub =
      this->node.Advertise<msgs::Double>(cabinJointCmdTopicName);

  return true;
}

//////////////////////////////////////////////////
bool ElevatorPrivate::InitDoors(const std::string &_doorJointPrefix,
                                const std::string &_topicPrefix,
                                EntityComponentManager &_ecm)
{
  for (size_t i = 0; i < this->cabinTargets.size(); ++i)
  {
    // Validate and initialize door joint
    auto name = _doorJointPrefix + std::to_string(i);
    auto joint = this->model.JointByName(_ecm, name);
    if (joint == kNullEntity)
    {
      gzerr << "Failed to find door joint " << name << std::endl;
      return false;
    }
    if (!_ecm.EntityHasComponentType(joint,
                                     components::JointPosition().TypeId()))
      _ecm.CreateComponent(joint, components::JointPosition());
    if (!_ecm.EntityHasComponentType(joint,
                                     components::JointVelocity().TypeId()))
      _ecm.CreateComponent(joint, components::JointVelocity());
    this->doorJoints.push_back(joint);

    // Initialize open door target
    auto upper = _ecm.Component<components::JointAxis>(joint)->Data().Upper();
    this->doorTargets.push_back(upper);

    // Initialize door joint command publisher
    std::string topicName = _topicPrefix + "/joint/" + name + "/0/cmd_pos";
    auto pub = this->node.Advertise<msgs::Double>(topicName);
    this->doorJointCmdPubs.push_back(pub);
  }

  // Initialize blocked doorway states
  this->isDoorwayBlockedStates =
      std::vector<bool>(this->doorJoints.size(), false);

  // Subscribe to lidar topics
  for (size_t i = 0; i < this->doorJoints.size(); ++i)
  {
    auto jointName = _doorJointPrefix + std::to_string(i);
    std::string topicName = _topicPrefix + "/" + jointName + "/lidar";
    std::function<void(const msgs::LaserScan &)> callback =
        std::bind(&ElevatorPrivate::OnLidarMsg, this, i, std::placeholders::_1);
    this->node.Subscribe(topicName, callback);
  }

  return true;
}

//////////////////////////////////////////////////
void ElevatorPrivate::StartDoorTimer(
    int32_t /*_floorTarget*/, const std::function<void()> &_timeoutCallback)
{
  std::lock_guard<std::recursive_mutex> lock(this->mutex);
  this->doorTimer->Configure(this->lastUpdateTime, _timeoutCallback);
}

//////////////////////////////////////////////////
void ElevatorPrivate::SetDoorMonitor(
    int32_t _floorTarget, double _jointTarget, double _posEps, double _velEps,
    const std::function<void()> &_jointTargetReachedCallback)
{
  std::lock_guard<std::recursive_mutex> lock(this->mutex);
  this->doorJointMonitor.Configure(this->doorJoints[_floorTarget], _jointTarget,
                                   _posEps, _velEps,
                                   _jointTargetReachedCallback);
}

//////////////////////////////////////////////////
void ElevatorPrivate::SetCabinMonitor(
    int32_t /*_floorTarget*/, double _jointTarget, double _posEps,
    double _velEps, const std::function<void()> &_jointTargetReachedCallback)
{
  std::lock_guard<std::recursive_mutex> lock(this->mutex);
  this->cabinJointMonitor.Configure(this->cabinJoint, _jointTarget, _posEps,
                                    _velEps, _jointTargetReachedCallback);
}

//////////////////////////////////////////////////
void ElevatorPrivate::UpdateState(const gz::sim::UpdateInfo &_info,
                                  const EntityComponentManager &_ecm)
{
  // Update state
  double pos =
      _ecm.ComponentData<components::JointPosition>(this->cabinJoint)->front();
  std::vector<int32_t> diffs(this->cabinTargets.size());
  std::transform(this->cabinTargets.begin(), this->cabinTargets.end(),
                 diffs.begin(),
                 [&pos](auto target) { return std::fabs(target - pos); });
  auto it = std::min_element(diffs.begin(), diffs.end());
  this->state = static_cast<int32_t>(std::distance(diffs.begin(), it));

  // Throttle publish rate
  auto elapsed = _info.simTime - this->lastStatePubTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->statePubPeriod)
    return;
  this->lastStatePubTime = _info.simTime;

  this->stateMsg.set_data(this->state);
  this->statePub.Publish(this->stateMsg);
}

//////////////////////////////////////////////////
void ElevatorPrivate::OnLidarMsg(size_t _floorLevel,
                                 const msgs::LaserScan &_msg)
{
  if (_msg.ranges_size() <= 0)
    return;

  bool isDoorwayBlocked = _msg.ranges(0) < _msg.range_max() - 0.005;
  if (isDoorwayBlocked == this->isDoorwayBlockedStates[_floorLevel]) return;
  std::lock_guard<std::recursive_mutex> lock(this->mutex);
  this->isDoorwayBlockedStates[_floorLevel] = isDoorwayBlocked;
}

//////////////////////////////////////////////////
void ElevatorPrivate::OnCmdMsg(const msgs::Int32 &_msg)
{
  auto target = _msg.data();
  if (target < 0 || target >= static_cast<int32_t>(this->cabinTargets.size()))
  {
    gzwarn << "Invalid target [" << target << "]; command must be in [0,"
            << this->cabinTargets.size() << ")" << std::endl;
    return;
  }
  this->stateMachine->process_event(events::EnqueueNewTarget(_msg.data()));
}

GZ_ADD_PLUGIN(Elevator, System, Elevator::ISystemConfigure,
                    Elevator::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(Elevator, "gz::sim::systems::Elevator")
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
