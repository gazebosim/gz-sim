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

#ifndef GZ_SIM_SYSTEMS_ELEVATOR_COMMON_PRIVATE_HH_
#define GZ_SIM_SYSTEMS_ELEVATOR_COMMON_PRIVATE_HH_

#include <functional>
#include <mutex>
#include <vector>

#include <gz/transport/Node.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
class ElevatorCommonPrivate
{
  /// \brief Destructor
  public: virtual ~ElevatorCommonPrivate() = default;

  /// \brief Starts the timer that keeps the door at the target floor level open
  /// \param[in] _floorTarget Target floor level
  /// \param[in] _timeoutCallback Function to call upon timeout
  public: virtual void StartDoorTimer(
      int32_t _floorTarget, const std::function<void()> &_timeoutCallback) = 0;

  /// \brief Configures the monitor that checks the state of the joint of the
  /// door at the target floor level
  /// \param[in] _floorTarget Target floor level
  /// \param[in] _jointTarget Last joint target (command)
  /// \param[in] _posEps Position tolerance
  /// \param[in] _velEps Velocity tolerance
  /// \param[in] _jointTargetReachedCallback Function to call when the door
  /// joint reaches its target
  public: virtual void SetDoorMonitor(
      int32_t _floorTarget, double _jointTarget, double _posEps, double _velEps,
      const std::function<void()> &_jointTargetReachedCallback) = 0;

  /// \brief Configures the monitor that checks the state of the cabin joint
  /// \param[in] _floorTarget Target floor level
  /// \param[in] _jointTarget Last joint target (command)
  /// \param[in] _posEps Position tolerance
  /// \param[in] _velEps Velocity tolerance
  /// \param[in] _jointTargetReachedCallback Function to call when the cabin
  /// joint reaches its target
  public: virtual void SetCabinMonitor(
      int32_t _floorTarget, double _jointTarget, double _posEps, double _velEps,
      const std::function<void()> &_jointTargetReachedCallback) = 0;

  /// \brief Door joint command publishers
  public: std::vector<transport::Node::Publisher> doorJointCmdPubs;

  /// \brief Cabin joint command publisher
  public: transport::Node::Publisher cabinJointCmdPub;

  /// \brief Joint commands for opening the door at each floor level
  public: std::vector<double> doorTargets;

  /// \brief Cabin joint commands for each floor level
  public: std::vector<double> cabinTargets;

  /// \brief Current floor at which the cabin is
  public: int32_t state{0};

  /// \brief A mutex to protect access to targets and state
  public: std::recursive_mutex mutex;
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_SYSTEMS_ELEVATOR_COMMON_PRIVATE_HH_
