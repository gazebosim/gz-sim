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

#ifndef GZ_SIM_SYSTEMS_JOINT_MONITOR_HH_
#define GZ_SIM_SYSTEMS_JOINT_MONITOR_HH_

#include <functional>
#include <memory>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
// Data forward declaration
class JointMonitorPrivate;

/// \brief Monitor that checks the state of a joint. When the joint reaches the
/// configured target, it calls a function to let the state machine know to
/// transition to the next state.
class JointMonitor
{
  /// \brief Constructor
  public: JointMonitor();

  /// \brief Destructor
  public: ~JointMonitor();

  /// \brief Starts monitoring of the position and velocity of the given joint
  /// \param[in] _joint Joint to monitor
  /// \param[in] _jointTarget Last joint target (command)
  /// \param[in] _posEps Position tolerance
  /// \param[in] _velEps Velocity tolerance
  /// \param[in] _jointTargetReachedCallback Function to call when the joint
  /// reaches its target
  public: void Configure(
      Entity _joint, double _target, double _posEps, double _velEps,
      const std::function<void()> &_jointTargetReachedCallback);

  /// \brief Checks whether the position and velocity of the joint are within
  /// the configured tolerances
  /// \param[in] _ecm Entity component manager
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Private data pointer
  private: std::unique_ptr<JointMonitorPrivate> dataPtr;
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_SYSTEMS_JOINT_MONITOR_HH_
