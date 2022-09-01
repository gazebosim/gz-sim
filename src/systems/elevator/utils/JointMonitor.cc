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

#include <memory>

#include "JointMonitor.hh"

#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
class JointMonitorPrivate
{
  /// \brief Joint under monitoring
  public: Entity joint;

  /// \brief Last joint target
  public: double target;

  /// \brief Position tolerance
  public: double posEps;

  /// \brief Velocity tolerance
  public: double velEps;

  /// \brief Function that gets called when the joint reaches its target
  public: std::function<void()> targetReachedCallback;

  /// \brief Flag to indicate whether the monitor is active
  public: bool isActive{false};
};

//////////////////////////////////////////////////
JointMonitor::JointMonitor() : dataPtr(std::make_unique<JointMonitorPrivate>())
{
}

//////////////////////////////////////////////////
JointMonitor::~JointMonitor() = default;

//////////////////////////////////////////////////
void JointMonitor::Configure(
    Entity _joint, double _target, double _posEps, double _velEps,
    const std::function<void()> &_jointTargetReachedCallback)
{
  this->dataPtr->isActive = true;
  this->dataPtr->joint = _joint;
  this->dataPtr->target = _target;
  this->dataPtr->posEps = _posEps;
  this->dataPtr->velEps = _velEps;
  this->dataPtr->targetReachedCallback = _jointTargetReachedCallback;
}

//////////////////////////////////////////////////
void JointMonitor::Update(const EntityComponentManager &_ecm)
{
  if (!this->dataPtr->isActive)
    return;
  double pos =
      _ecm.ComponentData<components::JointPosition>(this->dataPtr->joint)
          ->front();
  double vel =
      _ecm.ComponentData<components::JointVelocity>(this->dataPtr->joint)
          ->front();
  if (std::fabs(this->dataPtr->target - pos) > this->dataPtr->posEps ||
      vel > this->dataPtr->velEps)
    return;
  this->dataPtr->isActive = false;
  this->dataPtr->targetReachedCallback();
}

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
