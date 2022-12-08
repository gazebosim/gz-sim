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

#include <gz/msgs/double.pb.h>

#include <deque>
#include <memory>
#include <sstream>
#include <utility>

#include <gz/common/Console.hh>

#include "../ElevatorStateMachine.hh"

#include "../ElevatorCommonPrivate.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
class ElevatorStateMachinePrivate
{
  /// \brief Constructor
  /// \param[in] _system Data of the enclosing system
  public: ElevatorStateMachinePrivate(
      const std::shared_ptr<ElevatorCommonPrivate> &_system);

  /// \brief Adds a new target to the queue
  /// \details If there is another target in the queue, and the new target is
  /// an intermediate stop, the new target will take precedence
  /// \param[in] _target Target to add to the queue
  public: void EnqueueNewTarget(double _target);

  /// \brief Publishes a command message
  /// \param[in] _pub Joint command publisher
  /// \param[in] _cmd Joint command
  public: void SendCmd(transport::Node::Publisher &_pub, double _cmd);

  /// \brief Data of the enclosing system
  public: std::shared_ptr<ElevatorCommonPrivate> system;

  /// \brief Floor target queue
  public: std::deque<int32_t> targets;
};

//////////////////////////////////////////////////
ElevatorStateMachinePrivate::ElevatorStateMachinePrivate(
    const std::shared_ptr<ElevatorCommonPrivate> &_system)
    : system(_system)
{
}

//////////////////////////////////////////////////
void ElevatorStateMachinePrivate::EnqueueNewTarget(double _target)
{
  // Ignore duplicate targets
  auto it = std::find(this->targets.cbegin(), this->targets.cend(), _target);
  if (it != this->targets.cend())
    return;

  // Prioritize target in the queue
  bool enqueued = false;
  int32_t prevTarget = this->system->state;
  for (it = this->targets.cbegin(); it != this->targets.cend(); ++it)
  {
    int32_t lower = prevTarget;
    int32_t upper = *it;
    if (upper < lower) std::swap(lower, upper);
    if (_target >= lower && _target < upper)
    {
      this->targets.insert(it, _target);
      enqueued = true;
      break;
    }
    prevTarget = *it;
  }
  if (!enqueued)
    this->targets.push_back(_target);

  std::ostringstream ss;
  ss << "The elevator enqueued target " << _target << " [ ";
  for (const auto &target : this->targets) ss << target << " ";
  gzmsg << ss.str() << "]" << std::endl;
}

//////////////////////////////////////////////////
void ElevatorStateMachinePrivate::SendCmd(transport::Node::Publisher &_pub,
                                          double _cmd)
{
  msgs::Double msg;
  msg.set_data(_cmd);
  _pub.Publish(msg);
}

//////////////////////////////////////////////////
ElevatorStateMachineDef::ElevatorStateMachineDef(
    const std::shared_ptr<ElevatorCommonPrivate> &_system)
    : dataPtr(std::make_unique<ElevatorStateMachinePrivate>(_system))
{
}

//////////////////////////////////////////////////
ElevatorStateMachineDef::~ElevatorStateMachineDef() = default;

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#include "EventsImpl.hh"

#include "ActionsImpl.hh"
#include "GuardsImpl.hh"
#include "StatesImpl.hh"
