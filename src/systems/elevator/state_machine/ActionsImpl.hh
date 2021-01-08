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

#include "../ElevatorStateMachine.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
namespace systems
{
/// \brief Action that enqueues a new target in the target queue.
/// \details After the new target has been added in the queue, depending on the
/// template parameter, a NewTargetEvent is triggered.
template <bool trigger>
class EnqueueNewTargetAction
{
  /// \brief Function call operator
  /// \param[in] _event Event that triggered the action
  /// \param[in] _fsm State machine with which the action is associated
  public: template <typename Event, typename Fsm, typename Source,
                    typename Target>
  void operator()(const Event &_event, Fsm &_fsm, Source &, Target &)
  {
    const auto &data = _fsm.Data();
    data->EnqueueNewTarget(_event.target);
    if (trigger) _fsm.process_event(NewTargetEvent());
  }
};

/// \brief Action that cleans up the target queue when a new target is
/// processed.
class NewTargetAction
{
  /// \brief Function call operator
  /// \param[in] _fsm State machine with which the action is associated
  public: template <typename Event, typename Fsm, typename Source,
                    typename Target>
  void operator()(const Event &, Fsm &_fsm, Source &, Target &)
  {
    const auto &data = _fsm.Data();
    std::lock_guard<std::mutex> lock(data->system->mutex);
    if (data->targets.front() == data->system->state) data->targets.pop_front();
  }
};

/// \brief Action that cleans up the target queue when the cabin reaches the
/// target floor level.
class CabinAtTargetAction
{
  /// \brief Function call operator
  /// \param[in] _fsm State machine with which the action is associated
  public: template <typename Event, typename Fsm, typename Source,
                    typename Target>
  void operator()(const Event &, Fsm &_fsm, Source &, Target &)
  {
    const auto &data = _fsm.Data();
    std::lock_guard<std::mutex> lock(data->system->mutex);
    data->targets.pop_front();
  }
};

}  // namespace systems
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition
