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

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
namespace actions
{
/// \brief Action that enqueues a new target in the target queue.
/// \details After the new target has been added in the queue, depending on the
/// template parameter, an events::NewTarget is triggered.
template <bool trigger>
struct EnqueueNewTarget
{
  /// \brief Function call operator
  /// \param[in] _event Event that triggered the action
  /// \param[in] _fsm State machine with which the action is associated
  /// \param[in] _source Source state
  /// \param[in] _target Target state
  public: template <typename Event, typename Fsm, typename Source,
                    typename Target>
   void operator()(const Event &_event, Fsm &_fsm, Source & /*_source*/,
                   Target & /*_target*/)
  {
    _fsm.dataPtr->EnqueueNewTarget(_event.target);
    if (trigger)
      _fsm.process_event(events::NewTarget());
  }
};

/// \brief Action that cleans up the target queue when a new target is
/// processed.
struct NewTarget
{
  /// \brief Function call operator
  /// \param[in] _event Event that triggered the action
  /// \param[in] _fsm State machine with which the action is associated
  /// \param[in] _source Source state
  /// \param[in] _target Target state
  public: template <typename Event, typename Fsm, typename Source,
                    typename Target>
  void operator()(const Event & /*_event*/, Fsm &_fsm, Source & /*_source*/,
                  Target & /*_target*/)
  {
    std::lock_guard<std::recursive_mutex> lock(_fsm.dataPtr->system->mutex);
    if (_fsm.dataPtr->targets.front() == _fsm.dataPtr->system->state)
      _fsm.dataPtr->targets.pop_front();
  }
};

/// \brief Action that cleans up the target queue when the cabin reaches the
/// target floor level.
struct CabinAtTarget
{
  /// \brief Function call operator
  /// \param[in] _event Event that triggered the action
  /// \param[in] _fsm State machine with which the action is associated
  /// \param[in] _source Source state
  /// \param[in] _target Target state
  public: template <typename Event, typename Fsm, typename Source,
                    typename Target>
  void operator()(const Event & /*_event*/, Fsm &_fsm, Source & /*_source*/,
                  Target & /*_target*/)
  {
    std::lock_guard<std::recursive_mutex> lock(_fsm.dataPtr->system->mutex);
    _fsm.dataPtr->targets.pop_front();
  }
};

}  // namespace actions
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
