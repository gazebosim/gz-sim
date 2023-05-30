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

#include <mutex>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
namespace guards
{
/// \brief Guard that checks whether the state machine is in a given state.
/// \note The template parameter generalizes the target state.
template <typename TargetState>
struct IsInState
{
  /// \brief Function call operator
  /// \param[in] _fsm State machine with which the guard is associated
  public: template <typename Fsm, typename State>
  bool operator()(const Fsm &_fsm, const State &)
  {
    // NOTE Mind the inversion; the internal transition table needs
    // the guard to return false in order to trigger a transition
    return !_fsm.template is_in_state<TargetState()>();
  }
};

/// \brief Guard that checks whether the cabin is at the target floor level.
struct CabinAtTarget
{
  /// \brief Function call operator
  /// \param[in] _fsm State machine with which the guard is associated
  public: template <typename Fsm, typename State>
  bool operator()(const Fsm &_fsm, const State &)
  {
    std::lock_guard<std::recursive_mutex> lock(_fsm.dataPtr->system->mutex);
    return _fsm.dataPtr->targets.front() == _fsm.dataPtr->system->state;
  }
};

/// \brief Guard that checks whether the target queue is empty.
struct NoQueuedTarget
{
  /// \brief Function call operator
  /// \param[in] _fsm State machine with which the guard is associated
  public: template <typename Fsm, typename State>
  bool operator()(const Fsm &_fsm, const State &)
  {
    std::lock_guard<std::recursive_mutex> lock(_fsm.dataPtr->system->mutex);
    return _fsm.dataPtr->targets.empty();
  }
};

}  // namespace guards
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
