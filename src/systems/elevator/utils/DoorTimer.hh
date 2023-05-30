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

#ifndef GZ_SIM_SYSTEMS_DOOR_TIMER_HH_
#define GZ_SIM_SYSTEMS_DOOR_TIMER_HH_

#include <chrono>
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
class DoorTimerPrivate;

/// \brief Timer that's used to keep a door open. It has a configurable default
/// wait duration that when exceeded, it calls a function to let the state
/// machine know to transition to the next state. The timer also checks whether
/// the doorway is blocked, in which case it keeps the door open until whatever
/// blocks the doorway moves out of the way.
class DoorTimer
{
  /// \brief Constructor
  /// \param[in] _waitDuration Duration
  public: DoorTimer(const std::chrono::steady_clock::duration &_waitDuration);

  /// \brief Destructor
  public: ~DoorTimer();

  /// \brief Starts the timer and sets the timeout time based on the given
  /// start time
  /// \param[in] _startTime Start time
  /// \param[in] _timeoutCallback Function to call upon timeout
  public: void Configure(const std::chrono::steady_clock::duration &_startTime,
                         const std::function<void()> &_timeoutCallback);

  /// \brief Checks whether the timer has timed out
  /// \param[in] _info Current simulation step info
  /// \param[in] _isDoorwayBlocked Flag that indicates whether the doorway is
  /// blocked
  public: void Update(const UpdateInfo &_info, bool _isDoorwayBlocked);

  /// \brief Private data pointer
  private: std::unique_ptr<DoorTimerPrivate> dataPtr;
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_SYSTEMS_DOOR_TIMER_HH_
