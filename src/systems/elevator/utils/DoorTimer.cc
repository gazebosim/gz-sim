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

#include "DoorTimer.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
class DoorTimerPrivate
{
  /// \brief Constructor
  /// \param[in] _waitDuration Duration
  public: DoorTimerPrivate(
       const std::chrono::steady_clock::duration &_waitDuration);

  /// \brief Wait duration
  public: std::chrono::steady_clock::duration waitDuration;

  /// \brief Timeout time
  public: std::chrono::steady_clock::duration timeoutTime;

  /// \brief Function that gets called upon timeout
  public: std::function<void()> timeoutCallback;

  /// \brief Flag to indicate whether the doorway was blocked on the last update
  /// time
  public: bool wasDoorwayBlocked{false};

  /// \brief Flag to indicate whether the timer is active
  public: bool isActive{false};
};

//////////////////////////////////////////////////
DoorTimerPrivate::DoorTimerPrivate(
    const std::chrono::steady_clock::duration &_waitDuration)
    : waitDuration(_waitDuration)
{
}

//////////////////////////////////////////////////
DoorTimer::DoorTimer(const std::chrono::steady_clock::duration &_waitDuration)
    : dataPtr(std::make_unique<DoorTimerPrivate>(_waitDuration))
{
}

//////////////////////////////////////////////////
DoorTimer::~DoorTimer() = default;

//////////////////////////////////////////////////
void DoorTimer::Configure(const std::chrono::steady_clock::duration &_startTime,
                          const std::function<void()> &_timeoutCallback)
{
  this->dataPtr->isActive = true;
  this->dataPtr->wasDoorwayBlocked = false;
  this->dataPtr->timeoutTime = _startTime + this->dataPtr->waitDuration;
  this->dataPtr->timeoutCallback = _timeoutCallback;
}

//////////////////////////////////////////////////
void DoorTimer::Update(const UpdateInfo &_info, bool _isDoorwayBlocked)
{
  // Reset timeout time when doorway gets unblocked
  if (!_isDoorwayBlocked && this->dataPtr->wasDoorwayBlocked)
    this->dataPtr->timeoutTime = _info.simTime + this->dataPtr->waitDuration;
  this->dataPtr->wasDoorwayBlocked = _isDoorwayBlocked;

  if (!this->dataPtr->isActive || _isDoorwayBlocked ||
      (_info.dt > std::chrono::steady_clock::duration::zero() &&
       _info.simTime < this->dataPtr->timeoutTime))
    return;
  this->dataPtr->isActive = false;
  this->dataPtr->timeoutCallback();
}

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
