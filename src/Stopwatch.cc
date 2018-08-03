/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <chrono>
#include "ignition/math/Stopwatch.hh"

using namespace ignition::math;

// Private data class
class ignition::math::StopwatchPrivate
{
  /// \brief Default constructor.
  public: StopwatchPrivate() = default;

  /// \brief Copy constructor.
  /// \param[in] _watch Watch to copy.
  public: explicit StopwatchPrivate(const StopwatchPrivate &_watch)
          : running(_watch.running),
            startTime(_watch.startTime),
            stopTime(_watch.stopTime),
            stopDuration(_watch.stopDuration),
            runDuration(_watch.runDuration)
  {
  }

  /// \brief True if the real time clock is running.
  public: bool running = false;

  /// \brief Time point that marks the start of the real-time clock.
  public: clock::time_point startTime = clock::time_point::min();

  /// \brief Time point that marks the stop of the real-time clock.
  public: clock::time_point stopTime = clock::time_point::min();

  /// \brief Amount of stop time.
  public: clock::duration stopDuration = clock::duration::zero();

  /// \brief Amount of run time.
  public: clock::duration runDuration = clock::duration::zero();
};

//////////////////////////////////////////////////
Stopwatch::Stopwatch()
  : dataPtr(new StopwatchPrivate)
{
}

//////////////////////////////////////////////////
Stopwatch::Stopwatch(const Stopwatch &_watch)
  : dataPtr(new StopwatchPrivate(*_watch.dataPtr))
{
}

//////////////////////////////////////////////////
Stopwatch::Stopwatch(Stopwatch &&_watch) noexcept
  : dataPtr(std::move(_watch.dataPtr))
{
}

//////////////////////////////////////////////////
Stopwatch::~Stopwatch()
{
}

//////////////////////////////////////////////////
bool Stopwatch::Start(const bool _reset)
{
  if (_reset)
    this->Reset();

  if (!this->dataPtr->running)
  {
    if (this->dataPtr->startTime != this->dataPtr->stopTime)
    {
      this->dataPtr->stopDuration +=
        clock::now() - this->dataPtr->stopTime;
    }

    this->dataPtr->running = true;
    this->dataPtr->startTime = clock::now();
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
clock::time_point Stopwatch::StartTime() const
{
  return this->dataPtr->startTime;
}

//////////////////////////////////////////////////
bool Stopwatch::Stop()
{
  if (this->dataPtr->running)
  {
    this->dataPtr->running = false;
    this->dataPtr->stopTime = clock::now();
    this->dataPtr->runDuration +=
      this->dataPtr->stopTime - this->dataPtr->startTime;
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
clock::time_point Stopwatch::StopTime() const
{
  return this->dataPtr->stopTime;
}

//////////////////////////////////////////////////
bool Stopwatch::Running() const
{
  return this->dataPtr->running;
}

//////////////////////////////////////////////////
void Stopwatch::Reset()
{
  this->dataPtr->running = false;
  this->dataPtr->startTime = clock::time_point::min();
  this->dataPtr->stopTime = clock::time_point::min();
  this->dataPtr->stopDuration = clock::duration::zero();
  this->dataPtr->runDuration = clock::duration::zero();
}

//////////////////////////////////////////////////
clock::duration Stopwatch::ElapsedRunTime() const
{
  if (this->dataPtr->running)
  {
    return clock::now() - this->dataPtr->startTime + this->dataPtr->runDuration;
  }
  else
  {
    return this->dataPtr->runDuration;
  }
}

//////////////////////////////////////////////////
clock::duration Stopwatch::ElapsedStopTime() const
{
  // If running, then return the stopDuration.
  if (this->dataPtr->running)
  {
    return this->dataPtr->stopDuration;
  }
  // The clock is not running, and Stop() has been called.
  else if (this->dataPtr->stopTime > clock::time_point::min())
  {
    return this->dataPtr->stopDuration +
      (clock::now() - this->dataPtr->stopTime);
  }

  // Otherwise, the stopwatch has been reset or never started.
  return clock::duration::zero();
}

//////////////////////////////////////////////////
bool Stopwatch::operator==(const Stopwatch &_watch) const
{
  return this->dataPtr->running == _watch.dataPtr->running &&
    this->dataPtr->startTime == _watch.dataPtr->startTime &&
    this->dataPtr->stopTime == _watch.dataPtr->stopTime &&
    this->dataPtr->stopDuration == _watch.dataPtr->stopDuration &&
    this->dataPtr->runDuration == _watch.dataPtr->runDuration;
}

//////////////////////////////////////////////////
bool Stopwatch::operator!=(const Stopwatch &_watch) const
{
  return !(*this == _watch);
}

//////////////////////////////////////////////////
Stopwatch &Stopwatch::operator=(const Stopwatch &_watch)
{
  this->dataPtr.reset(new StopwatchPrivate(*_watch.dataPtr));
  return *this;
}

//////////////////////////////////////////////////
Stopwatch &Stopwatch::operator=(Stopwatch &&_watch)
{
  this->dataPtr = std::move(_watch.dataPtr);
  return *this;
}
