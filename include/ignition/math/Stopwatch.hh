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

#ifndef IGNITION_MATH_STOPWATCH_HH_
#define IGNITION_MATH_STOPWATCH_HH_

#include <chrono>
#include <memory>

namespace ignition
{
  namespace math
  {
    // Use a steady clock
    using clock = std::chrono::steady_clock;

    // Forward declarations.
    class StopwatchPrivate;

    /// \brief A system that manages simulation time.
    class Stopwatch
    {
      /// \brief Constructor.
      public: Stopwatch();

      /// \brief Destructor.
      public: virtual ~Stopwatch();

      /// \brief Start the stopwatch.
      /// \param[in] _reset If true the stopwatch is reset first.
      /// \return True if the the stopwatch was started. This will return
      /// false if the stopwatch was already running.
      public: bool Start(bool _reset = false);

      /// \brief Get the time when the stopwatch was started.
      /// \return The time when stopwatch was started, or
      /// std::chrono::steady_clock::time_point::min() if the stopwatch
      /// has not been started.
      public: clock::time_point StartTime();

      /// \brief Stop the stopwatch
      /// \return True if the stopwatch was stopped. This will return false
      /// if the stopwatch is not running.
      public: bool Stop();

      /// \brief Get the time when the stopwatch was last stopped.
      /// \return The time when stopwatch was last stopped, or
      /// std::chrono::steady_clock::time_point::min() if the stopwatch
      /// has never been stopped.
      public: clock::time_point StopTime();

      /// \brief Get whether the stopwatch is running.
      /// \return True if the stopwatch is running.
      public: bool Running();

      /// \brief Reset the stopwatch. This resets the start time, stop time,
      /// elapsed duration and elapsed stop duration.
      public: void Reset();

      /// \brief Get the amount of time that the stop watch has been
      /// running. This is the total amount of run time, spannning all start
      /// and stop calls. The Reset function or passing true to the Start
      /// function will reset this value.
      /// \return Total amount of elapsed run time.
      public: clock::duration ElapsedRunTime() const;

      /// \brief Get the amount of time that the stop watch has been
      /// stopped. This is the total amount of stop time, spannning all start
      /// and stop calls. The Reset function or passing true to the Start
      /// function will reset this value.
      /// \return Total amount of elapsed stop time.
      public: clock::duration ElapsedStopTime() const;

      /// \brief Private data pointer.
      private: std::unique_ptr<StopwatchPrivate> dataPtr;
    };
  }
}
#endif
