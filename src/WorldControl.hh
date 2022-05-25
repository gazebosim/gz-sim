/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_SIM_WORLDCONTROL_HH_
#define GZ_SIM_WORLDCONTROL_HH_

#include <chrono>
#include <cstdint>

#include "gz/sim/config.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {

    /// \brief Helper struct to control world time. It's used to hold
    /// input from either msgs::WorldControl or msgs::LogPlaybackControl.
    struct WorldControl
    {
      /// \brief True to pause simulation.
      // cppcheck-suppress unusedStructMember
      bool pause{false};  // NOLINT

      /// \biref Run a given number of simulation iterations.
      // cppcheck-suppress unusedStructMember
      uint64_t multiStep{0u};  // NOLINT

      /// \brief Reset simulation back to time zero. Rewinding resets sim time,
      /// real time and iterations.
      // cppcheck-suppress unusedStructMember
      bool rewind{false};  // NOLINT

      /// \brief A simulation time in the future to run to and then pause.
      /// A negative number indicates that this variable it not being used.
      std::chrono::steady_clock::duration runToSimTime{-1};  // NOLINT

      /// \brief Sim time to jump to. A negative value means don't seek.
      /// Seeking changes sim time but doesn't affect real time.
      /// It also resets iterations back to zero.
      std::chrono::steady_clock::duration seek{-1};
    };
    }
  }  // namespace sim
}  // namespace gz
#endif  // GZ_SIM_WORLDCONTROL_HH_
