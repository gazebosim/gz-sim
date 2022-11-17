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
#ifndef GZ_SIM_NETWORK_NETWORKROLE_HH_
#define GZ_SIM_NETWORK_NETWORKROLE_HH_

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    /// \brief Enumeration of roles that a network participant can take.
    enum class NetworkRole
    {
      /// \brief participant has no role in the network
      None = 0,
      /// \brief participant is read-only, could potentially be used
      /// for visualization or logging, but is not executing simulation.
      ///
      /// This is reserved for future use.
      ReadOnly = 1,
      /// \brief participant is simulation primary, it is responsible for
      /// configuring simulation and assigning work to secondaries.
      ///
      /// There will be only one simulation primary in a given distributed
      /// simulation
      SimulationPrimary = 2,
      /// \brief participant is a simulation secondary, it receives work
      /// from the simulation primary and executes.
      SimulationSecondary = 3,
    };
    }
  }  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_NETWORKROLE_HH_
