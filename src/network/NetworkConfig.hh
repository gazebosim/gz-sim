/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_NETWORK_NETWORKCONFIG_HH_
#define IGNITION_GAZEBO_NETWORK_NETWORKCONFIG_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include "NetworkRole.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \class NetworkConfig NetworkConfig.hh ignition/gazebo/NetworkConfig.hh
    /// \brief Configuration parameters for a distributed simulation instance
    ///
    /// NetworkConfig can either be created programatically, or populated from
    /// environment variables set before the execution of the Gazebo server.
    ///
    /// Environment Variables:
    /// * IGN_GAZEBO_NETWORK_ROLE - Sets the role that the current Networked
    ///   runner will use in the distributed simulation environment.
    ///   * PRIMARY - Primary executor, dispatches work to other runners.
    ///   * SECONDARY - Secondary executor, receives work from primary.
    ///   * READONLY - Can be used to view state of entities and components.
    /// * IGN_GAZEBO_NETWORK_SECONDARIES - (PRIMARY only) - Expected number of
    ///   secondaries expected to join the distributed simulation environment.
    ///   Simulation will not run without the expected number of secondaries.
    class IGNITION_GAZEBO_VISIBLE NetworkConfig
    {
      /// \brief Populated a new NetworkConfig object based on
      /// environment variables.
      public: static NetworkConfig FromEnv();

      /// \brief Role of this network participant
      public: NetworkRole role { NetworkRole::None };

      /// \brief Expect number of network secondaries.
      public: size_t numSecondariesExpected { 0 };
    };
    }
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_NETWORKCONFIG_HH_
