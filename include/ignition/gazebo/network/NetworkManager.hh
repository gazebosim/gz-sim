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
#ifndef IGNITION_GAZEBO_NETWORK_NETWORKMANAGER_HH_
#define IGNITION_GAZEBO_NETWORK_NETWORKMANAGER_HH_

#include <memory>
#include <string>

#include <ignition/common/Uuid.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include <ignition/gazebo/network/NetworkConfig.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations
    class NetworkManagerPrivate;

    /// \class NetworkManager NetworkManager.hh
    ///   ignition/gazebo/NetworkManager.hh
    /// \brief The server instantiates and controls simulation.
    class IGNITION_GAZEBO_VISIBLE NetworkManager
    {
      /// \brief Constructor (reads config from environment)
      public: explicit NetworkManager();

      /// \brief Constructor with configuration passed in.
      public: explicit NetworkManager(const NetworkConfig &_config);

      /// \brief Destructor.
      public: ~NetworkManager();

      /// \brief Indicate if NetworkManager configuraiton is in a valid state.
      public: bool Valid() const;

      /// \brief Indicate if NetworkManager is ready to execute.
      public: bool Ready() const;

      /// \brief Get a unique namespace for this runner
      public: std::string Namespace() const;

      /// \brief Convenience method for retrieving role.
      public: bool IsPrimary() const;

      /// \brief Convenience method for retrieving role.
      public: bool IsSecondary() const;

      /// \brief Private data
      private: std::unique_ptr<NetworkManagerPrivate> dataPtr;
    };
    }
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_NETWORKMANAGER_HH_
