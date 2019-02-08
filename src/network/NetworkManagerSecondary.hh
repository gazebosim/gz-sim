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
#ifndef IGNITION_GAZEBO_NETWORK_NETWORKMANAGERSECONDARY_HH_
#define IGNITION_GAZEBO_NETWORK_NETWORKMANAGERSECONDARY_HH_

#include <string>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include <ignition/gazebo/network/NetworkManager.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \class NetworkManagerSecondary NetworkManagerSecondary.hh
    ///   ignition/gazebo/network/NetworkManagerSecondary.hh
    /// \brief Secondaryspecific behaviors
    class IGNITION_GAZEBO_VISIBLE NetworkManagerSecondary:
      public NetworkManager
    {
      // Documentation inherited
      public: explicit NetworkManagerSecondary(const NetworkConfig &_config);

      // Documentation inherited
      public: bool Valid() const override;

      // Documentation inherited
      public: bool Ready() const override;

      // Documentation inherited
      public: std::string Namespace() const override;

      // Documentation inherited
      public: bool IsPrimary() const override { return false; };

      // Documentation inherited
      public: bool IsSecondary() const override { return true; };

      // Documentation inherited
      public: bool IsReadOnly() const override { return false; };
    };
    }
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_NETWORKMANAGERSECONDARY_HH_

