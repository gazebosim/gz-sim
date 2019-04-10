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
#ifndef IGNITION_GAZEBO_NETWORK_NETWORKMANAGERPRIVATE_HH_
#define IGNITION_GAZEBO_NETWORK_NETWORKMANAGERPRIVATE_HH_

#include <functional>
#include <memory>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include "NetworkConfig.hh"
#include "PeerInfo.hh"
#include "PeerTracker.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \class NetworkManagerPrivate NetworkManagerPrivate.hh
    /// ignition/gazebo/NetworkManagerPrivate.hh
    class IGNITION_GAZEBO_VISIBLE NetworkManagerPrivate
    {
      /// \brief Network Configuration
      public: NetworkConfig config;

      /// \brief Information about this peer
      public: PeerInfo peerInfo;

      /// \brief EventManager to emit/connect to
      public: EventManager *eventMgr;

      /// \brief Object to manage information about discovered peers.
      public: std::unique_ptr<PeerTracker> tracker;

      /// \brief Track connection to "PeerRemoved" Event
      public: ignition::common::ConnectionPtr peerRemovedConn;

      /// \brief Traack connection to "PeerStale" Event
      public: ignition::common::ConnectionPtr peerStaleConn;

      public: std::function<void(const UpdateInfo &_info)> stepFunction;

      public: EntityComponentManager *ecm;
    };
    }
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_NETWORKMANAGER_HH_

