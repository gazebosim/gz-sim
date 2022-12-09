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
#ifndef GZ_SIM_NETWORK_NETWORKMANAGERPRIVATE_HH_
#define GZ_SIM_NETWORK_NETWORKMANAGERPRIVATE_HH_

#include <functional>
#include <memory>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

#include "NetworkConfig.hh"
#include "PeerInfo.hh"
#include "PeerTracker.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    /// \class NetworkManagerPrivate NetworkManagerPrivate.hh
    /// gz/sim/NetworkManagerPrivate.hh
    class GZ_SIM_VISIBLE NetworkManagerPrivate
    {
      /// \brief Network Configuration
      public: NetworkConfig config;

      /// \brief Information about this peer
      public: PeerInfo peerInfo;

      /// \brief EventManager to emit/connect to
      public: EventManager *eventMgr{nullptr};

      /// \brief Object to manage information about discovered peers.
      public: std::unique_ptr<PeerTracker> tracker;

      /// \brief Track connection to "PeerRemoved" Event
      public: gz::common::ConnectionPtr peerRemovedConn;

      /// \brief Traack connection to "PeerStale" Event
      public: gz::common::ConnectionPtr peerStaleConn;

      /// \brief Function from the SimulationRunner to call for stepping.
      /// It will update the systems.
      public: std::function<void(const UpdateInfo &_info)> stepFunction;

      /// \brief Pointer to ECM.
      public: EntityComponentManager *ecm{nullptr};

      /// \brief Flag to indicate if simulation server is stopping.
      public: std::atomic<bool> stopReceived {false};

      /// \brief Track connection to "events::Stop" Event
      public: gz::common::ConnectionPtr stoppingConn;
    };
    }
  }  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_NETWORKMANAGER_HH_
