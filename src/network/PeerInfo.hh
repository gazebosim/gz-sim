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
#ifndef IGNITION_GAZEBO_NETWORK_PEERINFO_HH_
#define IGNITION_GAZEBO_NETWORK_PEERINFO_HH_

#include <string>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include "NetworkRole.hh"
#include "msgs/peer_info.pb.h"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    class IGNITION_GAZEBO_VISIBLE PeerInfo {
      /// \brief Constructor
      public: explicit PeerInfo(const NetworkRole &_role = NetworkRole::None);

      /// \brief Get the namespace for this peer.
      /// \return The namespace.
      public: std::string Namespace() const;

      /// \brief Unique peer ID in the network
      public: std::string id;

      /// \brief Peer hostname
      public: std::string hostname;

      /// \brief Peer's role in the network
      public: NetworkRole role;
    };
    }

    /// \brief Construct a `PeerInfo` object from a message.
    /// \param[in] _proto Message
    /// \result Equivalent PeerInfo
    IGNITION_GAZEBO_VISIBLE PeerInfo fromProto(
        const private_msgs::PeerInfo &_proto);

    /// \brief Construct a `PeerInfo` message from an object.
    /// \param[in] _info Peer info object
    /// \result Equivalent message
    IGNITION_GAZEBO_VISIBLE private_msgs::PeerInfo toProto(
        const PeerInfo &_info);
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_NETWORK_PEERINFO_HH_

