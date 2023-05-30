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
#ifndef GZ_SIM_NETWORK_PEERTRACKER_HH_
#define GZ_SIM_NETWORK_PEERTRACKER_HH_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>
#include <gz/sim/EventManager.hh>

#include <gz/common/Event.hh>
#include <gz/transport/Node.hh>

#include "PeerInfo.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    // Network Events
    /// \brief PeerAdded fired when a peer announces itself or detected via
    /// heartbeat
    using PeerAdded = common::EventT<void(PeerInfo), struct PeerAddedTag>;

    /// \brief PeerRemoved fired when a peer announces a disconnect.
    using PeerRemoved = common::EventT<void(PeerInfo), struct PeerRemovedTag>;

    /// \brief PeerStale fired when a peer is detected as stale.
    using PeerStale = common::EventT<void(PeerInfo), struct PeerStaleTag>;

    /// \brief The PeerTracker is used to track the state of multiple peers in
    /// a distributed simulation environment.
    ///
    /// It is used to both announce the existence of a peer, as well as track
    /// announcements and heartbeats from other peers.
    class GZ_SIM_VISIBLE PeerTracker {
      /// \brief Convenience type alias for NodeOptions
      public: using NodeOptions = gz::transport::NodeOptions;

      /// \brief Convenience type alias for duration
      public: using Duration = std::chrono::steady_clock::duration;

      /// \brief Constructor
      ///
      /// Announce the existence of a peer with given information _info,
      /// and start executing heartbeats and peer tracking.
      ///
      /// \param[in] _eventMgr Event Manager to emit network events on.
      /// \param[in] _options Advanced options for underlying gz-transport
      /// \param[in] _info Peer information to announce
      public: explicit PeerTracker(
                  PeerInfo _info,
                  EventManager *_eventMgr = nullptr,
                  const NodeOptions &_options = NodeOptions());

      /// \brief Destructor
      public: ~PeerTracker();

      /// \brief Set heartbeat period for this peer.
      /// \param[in] _period Period at which heartbeat occurs.
      public: void SetHeartbeatPeriod(const Duration &_period);

      /// \brief Get heartbeat period.
      /// \return Heartbeat period.
      public: Duration HeartbeatPeriod() const;

      /// \brief Set number of heartbeats of this peer before a peer is marked
      /// stale.
      ///
      /// Note that the total time to consider a peer stale is based on this
      /// peer's heartbeat period. The maximum stale time is:
      ///
      /// max = heartbeatPeriod * staleMultiplier
      ///
      /// \param[in] _multipler Multiplier of heartbeat period.
      public: void SetStaleMultiplier(const size_t &_multiplier);

      /// \brief Get current heartbeat multiplier
      /// \return Number of hearbeats before a peer is marked stale.
      public: size_t StaleMultiplier() const;

      /// \brief Retrieve total number of detected peers in the network.
      public: size_t NumPeers() const;

      /// \brief Retrieve number of detected peers in the network by role.
      /// \param[in] _role Role of peers to enumerate
      /// \return Number of peers with the given role.
      public: size_t NumPeers(const NetworkRole &_role) const;

      /// \brief Retrieve number of detected primaries
      public: inline size_t NumPrimary() const
              {
                return NumPeers(NetworkRole::SimulationPrimary);
              }

      /// \brief Retrieve number of detected primaries
      public: inline size_t NumSecondary() const
              {
                return NumPeers(NetworkRole::SimulationSecondary);
              }

      /// \brief Retrieve number of detected primaries
      public: inline size_t NumReadOnly() const
              {
                return NumPeers(NetworkRole::ReadOnly);
              }

      /// \brief Retrieve the ids of discovered peers.
      public: std::vector<std::string> SecondaryPeers() const
              {
                std::vector<std::string> ret;
                for (const auto& it : this->peers)
                {
                  ret.push_back(it.first);
                }
                return ret;
              }

      /// \brief Internal loop to announce and check stale peers.
      private: void HeartbeatLoop();

      /// \brief Helper function for removing a peer
      /// \param[in] _info Peer to remove
      /// \return True if successfully removed.
      private: bool RemovePeer(const PeerInfo &_info);

      /// \brief Callback for the announcement of a peer
      /// \param[in] _info Announcement from another peer.
      private: void OnPeerAnnounce(const private_msgs::PeerAnnounce &_info);

      /// \brief Callback for peer heartbeat
      /// \param[in] _info Heartbeat from another peer.
      private: void OnPeerHeartbeat(const private_msgs::PeerInfo &_info);

      /// \brief Callback for when a peer is added.
      /// \param[in] _info Info from peer which was added.
      private: void OnPeerAdded(const PeerInfo &_info);

      /// \brief Callback for when a peer is removed.
      /// \param[in] _info Info from peer which was removed.
      private: void OnPeerRemoved(const PeerInfo &_info);

      /// \brief Callback for when a peer goes stale.
      /// \param[in] _info Info from peer which is stale.
      private: void OnPeerStale(const PeerInfo &_info);

      /// \brief Information about discovered peers
      struct PeerState
      {
        /// \brief Peer info
        PeerInfo info;

         /// \brief Keep last header time
        std::chrono::steady_clock::time_point lastHeader;

        /// \brief Keep last time heartbeat was received
        std::chrono::steady_clock::time_point lastSeen;
      };

      /// \brief Convenience type alias
      private: using PeerMutex = std::recursive_mutex;

      /// \brief Convenience type alias
      private: using PeerLock = std::lock_guard<PeerMutex>;

      /// \brief Used for guarding map of peers.
      /// N.b. marked mutable to allow const in places that are read only.
      private: mutable PeerMutex peersMutex;

      /// \brief Information about discovered peers
      private: std::map<std::string, PeerState> peers;

      /// \brief Thread for executing heartbeat loop
      private: std::thread heartbeatThread;

      /// \brief Flag for execution of heartbeat loop
      private: std::atomic<bool> heartbeatRunning;

      /// \brief Period to publish heartbeat at
      private: Duration heartbeatPeriod {std::chrono::milliseconds(100)};

      /// \brief Timeout to mark a peer as stale.
      private: size_t staleMultiplier {100};

      /// \brief Peer information that this tracker announces.
      private: PeerInfo info;

      /// \brief Event manager instance to be used to emit network events.
      private: EventManager *eventMgr;

      /// \brief Transport node
      private: gz::transport::Node node;

      /// \brief Heartbeat publisher
      private: gz::transport::Node::Publisher heartbeatPub;

      /// \brief Announcement publisher
      private: gz::transport::Node::Publisher announcePub;
    };
    }
  }  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_NETWORKCONFIG_HH_
