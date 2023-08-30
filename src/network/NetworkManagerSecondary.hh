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
#ifndef GZ_SIM_NETWORK_NETWORKMANAGERSECONDARY_HH_
#define GZ_SIM_NETWORK_NETWORKMANAGERSECONDARY_HH_

#include <atomic>
#include <memory>
#include <string>
#include <unordered_set>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/private_msgs/simulation_step.pb.h"
#include "gz/sim/private_msgs/peer_control.pb.h"

#include "NetworkManager.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    /// \class NetworkManagerSecondary NetworkManagerSecondary.hh
    ///   gz/sim/network/NetworkManagerSecondary.hh
    /// \brief Secondary specific behaviors
    class GZ_SIM_VISIBLE NetworkManagerSecondary:
      public NetworkManager
    {
      // Documentation inherited
      public: explicit NetworkManagerSecondary(
          const std::function<void(const UpdateInfo &_info)> &_stepFunction,
          EntityComponentManager &_ecm, EventManager *_eventMgr,
          const NetworkConfig &_config,
          const NodeOptions &_options);

      // Documentation inherited
      public: bool Ready() const override;

      // Documentation inherited
      public: void Handshake() override;

      // Documentation inherited
      public: std::string Namespace() const override;

      /// \brief Callback for when PeerControl service request is received.
      /// \param[in] _req Request
      /// \param[in] _resp Response
      /// \return True if successful.
      public: bool OnControl(const private_msgs::PeerControl &_req,
                             private_msgs::PeerControl &_resp);

      /// \brief Callback when step commands are received from the primary
      /// \param[in] _msg Step message.
      private: void OnStep(const private_msgs::SimulationStep &_msg);

      /// \brief Flag to control enabling/disabling simulation secondary.
      private: std::atomic<bool> enableSim {false};

      /// \brief Transport node used for communication with simulation graph.
      private: gz::transport::Node node;

      /// \brief Publish step acknowledgement messages.
      private: gz::transport::Node::Publisher stepAckPub;

      /// \brief Collection of performers associated with this secondary.
      private: std::unordered_set<Entity> performers;
    };
    }
  }  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_NETWORKMANAGERSECONDARY_HH_
