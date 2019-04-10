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

#include <atomic>
#include <memory>
#include <string>


#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/transport/Node.hh>

#include "NetworkManager.hh"
#include "msgs/simulation_step.pb.h"
#include "msgs/peer_control.pb.h"

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
      public: explicit NetworkManagerSecondary(
                  std::function<void(const UpdateInfo &_info)> _stepFunction,
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
      public: bool OnControl(const private_msgs::PeerControl &_req,
                             private_msgs::PeerControl &_resp);

      /// \brief Callback when step commands are received from the primary
      /// \param[in] _msg Step message.
      private: void OnStep(const private_msgs::SimulationStep &_msg);

      /// \brief Track connection to "events::Stop" Event
      public: ignition::common::ConnectionPtr stoppingConn;

      /// \brief Flag to indicate if simulation server is stopping.
      private: std::atomic<bool> stopReceived {false};

      /// \brief Flag to control enabling/disabling simulation secondary.
      private: std::atomic<bool> enableSim {false};

      /// \brief Transport node used for communication with simulation graph.
      private: ignition::transport::Node node;

      /// \brief Publish step acknowledgement messages.
      private: ignition::transport::Node::Publisher stepAckPub;

      /// \brief Collection of performers associated with this secondary.
      private: std::unordered_set<Entity> performers;
    };
    }
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_NETWORKMANAGERSECONDARY_HH_

