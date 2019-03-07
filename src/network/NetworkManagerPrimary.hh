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
#ifndef IGNITION_GAZEBO_NETWORK_NETWORKMANAGERPRIMARY_HH_
#define IGNITION_GAZEBO_NETWORK_NETWORKMANAGERPRIMARY_HH_

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/transport/Node.hh>

#include "NetworkManager.hh"
#include "msgs/simulation_step.pb.h"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    struct SecondaryControl
    {
      /// \brief indicate if the secondary is ready to execute
      std::atomic<bool> ready {false};
      /// \brief acknowledge received flag
      std::atomic<bool> recvStepAck {false};
      /// \brief last acknowledged iteration from secondary peer.
      std::atomic<uint64_t> recvIter {0};

      /// \brief id of the secondary peer
      std::string id;
      /// \brief prefix namespace of the secondary peer
      std::string prefix;
      /// \brief string identification of associated performers to this peer
      std::vector<std::string> performers;
      /// \brief entity identification of associated performers to this peer
      std::vector<Entity> performerIds;

      /// \brief Convenience alias for unique_ptr.
      using Ptr = std::unique_ptr<SecondaryControl>;
    };


    /// \class NetworkManagerPrimary NetworkManagerPrimary.hh
    ///   ignition/gazebo/network/NetworkManagerPrimary.hh
    /// \brief SimulationPrimary specific behaviors
    class IGNITION_GAZEBO_VISIBLE NetworkManagerPrimary:
      public NetworkManager
    {
      // Documentation inherited
      public: explicit NetworkManagerPrimary(EventManager *_eventMgr,
                                             const NetworkConfig &_config,
                                             const NodeOptions &_options);

      // Documentation inherited
      public: void Initialize() override;

      // Documentation inherited
      public: bool Ready() const override;

      /// \brief Populate simulation step data.
      /// On the network primary, the argument will be used to distribute
      /// simulation state to all of the network participants.
      /// \return True if simulation step is ready.
      public: bool Step(UpdateInfo &_info) override;

      /// \brief Acknowledge the completion of a simulation step.
      /// On the network primary, this will aggregate the acknowledgements of
      /// all simulation participants
      /// \return True if all participants acknowledge the simuation step.
      public: bool StepAck(uint64_t _iteration) override;

      // Documentation inherited
      public: std::string Namespace() const override;

      /// \brief Return a mutable reference to the currently detected secondary
      /// peers.
      public: std::map<std::string, SecondaryControl::Ptr>& Secondaries();

      /// \brief Called when a step acknowledgement is received from a
      /// secondary.
      public: void OnStepAck(const std::string &_secondary,
                  const msgs::SimulationStep &_msg);

      /// \brief Container of currently used secondary peers
      private: std::map<std::string, SecondaryControl::Ptr> secondaries;

      /// \brief Transport node
      private: ignition::transport::Node node;

      /// \brief Publisher for network clock sync
      private: ignition::transport::Node::Publisher simStepPub;
    };
    }
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_NETWORKMANAGERPRIMARY_HH_

