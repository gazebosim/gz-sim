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
#ifndef GZ_SIM_NETWORK_NETWORKMANAGERPRIMARY_HH_
#define GZ_SIM_NETWORK_NETWORKMANAGERPRIMARY_HH_

#include <gz/msgs/serialized_map.pb.h>
#include "gz/sim/private_msgs/simulation_step.pb.h"

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>
#include <gz/sim/Entity.hh>
#include <gz/transport/Node.hh>


#include "NetworkManager.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    struct SecondaryControl
    {
      /// \brief indicate if the secondary is ready to execute
      std::atomic<bool> ready{false};

      /// \brief id of the secondary peer
      std::string id;

      /// \brief prefix namespace of the secondary peer
      std::string prefix;

      /// \brief Convenience alias for unique_ptr.
      using Ptr = std::unique_ptr<SecondaryControl>;
    };

    /// \class NetworkManagerPrimary NetworkManagerPrimary.hh
    ///   gz/sim/network/NetworkManagerPrimary.hh
    /// \brief Simulation primary specific behaviors
    class GZ_SIM_VISIBLE NetworkManagerPrimary:
      public NetworkManager
    {
      // Documentation inherited
      public: explicit NetworkManagerPrimary(
          const std::function<void(const UpdateInfo &_info)> &_stepFunction,
          EntityComponentManager &_ecm, EventManager *_eventMgr,
          const NetworkConfig &_config,
          const NodeOptions &_options);

      // Documentation inherited
      public: void Handshake() override;

      // Documentation inherited
      public: bool Ready() const override;

      /// \brief Populate simulation step data
      /// This method is called at the beginning of a simulation iteration.
      /// It will populate the info argument with the appropriate values for
      /// the simuation iteration.
      /// \param[inout] _info current simulation update information
      /// \return True if simulation step was successfully synced.
      public: bool Step(const UpdateInfo &_info);

      // Documentation inherited
      public: std::string Namespace() const override;

      /// \brief Return a mutable reference to the currently detected secondary
      /// peers.
      public: std::map<std::string, SecondaryControl::Ptr>& Secondaries();

      /// \brief Callback for step ack messages.
      /// \param[in] _msg Message containing secondary's updated state.
      private: void OnStepAck(const msgs::SerializedStateMap &_msg);

      /// \brief Check if the step publisher has connections.
      private: bool SecondariesCanStep() const;

      /// \brief Populate the step message with the latest affinities according
      /// to levels.
      /// \param[in] _msg Step message.
      private: void PopulateAffinities(private_msgs::SimulationStep &_msg);

      /// \brief Set the performer to secondary affinity.
      /// \param[in] _performer Performer entity.
      /// \param[in] _secondary Secondary identifier.
      /// \param[out] _msg Message to be populated.
      private: void SetAffinity(Entity _performer,
          const std::string &_secondary, private_msgs::PerformerAffinity *_msg);

      /// \brief Container of currently used secondary peers
      private: std::map<std::string, SecondaryControl::Ptr> secondaries;

      /// \brief Transport node
      private: gz::transport::Node node;

      /// \brief Publisher for network step sync
      private: gz::transport::Node::Publisher simStepPub;

      /// \brief Keep track of states received from secondaries.
      private: std::vector<msgs::SerializedStateMap> secondaryStates;

      /// \brief Promise used to notify when all secondaryStates where received.
      private: std::promise<void> secondaryStatesPromise;
    };
    }
  }  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_NETWORKMANAGERPRIMARY_HH_
