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
#include <ignition/gazebo/network/NetworkManager.hh>

#include "ignition/msgs/empty.pb.h"
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
                  EventManager *_eventMgr,
                  const NetworkConfig &_config,
                  const NodeOptions &_options);

      // Documentation inherited
      public: bool Ready() const override;

      // Documentation inherited
      public: void Initialize() override;

      // Documentation inherited
      public: bool Step(
                  uint64_t &_iteration,
                  std::chrono::steady_clock::duration &_stepSize,
                  std::chrono::steady_clock::duration &_simTime) override;

      // Documentation inherited
      public: bool StepAck(uint64_t _iteration) override;

      // Documentation inherited
      public: std::string Namespace() const override;

      /// \brief Callback for when PeerControl service request is received.
      public: bool OnControl(const msgs::PeerControl &_req,
                             ignition::msgs::Empty &_resp);

      /// \brief Callback for when SimulationStep message is received.
      public: void OnStep(const msgs::SimulationStep &_msg);

      /// \brief Hold the data from the most current simulation step.
      private: std::unique_ptr<msgs::SimulationStep> currentStep;

      /// \brief Mutex to protect currentStep data.
      private: std::mutex stepMutex;

      /// \brief Condition variable to signal changes of currentStep data.
      private: std::condition_variable stepCv;

      /// \brief Flag to control enabling/disabling simulation secondary.
      private: std::atomic<bool> enableSim;

      /// \brief Flag to control pausing/unpausing simulation secondary.
      private: std::atomic<bool> pauseSim;

      /// \brief Transport node used for communication with simulation graph.
      private: ignition::transport::Node node;

      /// \brief Publisher for communication simulation step acknowledgement.
      private: ignition::transport::Node::Publisher stepAckPub;
    };
    }
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_NETWORKMANAGERSECONDARY_HH_

