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
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_set>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/msgs/serialized_map.pb.h>
#include <ignition/transport/Node.hh>

#include "msgs/simulation_step.pb.h"
#include "msgs/peer_control.pb.h"

#include "NetworkManager.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \class NetworkManagerSecondary NetworkManagerSecondary.hh
    ///   ignition/gazebo/network/NetworkManagerSecondary.hh
    /// \brief Secondary specific behaviors
    class IGNITION_GAZEBO_VISIBLE NetworkManagerSecondary:
      public NetworkManager
    {
      // Documentation inherited
      public: explicit NetworkManagerSecondary(
          const std::function<void(const UpdateInfo &_info)> &_stepFunction,
          EntityComponentManager &_ecm, EventManager *_eventMgr,
          const NetworkConfig &_config,
          const NodeOptions &_options);

      public: ~NetworkManagerSecondary();

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

      /// \brief Task that will be executed asynchronously in a separated thread.
      private: void AsyncStepTask();

      /// \brief Flag to control enabling/disabling simulation secondary.
      private: std::atomic<bool> enableSim {false};

      /// \brief Transport node used for communication with simulation graph.
      private: ignition::transport::Node node;

      /// \brief Publish step acknowledgement messages.
      private: ignition::transport::Node::Publisher stepAckPub;

      /// \brief Collection of performers associated with this secondary.
      private: std::unordered_set<Entity> performers;

      /// \brief Thread doing steps asynchronously.
      private: std::thread steppingThread;

      /// \brief Max iteration that the secondary can move ahead.
      private: uint64_t maxIteration{0};

      /// \brief Last (local) update info.
      private: UpdateInfo lastUpdateInfo;

      /// \brief Vector of received step messages.
      private: std::deque<private_msgs::SimulationStep> steps;

      /// \brief Mutex protecting `this->steps`.
      private: std::mutex stepsMutex;

      /// \brief Condition variable used to awake the thread doing steps.
      private: std::condition_variable moreStepsCv;

      /// \brief Boolean used to indicate that the thread doing asynchronous steps has to stop.
      private: bool stopAsyncStepThread {false};

      private: using Duration = std::chrono::steady_clock::duration;

      /// \brief History of previous map states, needed to be able to rewind.
      private: std::map<uint64_t, std::tuple<msgs::SerializedStateMap, Duration, Duration>> history;
    };
    }
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_NETWORKMANAGERSECONDARY_HH_

