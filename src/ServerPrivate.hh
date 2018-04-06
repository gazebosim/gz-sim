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
#ifndef IGNITION_GAZEBO_SERVERPRIVATE_HH_
#define IGNITION_GAZEBO_SERVERPRIVATE_HH_

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/System.hh"

namespace ignition
{
  namespace gazebo
  {
    // Private data for Server
    class ServerPrivate
    {
      /// \brief Constructor
      public: ServerPrivate() = default;

      /// \brief Destructor
      public: ~ServerPrivate();

      /// \brief Update all the systems
      public: void UpdateSystems();

      /// \brief Run the server.
      /// \param[in] _iterations Number of iterations.
      public: void Run(const uint64_t _iterations);

      /// \brief Transport service that responds with the scene graph.
      /// \param[out] _rep Scene reply message.
      /// \return True if the service successfully completed.
      public: bool SceneService(ignition::msgs::Scene &_rep);

      /// \brief Signal handler
      /// \param[in] _sig The signal number
      private: static void SigHandler(int _sig);

      /// \brief This is used to indicate that Run has been called, and the
      /// server is in the run state.
      public: static std::atomic<bool> running;

      /// \brief Thread that executes systems.
      public: std::thread runThread;

      /// \brief All of the systems.
      public: std::vector<std::unique_ptr<System>> systems;

      /// \brief Communication node.
      public: ignition::transport::Node node;

      /// \brief Number of iterations.
      public: uint64_t iterations = 0;
    };
  }
}
#endif
