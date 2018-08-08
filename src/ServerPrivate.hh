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
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sdf/Root.hh>

#include <ignition/transport/Node.hh>
#include <ignition/common/SignalHandler.hh>
#include <ignition/common/WorkerPool.hh>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/EntityQueryRegistrar.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Private data for Server
    class IGNITION_GAZEBO_HIDDEN SystemInternal
    {
      public: explicit SystemInternal(std::unique_ptr<System> _system)
              : system(std::move(_system))
              {
              }

      /// \brief All of the systems.
      public: std::unique_ptr<System> system;

      public: std::vector<
              std::pair<EntityQueryId, EntityQueryCallback>> updates;
    };

    // Private data for Server
    class IGNITION_GAZEBO_HIDDEN ServerPrivate
    {
      /// \brief Constructor
      public: ServerPrivate();

      /// \brief Destructor
      public: ~ServerPrivate();

      /// \brief Update all the systems
      public: void UpdateSystems();

      /// \brief Run the server.
      /// \param[in] _iterations Number of iterations.
      /// \param[in] _cond Optional condition variable. This condition is
      /// notified when the server has started running.
      public: bool Run(const uint64_t _iterations,
                 std::optional<std::condition_variable *> _cond = std::nullopt);

      /// \brief Create all entities that exist in the sdf::Root object.
      /// \param[in] _root SDF root object.
      public: void CreateEntities(const sdf::Root &_root);

      /// \brief Initialize all the systems.
      public: void InitSystems();

      /// \brief Signal handler callback
      /// \param[in] _sig The signal number
      private: void OnSignal(int _sig);

      /// \brief Thread that executes systems.
      public: std::thread runThread;

      /// \brief Communication node.
      public: ignition::transport::Node node;

      /// \brief Number of iterations.
      public: uint64_t iterations = 0;

      /// \brief This is used to indicate that Run has been called, and the
      /// server is in the run state.
      public: std::atomic<bool> running{false};

      /// \brief Mutex to protect the Run operation.
      public: std::mutex runMutex;

      /// \brief Our signal handler.
      public: ignition::common::SignalHandler sigHandler;

      /// \brief Manager of all components.
      public: std::shared_ptr<EntityComponentManager> entityCompMgr;

      /// \brief All the systems.
      public: std::vector<SystemInternal> systems;

      /// \brief A pool of worker threads.
      public: common::WorkerPool workerPool;

      /// \brief This is used to keep track of the world entity for
      /// convenience.
      public: EntityId worldEntity;

      public: std::chrono::steady_clock::time_point prevStepWallTime;
      public: std::chrono::steady_clock::duration sleepOffset{0};
    };
    }
  }
}
#endif
