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
#ifndef IGNITION_GAZEBO_SIMULATION_RUNNER_HH_
#define IGNITION_GAZEBO_SIMULATION_RUNNER_HH_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <ignition/common/WorkerPool.hh>

#include <ignition/math/Pose3.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/SystemQueryResponse.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

using namespace std::chrono_literals;

namespace sdf
{
  class World;
}

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class SimulationRunnerPrivate;

    // Private data for Server
    class SystemInternal
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

    class IGNITION_GAZEBO_VISIBLE SimulationRunner
    {
      /// \brief Constructor
      public: SimulationRunner(const sdf::World *_world);

      /// \brief Destructor.
      public: virtual ~SimulationRunner();

      public: void InitSystems();

      public: void Stop();

      public: bool Run(const uint64_t _iterations,
                  std::optional<std::condition_variable *> _cond);

      /// \brief Update all the systems
      public: void UpdateSystems();

      /// \brief Create all entities that exist in the sdf::World object.
      /// \param[in] _world SDF world object.
      public: void CreateEntities(const sdf::World *_world);

      /// \brief This is used to indicate that Run has been called, and the
      /// server is in the run state.
      public: std::atomic<bool> running{false};

      /// \brief Mutex to protect the Run operation.
      public: std::mutex runMutex;

      /// \brief All the systems.
      public: std::vector<SystemInternal> systems;

      /// \brief Manager of all components.
      public: EntityComponentManager entityCompMgr;

      /// \brief A pool of worker threads.
      public: common::WorkerPool workerPool;

      /// \brief Time of the previous update.
      public: std::chrono::steady_clock::time_point prevUpdateWallTime;

      /// \brief A duration used to account for inaccuracies associated with
      /// sleep durations.
      public: std::chrono::steady_clock::duration sleepOffset{0};

      /// \brief The default update rate is 500hz, which is a period of 2ms.
      public: std::chrono::steady_clock::duration updatePeriod{2ms};

      /// \brief Number of iterations.
      public: uint64_t iterations = 0;
    };
    }
  }
}
#endif
