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
#ifndef IGNITION_GAZEBO_SIMULATIONRUNNER_HH_
#define IGNITION_GAZEBO_SIMULATIONRUNNER_HH_

#include <atomic>
#include <chrono>
#include <functional>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ignition/common/WorkerPool.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/SystemManager.hh"
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

    /// \brief Class to hold systems internally
    class SystemInternal
    {
      /// \brief Constructor
      public: explicit SystemInternal(const std::shared_ptr<System> &_system)
              : system(_system)
      {
      }

      /// \brief All of the systems.
      public: std::shared_ptr<System> system;

      /// \brief Vector of queries and callbacks
      public: std::vector<
              std::pair<EntityQueryId, EntityQueryCallback>> updates;
    };

    class IGNITION_GAZEBO_VISIBLE SimulationRunner
    {
      public: using SystemPtr = std::shared_ptr<System>;

      /// \brief Constructor
      /// \param[in] _world Pointer to the SDF world.
      /// \param[in] _systems Systems to be loaded
      public: explicit SimulationRunner(const sdf::World *_world,
                const std::vector<SystemPtr> &_systems);

      /// \brief Destructor.
      public: virtual ~SimulationRunner();

      /// \brief Initialize the systems
      public: void InitSystems();

      /// \brief Stop running
      public: void Stop();

      /// \brief Run the simulationrunner.
      /// \param[in] _iterations Number of iterations.
      public: bool Run(const uint64_t _iterations);

      /// \brief Update all the systems
      /// \param[in] _info Information about the simulation status.
      public: void UpdateSystems(const UpdateInfo &_info);

      /// \brief Publish current world statistics.
      /// \param[in] _info Information about the simulation status.
      public: void PublishStats(const UpdateInfo &_info);

      /// \brief Create all entities that exist in the sdf::World object.
      /// \param[in] _world SDF world object.
      public: void CreateEntities(const sdf::World *_world);

      /// \brief Get whether this is running. When running is true,
      /// then simulation is stepping forward.
      /// \return True if the server is running.
      public: bool Running() const;

      /// \brief Get the number of iterations the server has executed.
      /// \return The current iteration count.
      public: uint64_t IterationCount() const;

      /// \brief Get the number of entities on the runner.
      /// \return Entity count.
      public: size_t EntityCount() const;

      /// \brief Get the number of systems on the runner.
      /// \return System count.
      public: size_t SystemCount() const;

      /// \brief Set the update period. The update period is the wall-clock time
      /// between updates.
      /// \param[in] _updatePeriod Duration between updates.
      public: void SetUpdatePeriod(
                  const std::chrono::steady_clock::duration &_updatePeriod);

      /// \brief World control service callback
      /// \param[in] _req Request
      /// \param[out] _res Response
      /// \return True for success
      private: bool OnWorldControl(const msgs::WorldControl &_req,
                                         msgs::Boolean &_res);

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

      /// \brief Wall time of the previous ECS update.
      public: std::chrono::steady_clock::time_point ecsPrevUpdateRealTime;

      /// \brief A duration used to account for inaccuracies associated with
      /// sleep durations.
      public: std::chrono::steady_clock::duration ecsSleepOffset{0};

      /// \brief This is the rate at which the ECS is updated.
      /// The default update rate is 500hz, which is a period of 2ms.
      public: std::chrono::steady_clock::duration ecsUpdatePeriod{2ms};

      /// \brief The default update rate is 500hz, which is a period of 2ms.
      public: std::chrono::steady_clock::duration simUpdatePeriod{2ms};

      /// \brief Number of ECS iterations.
      public: uint64_t ecsIterations{0};

      /// \brief Number of simulation iterations.
      public: uint64_t simIterations{0};

      /// \brief List of simulation times used to compute averages.
      public: std::list<std::chrono::steady_clock::duration> simTimes;

      /// \brief List of real times used to compute averages.
      public: std::list<std::chrono::steady_clock::duration> realTimes;

      /// \brief Node for communication.
      public: ignition::transport::Node node;

      /// \brief Publisher for this data.
      public: ignition::transport::Node::Publisher statsPub;

      /// \brief Name of world being simulated.
      public: std::string worldName;

      /// \brief Stopwatch to keep track of wall time.
      public: ignition::math::Stopwatch realTimeWatch;

      /// \brief Total simulation time.
      public: ignition::math::clock::duration simTime{0};

      /// \brief Step size
      public: ignition::math::clock::duration stepSize{10ms};

      /// \brief The real time factor calculated based on sim and real time
      /// averages.
      public: double realTimeFactor;

      /// \brief True if simulation currently paused, which means the simulation
      /// time is not currently running, but systems are still being updated.
      public: bool paused{false};
    };
    }
  }
}
#endif
