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
#ifndef GZ_SIM_SIMULATIONRUNNER_HH_
#define GZ_SIM_SIMULATIONRUNNER_HH_

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/gui.pb.h>
#include <gz/msgs/log_playback_control.pb.h>
#include <gz/msgs/sdf_generator_config.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/world_control_state.pb.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <sdf/World.hh>

#include <gz/common/Event.hh>
#include <gz/math/Stopwatch.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/config.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"

#include "network/NetworkManager.hh"
#include "LevelManager.hh"
#include "SystemManager.hh"
#include "Barrier.hh"
#include "WorldControl.hh"

using namespace std::chrono_literals;

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    // Forward declarations.
    class SimulationRunnerPrivate;

    class GZ_SIM_VISIBLE SimulationRunner
    {
      /// \brief Constructor
      /// \param[in] _world Pointer to the SDF world.
      /// \param[in] _systemLoader Reference to system manager.
      /// \param[in] _useLevels Whether to use levles or not. False by default.
      public: explicit SimulationRunner(const sdf::World &_world,
                                const SystemLoaderPtr &_systemLoader,
                                const ServerConfig &_config = ServerConfig());

      /// \brief Destructor.
      public: virtual ~SimulationRunner();

      /// \brief Stop running
      public: void Stop();

      /// \brief Internal method for handling stop event (to prevent recursion)
      private: void OnStop();

      /// \brief Stop and join all post update worker threads
      private: void StopWorkerThreads();

      /// \brief Run the simulationrunner.
      /// \param[in] _iterations Number of iterations.
      /// \return True if the operation completed successfully.
      public: bool Run(const uint64_t _iterations);

      /// \brief Perform a simulation step:
      /// * Publish stats and process control messages
      /// * Update levels and systems
      /// * Process entity creation / removal
      /// \param[in] _info Time information for the step.
      public: void Step(const UpdateInfo &_info);

      /// \brief Add system after the simulation runner has been instantiated
      /// \note This actually adds system to a queue. The system is added to the
      /// runner at the begining of the a simulation cycle (call to Run). It is
      /// also responsible for calling `Configure` on the system.
      /// \param[in] _system SystemPluginPtr to be added
      /// \param[in] _entity Entity that system is attached to. If nullopt,
      /// system is attached to a world.
      /// \param[in] _sdf Pointer to the SDF of the entity. Nullopt defaults to
      /// SDF of the entire world.
      public: void AddSystem(const SystemPluginPtr &_system,
          std::optional<Entity> _entity = std::nullopt,
          std::optional<std::shared_ptr<const sdf::Element>> _sdf =
              std::nullopt);

      /// \brief Add system after the simulation runner has been instantiated
      /// \note This actually adds system to a queue. The system is added to the
      /// runner at the begining of the a simulation cycle (call to Run). It is
      /// also responsible for calling `Configure` on the system.
      /// \param[in] _system System to be added
      /// \param[in] _entity Entity of system to be added. Nullopt if system
      /// doesn't connect to an entity.
      /// \param[in] _sdf Pointer to the SDF of the entity. Nullopt defaults to
      /// world.
      public: void AddSystem(const std::shared_ptr<System> &_system,
          std::optional<Entity> _entity = std::nullopt,
          std::optional<std::shared_ptr<const sdf::Element>> _sdf =
              std::nullopt);

      /// \brief Update all the systems
      public: void UpdateSystems();

      /// \brief Publish current world statistics.
      public: void PublishStats();

      /// \brief Load system plugin for a given entity.
      /// \param[in] _entity The plugins will be associated with this Entity
      /// \param[in] _plugin SDF Plugin to load
      public: void LoadPlugin(const Entity _entity, const sdf::Plugin &_plugin);

      /// \brief Load system plugins for a given entity.
      /// \param[in] _entity The plugins will be associated with this Entity
      /// \param[in] _plugins SDF Plugins to load
      public: void LoadPlugins(const Entity _entity,
          const sdf::Plugins &_plugins);

      /// \brief Load server plugins for a given entity.
      /// \param[in] _plugins Load any additional plugins from the
      /// Server Configuration
      public: void LoadServerPlugins(
          const std::list<ServerConfig::PluginInfo> &_plugins);

      /// \brief Load logging/playback plugins
      /// \param[in] _config Configuration to load plugins from.
      public: void LoadLoggingPlugins(const ServerConfig &_config);

      /// \brief Get whether this is running. When running is true,
      /// then simulation is stepping forward.
      /// \return True if the server is running.
      public: bool Running() const;

      /// \brief Get whether the runner has received a stop event
      /// \return True if the event has been received.
      public: bool StopReceived() const;

      /// \brief Get whether the runner is ready to execute.
      /// \return True if the runner is ready
      public: bool Ready() const;

      /// \brief Get the number of iterations the server has executed.
      /// \return The current iteration count.
      public: uint64_t IterationCount() const;

      /// \brief Get the number of entities on the runner.
      /// \return Entity count.
      public: size_t EntityCount() const;

      /// \brief Get the number of systems on the runner.
      /// \return System count.
      public: size_t SystemCount() const;

      /// \brief Set the update period. The update period is the wall-clock
      /// time between updates of all systems. Note that even if systems
      /// are being updated, this doesn't mean sim time is increasing.
      /// \param[in] _updatePeriod Duration between updates.
      public: void SetUpdatePeriod(
                  const std::chrono::steady_clock::duration &_updatePeriod);

      /// \brief Get the simulation epoch.
      /// \return The simulation epoch.
      public: const std::chrono::steady_clock::duration &SimTimeEpoch() const;

      /// \brief Get the update period.
      /// \return The update period.
      public: const std::chrono::steady_clock::duration &UpdatePeriod() const;

      /// \brief Set the paused state.
      /// \param[in] _paused True to pause the simulation runner.
      public: void SetPaused(const bool _paused);

      /// \brief Get the pause state.
      /// \return True if the simulation runner is paused, false otherwise.
      public: bool Paused() const;

      /// \brief Set if the simulation runner is stepping based on WorldControl
      /// info
      /// \param[in] _step True if stepping based on WorldControl info, false
      /// otherwise
      public: void SetStepping(bool _step);

      /// \brief Get if the simulation runner is stepping based on WorldControl
      /// info
      /// \return True if stepping based on WorldControl info, false otherwise
      public: bool Stepping() const;

      /// \brief Set the run to simulation time.
      /// \param[in] _time A simulation time in the future to run to and then
      /// pause. A time prior than the current simulation time disables the
      /// run-to feature.
      public: void SetRunToSimTime(
                  const std::chrono::steady_clock::duration &_time);

      /// \brief Get the EntityComponentManager
      /// \return Reference to the entity component manager.
      public: const EntityComponentManager &EntityCompMgr() const;

      /// \brief Return an entity with the provided name.
      /// \details If multiple entities with the same name exist, the first
      /// entity found will be returned.
      /// \param[in] _name Name of the entity.
      /// \return The entity, if the entity exists in the world. Otherwise
      /// std::nullopt.
      public: std::optional<Entity> EntityByName(
                  const std::string &_name) const;

      /// \brief Return true if an entity with the provided name exists.
      /// \param[in] _name Name of the entity.
      /// \return True if the entity exists in the world.
      public: bool HasEntity(const std::string &_name) const;

      /// \brief Return true if an entity exists with the
      /// provided name and the entity was queued for deletion. Note that
      /// the entity is not removed immediately. Entity deletion happens at
      /// the end of the next (or current depending on when this function is
      /// called) simulation step.
      /// \param[in] _name Name of the entity to delete.
      /// \param[in] _recursive Whether to recursively delete all child
      /// entities. True by default.
      /// \return True if the entity exists in the world and it was queued
      /// for deletion.
      public: bool RequestRemoveEntity(const std::string &_name,
          bool _recursive = true);

      /// \brief Return true if an entity exists with the
      /// provided id and the entity was queued for deletion. Note that
      /// the entity is not removed immediately. Entity deletion happens at
      /// the end of the next (or current depending on when this function is
      /// called) simulation step.
      /// \details If multiple entities with the same name exist, only the
      /// first entity found will be deleted.
      /// \param[in] _entity The entity to delete.
      /// \param[in] _recursive Whether to recursively delete all child
      /// entities. True by default.
      /// \return True if the entity exists in the world and it was queued
      /// for deletion.
      public: bool RequestRemoveEntity(const Entity _entity,
          bool _recursive = true);

      /// \brief Get the EventManager
      /// \return Reference to the event manager.
      public: EventManager &EventMgr();

      /// \brief Get the current info object.
      /// \return Current info.
      public: const UpdateInfo &CurrentInfo() const;

      /// \brief Get the step size;
      /// \return Step size.
      public: const std::chrono::steady_clock::duration &StepSize() const;

      /// \brief Set the step size;
      /// \param[in] _step Step size.
      public: void SetStepSize(
          const std::chrono::steady_clock::duration &_step);

      /// \brief World control service callback. This function stores the
      /// the request which will then be processed by the ProcessMessages
      /// function.
      /// \param[in] _req Request from client, currently handling play / pause
      /// and multistep.
      /// \param[out] _res Response to client, true if successful.
      /// \return True for success
      private: bool OnWorldControl(const msgs::WorldControl &_req,
                                         msgs::Boolean &_res);

      /// \brief World control state service callback. This function stores the
      /// the request which will then be processed by the ProcessMessages
      /// function.
      /// \param[in] _req Request from client, currently handling play / pause
      /// and multistep. This also may contain SerializedState information.
      /// \param[out] _res Response to client, true if successful.
      /// \return True for success
      private: bool OnWorldControlState(const msgs::WorldControlState &_req,
                                         msgs::Boolean &_res);

      /// \brief World control service callback. This function stores the
      /// the request which will then be processed by the ProcessMessages
      /// function.
      /// \param[in] _req Request from client, currently handling play / pause
      /// and multistep.
      /// \param[out] _res Response to client, true if successful.
      /// \return True for success
      private: bool OnPlaybackControl(const msgs::LogPlaybackControl &_req,
                                            msgs::Boolean &_res);

      /// \brief Callback for GUI info service.
      /// \param[out] _res Response containing the latest GUI message.
      /// \return True if successful.
      private: bool GuiInfoService(gz::msgs::GUI &_res);

      /// \brief Calculate real time factor and populate currentInfo.
      private: void UpdateCurrentInfo();

      /// \brief Process all buffered messages. Ths function is called at
      /// the end of an update iteration.
      private: void ProcessMessages();

      /// \brief Process world control service messages.
      private: void ProcessWorldControl();

      /// \brief Actually add system to the runner
      /// \param[in] _system System to be added
      public: void AddSystemToRunner(SystemInternal _system);

      /// \brief Calls AddSystemToRunner to each system that is pending to be
      /// added.
      public: void ProcessSystemQueue();

      /// \brief Generate the current world's SDFormat representation.
      /// \param[in] _req Request message with options for saving a world to an
      /// SDFormat file.
      /// \param[out] _res Generated SDFormat string.
      /// \return True if successful.
      public: bool GenerateWorldSdf(const msgs::SdfGeneratorConfig &_req,
                                    msgs::StringMsg &_res);

      /// \brief Sets the file path to fuel URI map.
      /// \param[in] _map A populated map of file paths to fuel URIs.
      public: void SetFuelUriMap(
                  const std::unordered_map<std::string, std::string> &_map);

      /// \brief Add an entry to the file path to Fuel URI map
      /// \param[in] _path A file path to a resource (model.sdf, mesh, etc).
      /// \param[in] _uri A fuel URI.
      public: void AddToFuelUriMap(const std::string &_path,
                                   const std::string &_uri);

      /// \brief Get whether the next step is going to be executed as paused.
      /// \return True if the next step is being executed as paused, false
      /// otherwise.
      public: bool NextStepIsBlockingPaused() const;

      /// \brief Set the next step to be blocking and paused.
      public: void SetNextStepAsBlockingPaused(const bool value);

      /// \brief Updates the physics parameters of the simulation based on the
      /// Physics component of the world, if any.
      public: void UpdatePhysicsParams();

      /// \brief Create entities for the world simulated by this runner based
      /// on the provided SDF Root object.
      /// \param[in] _world SDF world created entities from.
      public: void CreateEntities(const sdf::World &_world);

      /// \brief Process entities with the components::Recreate component.
      /// Put in a request to make them as removed
      private: void ProcessRecreateEntitiesRemove();

      /// \brief Process entities with the components::Recreate component.
      /// Reccreate the entities by cloning from the original ones.
      private: void ProcessRecreateEntitiesCreate();

      /// \brief Process the new world state message, if it is present.
      /// See the newWorldControlState variable below.
      private: void ProcessNewWorldControlState();

      /// \brief This is used to indicate that a stop event has been received.
      private: std::atomic<bool> stopReceived{false};

      /// \brief This is used to indicate that Run has been called, and the
      /// server is in the run state.
      private: std::atomic<bool> running{false};

      /// \brief Manager of all systems.
      /// Note: must be before EntityComponentManager
      /// Note: must be before EventMgr
      /// Because systems have access to the ECM and Events, they need to be
      /// cleanly stopped and destructed before destroying the event manager
      /// and entity component manager.
      private: std::unique_ptr<SystemManager> systemMgr;

      /// \brief Manager of all events.
      /// Note: must be before EntityComponentManager
      private: EventManager eventMgr;

      /// \brief Manager all parameters
      private: std::unique_ptr<
        gz::transport::parameters::ParametersRegistry
      > parametersRegistry;

      /// \brief Manager of all components.
      private: EntityComponentManager entityCompMgr;

      /// \brief Copy of the EntityComponentManager immediately after the
      /// initial entity creation/world load.
      private: EntityComponentManager initialEntityCompMgr;

      /// \brief Manager of all levels.
      private: std::unique_ptr<LevelManager> levelMgr;

      /// \brief Manager of distributing/receiving network work.
      private: std::unique_ptr<NetworkManager> networkMgr{nullptr};

      /// \brief Wall time of the previous update.
      private: std::chrono::steady_clock::time_point prevUpdateRealTime;

      /// \brief A duration used to account for inaccuracies associated with
      /// sleep durations.
      private: std::chrono::steady_clock::duration sleepOffset{0};

      /// \brief This is the rate at which the systems are updated.
      /// The default update rate is 500hz, which is a period of 2ms.
      private: std::chrono::steady_clock::duration updatePeriod{2ms};

      /// \brief The simulation epoch.
      /// All simulation times will be larger than the epoch. It defaults to 0.
      private: std::chrono::steady_clock::duration simTimeEpoch{0};


      /// \brief Node for communication.
      private: std::unique_ptr<transport::Node> node{nullptr};

      /// \brief World statistics publisher.
      private: gz::transport::Node::Publisher statsPub;

      /// \brief Clock publisher for the root `/stats` topic.
      private: gz::transport::Node::Publisher rootStatsPub;

      /// \brief Clock publisher.
      private: gz::transport::Node::Publisher clockPub;

      /// \brief Clock publisher for the root `/clock` topic.
      private: gz::transport::Node::Publisher rootClockPub;

      /// \brief Name of world being simulated.
      private: std::string worldName;

      /// \brief Stopwatch to keep track of wall time.
      private: gz::math::Stopwatch realTimeWatch;

      /// \brief Step size
      private: std::chrono::steady_clock::duration stepSize{10ms};

      /// \brief Desired real time factor
      private: double desiredRtf{1.0};

      /// \brief Connection to the pause event.
      private: gz::common::ConnectionPtr pauseConn;

      /// \brief Connection to the stop event.
      private: gz::common::ConnectionPtr stopConn;

      /// \brief Connection to the load plugins event.
      private: common::ConnectionPtr loadPluginsConn;

      /// \brief The sdf::World object of this runner
      private: sdf::World sdfWorld;

      /// \brief The real time factor calculated based on sim and real time
      /// averages.
      private: double realTimeFactor{0.0};

      /// \brief Number of simulation steps requested that haven't been
      /// executed yet.
      private: unsigned int pendingSimIterations{0};

      /// \brief True if user requested to rewind simulation.
      private: bool requestedRewind{false};

      /// \brief If user asks to seek to a specific sim time, this holds the
      /// time.
      private: std::optional<std::chrono::steady_clock::duration> requestedSeek;

      /// \brief A simulation time past the epoch to run to and then pause.
      private: std::optional<std::chrono::steady_clock::duration>
        requestedRunToSimTime;

      /// \brief Keeps the latest simulation info.
      private: UpdateInfo currentInfo;

      /// \brief Buffer of world control messages.
      private: std::list<WorldControl> worldControls;

      /// \brief Mutex to protect message buffers.
      private: std::mutex msgBufferMutex;

      /// \brief Keep the latest GUI message.
      public: msgs::GUI guiMsg;

      /// \brief Copy of the server configuration.
      public: ServerConfig serverConfig;

      /// \brief Collection of threads running system PostUpdates
      private: std::vector<std::thread> postUpdateThreads;

      /// \brief Flag to indicate running status of PostUpdate threads
      private: std::atomic<bool> postUpdateThreadsRunning{false};

      /// \brief Barrier to signal beginning of PostUpdate thread execution
      private: std::unique_ptr<Barrier> postUpdateStartBarrier;

      /// \brief Barrier to signal end of PostUpdate thread execution
      private: std::unique_ptr<Barrier> postUpdateStopBarrier;

      /// \brief Map from file paths to Fuel URIs.
      private: std::unordered_map<std::string, std::string> fuelUriMap;

      /// \brief True if Server::RunOnce triggered a blocking paused step
      private: bool blockingPausedStepPending{false};

      /// \brief Whether the simulation runner is currently stepping based on
      /// WorldControl info (true) or not (false)
      private: bool stepping{false};

      /// \brief A set of entities that need to be recreated
      private: std::set<Entity> entitiesToRecreate;

      /// \brief Holds new world state information so that it can be processed
      /// at the appropriate time.
      private: std::unique_ptr<msgs::WorldControlState> newWorldControlState;

      /// \brief Set if we need to remove systems due to entity removal
      private: bool threadsNeedCleanUp{false};

      private: bool resetInitiated{false};
      friend class LevelManager;
    };
    }
  }
}
#endif
