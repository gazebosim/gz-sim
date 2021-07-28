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

#include <ignition/msgs/gui.pb.h>
#include <ignition/msgs/log_playback_control.pb.h>
#include <ignition/msgs/sdf_generator_config.pb.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <sdf/World.hh>

#include <ignition/common/Event.hh>
#include <ignition/common/WorkerPool.hh>
#include <ignition/math/Stopwatch.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/EventManager.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/ServerConfig.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/SystemPluginPtr.hh"
#include "ignition/gazebo/Types.hh"

#include "network/NetworkManager.hh"
#include "LevelManager.hh"
#include "Barrier.hh"

using namespace std::chrono_literals;

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class SimulationRunnerPrivate;

    /// \brief Helper struct to control world time. It's used to hold
    /// input from either msgs::WorldControl or msgs::LogPlaybackControl.
    struct WorldControl
    {
      /// \brief True to pause simulation.
      // cppcheck-suppress unusedStructMember
      bool pause{false};  // NOLINT

      /// \biref Run a given number of simulation iterations.
      // cppcheck-suppress unusedStructMember
      uint64_t multiStep{0u};  // NOLINT

      /// \brief Reset simulation back to time zero. Rewinding resets sim time,
      /// real time and iterations.
      // cppcheck-suppress unusedStructMember
      bool rewind{false};  // NOLINT

      /// \brief Sim time to jump to. A negative value means don't seek.
      /// Seeking changes sim time but doesn't affect real time.
      /// It also resets iterations back to zero.
      std::chrono::steady_clock::duration seek{-1};
    };

    /// \brief Class to hold systems internally. It supports systems loaded
    /// from plugins, as well as systems created at runtime.
    class SystemInternal
    {
      /// \brief Constructor
      /// \param[in] _systemPlugin A system loaded from a plugin.
      public: explicit SystemInternal(SystemPluginPtr _systemPlugin)
              : systemPlugin(std::move(_systemPlugin)),
                system(systemPlugin->QueryInterface<System>()),
                configure(systemPlugin->QueryInterface<ISystemConfigure>()),
                preupdate(systemPlugin->QueryInterface<ISystemPreUpdate>()),
                update(systemPlugin->QueryInterface<ISystemUpdate>()),
                postupdate(systemPlugin->QueryInterface<ISystemPostUpdate>())
      {
      }

      /// \brief Constructor
      /// \param[in] _system Pointer to a system.
      public: explicit SystemInternal(const std::shared_ptr<System> &_system)
              : systemShared(_system),
                system(_system.get()),
                configure(dynamic_cast<ISystemConfigure *>(_system.get())),
                preupdate(dynamic_cast<ISystemPreUpdate *>(_system.get())),
                update(dynamic_cast<ISystemUpdate *>(_system.get())),
                postupdate(dynamic_cast<ISystemPostUpdate *>(_system.get()))
      {
      }

      /// \brief Plugin object. This manages the lifecycle of the instantiated
      /// class as well as the shared library.
      /// This will be null if the system wasn't loaded from a plugin.
      public: SystemPluginPtr systemPlugin;

      /// \brief Pointer to a system.
      /// This will be null if the system wasn't loaded from a pointer.
      public: std::shared_ptr<System> systemShared{nullptr};

      /// \brief Access this system via the `System` interface
      public: System *system = nullptr;

      /// \brief Access this system via the ISystemConfigure interface
      /// Will be nullptr if the System doesn't implement this interface.
      public: ISystemConfigure *configure = nullptr;

      /// \brief Access this system via the ISystemPreUpdate interface
      /// Will be nullptr if the System doesn't implement this interface.
      public: ISystemPreUpdate *preupdate = nullptr;

      /// \brief Access this system via the ISystemUpdate interface
      /// Will be nullptr if the System doesn't implement this interface.
      public: ISystemUpdate *update = nullptr;

      /// \brief Access this system via the ISystemPostUpdate interface
      /// Will be nullptr if the System doesn't implement this interface.
      public: ISystemPostUpdate *postupdate = nullptr;

      /// \brief Vector of queries and callbacks
      public: std::vector<EntityQueryCallback> updates;
    };

    class IGNITION_GAZEBO_VISIBLE SimulationRunner
    {
      /// \brief Constructor
      /// \param[in] _world Pointer to the SDF world.
      /// \param[in] _systemLoader Reference to system manager.
      /// \param[in] _useLevels Whether to use levles or not. False by default.
      public: explicit SimulationRunner(const sdf::World *_world,
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
      /// \param[in] _entity Entity
      /// \param[in] _fname Filename of the plugin library
      /// \param[in] _name Name of the plugin
      /// \param[in] _sdf SDF element (content of plugin tag)
      public: void LoadPlugin(const Entity _entity,
          const std::string &_fname,
          const std::string &_name,
          const sdf::ElementPtr &_sdf);

      /// \brief Load system plugins for a given entity.
      /// \param[in] _entity Entity
      /// \param[in] _sdf SDF element
      public: void LoadPlugins(const Entity _entity,
          const sdf::ElementPtr &_sdf);

      /// \brief Load server plugins for a given entity.
      /// \param[in] _config Configuration to load plugins from.
      ///     plugins based on the _config contents
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

      /// \brief Get the update period.
      /// \return The update period.
      public: const std::chrono::steady_clock::duration &UpdatePeriod() const;

      /// \brief Set the paused state.
      /// \param[in] _paused True to pause the simulation runner.
      public: void SetPaused(const bool _paused);

      /// \brief Get the pause state.
      /// \return True if the simulation runner is paused, false otherwise.
      public: bool Paused() const;

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
      public: const ignition::math::clock::duration &StepSize() const;

      /// \brief Set the step size;
      /// \param[in] _step Step size.
      public: void SetStepSize(const ignition::math::clock::duration &_step);

      /// \brief World control service callback. This function stores the
      /// the request which will then be processed by the ProcessMessages
      /// function.
      /// \param[in] _req Request from client, currently handling play / pause
      /// and multistep.
      /// \param[out] _res Response to client, true if successful.
      /// \return True for success
      private: bool OnWorldControl(const msgs::WorldControl &_req,
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
      private: bool GuiInfoService(ignition::msgs::GUI &_res);

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

      /// \brief Implementation for AddSystem functions. This only adds systems
      /// to a queue, the actual addition is performed by `AddSystemToRunner` at
      /// the appropriate time.
      /// \param[in] _system Generic representation of a system.
      /// \param[in] _entity Entity received from AddSystem.
      /// \param[in] _sdf SDF received from AddSystem.
      private: void AddSystemImpl(SystemInternal _system,
        std::optional<Entity> _entity = std::nullopt,
        std::optional<std::shared_ptr<const sdf::Element>> _sdf = std::nullopt);

      /// \brief This is used to indicate that a stop event has been received.
      private: std::atomic<bool> stopReceived{false};

      /// \brief This is used to indicate that Run has been called, and the
      /// server is in the run state.
      private: std::atomic<bool> running{false};

      /// \brief All the systems.
      private: std::vector<SystemInternal> systems;

      /// \brief Pending systems to be added to systems.
      private: std::vector<SystemInternal> pendingSystems;

      /// \brief Mutex to protect pendingSystems
      private: mutable std::mutex pendingSystemsMutex;

      /// \brief Systems implementing Configure
      private: std::vector<ISystemConfigure *> systemsConfigure;

      /// \brief Systems implementing PreUpdate
      private: std::vector<ISystemPreUpdate *> systemsPreupdate;

      /// \brief Systems implementing Update
      private: std::vector<ISystemUpdate *> systemsUpdate;

      /// \brief Systems implementing PostUpdate
      private: std::vector<ISystemPostUpdate *> systemsPostupdate;

      /// \brief Manager of all events.
      private: EventManager eventMgr;

      /// \brief Manager of all components.
      private: EntityComponentManager entityCompMgr;

      /// \brief Manager of all levels.
      private: std::unique_ptr<LevelManager> levelMgr;

      /// \brief Manager of distributing/receiving network work.
      private: std::unique_ptr<NetworkManager> networkMgr{nullptr};

      /// \brief A pool of worker threads.
      private: common::WorkerPool workerPool{2};

      /// \brief Wall time of the previous update.
      private: std::chrono::steady_clock::time_point prevUpdateRealTime;

      /// \brief A duration used to account for inaccuracies associated with
      /// sleep durations.
      private: std::chrono::steady_clock::duration sleepOffset{0};

      /// \brief This is the rate at which the systems are updated.
      /// The default update rate is 500hz, which is a period of 2ms.
      private: std::chrono::steady_clock::duration updatePeriod{2ms};

      /// \brief List of simulation times used to compute averages.
      private: std::list<std::chrono::steady_clock::duration> simTimes;

      /// \brief List of real times used to compute averages.
      private: std::list<std::chrono::steady_clock::duration> realTimes;

      /// \brief System loader, for loading system plugins.
      private: SystemLoaderPtr systemLoader;

      /// \brief Mutex to protect systemLoader
      private: std::mutex systemLoaderMutex;

      /// \brief Node for communication.
      private: std::unique_ptr<transport::Node> node{nullptr};

      /// \brief World statistics publisher.
      private: ignition::transport::Node::Publisher statsPub;

      /// \brief Clock publisher for the root `/stats` topic.
      private: ignition::transport::Node::Publisher rootStatsPub;

      /// \brief Clock publisher.
      private: ignition::transport::Node::Publisher clockPub;

      /// \brief Clock publisher for the root `/clock` topic.
      private: ignition::transport::Node::Publisher rootClockPub;

      /// \brief Name of world being simulated.
      private: std::string worldName;

      /// \brief Stopwatch to keep track of wall time.
      private: ignition::math::Stopwatch realTimeWatch;

      /// \brief Step size
      private: ignition::math::clock::duration stepSize{10ms};

      /// \brief Desired real time factor
      private: double desiredRtf{1.0};

      /// \brief Connection to the pause event.
      private: ignition::common::ConnectionPtr pauseConn;

      /// \brief Connection to the stop event.
      private: ignition::common::ConnectionPtr stopConn;

      /// \brief Connection to the load plugins event.
      private: common::ConnectionPtr loadPluginsConn;

      /// \brief Pointer to the sdf::World object of this runner
      private: const sdf::World *sdfWorld;

      /// \brief The real time factor calculated based on sim and real time
      /// averages.
      private: double realTimeFactor{0.0};

      /// \brief Number of simulation steps requested that haven't been
      /// executed yet.
      private: unsigned int pendingSimIterations{0};

      /// \brief True if user requested to rewind simulation.
      private: bool requestedRewind{false};

      /// \brief If user asks to seek to a specific sim time, this holds the
      /// time.s A negative value means there's no request from the user.
      private: std::chrono::steady_clock::duration requestedSeek{-1};

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

      friend class LevelManager;
    };
    }
  }
}
#endif
