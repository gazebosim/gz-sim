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
#ifndef IGNITION_GAZEBO_SERVER_HH_
#define IGNITION_GAZEBO_SERVER_HH_

#include <cstdint>
#include <memory>
#include <string>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/SystemPluginPtr.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forware declarations
    class ServerPrivate;

    /// \class Server Server.hh ignition/gazebo/Server.hh
    /// \brief The server instantiates and controls simulation.
    ///
    /// ## Example Usage
    ///
    /// A basic simulation server can be instantiated and run using
    ///
    /// ```
    /// ignition::gazebo::Server server;
    /// server.Run();
    /// ```
    ///
    /// An SDF File can be passed into the server via a ServerConfig object.
    /// The server will parse the SDF file and create entities for the
    /// elements contained in the file.
    ///
    /// ```
    /// ignition::gazebo::ServerConfig config;
    /// config.SetSdfFile("path_to_file.sdf");
    /// ignition::gazebo::Server server(config);
    /// server.Run();
    /// ```
    ///
    /// The Run() function accepts a few arguments, one of which is whether
    /// simulation should start in a paused state. The default value of this
    /// argument is true, which starts simulation paused. This means that by
    /// default, running the server will cause systems to update but some
    /// systems may not update because paused == true. For example,
    /// a physics system will not update its state when paused is
    /// true. So, while a Server can be Running, simulation itself can be
    /// paused.
    ///
    /// Simulation is paused by default because a common use case is to load
    /// a world from the command line. If simulation starts running, the
    /// GUI client may miss the first few simulation iterations.
    ///
    /// ## Services
    ///
    /// The following are services provided by the Server.
    /// The <world_name> in the service list is the name of the
    /// simulated world.
    ///
    /// List syntax: *service_name(request_msg_type) : response_msg_type*
    ///
    ///   1. /world/<world_name>/scene/info(none) : ignition::msgs::Scene
    ///     + Returns the current scene information.
    ///
    ///   2. /gazebo/get_resource_paths : ignition::msgs::StringMsg_V
    ///     + Get list of resource paths.
    ///
    ///   3. /gazebo/add_resource_paths : ignition::msgs::Empty
    ///     + Add new resource paths.
    ///
    /// ## Topics
    ///
    /// The following are topics provided by the Server.
    /// The <world_name> in the service list is the name of the
    /// simulated world.
    ///
    /// List syntax: *topic_name : published_msg_type*
    ///
    /// 1. /world/<world_name>/clock : ignition::msgs::Clock
    ///
    /// 2. /world/<world_name>/stats : ignition::msgs::WorldStatistics
    ///   + This topic is throttled to 5Hz.
    ///
    /// 3. /gazebo/resource_paths : ignition::msgs::StringMsg_V
    ///   + Updated list of resource paths.
    ///
    class IGNITION_GAZEBO_VISIBLE Server
    {
      /// \brief Construct the server using the parameters specified in a
      /// ServerConfig.
      /// \param[in] _config Server configuration parameters. If this
      /// parameter is omitted, then an empty world is loaded.
      public: explicit Server(const ServerConfig &_config = ServerConfig());

      /// \brief Destructor
      public: ~Server();

      /// \brief Set the update period. The update period is the wall-clock time
      /// between ECS updates.
      /// Note that this is different from the simulation update rate. ECS
      /// systems will be updated even while sim time is paused.
      /// \param[in] _updatePeriod Duration between updates.
      /// \param[in] _worldIndex Index of the world to query.
      public: void SetUpdatePeriod(
                  const std::chrono::steady_clock::duration &_updatePeriod,
                  const unsigned int _worldIndex = 0);

      /// \brief Run the server. By default this is a non-blocking call,
      /// which means the server runs simulation in a separate thread. Pass
      /// in true to the _blocking argument to run the server in the current
      /// thread.
      /// \param[in] _blocking False to run the server in a new thread. True
      /// to run the server in the current thread.
      /// \param[in] _iterations Number of steps to perform. A value of
      /// zero will run indefinitely.
      /// \param[in] _paused True to start simulation in a paused state,
      /// false, to start simulation unpaused.
      /// \return In non-blocking mode, the return value is true if a thread
      /// was successfully created. In blocking mode, true will be returned
      /// if the Server ran for the specified number of iterations or was
      /// terminated. False will always be returned if signal handlers could
      /// not be initialized, and if the server is already running.
      public: bool Run(const bool _blocking = false,
                       const uint64_t _iterations = 0,
                       const bool _paused = true);

      /// \brief Get whether the server is running. The server can have zero
      /// or more simulation worlds, each of which may or may not be
      /// running. See Running(const unsigned int) to get the running status
      /// of a world.
      /// \param[in] _worldIndex Index of the world to query.
      /// \return True if the server is running, or std::nullopt
      ///  if _worldIndex is invalid.
      public: bool Running() const;

      /// \brief Get whether a world simulation instance is running. When
      /// running is true, then systems are being updated but simulation may
      /// or may not be stepping forward. Check the value of Paused() to
      /// determine if a world simulation instance is stepping forward.
      /// If Paused() returns true, then simulation is not stepping foward.
      /// \param[in] _worldIndex Index of the world to query.
      /// \return True if the server is running, or std::nullopt
      ///  if _worldIndex is invalid.
      public: std::optional<bool> Running(const unsigned int _worldIndex) const;

      /// \brief Set whether a world simulation instance is paused.
      /// When paused is true, then simulation for the world is not stepping
      /// forward.
      /// \param[in] _paused True to pause the world, false to unpause.
      /// \param[in] _worldIndex Index of the world to query.
      /// \return True if the world referenced by _worldIndex exists, false
      /// otherwise.
      public: bool SetPaused(const bool _paused,
                  const unsigned int _worldIndex = 0) const;

      /// \brief Get whether a world simulation instance is paused.
      /// When paused is true, then simulation for the world is not stepping
      /// forward.
      /// \param[in] _worldIndex Index of the world to query.
      /// \return True if the world simulation instance is paused, false if
      /// stepping forward, or std::nullopt if _worldIndex is invalid.
      public: std::optional<bool> Paused(
                  const unsigned int _worldIndex = 0) const;

      /// \brief Get the number of iterations the server has executed.
      /// \param[in] _worldIndex Index of the world to query.
      /// \return The current iteration count,
      /// or std::nullopt if _worldIndex is invalid.
      public: std::optional<uint64_t> IterationCount(
                  const unsigned int _worldIndex = 0) const;

      /// \brief Get the number of entities on the server.
      /// \param[in] _worldIndex Index of the world to query.
      /// \return Entity count, or std::nullopt if _worldIndex is invalid.
      public: std::optional<size_t> EntityCount(
                  const unsigned int _worldIndex = 0) const;

      /// \brief Get the number of systems on the server.
      /// \param[in] _worldIndex Index of the world to query.
      /// \return System count, or std::nullopt if _worldIndex is invalid.
      public: std::optional<size_t> SystemCount(
                  const unsigned int _worldIndex = 0) const;

      /// \brief Add a System to the server. The server must not be running when
      /// calling this.
      /// \param[in] _system system to be added
      /// \param[in] _worldIndex Index of the world to query.
      /// \return Whether the system was added successfully, or std::nullopt
      /// if _worldIndex is invalid.
      public: std::optional<bool> AddSystem(
                  const SystemPluginPtr &_system,
                  const unsigned int _worldIndex = 0);

      /// \brief Get an Entity based on a name.
      /// \details If multiple entities with the same name exist, the first
      /// entity found will be returned.
      /// \param [in] _name Name of the entity to get from the specified
      /// world.
      /// \param[in] _worldIndex Index of the world to query.
      /// \return The entity, or std::nullopt if the entity or world
      /// doesn't exist.
      public: std::optional<Entity> EntityByName(const std::string &_name,
                  const unsigned int _worldIndex = 0) const;

      /// \brief Return true if the specified world has an entity with the
      /// provided name.
      /// \param[in] _name Name of the entity.
      /// \param[in] _worldIndex Index of the world.
      /// \return True if the _worldIndex is valid and the
      /// entity exists in the world.
      public: bool HasEntity(const std::string &_name,
                             const unsigned int _worldIndex = 0) const;

      /// \brief Return true if the specified world has an entity with the
      /// provided name and the entity was queued for deletion. Note that
      /// the entity is not removed immediately. Entity deletion happens at
      /// the end of the next (or current depending on when this function is
      /// called) simulation step.
      /// \details If multiple entities with the same name exist, only the
      /// first entity found will be deleted.
      /// \param[in] _name Name of the entity to delete.
      /// \param[in] _recursive Whether to recursively delete all child
      /// entities. True by default.
      /// \param[in] _worldIndex Index of the world.
      /// \return True if the entity exists in the world and it was queued
      /// for deletion.
      public: bool RequestRemoveEntity(const std::string &_name,
                                      bool _recursive = true,
                                      const unsigned int _worldIndex = 0);

      /// \brief Return true if the specified world has an entity with the
      /// provided id and the entity was queued for deletion. Note that
      /// the entity is not removed immediately. Entity deletion happens at
      /// the end of the next (or current depending on when this function is
      /// called) simulation step.
      /// \param[in] _entity The entity to delete.
      /// \param[in] _recursive Whether to recursively delete all child
      /// entities. True by default.
      /// \param[in] _worldIndex Index of the world.
      /// \return True if the entity exists in the world and it was queued
      /// for deletion.
      public: bool RequestRemoveEntity(const Entity _entity,
                                      bool _recursive = true,
                                      const unsigned int _worldIndex = 0);

      /// \brief Private data
      private: std::unique_ptr<ServerPrivate> dataPtr;
    };
    }
  }
}

#endif
