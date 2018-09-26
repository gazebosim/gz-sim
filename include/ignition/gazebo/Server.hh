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
    // ## Services
    //
    // The following are services provided by the Server.
    // List syntax: *topic_name(request_message) : response_message*
    //
    // 1. /ign/gazebo/scene(none) : ignition::msgs::Scene
    //   + Returns the current scene information.
    //
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

      /// \brief Get whether this server is running. When running is true,
      /// then simulation is stepping forward.
      /// \param[in] _worldIndex Index of the world to query.
      /// \return True if the server is running, or std::nullopt
      ///  if _worldIndex is invalid.
      public: std::optional<bool> Running(
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

          /// \brief Private data
      private: std::unique_ptr<ServerPrivate> dataPtr;
    };
    }
  }
}

#endif
