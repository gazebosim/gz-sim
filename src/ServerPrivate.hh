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

#include <ignition/msgs/stringmsg_v.pb.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sdf/Root.hh>

#include <ignition/common/SignalHandler.hh>
#include <ignition/common/URI.hh>
#include <ignition/common/WorkerPool.hh>

#include <ignition/fuel_tools/FuelClient.hh>

#include <ignition/transport/Node.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/ServerConfig.hh"
#include "ignition/gazebo/SystemLoader.hh"

using namespace std::chrono_literals;

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    class SimulationRunner;

    // Private data for Server
    class IGNITION_GAZEBO_HIDDEN ServerPrivate
    {
      /// \brief Constructor
      public: ServerPrivate();

      /// \brief Destructor
      public: ~ServerPrivate();

      /// \brief Run the server, and all the simulation runners.
      /// \param[in] _iterations Number of iterations.
      /// \param[in] _cond Optional condition variable. This condition is
      /// notified when the server has started running.
      public: bool Run(const uint64_t _iterations,
                 std::optional<std::condition_variable *> _cond = std::nullopt);

      /// \brief Add logging record plugin.
      /// \param[in] _config Server configuration parameters.
      public: void AddRecordPlugin(const ServerConfig &_config);

      /// \brief Create all entities that exist in the sdf::Root object.
      public: void CreateEntities();

      /// \brief Stop server.
      public: void Stop();

      /// \brief Sets up all transport.
      /// \detail Future publishers and subscribers should be created within
      /// this function.
      public: void SetupTransport();

      /// \brief Fetch a resource from Fuel using fuel-tools.
      /// \param[in] _uri The resource URI to fetch.
      /// \return Path to the downloaded resource, empty on error.
      public: std::string FetchResource(const std::string &_uri);

      /// \brief Fetch a resource from Fuel using fuel-tools.
      /// \param[in] _uri The resource URI to fetch.
      /// \return Path to the downloaded resource, empty on error.
      public: std::string FetchResourceUri(const common::URI &_uri);

      /// \brief Signal handler callback
      /// \param[in] _sig The signal number
      private: void OnSignal(int _sig);

      /// \brief Callback for worlds service.
      /// \param[out] _res Response containing the names of all the worlds.
      /// \return True if successful.
      private: bool WorldsService(ignition::msgs::StringMsg_V &_res);

      /// \brief A pool of worker threads.
      public: common::WorkerPool workerPool{2};

      /// \brief All the simulation runners.
      public: std::vector<std::unique_ptr<SimulationRunner>> simRunners;

      /// \brief Mutex to protect the Run operation.
      public: std::mutex runMutex;

      /// \brief This is used to indicate that Run has been called, and the
      /// server is in the run state.
      public: std::atomic<bool> running{false};

      /// \brief Thread that executes systems.
      public: std::thread runThread;

      /// \brief Our signal handler.
      public: ignition::common::SignalHandler sigHandler;

      /// \brief Our system loader.
      public: SystemLoaderPtr systemLoader;

      /// \brief The SDF root object.
      /// This keeps the SDF object in memory so that other classes can keep a
      /// pointer to child nodes of the root
      public: sdf::Root sdfRoot;

      /// \brief The server configuration.
      public: ServerConfig config;

      /// \brief Client used to download resources from Ignition Fuel.
      public: std::unique_ptr<fuel_tools::FuelClient> fuelClient = nullptr;

      /// \brief Map from file paths to fuel URIs. This is set and updated by
      /// Server. It is used in the SDFormat world generator when saving worlds
      public: std::unordered_map<std::string, std::string> fuelUriMap;

      /// \brief List of names for all worlds loaded in this server.
      private: std::vector<std::string> worldNames;

      /// \brief Protects worldNames.
      private: std::mutex worldsMutex;

      /// \brief Node for transport.
      private: transport::Node node;
    };
    }
  }
}
#endif
