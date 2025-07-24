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
#ifndef GZ_SIM_SERVERPRIVATE_HH_
#define GZ_SIM_SERVERPRIVATE_HH_

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/server_control.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sdf/Root.hh>

#include <gz/common/SignalHandler.hh>
#include <gz/common/URI.hh>
#include <gz/common/WorkerPool.hh>

#include <gz/fuel_tools/FuelClient.hh>

#include <gz/transport/Node.hh>

#include "gz/sim/config.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/SystemLoader.hh"

using namespace std::chrono_literals;

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    class SimulationRunner;

    // Private data for Server
    class GZ_SIM_HIDDEN ServerPrivate
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

      /// \brief Helper function that loads an SDF root object based on
      /// values in a ServerConfig object.
      /// \param[in] _config Server config to read from.
      /// \return Set of SDF errors.
      public: sdf::Errors LoadSdfRootHelper(const ServerConfig &_config);

      /// \brief Signal handler callback
      /// \param[in] _sig The signal number
      private: void OnSignal(int _sig);

      /// \brief Callback for worlds service.
      /// \param[out] _res Response containing the names of all the worlds.
      /// \return True if successful.
      private: bool WorldsService(gz::msgs::StringMsg_V &_res);

      /// \brief Callback for add resource paths service.
      /// \param[out] _req Request containing the paths to be added.
      private: void AddResourcePathsService(
          const gz::msgs::StringMsg_V &_req);

      /// \brief Callback for get resource paths service.
      /// \param[out] _res Response filled with all current paths.
      /// \return True if successful.
      private: bool ResourcePathsService(gz::msgs::StringMsg_V &_res);

      /// \brief Callback for a resource path resolve service. This service
      /// will return the full path to a provided resource's URI. An empty
      /// string and return value of false will be used if the resource could
      /// not be found.
      ///
      /// Fuel will be checked and then the GZ_SIM_RESOURCE_PATH environment
      /// variable paths. This service will not check for files relative to
      /// working directory of the Gazebo server.
      ///
      /// \param[in] _req Request filled with a resource URI to resolve.
      /// Example values could be:
      ///   * https://URI_TO_A_FUEL_RESOURCE
      ///   * model://MODLE_NAME/meshes/MESH_NAME
      ///   * file://PATH/TO/FILE
      ///
      /// \param[out] _res Response filled with the resolved path, or empty
      /// if the resource could not be found.
      /// \return True if successful, false otherwise.
      private: bool ResourcePathsResolveService(
                   const msgs::StringMsg &_req,
                   msgs::StringMsg &_res);

      /// \brief Callback for server control service.
      /// \param[out] _req The control request.
      /// \param[out] _res Whether the request was successfully fullfilled.
      /// \return True if successful.
      private: bool ServerControlService(
        const gz::msgs::ServerControl &_req, msgs::Boolean &_res);

      /// \brief A pool of worker threads.
      /// \note We use optional here since most of the time, there will be a
      /// single simulation runner and a workerpool is not needed. We will
      /// initialize the workerpool as necessary later on.
      public: std::optional<common::WorkerPool> workerPool;

      /// \brief All the simulation runners.
      public: std::vector<std::unique_ptr<SimulationRunner>> simRunners;

      /// \brief Mutex to protect the Run operation.
      public: std::mutex runMutex;

      /// \brief This is used to indicate that Run has been called, and the
      /// server is in the run state.
      public: std::atomic<bool> running{false};

      /// \brief Thread that executes systems.
      public: std::thread runThread;

      /// \brief Thread that shuts down the system.
      public: std::shared_ptr<std::thread> stopThread;

      /// \brief Our signal handler.
      public: gz::common::SignalHandler sigHandler;

      /// \brief Our system loader.
      public: SystemLoaderPtr systemLoader;

      /// \brief The SDF root object.
      /// This keeps the SDF object in memory so that other classes can keep a
      /// pointer to child nodes of the root
      public: sdf::Root sdfRoot;

      /// \brief The server configuration.
      public: ServerConfig config;

      /// \brief Client used to download resources from Gazebo Fuel.
      public: std::unique_ptr<fuel_tools::FuelClient> fuelClient = nullptr;

      /// \brief Map from file paths to fuel URIs. This is set and updated by
      /// Server. It is used in the SDFormat world generator when saving worlds
      public: std::unordered_map<std::string, std::string> fuelUriMap;

      /// \brief Gazebo classic material URI string
      /// A URI matching this string indicates that it is a gazebo classic
      /// material.
      public: static const char kClassicMaterialScriptUri[];

      /// \brief List of names for all worlds loaded in this server.
      private: std::vector<std::string> worldNames;

      /// \brief Protects worldNames.
      private: std::mutex worldsMutex;

      /// \brief Node for transport.
      private: transport::Node node;

      /// \brief Publisher of resrouce paths.
      private: transport::Node::Publisher pathPub;
    };
    }
  }
}
#endif
