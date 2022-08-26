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
#ifndef GZ_SIM_NETWORK_NETWORKMANAGER_HH_
#define GZ_SIM_NETWORK_NETWORKMANAGER_HH_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <gz/transport/NodeOptions.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>
#include <gz/sim/EventManager.hh>

#include "NetworkConfig.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    // Forward declarations
    class NetworkManagerPrivate;

    /// \class NetworkManager NetworkManager.hh
    ///   gz/sim/NetworkManager.hh
    /// \brief The NetworkManager provides a common interface to derived
    /// objects that control the flow of information in the distributed
    /// simulation environment.
    class GZ_SIM_VISIBLE NetworkManager
    {
      /// \brief Convenience type alias for NodeOptions
      public: using NodeOptions = gz::transport::NodeOptions;

      /// \brief Create a class derived from NetworkManager based on
      /// a given configuration
      /// \param[in] _stepFunction The `SimulationRunner`'s `Step` function,
      /// so the network can perform actions before and after stepping.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _eventMgr EventManager to associate with this
      /// NetworkManager
      /// \param[in] _config configuration object to use. If not given,
      ///   configuration will be populated from environment variables.
      /// \param[in] _options Advanced options for underlying gz-transport
      /// \return A pointer to a network manager, or null if the network
      /// manager could not be created.
      public: static std::unique_ptr<NetworkManager> Create(
          const std::function<void(const UpdateInfo &_info)> &_stepFunction,
          EntityComponentManager &_ecm, EventManager *_eventMgr = nullptr,
          const NetworkConfig &_config = NetworkConfig(),
          const NodeOptions &_options = NodeOptions());

      /// \brief Constructor with configuration passed in.
      /// \param[in] _stepFunction The `SimulationRunner`'s `Step` function,
      /// so the network can perform actions before and after stepping.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _eventMgr EventManager to associate with this
      /// NetworkManager
      /// \param[in] _config configuration object to use.
      /// \param[in] _options Advanced options for underlying gz-transport
      protected: explicit NetworkManager(
          const std::function<void(const UpdateInfo &_info)> &_stepFunction,
          EntityComponentManager &_ecm, EventManager *_eventMgr,
          const NetworkConfig &_config,
          const NodeOptions &_options);

      /// \brief Destructor.
      public: virtual ~NetworkManager() = 0;

      /// \brief Indicate if NetworkManager is ready to initialize.
      /// \return True when NetworkManager is ready to be initialized and
      /// execute
      public: virtual bool Ready() const = 0;

      /// \brief Initialize communications with peers.
      /// The NetworkManager goes through two phases of initialization. The
      /// constructor creates the peer tracker and begins to discover peers
      /// in the network. Once the appropriate number of peers is discovered
      /// (based on participant role), then the NetworkManager will indicate
      /// that it is ready to initialize and execute via the `Ready`.
      ///
      /// The `Handshake` call will then set up any additional communications
      /// infrastructure required for distributed simulation to proceed.
      public: virtual void Handshake() = 0;

      /// \brief Get a unique namespace for this runner
      public: virtual std::string Namespace() const = 0;

      /// \brief Convenience method for retrieving role.
      public: NetworkRole Role() const;

      /// \brief Convenience method for retrieving primary role.
      public: bool IsPrimary() const;

      /// \brief Convenience method for retrieving secondary role.
      public: bool IsSecondary() const;

      /// \brief Convenience method for retrieving readonly role.
      public: bool IsReadOnly() const;

      /// \brief Get the manager's config.
      /// \return The manager's config.
      public: NetworkConfig Config() const;

      /// \brief Private data
      protected: std::unique_ptr<NetworkManagerPrivate> dataPtr;
    };
    }
  }  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_NETWORKMANAGER_HH_
