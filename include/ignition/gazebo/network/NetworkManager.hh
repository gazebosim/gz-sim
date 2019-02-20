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
#ifndef IGNITION_GAZEBO_NETWORK_NETWORKMANAGER_HH_
#define IGNITION_GAZEBO_NETWORK_NETWORKMANAGER_HH_

#include <chrono>
#include <memory>
#include <string>

#include <ignition/transport/NodeOptions.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/network/NetworkConfig.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations
    class NetworkManagerPrivate;

    /// \class NetworkManager NetworkManager.hh
    ///   ignition/gazebo/NetworkManager.hh
    /// \brief The NetworkManager provides a common interface to derived
    /// objects that control the flow of information in the distributed
    /// simulation environment.
    class IGNITION_GAZEBO_VISIBLE NetworkManager
    {
      /// \brief Convenience type alias for NodeOptions
      public: using NodeOptions = ignition::transport::NodeOptions;

      /// \brief Create a class derived from NetworkManager based on
      /// a given configuration
      /// \param[in] _eventMgr EventManager to associate with this
      /// NetworkManager
      /// \param[in] _config configuration object to use. If not given,
      ///   configuration will be populated from environment variables.
      /// \param[in] _options Advanced options for underlying ign-transport
      public: static std::unique_ptr<NetworkManager> Create(
                  EventManager *_eventMgr = nullptr,
                  const NetworkConfig &_config = NetworkConfig::FromEnv(),
                  const NodeOptions &_options = NodeOptions());

      /// \brief Constructor with configuration passed in.
      /// \param[in] _eventMgr EventManager to associate with this
      /// NetworkManager
      /// \param[in] _config configuration object to use.
      /// \param[in] _options Advanced options for underlying ign-transport
      protected: explicit NetworkManager(EventManager *_eventMgr,
                                         const NetworkConfig &_config,
                                         const NodeOptions &_options);

      /// \brief Destructor.
      public: virtual ~NetworkManager() = 0;

      /// \brief Indicate if NetworkManager configuration is in a valid state.
      public: virtual bool Valid() const = 0;

      public: virtual void Initialize() = 0;

      /// \brief Indicate if NetworkManager is ready to execute.
      public: virtual bool Ready() const = 0;

      /// \brief Populate simulation step data
      /// This method is called at the beginning of a simulation iteration.
      /// It will populate the iteration, stepSize and simTime arguments with
      /// their appropriate values for the simuation iteration.
      /// \param[inout] _iteration current simulation iteration
      /// \param[inout] _stepSize current simulation step size
      /// \param[inout] _stepSize current simulation time
      public: virtual bool Step(
                  uint64_t &_iteration,
                  std::chrono::steady_clock::duration &_stepSize,
                  std::chrono::steady_clock::duration &_simTime) = 0;

      /// \brief Acknowledge completion of a step
      /// This method is called at the end of a simulation iteration to provide
      /// a syncronization point for all distributed simulation runner
      /// instances.
      public: virtual bool StepAck(uint64_t _iteration) = 0;

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

      /// \brief Private data
      protected: std::unique_ptr<NetworkManagerPrivate> dataPtr;
    };
    }
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_NETWORKMANAGER_HH_
