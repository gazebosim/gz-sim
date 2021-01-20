/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SYSTEMS_USERCOMMANDS_HH_
#define IGNITION_GAZEBO_SYSTEMS_USERCOMMANDS_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class UserCommandsPrivate;

  /// \brief This system provides an Ignition Transport interface to execute
  /// commands while simulation is running.
  ///
  /// \todo(louise) In the future, an interface undo/redo commands will also
  /// be provided.
  ///
  /// # Spawn entity
  ///
  /// * **Service**: `/world/<world name>/create`
  /// * **Request type*: ignition.msgs.EntityFactory
  /// * **Response type*: ignition.msgs.Boolean
  ///
  /// # Spawn multiple entities
  ///
  /// This service can spawn multiple entities in the same iteration,
  /// thereby eliminating simulation steps between entity spawn times.
  ///
  /// * **Service**: `/world/<world name>/create_multiple`
  /// * **Request type*: ignition.msgs.EntityFactory_V
  /// * **Response type*: ignition.msgs.Boolean
  ///
  /// Try some examples described on examples/worlds/empty.sdf
  class UserCommands:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    /// \brief Constructor
    public: explicit UserCommands();

    /// \brief Destructor
    public: ~UserCommands() final;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// \brief All received commands are queued in order of reception and
    /// executed in order during PreUpdate.
    /// \param[in] _info Contains information about the current simulation
    /// iteration.
    /// \param[in] _ecm The entity component manager.
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<UserCommandsPrivate> dataPtr;
  };
  }
}
}
}
#endif
