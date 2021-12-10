/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_PYTHON__HELPER_SYSTEM_HH_
#define IGNITION_GAZEBO_PYTHON__HELPER_SYSTEM_HH_

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include <memory>
#include <string>
#include <functional>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"

#include "Destroyable.hh"
#include "EntityComponentManager.hh"

namespace ignition
{
namespace gazebo
{
namespace python
{
  class HelperFixturePrivate;

  class HelperFixture :
    public ignition::gazebo::python::Destroyable,
    public std::enable_shared_from_this<HelperFixture>
  {
    /// \brief Constructor
    /// \param[in] _path Path to SDF file.
    public: explicit HelperFixture(const std::string &_path);

    /// \brief Constructor
    /// \param[in] _config Server config file
    public: explicit HelperFixture(const ServerConfig &_config);

    /// \brief Destructor
    public: virtual ~HelperFixture();

    /// Force an early destruction of this object
    void
    Destroy() override;

    /// \brief Wrapper around a system's pre-update callback
    /// \param[in] _cb Function to be called every pre-update
    /// The _entity and _sdf will correspond to the world entity.
    /// \return Reference to self.
    public: HelperFixture &OnConfigure(std::function<void(
        const Entity &_entity,
        ignition::gazebo::python::EntityComponentManager &_ecm)> _cb);

    /// \brief Wrapper around a system's pre-update callback
    /// \param[in] _cb Function to be called every pre-update
    /// \return Reference to self.
    public: HelperFixture &OnPreUpdate(std::function<void(
        const UpdateInfo &,
        ignition::gazebo::python::EntityComponentManager &)> _cb);

    /// \brief Wrapper around a system's update callback
    /// \param[in] _cb Function to be called every update
    /// \return Reference to self.
    public: HelperFixture &OnUpdate(std::function<void(
        const UpdateInfo &,
        ignition::gazebo::python::EntityComponentManager &)> _cb);

    /// \brief Wrapper around a system's post-update callback
    /// \param[in] _cb Function to be called every post-update
    /// \return Reference to self.
    public: HelperFixture &OnPostUpdate(std::function<void(
        const UpdateInfo &,
        const ignition::gazebo::python::EntityComponentManager &)> _cb);

    /// \brief Finalize all the functions and add fixture to server.
    /// Finalize must be called before running the server, otherwise none of
    /// the `On*` functions will be called.
    /// The `OnConfigure` callback is called immediately on finalize.
    public: HelperFixture &Finalize();

    /// \brief Get pointer to underlying server.
    public: std::shared_ptr<ignition::gazebo::Server> Server() const;

    /// \internal
    /// \brief Pointer to private data.
    // TODO(ahcorde) Use IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr) when porting to v6
    private: std::shared_ptr<HelperFixturePrivate> dataPtr;
  };

/// \brief System that is inserted into the simulation loop to observe the ECM.
class HelperSystem :
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate,
  public ISystemUpdate,
  public ISystemPostUpdate
{
  // Documentation inherited
  public: void Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr) override;

  // Documentation inherited
  public: void PreUpdate(const UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void Update(const UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void PostUpdate(const UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm) override;

  /// \brief Function to call every time  we configure a world
  public: std::function<void(const Entity &_entity,
    ignition::gazebo::python::EntityComponentManager &_ecm)>
      configureCallback_internal;

  /// \brief Function to call every pre-update
  public: std::function<void(const UpdateInfo &,
    ignition::gazebo::python::EntityComponentManager &)>
      preUpdateCallback_internal;

  /// \brief Function to call every update
  public: std::function<void(const UpdateInfo &,
    ignition::gazebo::python::EntityComponentManager &)>
      updateCallback_internal;

  /// \brief Function to call every post-update
  public: std::function<void(const UpdateInfo &,
      const ignition::gazebo::python::EntityComponentManager &)>
        postUpdateCallback_internal;
};

/// Define a pybind11 wrapper for an ignition::gazebo::HelperFixture
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
defineGazeboHelperFixture(pybind11::object module);
}
}
}

#endif  // IGNITION_GAZEBO_PYTHON__HELPER_SYSTEM_HH_
