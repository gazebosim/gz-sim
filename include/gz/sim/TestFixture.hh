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
#ifndef GZ_SIM_TESTFIXTURE_HH_
#define GZ_SIM_TESTFIXTURE_HH_

#include <memory>
#include <string>

#include <gz/utils/ImplPtr.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
//
/// \brief Helper class to write automated tests. It provides a convenient API
/// to load a world file, step simulation and check entities and components.
///
/// ## Usage
///
/// ```
/// // Load a world with a fixture
/// gz::sim::TestFixture fixture("path_to.sdf");
///
/// // Register callbacks, for example:
/// fixture.OnPostUpdate([&](const sim::UpdateInfo &,
///   const sim::EntityComponentManager &_ecm)
///   {
///     // Add expectations here
///   }).Finalize();
/// // Be sure to call finalize before running the server.
///
/// // Run the server
/// fixture.Server()->Run(true, 1000, false);
/// ```
///
class GZ_SIM_VISIBLE TestFixture
{
  /// \brief Constructor
  /// \param[in] _path Path to SDF file.
  public: explicit TestFixture(const std::string &_path);

  /// \brief Constructor
  /// \param[in] _config Server config file
  public: explicit TestFixture(const ServerConfig &_config);

  /// \brief Destructor
  public: virtual ~TestFixture() = default;

  /// \brief Wrapper around a system's pre-update callback
  /// \param[in] _cb Function to be called every pre-update
  /// The _entity and _sdf will correspond to the world entity.
  /// \return Reference to self.
  public: TestFixture &OnConfigure(std::function<void(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &_eventMgr)> _cb);

  /// \brief Wrapper around a system's pre-update callback
  /// \param[in] _cb Function to be called every pre-update
  /// \return Reference to self.
  public: TestFixture &OnPreUpdate(std::function<void(
      const UpdateInfo &, EntityComponentManager &)> _cb);

  /// \brief Wrapper around a system's update callback
  /// \param[in] _cb Function to be called every update
  /// \return Reference to self.
  public: TestFixture &OnUpdate(std::function<void(
      const UpdateInfo &, EntityComponentManager &)> _cb);

  /// \brief Wrapper around a system's post-update callback
  /// \param[in] _cb Function to be called every post-update
  /// \return Reference to self.
  public: TestFixture &OnPostUpdate(std::function<void(
      const UpdateInfo &, const EntityComponentManager &)> _cb);

  /// \brief Finalize all the functions and add fixture to server.
  /// Finalize must be called before running the server, otherwise none of the
  /// `On*` functions will be called.
  /// The `OnConfigure` callback is called immediately on finalize.
  public: TestFixture &Finalize();

  /// \brief Get pointer to underlying server.
  public: std::shared_ptr<sim::Server> Server() const;

  /// \internal
  /// \brief Pointer to private data.
  private: GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};
}
}
}
#endif
