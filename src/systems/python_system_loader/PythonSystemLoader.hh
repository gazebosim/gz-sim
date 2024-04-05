/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#ifndef GZ_SIM_SYSTEMS_PYTHONSYSTEMLOADER_HH_
#define GZ_SIM_SYSTEMS_PYTHONSYSTEMLOADER_HH_

#include <pybind11/embed.h>

#include <memory>
#include <sdf/Element.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/config.hh"
#include "gz/sim/python-system-loader-system/Export.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
/// \brief Allows systems to be written in Python.
///
/// The convention for a system written in Python supported by the
/// `PythonSystemLoader` is that it's a Python module providing a `get_system`
/// function which itself returns an instance of a class that implements the
/// various interfaces in gz::sim::System. The spelling of the interfaces
/// conforms to Python code style guides, so the following name mapping applies
///
/// * `configure` : Corresponds to System::ISystemConfigure::Configure
/// * `pre_update` : Corresponds to System::ISystemPreUpdate::PreUpdate
/// * `update` : Corresponds to System::ISystemUpdate::Update
/// * `post_update` : Corresponds to System::ISystemPostUpdate::PostUpdate
/// * `reset` : Corresponds to System::ISystemReset::Reset
///
/// It is not necessary to implement all the interfaces. PythonSystemLoader will
/// check if the corresponding method is implemented in the Python system and
/// skip it if it's not found.
///
/// See `examples/scripts/python_api/systems/test_system.py` for an example
///
/// ## System Parameters
/// * `<module_name>` : Name of python module to be loaded. The search path
///                   includes `GZ_SIM_SYSTEM_PLUGIN_PATH` as well as
///                   `PYTHONPATH`.
///
/// The contents of the `<plugin>` tag will be available in the configure method
/// of the Python system
// TODO(azeey) Add ParameterConfigure
class GZ_SIM_PYTHON_SYSTEM_LOADER_SYSTEM_HIDDEN PythonSystemLoader final
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemUpdate,
      public ISystemPostUpdate,
      public ISystemReset
{
  public: ~PythonSystemLoader() final;
  // Documentation inherited
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) final;

  // Documentation inherited
  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) final;

  // Documentation inherited
  public: void Update(const UpdateInfo &_info,
                      EntityComponentManager &_ecm) final;

  // Documentation inherited
  public: void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm) final;

  // Documentation inherited
  public: void Reset(const UpdateInfo &_info,
                     EntityComponentManager &_ecm) final;

  /// \brief Function that calls each of the python equivalents of Configure,
  /// PreUpdate, etc.
  private: template <typename ...Args>
  void CallPythonMethod(pybind11::object _method, Args&&...);

  /// \brief Whether we have a valid configuration after Configure has run. This
  /// includes checking if the Python module is found and that the system is
  /// instantiated. This is set to false in the *Update/Reset calls if an
  /// exception is thrown by the Python module.
  private: bool validConfig{false};
  /// \brief Represents the imported Python module
  private: pybind11::module_ pythonModule;
  /// \brief Represents the instantiated Python System
  private: pybind11::object pythonSystem;
  /// \brief Pointer to the PreUpdate method inside the Python System
  private: pybind11::object preUpdateMethod;
  /// \brief Pointer to the Update method inside the Python System
  private: pybind11::object updateMethod;
  /// \brief Pointer to the PostUpdate method inside the Python System
  private: pybind11::object postUpdateMethod;
  /// \brief Pointer to the Reset method inside the Python System
  private: pybind11::object resetMethod;
};
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif /* end of include guard: GZ_SIM_SYSTEMS_PYTHONSYSTEMLOADER_HH_ */
