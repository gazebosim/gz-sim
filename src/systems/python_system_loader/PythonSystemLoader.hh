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

#include <gz/sim/System.hh>
#include <sdf/Element.hh>

#include <gz/sim/config.hh>
#include <gz/sim/python-system-loader-system/Export.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
// TODO(azeey) Add ParameterConfigure
class GZ_SIM_PYTHON_SYSTEM_LOADER_SYSTEM_HIDDEN PythonSystemLoader
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemUpdate,
      public ISystemPostUpdate,
      public ISystemReset
{
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

  private: template <typename ...Args>
  void CallPythonMethod(pybind11::object _method, Args&&...);

  private: bool validConfig{false};
  private: pybind11::module_ pythonModule;
  private: pybind11::object pythonSystem;
  private: pybind11::object preUpdateMethod;
  private: pybind11::object updateMethod;
  private: pybind11::object postUpdateMethod;
  private: pybind11::object resetMethod;
};
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif /* end of include guard: GZ_SIM_SYSTEMS_PYTHONSYSTEMLOADER_HH_ */
