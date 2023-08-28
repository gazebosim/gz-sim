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

#include "PythonSystemLoader.hh"

#include <pybind11/stl.h>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <memory>
#include <sdf/Element.hh>
#include <string>
#include <utility>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/SystemLoader.hh"

namespace py = pybind11;


using namespace gz;
using namespace sim;
using namespace systems;

PythonSystemLoader::~PythonSystemLoader()
{
  if (this->pythonSystem)
  {
    if (py::hasattr(this->pythonSystem, "shutdown"))
    {
      this->pythonSystem.attr("shutdown")();
    }
  }
}

void PythonSystemLoader::Configure(
    const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  auto [moduleName, hasModule] = _sdf->Get<std::string>("module_name", "");
  if (!hasModule)
  {
    gzerr << "PythonSystemLoader missing required argument <module_name>. "
          << "Failed to initialize." << std::endl;
    return;
  }

  // Load the `gz.sim` and sdformat modules to register all pybind bindings
  // necessary for System interface functions
  const auto gzSimModule =
      std::string("gz.sim") + std::to_string(GZ_SIM_MAJOR_VERSION);
  const auto sdformatModule =
      std::string("sdformat") + std::to_string(SDF_MAJOR_VERSION);
  py::module_ sysModule;
  try
  {
    py::module::import(gzSimModule.c_str());
    py::module::import(sdformatModule.c_str());
    sysModule = py::module_::import("sys");
  }
  catch (const pybind11::error_already_set &_err)
  {
    gzerr << "Error while loading required modules:\n"
          << _err.what() << std::endl;
    return;
  }

  // Add GZ_SIM_SYSTEM_PLUGIN_PATH and other default plugin
  // locations to PYTHONPATH
  try
  {
    SystemLoader systemLoader;
    for (const auto &pluginPath : systemLoader.PluginPaths())
    {
      sysModule.attr("path").attr("append")(pluginPath);
    }
  }
  catch (const std::runtime_error &_err)
  {
    gzerr << _err.what() << std::endl;
    return;
  }

  try
  {
    this->pythonModule = py::module_::import(moduleName.c_str());
  }
  catch (const pybind11::error_already_set &_err)
  {
    gzerr << "Error while loading module " << moduleName << "\n"
          << _err.what() << "\n";

    gzerr << "Searched in:\n";
    for (const auto &path : sysModule.attr("path"))
    {
      gzerr << "  \"" << path << "\"\n";
    }
    return;
  }

  try
  {
    py::object getSystem = this->pythonModule.attr("get_system");
    if (!getSystem)
    {
      gzerr << "Could not call 'get_system' on the module " << moduleName
            << "\n";
      return;
    }
    this->pythonSystem = getSystem();
  }
  catch (const pybind11::error_already_set &_err)
  {
    gzerr << _err.what() << std::endl;
    return;
  }
  if (py::hasattr(this->pythonSystem, "configure"))
  {
    try
    {
      // _ecm and _eventMgr are passed in as pointers so pybind11 would use the
      // write `return_value_policy`. Otherwise, it would use
      // `return_value_policy = copy`.
      this->pythonSystem.attr("configure")(_entity, _sdf, &_ecm, &_eventMgr);
    }
    catch (const pybind11::error_already_set &_err)
    {
      gzerr << _err.what() << std::endl;
      return;
    }
  }
  // TODO(azeey) Add support for ConfigureParameters
  if (py::hasattr(this->pythonSystem, "pre_update"))
  {
    this->preUpdateMethod = this->pythonSystem.attr("pre_update");
  }
  if (py::hasattr(this->pythonSystem, "update"))
  {
    this->updateMethod = this->pythonSystem.attr("update");
  }
  if (py::hasattr(this->pythonSystem, "post_update"))
  {
    this->postUpdateMethod = this->pythonSystem.attr("post_update");
  }
  if (py::hasattr(this->pythonSystem, "reset"))
  {
    this->resetMethod = this->pythonSystem.attr("reset");
  }
  this->validConfig = true;
}

//////////////////////////////////////////////////
template <typename... Args>
void PythonSystemLoader::CallPythonMethod(py::object _method, Args &&..._args)
{
  if (!this->validConfig)
  {
    return;
  }

  if (_method)
  {
    try
    {
      _method(std::forward<Args>(_args)...);
    }
    catch (const pybind11::error_already_set &_err)
    {
      gzerr << _err.what() << std::endl;
      this->validConfig = false;
      return;
    }
  }
}

//////////////////////////////////////////////////
void PythonSystemLoader::PreUpdate(const UpdateInfo &_info,
                                   EntityComponentManager &_ecm)
{
  CallPythonMethod(this->preUpdateMethod, _info, &_ecm);
}

//////////////////////////////////////////////////
void PythonSystemLoader::Update(const UpdateInfo &_info,
                                EntityComponentManager &_ecm)
{
  CallPythonMethod(this->updateMethod, _info, &_ecm);
}

//////////////////////////////////////////////////
void PythonSystemLoader::PostUpdate(const UpdateInfo &_info,
                                    const EntityComponentManager &_ecm)
{
  py::gil_scoped_acquire gil;
  CallPythonMethod(this->postUpdateMethod, _info, &_ecm);
}
//////////////////////////////////////////////////
void PythonSystemLoader::Reset(const UpdateInfo &_info,
                               EntityComponentManager &_ecm)
{
  CallPythonMethod(this->resetMethod, _info, &_ecm);
}

GZ_ADD_PLUGIN(PythonSystemLoader, System, ISystemConfigure, ISystemPreUpdate,
              ISystemUpdate, ISystemPostUpdate, ISystemReset)
GZ_ADD_PLUGIN_ALIAS(PythonSystemLoader, "gz::sim::systems::PythonSystemLoader")
