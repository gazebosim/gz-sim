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

#include <dlfcn.h>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/SystemLoader.hh>

namespace py = pybind11;

namespace gz::sim::systems
{
void PythonSystemLoader::Configure(
    const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &, EventManager &)
{
  auto [moduleName, hasModule] = _sdf->Get<std::string>("module_name", "");
  if (!hasModule)
  {
    gzerr << "PythonSystemLoader missing required argument <module_name>. "
          << "Failed to initialize." << std::endl;
    return;
  }
  // TODO (azeey) Where do we look for the python file? Do we need a separate
  // environment variable similar to GZ_SIM_SYSTEM_PLUGIN_PATH or can we use the
  // same variable? For now, only look for the file as an absolute path or
  // relative to CWD.
  // Alternatively, we can consider PYTHONPATH and the requirement would be that
  // we can just import the module.
  try
  {
    py::module_ sys = py::module_::import("sys");
    SystemLoader systemLoader;
    for (const auto &pluginPath : systemLoader.PluginPaths())
    {
      sys.attr("path").attr("append")(pluginPath);
    }

    gzdbg << "Searching for python module in:\n";
    auto searchPaths = sys.attr("path");
    // TODO (azeey) Improve formatting.
    gzdbg << py::str(searchPaths).cast<std::string>() << std::endl;
  }
  catch (const pybind11::error_already_set &_err)
  {
    gzerr << "Error while loading module 'sys'\n"
      << _err.what() << std::endl;
    return;
  }

  try
  {
    this->pythonModule = py::module_::import(moduleName.c_str());
  }
  catch (const pybind11::error_already_set &_err)
  {
    gzerr << "Error while loading module " << moduleName << "\n"
          << _err.what() << std::endl;
    return;
  }

  try
  {
    py::object getSystem = this->pythonModule.attr("get_system");
    if (!getSystem)
    {
      gzerr << "Could not call 'get_system' on the module" << moduleName << "\n";
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
    this->pythonSystem.attr("configure")(_entity);
  }

  if (py::hasattr(this->pythonSystem, "pre_update"))
  {
    this->preUpdateMethod = this->pythonSystem.attr("pre_update");
  }
  this->validConfig = true;
}

void PythonSystemLoader::PreUpdate(const UpdateInfo &_info,
                                   EntityComponentManager &_ecm)
{
  if (!this->validConfig)
  {
    return;
  }

  if (this->preUpdateMethod)
  {
    try
    {
      this->preUpdateMethod(
          _info, py::cast(_ecm, py::return_value_policy::reference));
    }
    catch (const pybind11::error_already_set &_err)
    {
      gzerr << _err.what() << std::endl;
      return;
    }
  }
}
void PythonSystemLoader::PostUpdate(const UpdateInfo &/* _info */,
                                    const EntityComponentManager &/* _ecm */)
{
}

GZ_ADD_PLUGIN(PythonSystemLoader, System, ISystemConfigure, ISystemPreUpdate,
              ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(PythonSystemLoader, "gz::sim::systems::PythonSystemLoader")
}  // namespace gz::sim::systems
