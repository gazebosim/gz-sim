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
#include <gz/plugin/Register.hh>

namespace gz::sim::systems
{
void PythonSystemLoader::Configure(
    const Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &, EventManager &)
{
  auto [pythonFile, hasFile] = _sdf->Get<std::string>("python_file", "");
  if (!hasFile)
  {
    gzerr << "PythonSystemLoader missing required argument <python_file>. "
          << "Failed to initialize." << std::endl;
    return;
  }
  // TODO(azeey) Where do we look for the python file? Do we need a separate 
  // environment variable similar to GZ_SYSTEM_PLUGIN_PATH or can we use the 
  // same variable? For now, only look for the file as an absolute path or 
  // relative to CWD.
  // Alternatively, we can consider PYTHONPATH and the requirement would be that 
  // we can just import the module.
}

void PythonSystemLoader::PreUpdate(const UpdateInfo &/* _info */,
                                   EntityComponentManager &/* _ecm */)
{
}
void PythonSystemLoader::PostUpdate(const UpdateInfo &/* _info */,
                                    const EntityComponentManager &/* _ecm */)
{
}

GZ_ADD_PLUGIN(PythonSystemLoader, System, ISystemConfigure, ISystemPreUpdate,
              ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(PythonSystemLoader, "gz::sim::systems::PythonSystemLoader")
}  // namespace gz::sim::systems
