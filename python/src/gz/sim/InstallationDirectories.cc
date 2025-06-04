/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
 */


#include <pybind11/pybind11.h>

#include <gz/sim/InstallationDirectories.hh>

#include "InstallationDirectories.hh"

namespace gz
{
namespace sim
{
namespace python
{
void defineSimInstallationDirectories(pybind11::module &_module)
{

  _module.def("get_install_prefix", [](){
    return getInstallPrefix();
  },
  "getInstallPrefix return the install prefix of the library")
  .def("get_gui_config_path", [](){
    return getGUIConfigPath();
  },
  "getGUIConfigPath return the GUI config path")
  .def("get_system_config_path", [](){
    return getSystemConfigPath();
  },
  "getSystemConfigPath return the system config path")
  .def("get_server_config_path", [](){
    return getServerConfigPath();
  },
  "getServerConfigPath return the server config path")
  .def("get_plugin_install_dir", [](){
    return getPluginInstallDir();
  },
  "getPluginInstallDir return the plugin install dir")
  .def("get_gui_plugin_install_dir", [](){
    return getGUIPluginInstallDir();
  },
  "getGUIPluginInstallDir return the GUI plugin install dir")
  .def("get_world_install_dir", [](){
    return getWorldInstallDir();
  },
  "getWorldInstallDir return the world install dir")
  .def("get_media_install_dir", [](){
    return getMediaInstallDir();
  },
  "getMediaInstallDir return the media install dir");
}
}  // namespace python
}  // namespace sim
}  // namespace gz
