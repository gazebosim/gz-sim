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

#include <gz/sim/config.hh>
#include <gz/sim/InstallationDirectories.hh>

#include <gz/common/Filesystem.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {

std::string getGUIConfigPath()
{
  return gz::common::joinPaths(
      getInstallPrefix(), GZ_SIM_GUI_CONFIG_RELATIVE_PATH);
}

std::string getSystemConfigPath()
{
  return gz::common::joinPaths(
      getInstallPrefix(), GZ_SIM_SYSTEM_CONFIG_RELATIVE_PATH);
}

std::string getServerConfigPath()
{
  return gz::common::joinPaths(
      getInstallPrefix(), GZ_SIM_SERVER_CONFIG_RELATIVE_PATH);
}

std::string getPluginInstallDir()
{
  return gz::common::joinPaths(
      getInstallPrefix(), GZ_SIM_PLUGIN_RELATIVE_INSTALL_DIR);
}

std::string getGUIPluginInstallDir()
{
  return gz::common::joinPaths(
      getInstallPrefix(), GZ_SIM_GUI_PLUGIN_RELATIVE_INSTALL_DIR);
}

std::string getWorldInstallDir()
{
  return gz::common::joinPaths(
      getInstallPrefix(), GZ_SIM_WORLD_RELATIVE_INSTALL_DIR);
}

std::string getMediaInstallDir()
{
  return gz::common::joinPaths(
      getInstallPrefix(), GZ_SIM_MEDIA_RELATIVE_INSTALL_DIR);
}

}
}
}
