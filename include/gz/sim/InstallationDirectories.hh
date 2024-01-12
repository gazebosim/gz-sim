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

#ifndef GZ_SIM_INSTALLATION_DIRECTORIES_HH_
#define GZ_SIM_INSTALLATION_DIRECTORIES_HH_

#include <string>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

namespace gz
{
  namespace sim
  {
    inline namespace GZ_SIM_VERSION_NAMESPACE {

    /// \brief getInstallPrefix return the install prefix of the library
    /// i.e. CMAKE_INSTALL_PREFIX unless the library has been moved
    GZ_SIM_VISIBLE std::string getInstallPrefix();

    /// \brief getGUIConfigPath return the GUI config path
    GZ_SIM_VISIBLE std::string getGUIConfigPath();

    /// \brief getSystemConfigPath return the system config path
    GZ_SIM_VISIBLE std::string getSystemConfigPath();

    /// \brief getServerConfigPath return the server config path
    GZ_SIM_VISIBLE std::string getServerConfigPath();

    /// \brief getPluginInstallDir return the plugin install dir
    GZ_SIM_VISIBLE std::string getPluginInstallDir();

    /// \brief getGUIPluginInstallDir return the GUI plugin install dir
    GZ_SIM_VISIBLE std::string getGUIPluginInstallDir();

    /// \brief getWorldInstallDir return the world install dir
    GZ_SIM_VISIBLE std::string getWorldInstallDir();

    /// \brief getMediaInstallDir return the media install dir
    GZ_SIM_VISIBLE std::string getMediaInstallDir();
    }
  }
}

#endif
