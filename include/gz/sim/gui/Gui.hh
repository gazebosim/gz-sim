/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef GZ_SIM_GUI_GUI_HH_
#define GZ_SIM_GUI_GUI_HH_

#include <memory>
#include <string>
#include <gz/gui/Application.hh>

#include "gz/sim/config.hh"
#include "gz/sim/gui/Export.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace gui
{
  /// \brief Run GUI application
  /// \param[in] _argc Number of command line arguments (Used when running
  /// without gz-tools. Set to 1 if using gz-tools). Note: The object
  /// referenced by this variable must continue to exist for the lifetime of the
  /// application.
  /// \param[in] _argv Command line arguments (Used when running without
  /// gz-tools. Set to the name of the application if using gz-tools)
  /// \param[in] _guiConfig The GUI configuration file. If nullptr, the default
  /// configuration from GZ_HOMEDIR/.gz/sim/gui.config will be used.
  /// \param[in] _renderEngine --render-engine-gui option
  /// \return -1 on failure, 0 on success
  GZ_SIM_GUI_VISIBLE int runGui(int &_argc,
    char **_argv, const char *_guiConfig, const char *_renderEngine = nullptr);

  /// \brief Run GUI application
  /// \param[in] _argc Number of command line arguments (Used when running
  /// without gz-tools. Set to 1 if using gz-tools). Note: The object
  /// referenced by this variable must continue to exist for the lifetime of the
  /// application.
  /// \param[in] _argv Command line arguments (Used when running without
  /// gz-tools. Set to the name of the application if using gz-tools)
  /// \param[in] _guiConfig The GUI configuration file. If nullptr, the default
  /// configuration from GZ_HOMEDIR/.gz/sim/gui.config will be used.
  /// \param[in] _sdfFile The world file path passed as a command line argument.
  /// If set, QuickStart Dialog will not be shown.
  /// \param[in] _waitGui Flag indicating whether the server waits until
  /// it receives a world path from GUI.
  /// \param[in] _renderEngine --render-engine-gui option
  /// \param[in] _renderEngineGuiApiBackend --render-engine-gui-api-backend
  /// \return -1 on failure, 0 on success
  GZ_SIM_GUI_VISIBLE int runGui(int &_argc, char **_argv,
        const char *_guiConfig, const char *_sdfFile, int _waitGui,
        const char *_renderEngine = nullptr,
        const char *_renderEngineApiBackend = nullptr);

  /// \brief Create a Gazebo GUI application
  /// \param[in] _argc Number of command line arguments (Used when running
  /// without gz-tools. Set to 1 if using gz-tools). Note: The object
  /// referenced by this variable must continue to exist for the lifetime of the
  /// application.
  /// \param[in] _argv Command line arguments (Used when running without
  /// gz-tools. Set to the name of the application if using gz-tools)
  /// \param[in] _guiConfig The GUI configuration file. If nullptr, the default
  /// configuration from GZ_HOMEDIR/.gz/sim/gui.config will be used.
  /// \param[in] _defaultGuiConfig The default GUI configuration file. If no
  /// plugins were added from a world file or from _guiConfig, this
  /// configuration file will be loaded. If this argument is a nullptr or if the
  /// file does not exist, the default configuration from
  /// GZ_HOMEDIR/.gz/sim/gui.config will be used.
  /// \param[in] _loadPluginsFromSdf If true, plugins specified in the world
  /// SDFormat file will get loaded.
  /// \param[in] _renderEngine --render-engine-gui option
  /// \return Newly created application.
  GZ_SIM_GUI_VISIBLE
  std::unique_ptr<gz::gui::Application> createGui(
      int &_argc, char **_argv, const char *_guiConfig,
      const char *_defaultGuiConfig = nullptr, bool _loadPluginsFromSdf = true,
      const char *_renderEngine = nullptr);

  /// \brief Create a Gazebo GUI application
  /// \param[in] _argc Number of command line arguments (Used when running
  /// without gz-tools. Set to 1 if using gz-tools). Note: The object
  /// referenced by this variable must continue to exist for the lifetime of the
  /// application.
  /// \param[in] _argv Command line arguments (Used when running without
  /// gz-tools. Set to the name of the application if using gz-tools)
  /// \param[in] _guiConfig The GUI configuration file. If nullptr, the default
  /// configuration from GZ_HOMEDIR/.gz/sim/gui.config will be used.
  /// \param[in] _defaultGuiConfig The default GUI configuration file. If no
  /// plugins were added from a world file or from _guiConfig, this
  /// configuration file will be loaded. If this argument is a nullptr or if the
  /// file does not exist, the default configuration from
  /// GZ_HOMEDIR/.gz/sim/gui.config will be used.
  /// \param[in] _loadPluginsFromSdf If true, plugins specified in the world
  /// SDFormat file will get loaded.
  /// \param[in] _sdfFile SDF world file, or nullptr if not set.
  /// \param[in] _waitGui True if the server is waiting for the GUI to decide on
  /// a starting world.
  /// \param[in] _renderEngine --render-engine-gui option
  /// \param[in] _renderEngineGuiApiBackend --render-engine-gui-api-backend
  /// option
  /// \return Newly created application.
  GZ_SIM_GUI_VISIBLE
  std::unique_ptr<gz::gui::Application> createGui(
      int &_argc, char **_argv, const char *_guiConfig,
      const char *_defaultGuiConfig, bool _loadPluginsFromSdf,
      const char *_sdfFile, int _waitGui, const char *_renderEngine = nullptr,
      const char *_renderEngineGuiApiBackend = nullptr );
}  // namespace gui
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif
