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

#ifndef IGNITION_GAZEBO_GUI_GUI_HH_
#define IGNITION_GAZEBO_GUI_GUI_HH_

#include <memory>
#include <ignition/gui/Application.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/gui/Export.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace gui
{
  /// \brief Run GUI application
  /// \param[in] _argc Number of command line arguments (Used when running
  /// without ign-tools. Set to 1 if using ign-tools). Note: The object
  /// referenced by this variable must continue to exist for the lifetime of the
  /// application.
  /// \param[in] _argv Command line arguments (Used when running without
  /// ign-tools. Set to the name of the application if using ign-tools)
  /// \param[in] _guiConfig The GUI configuration file. If nullptr, the default
  /// configuration from IGN_HOMEDIR/.ignition/gazebo/gui.config will be used.
  IGNITION_GAZEBO_GUI_VISIBLE int runGui(int &_argc, char **_argv,
                                     const char *_guiConfig);

  /// \brief Create a Gazebo GUI application
  /// \param[in] _argc Number of command line arguments (Used when running
  /// without ign-tools. Set to 1 if using ign-tools). Note: The object
  /// referenced by this variable must continue to exist for the lifetime of the
  /// application.
  /// \param[in] _argv Command line arguments (Used when running without
  /// ign-tools. Set to the name of the application if using ign-tools)
  /// \param[in] _guiConfig The GUI configuration file. If nullptr, the default
  /// configuration from IGN_HOMEDIR/.ignition/gazebo/gui.config will be used.
  /// \param[in] _defaultGuiConfig The default GUI configuration file. If no
  /// plugins were added from a world file or from _guiConfig, this
  /// configuration file will be loaded. If this argument is a nullptr or if the
  /// file does not exist, the default configuration from
  /// IGN_HOMEDIR/.ignition/gazebo/gui.config will be used.
  /// \param[in] _loadPluginsFromSdf If true, plugins specified in the world
  /// SDFormat file will get loaded.
  IGNITION_GAZEBO_GUI_VISIBLE
  std::unique_ptr<ignition::gui::Application> createGui(
      int &_argc, char **_argv, const char *_guiConfig,
      const char *_defaultGuiConfig = nullptr, bool _loadPluginsFromSdf = true);

}  // namespace gui
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition

#endif
