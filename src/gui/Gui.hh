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

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Export.hh"

namespace ignition
{
namespace gazebo
{
namespace gui
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  /// \brief Run GUI application
  /// \param[in] _argc Number of command line arguments (Used when running
  /// without ign-tools. Set to 0 if using ign-tools)
  /// \param[in] _argv Command line arguments (Used when running without
  /// ign-tools. Set to nullptr if using ign-tools)
  /// \param[in] _guiConfig The GUI configuration file. If nullptr, the default
  /// configuration from IGN_HOMEDIR/.ignition/gazebo/gui.config will be used.
  int runGui(int _argc, char **_argv, const char *_guiConfig);
}
}  // namespace gui
}  // namespace gazebo
}  // namespace ignition

#endif
