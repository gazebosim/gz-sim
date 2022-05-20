/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef IGNITION_GAZEBO__CONFIG_HH_
#define IGNITION_GAZEBO__CONFIG_HH_

#include <gz/sim/config.hh>

#define IGNITION_GAZEBO_MAJOR_VERSION GZ_SIM_MAJOR_VERSION
#define IGNITION_GAZEBO_MINOR_VERSION GZ_SIM_MINOR_VERSION
#define IGNITION_GAZEBO_PATCH_VERSION GZ_SIM_PATCH_VERSION

#define IGNITION_GAZEBO_VERSION GZ_SIM_VERSION
#define IGNITION_GAZEBO_VERSION_FULL GZ_SIM_VERSION_FULL
#define IGNITION_GAZEBO_MAJOR_VERSION_STR GZ_SIM_MAJOR_VERSION_STR

#define IGNITION_GAZEBO_VERSION_NAMESPACE GZ_SIM_VERSION_NAMESPACE

#define IGNITION_GAZEBO_VERSION_HEADER GZ_SIM_VERSION_HEADER

#define IGNITION_GAZEBO_GUI_CONFIG_PATH GZ_SIM_GUI_CONFIG_PATH
#define IGNITION_GAZEBO_SYSTEM_CONFIG_PATH GZ_SIM_SYSTEM_CONFIG_PATH
#define IGNITION_GAZEBO_SERVER_CONFIG_PATH GZ_SIM_SERVER_CONFIG_PATH
#define IGN_GAZEBO_PLUGIN_INSTALL_DIR GZ_SIM_PLUGIN_INSTALL_DIR
#define IGN_GAZEBO_GUI_PLUGIN_INSTALL_DIR GZ_SIM_GUI_PLUGIN_INSTALL_DIR
#define IGN_GAZEBO_WORLD_INSTALL_DIR GZ_SIM_WORLD_INSTALL_DIR

#cmakedefine IGNITION_GAZEBO_BUILD_TYPE_PROFILE 1
#cmakedefine IGNITION_GAZEBO_BUILD_TYPE_DEBUG 1
#cmakedefine IGNITION_GAZEBO_BUILD_TYPE_RELEASE 1

namespace gz
{
}

namespace ignition
{
  #ifndef SUPPRESS_IGNITION_HEADER_DEPRECATION
    #pragma message("ignition namespace is deprecated! Use gz instead!")
  #endif
  using namespace gz;
}

#endif
