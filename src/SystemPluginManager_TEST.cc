/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/SystemPluginManager.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(SystemPluginManager, Constructor)
{
  gazebo::SystemPluginManager sm;
  sm.LoadLibrary("libignition-gazebo-system_plugins.so");

  {
    auto plugins = sm.PluginsByType(gazebo::SystemTypeId::UNKNOWN);
    ASSERT_EQ(1u, plugins.size());
  }

  {
    auto plugins = sm.PluginsByType(gazebo::SystemTypeId::PHYSICS);
    ASSERT_EQ(1u, plugins.size());
  }

  {
    auto plugins = sm.PluginsByType(gazebo::SystemTypeId::RENDERING);
    ASSERT_EQ(0u, plugins.size());
  }
}

