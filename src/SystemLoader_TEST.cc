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

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <ignition/common/Filesystem.hh>
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/SystemLoader.hh"

#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

using namespace ignition;

/////////////////////////////////////////////////
TEST(SystemLoader, Constructor)
{
  gazebo::SystemLoader sm;

  // Add test plugin to path (referenced in config)
  auto testBuildPath = ignition::common::joinPaths(
      std::string(PROJECT_BINARY_PATH), "lib");
  sm.AddSystemPluginPath(testBuildPath);

  sdf::Root root;
  root.LoadSdfString(std::string("<?xml version='1.0'?><sdf version='1.6'>"
      "<world name='default'>"
      "<plugin filename='libignition-gazebo-physics-system.so' "
      "        name='ignition::gazebo::systems::Physics'></plugin>"
      "</world></sdf>"));

  auto worldElem = root.WorldByIndex(0)->Element();
  if (worldElem->HasElement("plugin")) {
    sdf::ElementPtr pluginElem = worldElem->GetElement("plugin");
    while (pluginElem)
    {
      auto system = sm.LoadPlugin(pluginElem);
      ASSERT_TRUE(system.has_value());
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }
}

TEST(SystemLoader, EmptyNames)
{
  gazebo::SystemLoader sm;
  sdf::ElementPtr element;
  auto system = sm.LoadPlugin("", "", element);
  ASSERT_FALSE(system.has_value());
}
