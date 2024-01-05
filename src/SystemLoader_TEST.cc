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

#include <gz/common/Filesystem.hh>
#include <gz/common/SystemPaths.hh>
#include "gz/sim/System.hh"
#include "gz/sim/SystemLoader.hh"

#include "gz/sim/test_config.hh"  // NOLINT(build/include)

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
TEST(SystemLoader, Constructor)
{
  gz::sim::SystemLoader sm;

  // Add test plugin to path (referenced in config)
  auto testBuildPath = common::joinPaths(
      std::string(PROJECT_BINARY_PATH), "lib");
  sm.AddSystemPluginPath(testBuildPath);

  sdf::Root root;
  root.LoadSdfString(std::string("<?xml version='1.0'?><sdf version='1.6'>"
      "<world name='default'>"
      "<plugin filename='libignition-gazebo") +
      IGNITION_GAZEBO_MAJOR_VERSION_STR + "-physics-system.so' "
      "name='gz::sim::systems::Physics'></plugin>"
      "<plugin filename='ignition-gazebo" +
      IGNITION_GAZEBO_MAJOR_VERSION_STR + "-physics-system' "
      "name='gz::sim::systems::Physics'></plugin>"
      "<plugin filename='ignition-gazebo-physics-system' "
      "name='gz::sim::systems::Physics'></plugin>"
      "<plugin filename='libgz-sim" +
      IGNITION_GAZEBO_MAJOR_VERSION_STR + "-physics-system.so' "
      "name='gz::sim::systems::Physics'></plugin>"
      "<plugin filename='gz-sim" +
      IGNITION_GAZEBO_MAJOR_VERSION_STR + "-physics-system' "
      "name='gz::sim::systems::Physics'></plugin>"
      "<plugin filename='gz-sim-physics-system' "
      "name='gz::sim::systems::Physics'></plugin>"
      "</world></sdf>");

  auto worldElem = root.WorldByIndex(0)->Element();
  if (worldElem->HasElement("plugin")) {
    sdf::ElementPtr pluginElem = worldElem->GetElement("plugin");
    while (pluginElem)
    {
      sdf::Plugin plugin;
      plugin.Load(pluginElem);
      auto system = sm.LoadPlugin(plugin);
      ASSERT_TRUE(system.has_value());
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }
}
/////////////////////////////////////////////////
TEST(SystemLoader, FromPluginPathEnv)
{
  sdf::Root root;
  root.LoadSdfString(R"(<?xml version='1.0'?>
    <sdf version='1.6'>
      <world name='default'>
        <plugin filename='libMockSystem.so' name='gz::sim::MockSystem'/>
      </world>
    </sdf>)");

  ASSERT_NE(root.WorldCount(), 0u);
  auto world = root.WorldByIndex(0);
  ASSERT_TRUE(world != nullptr);
  ASSERT_FALSE(world->Plugins().empty());
  auto plugin = world->Plugins()[0];

  {
    gz::sim::SystemLoader sm;
    auto system = sm.LoadPlugin(plugin);
    EXPECT_FALSE(system.has_value());
  }

  const auto libPath = common::joinPaths(PROJECT_BINARY_PATH, "lib");

  {
    common::setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", libPath.c_str());

    gz::sim::SystemLoader sm;
    auto system = sm.LoadPlugin(plugin);
    EXPECT_TRUE(system.has_value());
    EXPECT_TRUE(common::unsetenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH"));
  }
  {
    common::setenv("GZ_SIM_SYSTEM_PLUGIN_PATH", libPath.c_str());

    gz::sim::SystemLoader sm;
    auto system = sm.LoadPlugin(plugin);
    EXPECT_TRUE(system.has_value());
    EXPECT_TRUE(common::unsetenv("GZ_SIM_SYSTEM_PLUGIN_PATH"));
  }
}

TEST(SystemLoader, EmptyNames)
{
  gz::sim::SystemLoader sm;
  sdf::Plugin plugin;
  auto system = sm.LoadPlugin(plugin);
  ASSERT_FALSE(system.has_value());
}

/////////////////////////////////////////////////
TEST(SystemLoader, PluginPaths)
{
  SystemLoader sm;

  // verify that there should exist some default paths
  std::list<std::string> paths = sm.PluginPaths();
  unsigned int pathCount = paths.size();
  EXPECT_LT(0u, pathCount);

  // Add test path and verify that the loader now contains this path
  auto testBuildPath = common::joinPaths(
      std::string(PROJECT_BINARY_PATH), "lib");
  sm.AddSystemPluginPath(testBuildPath);
  paths = sm.PluginPaths();

  // Number of paths should increase by 1
  EXPECT_EQ(pathCount + 1, paths.size());

  // verify newly added paths exists
  bool hasPath = false;
  for (const auto &s : paths)
  {
    // the returned path string may not be exact match due to extra '/'
    // appended at the end of the string. So use NormalizeDirectoryPath
    if (common::SystemPaths::NormalizeDirectoryPath(s) ==
        common::SystemPaths::NormalizeDirectoryPath(testBuildPath))
    {
      hasPath = true;
      break;
    }
  }
  EXPECT_TRUE(hasPath) << testBuildPath;
}
