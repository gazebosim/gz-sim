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
#include "gz/sim/System.hh"
#include "gz/sim/SystemLoader.hh"

#include "test_config.hh"  // NOLINT(build/include)

using namespace gz;
using namespace sim;

#ifdef _WIN32
  constexpr const char *kPluginDir = "bin";
#else
  constexpr const char *kPluginDir = "lib";
#endif
/////////////////////////////////////////////////
TEST(SystemLoader, Constructor)
{
  // Add test plugin to path (referenced in config)
  auto testBuildPath = gz::common::joinPaths(
      std::string(PROJECT_BINARY_PATH), kPluginDir);

  sdf::Root root;
  root.LoadSdfString(std::string("<?xml version='1.0'?><sdf version='1.6'>"
      "<world name='default'>") +
      "<plugin filename='libgz-sim" +
      GZ_SIM_MAJOR_VERSION_STR + "-user-commands-system.so' "
      "name='gz::sim::systems::UserCommands'></plugin>"
      "<plugin filename='gz-sim" +
      GZ_SIM_MAJOR_VERSION_STR + "-user-commands-system' "
      "name='gz::sim::systems::UserCommands'></plugin>"
      "<plugin filename='gz-sim-user-commands-system' "
      "name='gz::sim::systems::UserCommands'></plugin>"
      "</world></sdf>");
  ASSERT_NE(root.WorldByIndex(0), nullptr);
  auto worldElem = root.WorldByIndex(0)->Element();
  if (worldElem->HasElement("plugin")) {
    sdf::ElementPtr pluginElem = worldElem->GetElement("plugin");
    while (pluginElem)
    {
      gz::sim::SystemLoader sm;
      sm.AddSystemPluginPath(testBuildPath);
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
        <plugin filename='MockSystem' name='gz::sim::MockSystem'/>
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

  const auto libPath = common::joinPaths(PROJECT_BINARY_PATH, kPluginDir);

  {
    common::setenv("GZ_SIM_SYSTEM_PLUGIN_PATH", libPath.c_str());

    gz::sim::SystemLoader sm;
    auto system = sm.LoadPlugin(plugin);
    EXPECT_TRUE(system.has_value());
    EXPECT_TRUE(common::unsetenv("GZ_SIM_SYSTEM_PLUGIN_PATH"));
  }
}

/////////////////////////////////////////////////
TEST(SystemLoader, EmptyNames)
{
  sim::SystemLoader sm;
  sdf::Plugin plugin;
  ::testing::internal::CaptureStderr();
  auto system = sm.LoadPlugin(plugin);
  ASSERT_FALSE(system.has_value());
  auto output = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(std::string::npos, output.find("empty argument"));
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
      std::string(PROJECT_BINARY_PATH), "lib/");
  sm.AddSystemPluginPath(testBuildPath);
  paths = sm.PluginPaths();

  // Number of paths should increase by 1
  EXPECT_EQ(pathCount + 1, paths.size());

  // verify newly added paths exists
  bool hasPath = false;
  for (const auto &s : paths)
  {
    // the returned path string may not be exact match due to extra '/'
    // appended at the end of the string. So use absPath to compare paths
    if (common::absPath(s) == common::absPath(testBuildPath))
    {
      hasPath = true;
      break;
    }
  }
  EXPECT_TRUE(hasPath);
}

/////////////////////////////////////////////////
TEST(SystemLoader, BadLibraryPath)
{
  sim::SystemLoader sm;

  // Add test plugin to path (referenced in config)
  auto testBuildPath = gz::common::joinPaths(
      std::string(PROJECT_BINARY_PATH), "lib");
  sm.AddSystemPluginPath(testBuildPath);

  sdf::Root root;
  root.LoadSdfString(std::string("<?xml version='1.0'?><sdf version='1.6'>"
      "<world name='default'>"
      "<plugin filename='foo.so'"
      "name='gz::sim::systems::Physics'></plugin>"
      "</world></sdf>"));

  auto worldElem = root.WorldByIndex(0)->Element();
  if (worldElem->HasElement("plugin")) {
    sdf::ElementPtr pluginElem = worldElem->GetElement("plugin");
    while (pluginElem)
    {
      ::testing::internal::CaptureStderr();
      sdf::Plugin plugin;
      plugin.Load(pluginElem);
      auto system = sm.LoadPlugin(plugin);
      ASSERT_FALSE(system.has_value());
      auto output = ::testing::internal::GetCapturedStderr();
      EXPECT_NE(std::string::npos,
          output.find("Could not find shared library")) << output.c_str();
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }
}

/////////////////////////////////////////////////
TEST(SystemLoader, BadPluginName)
{
  sim::SystemLoader sm;

  // Add test plugin to path (referenced in config)
  auto testBuildPath = gz::common::joinPaths(
      std::string(PROJECT_BINARY_PATH), "lib");
  sm.AddSystemPluginPath(testBuildPath);

  sdf::Root root;
  root.LoadSdfString(std::string("<?xml version='1.0'?><sdf version='1.6'>"
      "<world name='default'>"
      "<plugin filename='libgz-sim") +
      GZ_SIM_MAJOR_VERSION_STR + "-physics-system.so' "
      "name='gz::sim::systems::Foo'></plugin>"
      "</world></sdf>");

  auto worldElem = root.WorldByIndex(0)->Element();
  if (worldElem->HasElement("plugin")) {
    sdf::ElementPtr pluginElem = worldElem->GetElement("plugin");
    while (pluginElem)
    {
      ::testing::internal::CaptureStderr();
      sdf::Plugin plugin;
      plugin.Load(pluginElem);
      auto system = sm.LoadPlugin(plugin);
      ASSERT_FALSE(system.has_value());
      auto output = ::testing::internal::GetCapturedStderr();
      EXPECT_NE(std::string::npos,
          output.find("library does not contain")) << output.c_str();
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }
}
