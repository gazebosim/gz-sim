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

/////////////////////////////////////////////////
TEST(SystemLoader, Constructor)
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

TEST(SystemLoader, EmptyNames)
{
  sim::SystemLoader sm;
  sdf::Plugin plugin;
  auto system = sm.LoadPlugin(plugin);
  ASSERT_FALSE(system.has_value());
}
