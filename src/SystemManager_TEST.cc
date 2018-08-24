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

#include "ignition/common/Console.hh"
#include "ignition/common/Filesystem.hh"

#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/SystemManager.hh"

#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

using namespace ignition;

/////////////////////////////////////////////////
TEST(SystemManager, Constructor)
{
  gazebo::SystemManager sm;

  // Add test plugin to path (referenced in config)
  auto testBuildPath = ignition::common::joinPaths(
      std::string(PROJECT_BINARY_PATH), "lib");
  sm.AddSystemPluginPath(testBuildPath);

  // Load test config file
  auto testSourcePath = std::string(PROJECT_SOURCE_PATH) + "/test/";
  ASSERT_TRUE(sm.LoadSystemConfig(testSourcePath + "config/test.config"));

  auto system = sm.Instantiate("Null");
  ASSERT_NE(nullptr, system);
}

