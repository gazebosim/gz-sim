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

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"

#include "../test/helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace ignition::gazebo;
using namespace std::chrono_literals;

/////////////////////////////////////////////////
TEST_P(ServerFixture, CachedFuelWorld)
{
  auto cachedWorldPath =
    common::joinPaths(std::string(PROJECT_SOURCE_PATH), "test", "worlds");
  setenv("IGN_FUEL_CACHE_PATH", cachedWorldPath.c_str(), 1);

  ServerConfig serverConfig;
  auto fuelWorldURL =
    "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/worlds/Test%20world";
  EXPECT_TRUE(serverConfig.SetSdfFile(fuelWorldURL));

  EXPECT_EQ(fuelWorldURL, serverConfig.SdfFile());
  EXPECT_TRUE(serverConfig.SdfString().empty());

  // Check that world was loaded
  auto server = Server(serverConfig);
  EXPECT_NE(std::nullopt, server.Running(0));
  EXPECT_FALSE(*server.Running(0));

  server.Run(true /*blocking*/, 1, false/*paused*/);

  EXPECT_NE(std::nullopt, server.Running(0));
  EXPECT_FALSE(*server.Running(0));
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_SUITE_P(ServerRepeat, ServerFixture, ::testing::Range(1, 2));
