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
#include <gz/common/Util.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"

#include "../test/helpers/EnvTestFixture.hh"

using namespace gz;
using namespace gz::sim;
using namespace std::chrono_literals;

/////////////////////////////////////////////////
class FuelCachedServer : public InternalFixture<::testing::TestWithParam<int>>
{
};

/////////////////////////////////////////////////
TEST_P(FuelCachedServer, CachedFuelWorld)
{
  auto cachedWorldPath =
    common::joinPaths(std::string(PROJECT_SOURCE_PATH), "test", "worlds");
  common::setenv("GZ_FUEL_CACHE_PATH", cachedWorldPath.c_str());

  ServerConfig serverConfig;
  auto fuelWorldURL =
    "https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Test%20world";
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
INSTANTIATE_TEST_SUITE_P(ServerRepeat, FuelCachedServer,
    ::testing::Range(1, 2));
