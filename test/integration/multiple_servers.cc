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

#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"

#include "../test/helpers/EnvTestFixture.hh"

using namespace gz;
using namespace gz::sim;
using namespace std::chrono_literals;

/////////////////////////////////////////////////
class MultipleServers : public InternalFixture<::testing::TestWithParam<int>>
{
};

/////////////////////////////////////////////////
// See: https://github.com/gazebosim/gz-sim/issues/1544
TEST_P(MultipleServers, GZ_UTILS_TEST_DISABLED_ON_MAC(TwoServersNonBlocking))
{
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfString(TestWorldSansPhysics::World());

  sim::Server server1(serverConfig);
  sim::Server server2(serverConfig);
  EXPECT_FALSE(server1.Running());
  EXPECT_FALSE(*server1.Running(0));
  EXPECT_FALSE(server2.Running());
  EXPECT_FALSE(*server2.Running(0));
  EXPECT_EQ(0u, *server1.IterationCount());
  EXPECT_EQ(0u, *server2.IterationCount());

  // Make the servers run fast.
  server1.SetUpdatePeriod(1ns);
  server2.SetUpdatePeriod(1ns);

  // Start non-blocking
  const size_t iters1 = 9999;
  EXPECT_TRUE(server1.Run(false, iters1, false));

  // Expect that we can't start another instance.
  EXPECT_FALSE(server1.Run(true, 10, false));

  // It's okay to start another server
  EXPECT_TRUE(server2.Run(false, 500, false));

  while (*server1.IterationCount() < iters1 || *server2.IterationCount() < 500)
    GZ_SLEEP_MS(100);

  EXPECT_EQ(iters1, *server1.IterationCount());
  EXPECT_EQ(500u, *server2.IterationCount());
  EXPECT_FALSE(server1.Running());
  EXPECT_FALSE(*server1.Running(0));
  EXPECT_FALSE(server2.Running());
  EXPECT_FALSE(*server2.Running(0));
}

/////////////////////////////////////////////////
// See: https://github.com/gazebosim/gz-sim/issues/1544
TEST_P(MultipleServers, GZ_UTILS_TEST_DISABLED_ON_MAC(TwoServersMixedBlocking))
{
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfString(TestWorldSansPhysics::World());

  sim::Server server1(serverConfig);
  sim::Server server2(serverConfig);
  EXPECT_FALSE(server1.Running());
  EXPECT_FALSE(*server1.Running(0));
  EXPECT_FALSE(server2.Running());
  EXPECT_FALSE(*server2.Running(0));
  EXPECT_EQ(0u, *server1.IterationCount());
  EXPECT_EQ(0u, *server2.IterationCount());

  // Make the servers run fast.
  server1.SetUpdatePeriod(1ns);
  server2.SetUpdatePeriod(1ns);

  server1.Run(false, 10, false);
  server2.Run(true, 1000, false);

  while (*server1.IterationCount() < 10)
    GZ_SLEEP_MS(100);

  EXPECT_EQ(10u, *server1.IterationCount());
  EXPECT_EQ(1000u, *server2.IterationCount());
  EXPECT_FALSE(server1.Running());
  EXPECT_FALSE(*server1.Running(0));
  EXPECT_FALSE(server2.Running());
  EXPECT_FALSE(*server2.Running(0));
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_SUITE_P(ServerRepeat, MultipleServers, ::testing::Range(1, 2));
