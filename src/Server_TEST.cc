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
#include <csignal>
#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"

using namespace ignition;

class ServerFixture : public ::testing::TestWithParam<int>
{
};

/////////////////////////////////////////////////
TEST_P(ServerFixture, Constructor)
{
  gazebo::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_EQ(0u, server.IterationCount());
  EXPECT_EQ(0u, server.EntityCount());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, DefaultServerConfig)
{
  ignition::gazebo::ServerConfig serverConfig;
  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_EQ(0u, server.IterationCount());
  EXPECT_EQ(0u, server.EntityCount());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, SdfServerConfig)
{
  ignition::gazebo::ServerConfig serverConfig;

  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_EQ(0u, server.IterationCount());
  EXPECT_EQ(3u, server.EntityCount());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, RunBlocking)
{
  gazebo::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_EQ(0u, server.IterationCount());

  uint64_t expectedIters = 0;
  for (uint64_t i = 1; i < 100; ++i)
  {
    EXPECT_FALSE(server.Running());
    server.Run(true, i);
    EXPECT_FALSE(server.Running());

    expectedIters += i;
    EXPECT_EQ(expectedIters, server.IterationCount());
  }
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, RunNonBlocking)
{
  gazebo::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_EQ(0u, server.IterationCount());

  server.Run(false, 100);
  while (server.IterationCount() < 100)
    IGN_SLEEP_MS(100);

  EXPECT_EQ(100u, server.IterationCount());
  EXPECT_FALSE(server.Running());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, RunNonBlockingMultiple)
{
  gazebo::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_EQ(0u, server.IterationCount());

  EXPECT_TRUE(server.Run(false, 100));
  EXPECT_FALSE(server.Run(false, 100));

  while (server.IterationCount() < 100)
    IGN_SLEEP_MS(100);

  EXPECT_EQ(100u, server.IterationCount());
  EXPECT_FALSE(server.Running());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, SigInt)
{
  gazebo::Server server;
  EXPECT_FALSE(server.Running());

  // Run forever, non-blocking.
  server.Run();

  IGN_SLEEP_MS(500);

  EXPECT_TRUE(server.Running());

  std::raise(SIGTERM);

  EXPECT_FALSE(server.Running());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, TwoServersNonBlocking)
{
  ignition::common::Console::SetVerbosity(4);
  gazebo::Server server1;
  gazebo::Server server2;
  EXPECT_FALSE(server1.Running());
  EXPECT_FALSE(server2.Running());
  EXPECT_EQ(0u, server1.IterationCount());
  EXPECT_EQ(0u, server2.IterationCount());

  // Start non-blocking
  EXPECT_TRUE(server1.Run(false, 999999));

  // Expect that we can't start another instance.
  EXPECT_FALSE(server1.Run(true, 10));

  // It's okay to start another server
  EXPECT_TRUE(server2.Run(false, 500));

  while (server1.IterationCount() < 999999 || server2.IterationCount() < 500)
    IGN_SLEEP_MS(100);

  EXPECT_EQ(999999u, server1.IterationCount());
  EXPECT_EQ(500u, server2.IterationCount());
  EXPECT_FALSE(server1.Running());
  EXPECT_FALSE(server2.Running());
}

/////////////////////////////////////////////////
TEST_P(ServerFixture, TwoServersMixedBlocking)
{
  gazebo::Server server1;
  gazebo::Server server2;
  EXPECT_FALSE(server1.Running());
  EXPECT_FALSE(server2.Running());
  EXPECT_EQ(0u, server1.IterationCount());
  EXPECT_EQ(0u, server2.IterationCount());

  server1.Run(false, 10);
  server2.Run(true, 1000);

  while (server1.IterationCount() < 10)
    IGN_SLEEP_MS(100);

  EXPECT_EQ(10u, server1.IterationCount());
  EXPECT_EQ(1000u, server2.IterationCount());
  EXPECT_FALSE(server1.Running());
  EXPECT_FALSE(server2.Running());
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_CASE_P(ServerRepeat, ServerFixture, ::testing::Range(1, 10));
