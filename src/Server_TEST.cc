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
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include <ignition/common/Util.hh>

#include "ignition/gazebo/Server.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Server, Constructor)
{
  gazebo::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_EQ(0u, server.IterationCount());
}

/////////////////////////////////////////////////
TEST(Server, RunBlocking)
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
TEST(Server, RunNonBlocking)
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
TEST(Server, RunNonBlockingMultiple)
{
  gazebo::Server server;
  EXPECT_FALSE(server.Running());
  EXPECT_EQ(0u, server.IterationCount());

  server.Run(false, 100);
  server.Run(false, 100);

  while (server.IterationCount() < 100)
    IGN_SLEEP_MS(100);

  EXPECT_EQ(100u, server.IterationCount());
  EXPECT_FALSE(server.Running());
}

/////////////////////////////////////////////////
TEST(Server, SigInt)
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
TEST(Server, TwoServersNonBlocking)
{
  gazebo::Server server1;
  gazebo::Server server2;
  EXPECT_FALSE(server1.Running());
  EXPECT_FALSE(server2.Running());
  EXPECT_EQ(0u, server1.IterationCount());
  EXPECT_EQ(0u, server2.IterationCount());

  server1.Run(false, 100);
  server2.Run(false, 500);

  while (server1.IterationCount() < 100 || server2.IterationCount() < 500)
    IGN_SLEEP_MS(100);

  EXPECT_EQ(100u, server1.IterationCount());
  EXPECT_EQ(500u, server2.IterationCount());
  EXPECT_FALSE(server1.Running());
  EXPECT_FALSE(server2.Running());
}

/////////////////////////////////////////////////
TEST(Server, TwoServersMixedBlocking)
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
