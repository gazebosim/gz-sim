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
#include <signal.h>
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
    server.Run(i, true);
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

  server.Run(100, false);
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

  server.Run(100, false);
  server.Run(100, false);

  while (server.IterationCount() < 100)
    IGN_SLEEP_MS(100);

  EXPECT_EQ(100u, server.IterationCount());
  EXPECT_FALSE(server.Running());
}

/////////////////////////////////////////////////
TEST(Server, SceneService)
{
  gazebo::Server server;
  EXPECT_FALSE(server.Running());
  transport::Node node;

  msgs::Scene response;
  unsigned int timeout = 100;
  bool result;
  node.Request("/ign/gazebo/scene", timeout, response, result);
  EXPECT_TRUE(result);

  // This value is hardcoded in the service. Change this when we can load
  // the server with an SDF world.
  EXPECT_EQ("gazebo", response.name());
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

  raise(SIGINT);

  EXPECT_FALSE(server.Running());
}
