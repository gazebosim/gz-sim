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
#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"

using namespace ignition;

class ScenePublisherTest : public ::testing::TestWithParam<int>
{
};

/////////////////////////////////////////////////
TEST_P(ScenePublisherTest, Shapes)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(*server.Running());
  EXPECT_EQ(13u, *server.EntityCount());

  // Create scene subscriber
  transport::Node node;

  bool received{false};
  std::function<void(const msgs::Scene &)> cb = [&](const msgs::Scene &_msg)
  {
    EXPECT_EQ(3, _msg.model_size());

    for (auto m = 0; m < _msg.model_size(); ++m)
    {
      ASSERT_EQ(1, _msg.model(m).link_size());
      EXPECT_EQ(_msg.model(m).name() + "_link", _msg.model(m).link(0).name());

      ASSERT_EQ(1, _msg.model(m).link(0).visual_size());
      EXPECT_EQ(_msg.model(m).name() + "_visual",
          _msg.model(m).link(0).visual(0).name());
    }

    received = true;
  };
  EXPECT_TRUE(node.Subscribe("/world/default/scene", cb));

  // Run server
  server.Run(true, 1);

  unsigned int sleep{0u};
  unsigned int maxSleep{10u};
  while (!received && sleep++ < maxSleep)
    IGN_SLEEP_MS(100);

  EXPECT_TRUE(received);
}

// Run multiple times
INSTANTIATE_TEST_CASE_P(ServerRepeat, ScenePublisherTest,
    ::testing::Range(1, 2));
