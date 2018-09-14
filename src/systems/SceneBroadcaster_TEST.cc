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

/// \brief Test SceneBroadcaster system
class SceneBroadcasterTest : public ::testing::TestWithParam<int>
{
  // Documentation inherited
  protected: virtual void SetUp()
  {
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

/////////////////////////////////////////////////
TEST_P(SceneBroadcasterTest, PoseInfo)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(*server.Running());
  EXPECT_EQ(13u, *server.EntityCount());

  // Create pose subscriber
  transport::Node node;

  bool received{false};
  std::function<void(const msgs::Pose_V &)> cb = [&](const msgs::Pose_V &_msg)
  {
    EXPECT_EQ(9, _msg.pose_size());

    std::map<int, std::string> entityMap;
    for (auto p = 0; p < _msg.pose_size(); ++p)
    {
      entityMap.insert(std::make_pair(_msg.pose(p).id(), _msg.pose(p).name()));
    }

    EXPECT_EQ(9u, entityMap.size());

    received = true;
  };
  EXPECT_TRUE(node.Subscribe("/world/default/pose/info", cb));

  // Run server
  server.Run(true, 1);

  unsigned int sleep{0u};
  unsigned int maxSleep{10u};
  while (!received && sleep++ < maxSleep)
    IGN_SLEEP_MS(100);

  EXPECT_TRUE(received);
}

/////////////////////////////////////////////////
TEST_P(SceneBroadcasterTest, SceneInfo)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(*server.Running());
  EXPECT_EQ(13u, *server.EntityCount());

  // Run server
  server.Run(true, 1);

  // Create requester
  transport::Node node;

  bool result{false};
  unsigned int timeout{5000};
  ignition::msgs::Scene res;

  EXPECT_TRUE(node.Request("/world/default/scene/info", timeout, res, result));
  EXPECT_TRUE(result);

  EXPECT_EQ(3, res.model_size());

  for (auto m = 0; m < res.model_size(); ++m)
  {
    ASSERT_EQ(1, res.model(m).link_size());
    EXPECT_EQ(res.model(m).name() + "_link", res.model(m).link(0).name());

    ASSERT_EQ(1, res.model(m).link(0).visual_size());
    EXPECT_EQ(res.model(m).name() + "_visual",
        res.model(m).link(0).visual(0).name());
  }
}

/////////////////////////////////////////////////
TEST_P(SceneBroadcasterTest, SceneGraph)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(*server.Running());
  EXPECT_EQ(13u, *server.EntityCount());

  // Run server
  server.Run(true, 1);

  // Create requester
  transport::Node node;

  bool result{false};
  unsigned int timeout{5000};
  ignition::msgs::StringMsg res;

  EXPECT_TRUE(node.Request("/world/default/scene/graph", timeout, res, result));
  EXPECT_TRUE(result);

  EXPECT_FALSE(res.data().empty());
  EXPECT_NE(res.data().find("default (0)"), std::string::npos);
  EXPECT_NE(res.data().find("box (1)"), std::string::npos);
  EXPECT_NE(res.data().find("box_link (2)"), std::string::npos);
  EXPECT_NE(res.data().find("box_visual (3)"), std::string::npos);
  EXPECT_NE(res.data().find("cylinder (5)"), std::string::npos);
  EXPECT_NE(res.data().find("cylinder_link (6)"), std::string::npos);
  EXPECT_NE(res.data().find("cylinder_visual (7)"), std::string::npos);
  EXPECT_NE(res.data().find("sphere (9)"), std::string::npos);
  EXPECT_NE(res.data().find("sphere_link (10)"), std::string::npos);
  EXPECT_NE(res.data().find("sphere_visual (11)"), std::string::npos);
}

// Run multiple times
INSTANTIATE_TEST_CASE_P(ServerRepeat, SceneBroadcasterTest,
    ::testing::Range(1, 2));
