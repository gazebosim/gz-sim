/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/msgs/double.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Model.hh"

#include "ignition/gazebo/test_config.hh"
#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

class ThrusterTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(ThrusterTest, UniformWorldMovement)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "thruster.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  std::vector<math::Pose3d> modelPoses;
  std::vector<math::Pose3d> linkPoses;
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Model
    Entity sub = _ecm.EntityByComponents(
        components::Model(), components::Name("sub"));
    ASSERT_NE(sub, kNullEntity);

    auto modelPose = _ecm.Component<components::Pose>(sub);
    ASSERT_NE(modelPose , nullptr);
    modelPoses.push_back(modelPose->Data());

    // Link
    auto subLink = _ecm.EntityByComponents(
      components::ParentEntity(sub),
      components::Name("body"),
      components::Link());

    ASSERT_NE(subLink, kNullEntity);

    auto linkPose = _ecm.Component<components::Pose>(sub);
    ASSERT_NE(linkPose , nullptr);
    linkPoses.push_back(linkPose->Data());
  });
  server.AddSystem(testSystem.systemPtr);

  // Check initial position
  server.Run(true, 100, false);
  EXPECT_EQ(100u, modelPoses.size());
  EXPECT_EQ(100u, linkPoses.size());

  for (const auto &pose : modelPoses)
  {
    EXPECT_EQ(math::Pose3d(), pose);
  }
  for (const auto &pose : linkPoses)
  {
    EXPECT_EQ(math::Pose3d(), pose);
  }

  // Publish command and check that vehicle moved
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/sub/joint/propeller_joint/cmd_thrust");

  int sleep{0};
  int maxSleep{30};
  for (; !pub.HasConnections() && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_LT(sleep, maxSleep);
  EXPECT_TRUE(pub.HasConnections());

  msgs::Double msg;
  msg.set_data(300);
  pub.Publish(msg);

  // Check movement
  for (sleep = 0; modelPoses.back().Pos().X() < 5.0 && sleep < maxSleep;
      ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    server.Run(true, 100, false);
  }
  EXPECT_LT(sleep, maxSleep) << modelPoses.back().Pos().X();

  EXPECT_EQ(100u * (sleep + 1), modelPoses.size());
  EXPECT_EQ(100u * (sleep + 1), linkPoses.size());

  // F = m * a
  // F = m * v * t^2 / 2
  // F = m * s * t * t^2 / 2
  // TODO(louise) Add expectations based on applied force
  for (const auto &pose : modelPoses)
  {
//    EXPECT_EQ(math::Pose3d(), pose);
  }
  for (const auto &pose : linkPoses)
  {
//    EXPECT_EQ(math::Pose3d(), pose);
  }
}

