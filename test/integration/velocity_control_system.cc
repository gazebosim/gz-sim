/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Util.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test VelocityControl system
class VelocityControlTest
  : public InternalFixture<::testing::TestWithParam<int>>
{
  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _cmdVelTopic Command velocity topic.
  protected: void TestPublishCmd(const std::string &_sdfFile,
                                 const std::string &_cmdVelTopic)
  {
    // Start server
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);

    Server server(serverConfig);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    // Create a system that records the vehicle poses
    test::Relay testSystem;

    std::vector<math::Pose3d> poses;
    testSystem.OnPostUpdate([&poses](const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        auto id = _ecm.EntityByComponents(
          components::Model(),
          components::Name("vehicle_blue"));
        EXPECT_NE(kNullEntity, id);

        auto poseComp = _ecm.Component<components::Pose>(id);
        ASSERT_NE(nullptr, poseComp);

        poses.push_back(poseComp->Data());
      });
    server.AddSystem(testSystem.systemPtr);

    // Run server and check that vehicle didn't move
    server.Run(true, 1000, false);

    EXPECT_EQ(1000u, poses.size());

    for (const auto &pose : poses)
    {
      EXPECT_EQ(poses[0], pose);
    }

    // Publish command and check that vehicle moved
    transport::Node node;
    auto pub = node.Advertise<msgs::Twist>(_cmdVelTopic);

    msgs::Twist msg;

    // Avoid wheel slip by limiting acceleration (1 m/s^2)
    // and max velocity (0.5 m/s).
    // See <max_velocity< and <max_aceleration> parameters
    // in "/test/worlds/velocity_control.sdf".
    const double desiredLinVel = 10.5;
    const double desiredAngVel = 0.2;
    msgs::Set(msg.mutable_linear(),
              math::Vector3d(desiredLinVel, 0, 0));
    msgs::Set(msg.mutable_angular(),
              math::Vector3d(0.0, 0, desiredAngVel));
    pub.Publish(msg);

    // Give some time for message to be received
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    server.Run(true, 3000, false);

    // Poses for 4s
    ASSERT_EQ(4000u, poses.size());

    // verify that the vehicle is moving in +x and rotating towards +y
    for (unsigned int i = 1001; i < poses.size(); ++i)
    {
      EXPECT_GT(poses[i].Pos().X(), poses[i-1].Pos().X()) << i;
      EXPECT_GT(poses[i].Pos().Y(), poses[i-1].Pos().Y()) << i;
      EXPECT_NEAR(poses[i].Pos().Z(), poses[i-1].Pos().Z(), 1e-5);
      EXPECT_NEAR(poses[i].Rot().Euler().X(),
          poses[i-1].Rot().Euler().X(), 1e-5) << i;
      EXPECT_NEAR(poses[i].Rot().Euler().Y(),
          poses[i-1].Rot().Euler().Y(), 1e-5) << i;
      EXPECT_GT(poses[i].Rot().Euler().Z(), poses[i-1].Rot().Euler().Z()) << i;
    }
  }

  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _cmdVelTopic Command velocity topic.
  protected: void TestPublishLinkCmd(const std::string &_sdfFile,
                                 const std::string &_cmdVelTopic)
  {
    // Start server
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);

    // currently only tpe supports link velocity cmds
    serverConfig.SetPhysicsEngine("gz-physics-tpe-plugin");

    Server server(serverConfig);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    // Create a system that records the vehicle link poses
    test::Relay testSystem;

    std::vector<math::Pose3d> modelPoses;
    std::vector<math::Pose3d> linkPoses;
    testSystem.OnPostUpdate([&linkPoses, &modelPoses](
      const UpdateInfo &,
      const EntityComponentManager &_ecm)
      {
        auto modelId = _ecm.EntityByComponents(
          components::Model(),
          components::Name("vehicle_blue"));
        EXPECT_NE(kNullEntity, modelId);

        auto modelPoseComp = _ecm.Component<components::Pose>(modelId);
        ASSERT_NE(nullptr, modelPoseComp);

        modelPoses.push_back(modelPoseComp->Data());

        auto links = _ecm.ChildrenByComponents(modelId,
          components::Link(),
          components::Name("caster"));
        ASSERT_FALSE(links.empty());
        auto linkId = links.front();
        EXPECT_NE(kNullEntity, linkId);

        auto linkPoseComp = _ecm.Component<components::Pose>(linkId);
        ASSERT_NE(nullptr, linkPoseComp);

        linkPoses.push_back(linkPoseComp->Data());

      });
    server.AddSystem(testSystem.systemPtr);

    // Run server and check that vehicle didn't move
    server.Run(true, 1000, false);

    EXPECT_EQ(1000u, modelPoses.size());

    for (const auto &pose : modelPoses)
    {
      EXPECT_EQ(modelPoses[0], pose);
    }
    for (const auto &pose : linkPoses)
    {
      EXPECT_EQ(linkPoses[0], pose);
    }

    // Publish command and check that link moved
    transport::Node node;
    auto pub = node.Advertise<msgs::Twist>(_cmdVelTopic);

    msgs::Twist msg;

    const double desiredLinVel = 10.5;
    const double desiredAngVel = 0.2;
    msgs::Set(msg.mutable_linear(),
              math::Vector3d(desiredLinVel, 0, 0));
    msgs::Set(msg.mutable_angular(),
              math::Vector3d(0.0, 0, desiredAngVel));
    pub.Publish(msg);

    // Give some time for message to be received
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    server.Run(true, 3000, false);

    // Poses for 4s
    ASSERT_EQ(4000u, modelPoses.size());
    ASSERT_EQ(4000u, linkPoses.size());

    // verify that the model is stationary
    for (unsigned int i = 1001; i < modelPoses.size(); ++i)
    {
      EXPECT_EQ(modelPoses[0], modelPoses[i]);
    }

    // verify that the link is moving in +x and rotating about its origin
    for (unsigned int i = 1001; i < linkPoses.size(); ++i)
    {
      EXPECT_GT(linkPoses[i].Pos().X(), linkPoses[i-1].Pos().X()) << i;
      EXPECT_NEAR(linkPoses[i].Pos().Y(), linkPoses[i-1].Pos().Y(), 1e-5);
      EXPECT_NEAR(linkPoses[i].Pos().Z(), linkPoses[i-1].Pos().Z(), 1e-5);
      EXPECT_NEAR(linkPoses[i].Rot().Euler().X(),
          linkPoses[i-1].Rot().Euler().X(), 1e-5) << i;
      EXPECT_NEAR(linkPoses[i].Rot().Euler().Y(),
          linkPoses[i-1].Rot().Euler().Y(), 1e-5) << i;
      EXPECT_GT(linkPoses[i].Rot().Euler().Z(),
          linkPoses[i-1].Rot().Euler().Z()) << i;
    }
  }
};

/////////////////////////////////////////////////
// See: https://github.com/gazebosim/gz-sim/issues/1175
// See: https://github.com/gazebosim/gz-sim/issues/630
TEST_P(VelocityControlTest, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(PublishCmd))
{
  TestPublishCmd(
      std::string(PROJECT_SOURCE_PATH) + "/test/worlds/velocity_control.sdf",
      "/model/vehicle_blue/cmd_vel");
}

/////////////////////////////////////////////////
TEST_P(VelocityControlTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(PublishLinkCmd))
{
  TestPublishLinkCmd(
      std::string(PROJECT_SOURCE_PATH) + "/test/worlds/velocity_control.sdf",
      "/model/vehicle_blue/link/caster/cmd_vel");
}

/////////////////////////////////////////////////
TEST_P(VelocityControlTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ResetClearsModelCommand))
{
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/velocity_control.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  test::Relay testSystem;
  std::vector<math::Pose3d> poses;
  testSystem.OnPostUpdate([&poses](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      auto id = _ecm.EntityByComponents(
        components::Model(), components::Name("vehicle_blue"));
      ASSERT_NE(kNullEntity, id);

      auto poseComp = _ecm.Component<components::Pose>(id);
      ASSERT_NE(nullptr, poseComp);

      poses.push_back(poseComp->Data());
    });
  server.AddSystem(testSystem.systemPtr);

  server.Run(true, 500, false);
  ASSERT_EQ(500u, poses.size());
  const auto initialPose = poses.back();

  transport::Node node;
  auto pub = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  msgs::Twist msg;
  msgs::Set(msg.mutable_linear(), math::Vector3d(10.5, 0, 0));
  msgs::Set(msg.mutable_angular(), math::Vector3d(0, 0, 0.2));
  pub.Publish(msg);
  std::this_thread::sleep_for(100ms);

  server.Run(true, 1000, false);
  ASSERT_EQ(1500u, poses.size());
  EXPECT_GT(poses.back().Pos().Distance(initialPose.Pos()), 0.05);

  server.ResetAll();
  ASSERT_TRUE(test::StepUntil(server, 100,
      [&]
      {
        return poses.back().Pos().Distance(initialPose.Pos()) < 1e-3;
      }));

  const auto postResetStartIndex = poses.size();
  server.Run(true, 500, false);
  ASSERT_GT(poses.size(), postResetStartIndex + 1u);

  // Without a fresh command, the pre-reset command must not keep driving the
  // model in the new episode.
  const auto postResetStart = poses[postResetStartIndex];
  const auto postResetEnd = poses.back();
  EXPECT_LT(postResetEnd.Pos().Distance(postResetStart.Pos()), 0.03);

  pub.Publish(msg);
  std::this_thread::sleep_for(100ms);
  const auto freshStartIndex = poses.size();
  server.Run(true, 500, false);
  ASSERT_GT(poses.size(), freshStartIndex + 1u);
  EXPECT_GT(poses.back().Pos().Distance(poses[freshStartIndex].Pos()), 0.02);
}

/////////////////////////////////////////////////
TEST_P(VelocityControlTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ResetClearsLinkCommand))
{
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/velocity_control.sdf");
  serverConfig.SetPhysicsEngine("gz-physics-tpe-plugin");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  test::Relay testSystem;
  std::vector<math::Pose3d> linkPoses;
  testSystem.OnPostUpdate([&linkPoses](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      auto linkId = _ecm.EntityByComponents(
        components::Link(), components::Name("caster"));
      ASSERT_NE(kNullEntity, linkId);

      auto poseComp = _ecm.Component<components::Pose>(linkId);
      ASSERT_NE(nullptr, poseComp);

      linkPoses.push_back(poseComp->Data());
    });
  server.AddSystem(testSystem.systemPtr);

  server.Run(true, 500, false);
  ASSERT_EQ(500u, linkPoses.size());
  const auto initialPose = linkPoses.back();

  transport::Node node;
  auto pub = node.Advertise<msgs::Twist>(
      "/model/vehicle_blue/link/caster/cmd_vel");
  msgs::Twist msg;
  msgs::Set(msg.mutable_linear(), math::Vector3d(10.5, 0, 0));
  msgs::Set(msg.mutable_angular(), math::Vector3d(0, 0, 0.2));
  pub.Publish(msg);
  std::this_thread::sleep_for(100ms);

  server.Run(true, 1000, false);
  ASSERT_EQ(1500u, linkPoses.size());
  EXPECT_GT(linkPoses.back().Pos().Distance(initialPose.Pos()), 0.05);

  const auto preResetPoseCount = linkPoses.size();
  server.ResetAll();
  ASSERT_TRUE(test::StepUntil(server, 100,
      [&]
      {
        return linkPoses.size() > preResetPoseCount;
      }));

  const auto postResetStartIndex = linkPoses.size();
  server.Run(true, 500, false);
  ASSERT_GT(linkPoses.size(), postResetStartIndex + 1u);

  // The link may settle to a reset baseline, but the old link command should
  // not continue to move it across the post-reset window.
  const auto postResetStart = linkPoses[postResetStartIndex];
  const auto postResetEnd = linkPoses.back();
  EXPECT_LT(postResetEnd.Pos().Distance(postResetStart.Pos()), 0.03);

  pub.Publish(msg);
  std::this_thread::sleep_for(100ms);
  const auto freshStartIndex = linkPoses.size();
  server.Run(true, 500, false);
  ASSERT_GT(linkPoses.size(), freshStartIndex + 1u);
  EXPECT_GT(linkPoses.back().Pos().Distance(linkPoses[freshStartIndex].Pos()),
      0.02);
}

// Run multiple times
INSTANTIATE_TEST_SUITE_P(ServerRepeat, VelocityControlTest,
    ::testing::Range(1, 2));
