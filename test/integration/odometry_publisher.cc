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
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/AngularVelocityCmd.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/LinearVelocityCmd.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

#define tol 0.005

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

/// \brief Test OdometryPublisher system
class OdometryPublisherTest : public ::testing::TestWithParam<int>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }

  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _odomTopic Odometry topic.
  protected: void TestMovement(const std::string &_sdfFile,
                               const std::string &_odomTopic)
  {
    // Start server
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);

    Server server(serverConfig);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    // Create a system that records the vehicle poses and velocities
    test::Relay testSystem;

    std::vector<math::Pose3d> poses;
    testSystem.OnPostUpdate([&poses](const gazebo::UpdateInfo &,
      const gazebo::EntityComponentManager &_ecm)
      {
        auto id = _ecm.EntityByComponents(
          components::Model(),
          components::Name("vehicle"));
        EXPECT_NE(kNullEntity, id);

        auto poseComp = _ecm.Component<components::Pose>(id);
        ASSERT_NE(nullptr, poseComp);
        poses.push_back(poseComp->Data());
      });
    server.AddSystem(testSystem.systemPtr);

    // Run server while model is stationary
    server.Run(true, 1000, false);

    EXPECT_EQ(1000u, poses.size());

    for (const auto &pose : poses)
    {
      EXPECT_EQ(poses[0], pose);
    }

    // 50 Hz is the default publishing freq
    double period{1.0 / 50.0};
    double lastMsgTime{1.0};
    std::vector<math::Pose3d> odomPoses;
    std::vector<math::Vector3d> odomLinVels;
    std::vector<math::Vector3d> odomAngVels;
    // Create function to store data from odometry messages
    std::function<void(const msgs::Odometry &)> odomCb =
      [&](const msgs::Odometry &_msg)
      {
        ASSERT_TRUE(_msg.has_header());
        ASSERT_TRUE(_msg.header().has_stamp());

        double msgTime =
            static_cast<double>(_msg.header().stamp().sec()) +
            static_cast<double>(_msg.header().stamp().nsec()) * 1e-9;

        EXPECT_DOUBLE_EQ(msgTime, lastMsgTime + period);
        lastMsgTime = msgTime;

        odomPoses.push_back(msgs::Convert(_msg.pose()));
        odomLinVels.push_back(msgs::Convert(_msg.twist().linear()));
        odomAngVels.push_back(msgs::Convert(_msg.twist().angular()));
      };
    transport::Node node;
    node.Subscribe(_odomTopic, odomCb);

    test::Relay velocityRamp;
    math::Vector3d linVelCmd(1, 0.5, 0.0);
    math::Vector3d angVelCmd(0.0, 0.0, 0.2);
    velocityRamp.OnPreUpdate(
        [&](const gazebo::UpdateInfo &/*_info*/,
            gazebo::EntityComponentManager &_ecm)
        {
          auto en = _ecm.EntityByComponents(
            components::Model(),
            components::Name("vehicle"));
          EXPECT_NE(kNullEntity, en);

          // Set the linear velocity of the model
          auto linVelCmdComp =
            _ecm.Component<components::LinearVelocityCmd>(en);
          if (!linVelCmdComp)
          {
            _ecm.CreateComponent(en,
                components::LinearVelocityCmd(linVelCmd));
          }
          else
          {
            linVelCmdComp->Data() = linVelCmd;
          }

          // Set the angular velocity of the model
          auto angVelCmdComp =
            _ecm.Component<components::AngularVelocityCmd>(en);
          if (!angVelCmdComp)
          {
            _ecm.CreateComponent(en,
                components::AngularVelocityCmd(angVelCmd));
          }
          else
          {
            angVelCmdComp->Data() = angVelCmd;
          }
        });

    server.AddSystem(velocityRamp.systemPtr);

    // Run server while the model moves with the velocities set earlier
    server.Run(true, 3000, false);

    // Poses for 4s
    ASSERT_EQ(4000u, poses.size());

    int sleep = 0;
    int maxSleep = 30;
    for (; odomPoses.size() < 150 && sleep < maxSleep; ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_NE(maxSleep, sleep);

    // Odom for 3s
    ASSERT_FALSE(odomPoses.empty());
    EXPECT_EQ(150u, odomPoses.size());
    EXPECT_EQ(150u, odomLinVels.size());
    EXPECT_EQ(150u, odomAngVels.size());

    // Check accuracy of poses published in the odometry message
    auto finalModelFramePose = odomPoses.back();
    EXPECT_NEAR(poses[1020].Pos().X(), odomPoses[0].Pos().X(), 1e-2);
    EXPECT_NEAR(poses[1020].Pos().Y(), odomPoses[0].Pos().Y(), 1e-2);
    EXPECT_NEAR(poses[1020].Pos().Z(), odomPoses[0].Pos().Z(), 1e-2);
    EXPECT_NEAR(poses[1020].Rot().X(), odomPoses[0].Rot().X(), 1e-2);
    EXPECT_NEAR(poses[1020].Rot().Y(), odomPoses[0].Rot().Y(), 1e-2);
    EXPECT_NEAR(poses[1020].Rot().Z(), odomPoses[0].Rot().Z(), 1e-2);
    EXPECT_NEAR(poses.back().Pos().X(), finalModelFramePose.Pos().X(), 1e-2);
    EXPECT_NEAR(poses.back().Pos().Y(), finalModelFramePose.Pos().Y(), 1e-2);
    EXPECT_NEAR(poses.back().Pos().Z(), finalModelFramePose.Pos().Z(), 1e-2);
    EXPECT_NEAR(poses.back().Rot().X(), finalModelFramePose.Rot().X(), 1e-2);
    EXPECT_NEAR(poses.back().Rot().Y(), finalModelFramePose.Rot().Y(), 1e-2);
    EXPECT_NEAR(poses.back().Rot().Z(), finalModelFramePose.Rot().Z(), 1e-2);

    // Check accuracy of velocities published in the odometry message
    EXPECT_NEAR(odomLinVels[5].X(), linVelCmd[0], 1e-1);
    EXPECT_NEAR(odomLinVels[5].Y(), linVelCmd[1], 1e-1);
    EXPECT_NEAR(odomLinVels[5].Z(), 0.0, 1e-1);
    EXPECT_NEAR(odomAngVels[5].X(), 0.0, 1e-1);
    EXPECT_NEAR(odomAngVels[5].Y(), 0.0, 1e-1);
    EXPECT_NEAR(odomAngVels[5].Z(), angVelCmd[2], 1e-1);

    EXPECT_NEAR(odomLinVels.back().X(), linVelCmd[0], 1e-1);
    EXPECT_NEAR(odomLinVels.back().Y(), linVelCmd[1], 1e-1);
    EXPECT_NEAR(odomLinVels.back().Z(), 0.0, 1e-1);
    EXPECT_NEAR(odomAngVels.back().X(), 0.0, 1e-1);
    EXPECT_NEAR(odomAngVels.back().Y(), 0.0, 1e-1);
    EXPECT_NEAR(odomAngVels.back().Z(), angVelCmd[2], 1e-1);
  }

  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _odomTopic Odometry topic.
  /// \param[in] _frameId Name of the world-fixed coordinate frame
  /// for the odometry message.
  /// \param[in] _childFrameId Name of the coordinate frame rigidly
  /// attached to the mobile robot base.
  protected: void TestPublishCmd(const std::string &_sdfFile,
                                 const std::string &_odomTopic,
                                 const std::string &_frameId,
                                 const std::string &_childFrameId)
  {
    // Start server
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);

    Server server(serverConfig);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    server.SetUpdatePeriod(0ns);

    unsigned int odomPosesCount = 0;
    std::function<void(const msgs::Odometry &)> odomCb =
      [&odomPosesCount, &_frameId, &_childFrameId](const msgs::Odometry &_msg)
      {
        ASSERT_TRUE(_msg.has_header());
        ASSERT_TRUE(_msg.header().has_stamp());

        ASSERT_GT(_msg.header().data_size(), 1);

        EXPECT_STREQ(_msg.header().data(0).key().c_str(), "frame_id");
        EXPECT_STREQ(
          _msg.header().data(0).value().Get(0).c_str(),
          _frameId.c_str());

        EXPECT_STREQ(_msg.header().data(1).key().c_str(), "child_frame_id");
        EXPECT_STREQ(
              _msg.header().data(1).value().Get(0).c_str(),
              _childFrameId.c_str());

        odomPosesCount++;
      };

    transport::Node node;
    node.Subscribe(_odomTopic, odomCb);

    server.Run(true, 100, false);

    int sleep = 0;
    int maxSleep = 30;
    for (; odomPosesCount < 5 && sleep < maxSleep; ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_NE(maxSleep, sleep);

    EXPECT_EQ(5u, odomPosesCount);
  }
};

/////////////////////////////////////////////////
TEST_P(OdometryPublisherTest, Movement)
{
  TestMovement(
      std::string(PROJECT_SOURCE_PATH) + "/test/worlds/odometry_publisher.sdf",
      "/model/vehicle/odometry");
}

/////////////////////////////////////////////////
TEST_P(OdometryPublisherTest, MovementCustomTopic)
{
  TestMovement(
      std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/odometry_publisher_custom.sdf",
      "/model/bar/odom");
}

/////////////////////////////////////////////////
TEST_P(OdometryPublisherTest, OdomFrameId)
{
  TestPublishCmd(
      std::string(PROJECT_SOURCE_PATH) + "/test/worlds/odometry_publisher.sdf",
      "/model/vehicle/odometry",
      "vehicle/odom",
      "vehicle/base_footprint");
}

/////////////////////////////////////////////////
TEST_P(OdometryPublisherTest, OdomCustomFrameId)
{
  TestPublishCmd(
      std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/odometry_publisher_custom.sdf",
      "/model/bar/odom",
      "odomCustom",
      "baseCustom");
}

// Run multiple times
INSTANTIATE_TEST_SUITE_P(ServerRepeat, OdometryPublisherTest,
    ::testing::Range(1, 2));
