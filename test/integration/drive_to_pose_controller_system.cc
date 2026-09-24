/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/convert/Pose.hh>

#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include <chrono>
#include <functional>
#include <string>

#include "gz/sim/Server.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test DriveToPoseController system
class DriveToPoseControllerTest
  : public InternalFixture<::testing::Test>
{
  /// \param[in] _sdfFile SDF file to load
  /// \param[in] _topicPrefix Prefix for all topics
  /// \param[in] _pose Pose command
  protected: void TestPublishCmd(const std::string& _sdfFile,
                                 const std::string& _topicPrefix,
                                 const math::Pose3d& _pose)
  {
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);

    Server server(serverConfig);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    server.SetUpdatePeriod(0ns);

    const std::string cmdPoseTopic = _topicPrefix + "/cmd_pose";
    const std::string cmdVelTopic = _topicPrefix + "/cmd_vel";
    const std::string reachedPoseTopic = _topicPrefix + "/reached_pose";
    const std::string odomPoseTopic = _topicPrefix + "/pose";

    // Get the reached pose message
    math::Pose3d reachedPose;
    std::function<void(const msgs::Pose&)> reachedPoseCb =
      [&](const msgs::Pose &_msg)
      {
        reachedPose = msgs::Convert(_msg);
      };

    // Get the odom pose message
    math::Pose3d odomPose;
    std::function<void(const msgs::Pose_V&)> odomPoseCb =
      [&](const msgs::Pose_V &_msg)
      {
        odomPose = msgs::Convert(_msg.pose(0));
      };

    // Count the last published linear and angular velocities and the total
    // msg count
    double lastLinearVel;
    double lastAngularVel;
    int msgCount = 0;
    std::function<void(const msgs::Twist&)> cmdVelCb =
      [&](const msgs::Twist &_msg)
      {
        ++msgCount;
        lastLinearVel = _msg.linear().x();
        lastAngularVel = _msg.angular().y();
      };

    transport::Node node;
    auto pub = node.Advertise<msgs::Pose_V>(cmdPoseTopic);
    node.Subscribe(reachedPoseTopic, reachedPoseCb);
    node.Subscribe(odomPoseTopic, odomPoseCb);
    node.Subscribe(cmdVelTopic, cmdVelCb);

    // Publish a pose command
    msgs::Pose_V poseMsg;
    auto poseMsgPose = poseMsg.add_pose();
    poseMsgPose->CopyFrom(msgs::Convert(_pose));
    pub.Publish(poseMsg);

    // Run the system for 2s
    test::Relay testSystem;
    testSystem.OnPreUpdate(
      [&](const UpdateInfo&,
          const EntityComponentManager&){});
    server.AddSystem(testSystem.systemPtr);
    server.Run(true, 2000, false);

    // Verify the final pose with the actual odom pose
    ASSERT_NEAR(_pose.X(), odomPose.X(), 0.1);
    ASSERT_NEAR(_pose.Y(), odomPose.Y(), 0.1);
    ASSERT_NEAR(_pose.Yaw(), odomPose.Yaw(), 0.05);

    // Verify the final pose with the reached pose acknowledgement
    ASSERT_NEAR(_pose.X(), reachedPose.X(), 0.1);
    ASSERT_NEAR(_pose.Y(), reachedPose.Y(), 0.1);
    ASSERT_NEAR(_pose.Yaw(), reachedPose.Yaw(), 0.05);

    // Verify the final velocity and check if the msg count is more than 1
    ASSERT_EQ(lastLinearVel, 0.0);
    ASSERT_EQ(lastAngularVel, 0.0);
    ASSERT_GT(msgCount, 0);
  }
};

/////////////////////////////////////////////////
TEST_F(DriveToPoseControllerTest, CurrentPosePublish)
{
  math::Pose3d pose(0, 0, 0, 0, 0, 0);

  TestPublishCmd(
    gz::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "drive_to_pose_controller.sdf"),
    "/model/DeliveryBot",
    pose);
}

/////////////////////////////////////////////////
TEST_F(DriveToPoseControllerTest, XCoordinatePublish)
{
  math::Pose3d pose(1.5, 0, 0, 0, 0, 0);

  TestPublishCmd(
    gz::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "drive_to_pose_controller.sdf"),
   "/model/DeliveryBot",
    pose);
}

/////////////////////////////////////////////////
TEST_F(DriveToPoseControllerTest, YCoordinatePublish)
{
  math::Pose3d pose(0, 1.5, 0, 0, 0, 0);

  TestPublishCmd(
    gz::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "drive_to_pose_controller.sdf"),
   "/model/DeliveryBot",
    pose);
}

/////////////////////////////////////////////////
TEST_F(DriveToPoseControllerTest, YawPublish)
{
  math::Pose3d pose(0, 0, 0, 0, 0, -1.57);

  TestPublishCmd(
    gz::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "drive_to_pose_controller.sdf"),
   "/model/DeliveryBot",
    pose);
}

/////////////////////////////////////////////////
TEST_F(DriveToPoseControllerTest, XYCoordinateYawPublish)
{
  math::Pose3d pose(1.5, -1.5, 0, 0, 0, 1.57);

  TestPublishCmd(
    gz::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "drive_to_pose_controller.sdf"),
   "/model/DeliveryBot",
    pose);
}
