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

#include <string>
#include <vector>
#include <functional>
#include <thread>

#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>


#include "gz/sim/components/Name.hh"
#include "test_config.hh"

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Server.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

#define tol 10e-4

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test MecanumDrive system
class MecanumDriveTest : public InternalFixture<::testing::Test>
{
  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _cmdVelTopic Command velocity topic.
  /// \param[in] _odomTopic Odometry topic.
  protected: void TestPublishCmd(const std::string &_sdfFile,
                                 const std::string &_cmdVelTopic,
                                 const std::string &_odomTopic)
  {
    /// \param[in] fb_component forward/backward motion vector component.
    /// \param[in] lr_component left/right motion vector component.
    /// \param[in] yaw_component yaw rotation component.
    auto testCmdVel = [&](double fb_component,
                          double lr_component,
                          double yaw_component)
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

      // Get odometry messages
      double period{1.0 / 50.0};
      double lastMsgTime{1.0};
      std::vector<math::Pose3d> odomPoses;
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
        };

      // Publish command and check that vehicle moved
      transport::Node node;
      auto pub = node.Advertise<msgs::Twist>(_cmdVelTopic);
      node.Subscribe(_odomTopic, odomCb);

      msgs::Twist msg;

      // Avoid wheel slip by limiting acceleration (1 m/s^2)
      // and max velocity (0.5 m/s).
      // See <max_velocity> and <max_aceleration> parameters
      // in "/test/worlds/mecanum_drive.sdf".
      // See <min_velocity>, <min_aceleration>, <min_jerk> and
      // <max_jerk> parameters in "/test/worlds/mecanum_drive.sdf".
      test::Relay velocityRamp;
      double desiredLinVelX = fb_component;
      double desiredLinVelY = lr_component;
      double desiredAngVel  = yaw_component;

      velocityRamp.OnPreUpdate(
          [&](const UpdateInfo &/*_info*/,
              const EntityComponentManager &)
          {
            msgs::Set(msg.mutable_linear(),
                      math::Vector3d(desiredLinVelX, desiredLinVelY, 0));
            msgs::Set(msg.mutable_angular(),
                      math::Vector3d(0.0, 0, desiredAngVel));
            pub.Publish(msg);
          });

      server.AddSystem(velocityRamp.systemPtr);

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

      // Odometry calculates the pose of a point that is located half way
      // between the two wheels, not the origin of the model. For example,
      // if the vehicle is commanded to rotate in place, the vehicle will
      // rotate about the point half way between the two wheels, thus,
      // the odometry position will remain zero.
      // However, since the model origin is offset, the model position will
      // change. To find the final pose of the model, we have to do the
      // following similarity transformation

      math::Pose3d tOdomModel(-0.2, 0, 0, 0, 0, 0);
      auto finalModelFramePose =
          tOdomModel * odomPoses.back() * tOdomModel.Inverse();

      // Odom for 3s
      ASSERT_FALSE(odomPoses.empty());
      EXPECT_EQ(150u, odomPoses.size());

       auto relativeMotionPos = poses[3999].Pos() - poses[0].Pos();
       auto relativeMotionRot = poses[3999].Rot().Yaw() - poses[0].Rot().Yaw();

      // if the linear motion is non-zero
      // the vehicle should move in the commanded direction
      if (std::abs(fb_component) > 0.0 || std::abs(lr_component) > 0.0)
      {
        auto cmdVelDir = math::Vector2(desiredLinVelX,
                                       desiredLinVelY).Normalized();
        auto motionDir = math::Vector2(relativeMotionPos.X(),
                                       relativeMotionPos.Y()).Normalized();
        auto scalarProjection = cmdVelDir.Dot(motionDir);
        EXPECT_GT(motionDir.Length(), 0.5);  // is there motion?
        EXPECT_GT(scalarProjection, 0.9);  // is it in the right direction?

        // Verify velocity and acceleration upper boundaries.
        double t = 3.0;
        double d = poses[3999].Pos().Distance(poses[0].Pos());
        double v0 = 0;
        double v = d / t;
        double a = (v - v0) / t;

        // The vehicle should not exceed the max velocity and acceleration
        // notice the limits for the x and y direction are separately
        // enforced
        auto threshold_factor = math::Vector2(fb_component,
                                              lr_component).Length();

        EXPECT_LT(v, 0.5 * threshold_factor);
        EXPECT_LT(a, 1.0 * threshold_factor);
        EXPECT_GT(v, -0.5 * threshold_factor);
        EXPECT_GT(a, -1.0 * threshold_factor);
      }

      // The vehicle should rotate in the commanded direction
      if (std::abs(yaw_component) > 0.0)
      {
        // the relative rotation has the same sign as the commanded rotation
        EXPECT_GT(desiredAngVel * relativeMotionRot, 0.0);
      }

      // The value from odometry will be close, but not exactly the ground truth
      // pose of the robot model. This is partially due to throttling the
      // odometry publisher, and partially due to wheel slip.
      EXPECT_NEAR(poses[1020].Pos().X(), odomPoses[0].Pos().X(), 1e-2);
      EXPECT_NEAR(poses[1020].Pos().Y(), odomPoses[0].Pos().Y(), 1e-2);
      EXPECT_NEAR(poses.back().Pos().X(), finalModelFramePose.Pos().X(), 1e-2);
      EXPECT_NEAR(poses.back().Pos().Y(), finalModelFramePose.Pos().Y(), 1e-2);


    };

    testCmdVel(0., 0., 0.); /* no motion */

    testCmdVel(+1., 0., 0.); /* pure forward */
    testCmdVel(-1., 0., 0.); /* pure backward */
    testCmdVel(0., +1., 0.); /* pure left */
    testCmdVel(0., -1., 0.); /* pure right */

    testCmdVel(+1., +1., 0.); /* forward left */
    testCmdVel(+1., -1., 0.); /* forward right */
    testCmdVel(-1., +1., 0.); /* backward left */
    testCmdVel(-1., -1., 0.); /* backward right */

    testCmdVel(0., 0., +0.1); /* CW motion */
    testCmdVel(0., 0., -0.1); /* CCW motion */
  }
};

/////////////////////////////////////////////////
TEST_F(MecanumDriveTest, PublishCmd)
{
  TestPublishCmd(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds", "mecanum_drive.sdf"),
      "/model/vehicle_blue/cmd_vel", "/model/vehicle_blue/odometry");
}

/////////////////////////////////////////////////
TEST_F(MecanumDriveTest,
       GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(PublishCmdCustomTopics))
{
  TestPublishCmd(common::joinPaths(
      std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "mecanum_drive_custom_topics.sdf"),
      "/model/foo/cmdvel", "/model/bar/odom");
}

// /////////////////////////////////////////////////
TEST_F(MecanumDriveTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(OdomFrameId))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "mecanum_drive.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  unsigned int odomPosesCount = 0;
  std::function<void(const msgs::Odometry &)> odomCb =
    [&odomPosesCount](const msgs::Odometry &_msg)
    {
      ASSERT_TRUE(_msg.has_header());
      ASSERT_TRUE(_msg.header().has_stamp());

      ASSERT_GT(_msg.header().data_size(), 1);

      EXPECT_STREQ(_msg.header().data(0).key().c_str(), "frame_id");
      EXPECT_STREQ(
        _msg.header().data(0).value().Get(0).c_str(), "vehicle_blue/odom");

      EXPECT_STREQ(_msg.header().data(1).key().c_str(), "child_frame_id");
      EXPECT_STREQ(
        _msg.header().data(1).value().Get(0).c_str(), "vehicle_blue/chassis");

      odomPosesCount++;
    };

  transport::Node node;
  auto pub = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  node.Subscribe("/model/vehicle_blue/odometry", odomCb);

  msgs::Twist msg;
  msgs::Set(msg.mutable_linear(), math::Vector3d(0.5, 0, 0));
  msgs::Set(msg.mutable_angular(), math::Vector3d(0.0, 0, 0.2));

  pub.Publish(msg);

  server.Run(true, 100, false);

  int sleep = 0;
  int maxSleep = 30;
  // cppcheck-suppress knownConditionTrueFalse
  for (; odomPosesCount < 5 && sleep < maxSleep; ++sleep) // NOLINT
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(maxSleep, sleep);

  EXPECT_EQ(5u, odomPosesCount);
}

// /////////////////////////////////////////////////
TEST_F(MecanumDriveTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(OdomCustomFrameId))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "mecanum_drive_custom_frame_id.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  unsigned int odomPosesCount = 0;
  std::function<void(const msgs::Odometry &)> odomCb =
    [&odomPosesCount](const msgs::Odometry &_msg)
    {
      ASSERT_TRUE(_msg.has_header());
      ASSERT_TRUE(_msg.header().has_stamp());

      ASSERT_GT(_msg.header().data_size(), 1);

      EXPECT_STREQ(_msg.header().data(0).key().c_str(), "frame_id");
      EXPECT_STREQ(_msg.header().data(0).value().Get(0).c_str(), "odom");

      EXPECT_STREQ(_msg.header().data(1).key().c_str(), "child_frame_id");
      EXPECT_STREQ(
            _msg.header().data(1).value().Get(0).c_str(), "base_footprint");

      odomPosesCount++;
    };

  transport::Node node;
  auto pub = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  node.Subscribe("/model/vehicle_blue/odometry", odomCb);

  msgs::Twist msg;
  msgs::Set(msg.mutable_linear(), math::Vector3d(0.5, 0, 0));
  msgs::Set(msg.mutable_angular(), math::Vector3d(0.0, 0, 0.2));

  pub.Publish(msg);

  server.Run(true, 100, false);

  int sleep = 0;
  int maxSleep = 30;
  // cppcheck-suppress knownConditionTrueFalse
  for (; odomPosesCount < 5 && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(maxSleep, sleep);

  EXPECT_EQ(5u, odomPosesCount);
}

/////////////////////////////////////////////////
TEST_F(MecanumDriveTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Pose_VFrameId))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "mecanum_drive.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  unsigned int odomPosesCount = 0;
  std::function<void(const msgs::Pose_V &)> pose_VCb =
    [&odomPosesCount](const msgs::Pose_V &_msg)
    {
      ASSERT_TRUE(_msg.pose(0).has_header());
      ASSERT_TRUE(_msg.pose(0).header().has_stamp());

      ASSERT_GT(_msg.pose(0).header().data_size(), 1);

      EXPECT_STREQ(_msg.pose(0).header().data(0).key().c_str(),
                   "frame_id");
      EXPECT_STREQ(_msg.pose(0).header().data(0).value().Get(0).c_str(),
                   "vehicle_blue/odom");

      EXPECT_STREQ(_msg.pose(0).header().data(1).key().c_str(),
                   "child_frame_id");
      EXPECT_STREQ(_msg.pose(0).header().data(1).value().Get(0).c_str(),
                   "vehicle_blue/chassis");

      odomPosesCount++;
    };

  transport::Node node;
  auto pub = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  node.Subscribe("/model/vehicle_blue/tf", pose_VCb);

  msgs::Twist msg;
  msgs::Set(msg.mutable_linear(), math::Vector3d(0.5, 0, 0));
  msgs::Set(msg.mutable_angular(), math::Vector3d(0.0, 0, 0.2));

  pub.Publish(msg);

  server.Run(true, 100, false);

  int sleep = 0;
  int maxSleep = 30;
  // cppcheck-suppress knownConditionTrueFalse
  for (; odomPosesCount < 5 && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(maxSleep, sleep);

  EXPECT_EQ(5u, odomPosesCount);
}

/////////////////////////////////////////////////
TEST_F(MecanumDriveTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Pose_VCustomFrameId))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "mecanum_drive_custom_frame_id.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  unsigned int odomPosesCount = 0;
  std::function<void(const msgs::Pose_V &)> Pose_VCb =
    [&odomPosesCount](const msgs::Pose_V &_msg)
    {
      ASSERT_TRUE(_msg.pose(0).has_header());
      ASSERT_TRUE(_msg.pose(0).header().has_stamp());

      ASSERT_GT(_msg.pose(0).header().data_size(), 1);

      EXPECT_STREQ(_msg.pose(0).header().data(0).key().c_str(),
                   "frame_id");
      EXPECT_STREQ(_msg.pose(0).header().data(0).value().Get(0).c_str(),
                   "odom");

      EXPECT_STREQ(_msg.pose(0).header().data(1).key().c_str(),
                   "child_frame_id");
      EXPECT_STREQ(_msg.pose(0).header().data(1).value().Get(0).c_str(),
            "base_footprint");

      odomPosesCount++;
    };

  transport::Node node;
  auto pub = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  node.Subscribe("/model/vehicle_blue/tf", Pose_VCb);

  msgs::Twist msg;
  msgs::Set(msg.mutable_linear(), math::Vector3d(0.5, 0, 0));
  msgs::Set(msg.mutable_angular(), math::Vector3d(0.0, 0, 0.2));

  pub.Publish(msg);

  server.Run(true, 100, false);

  int sleep = 0;
  int maxSleep = 30;
  // cppcheck-suppress knownConditionTrueFalse
  for (; odomPosesCount < 5 && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(maxSleep, sleep);

  EXPECT_EQ(5u, odomPosesCount);
}

/////////////////////////////////////////////////
TEST_F(MecanumDriveTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Pose_VCustomTfTopic))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "mecanum_drive_custom_tf_topic.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  unsigned int odomPosesCount = 0;
  std::function<void(const msgs::Pose_V &)> pose_VCb =
    [&odomPosesCount](const msgs::Pose_V &_msg)
    {
      ASSERT_TRUE(_msg.pose(0).has_header());
      ASSERT_TRUE(_msg.pose(0).header().has_stamp());

      ASSERT_GT(_msg.pose(0).header().data_size(), 1);

      EXPECT_STREQ(_msg.pose(0).header().data(0).key().c_str(), "frame_id");
      EXPECT_STREQ(
            _msg.pose(0).header().data(0).value().Get(0).c_str(),
            "vehicle_blue/odom");

      EXPECT_STREQ(
            _msg.pose(0).header().data(1).key().c_str(), "child_frame_id");
      EXPECT_STREQ(
            _msg.pose(0).header().data(1).value().Get(0).c_str(),
            "vehicle_blue/chassis");

      odomPosesCount++;
    };

  transport::Node node;
  auto pub = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  node.Subscribe("/tf_foo", pose_VCb);

  msgs::Twist msg;
  msgs::Set(msg.mutable_linear(), math::Vector3d(0.5, 0, 0));
  msgs::Set(msg.mutable_angular(), math::Vector3d(0.0, 0, 0.2));

  pub.Publish(msg);

  server.Run(true, 100, false);

  int sleep = 0;
  int maxSleep = 30;
  // cppcheck-suppress knownConditionTrueFalse
  for (; odomPosesCount < 5 && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(maxSleep, sleep);

  EXPECT_EQ(5u, odomPosesCount);
}
