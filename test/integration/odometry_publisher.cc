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

#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

#define tol 0.005

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test OdometryPublisher system
class OdometryPublisherTest
  : public InternalFixture<::testing::TestWithParam<int>>
{
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
    testSystem.OnPostUpdate([&poses](const sim::UpdateInfo &,
      const sim::EntityComponentManager &_ecm)
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
        [&](const sim::UpdateInfo &/*_info*/,
            sim::EntityComponentManager &_ecm)
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
  /// \param[in] _tfTopic TF / Pose_V topic.
  /// \param[in] _frameId Name of the world-fixed coordinate frame
  /// for the odometry message.
  /// \param[in] _childFrameId Name of the coordinate frame rigidly
  /// attached to the mobile robot base.
  protected: void TestMovement3d(const std::string &_sdfFile,
                                 const std::string &_odomTopic,
                                 const std::string &_tfTopic,
                                 const std::string &_frameId,
                                 const std::string &_childFrameId)
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
    testSystem.OnPostUpdate([&poses](const sim::UpdateInfo &,
      const sim::EntityComponentManager &_ecm)
      {
        auto id = _ecm.EntityByComponents(
          components::Model(),
          components::Name("X3"));
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
    std::vector<math::Pose3d> tfPoses;
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
    // Create function to store data from Pose_V messages
    std::function<void(const msgs::Pose_V &)> tfCb =
      [&](const msgs::Pose_V &_msg)
      {
        ASSERT_EQ(_msg.pose_size(), 1);
        EXPECT_TRUE(_msg.pose(0).has_header());
        EXPECT_TRUE(_msg.pose(0).has_position());
        EXPECT_TRUE(_msg.pose(0).has_orientation());

        ASSERT_EQ(_msg.pose(0).header().data_size(), 2);

        EXPECT_EQ(_msg.pose(0).header().data(0).key(), "frame_id");
        EXPECT_EQ(_msg.pose(0).header().data(0).value().Get(0), _frameId);

        EXPECT_EQ(_msg.pose(0).header().data(1).key(), "child_frame_id");
        EXPECT_EQ(_msg.pose(0).header().data(1).value().Get(0), _childFrameId);

        tfPoses.push_back(msgs::Convert(_msg.pose(0)));
      };
    // Create node for publishing twist messages
    transport::Node node;
    auto cmdVel = node.Advertise<msgs::Twist>("/X3/gazebo/command/twist");
    node.Subscribe(_odomTopic, odomCb);
    node.Subscribe(_tfTopic, tfCb);

    test::Relay velocityRamp;
    math::Vector3d linVelCmd(0.5, 0.3, 1.5);
    math::Vector3d angVelCmd(0.0, 0.0, 0.2);
    velocityRamp.OnPreUpdate(
        [&](const sim::UpdateInfo &/*_info*/,
            sim::EntityComponentManager &/*_ecm*/)
        {
          msgs::Twist msg;
          msgs::Set(msg.mutable_linear(), linVelCmd);
          msgs::Set(msg.mutable_angular(), angVelCmd);
          cmdVel.Publish(msg);
        });

    server.AddSystem(velocityRamp.systemPtr);

    // Run server while the model moves with the velocities set earlier
    server.Run(true, 3000, false);

    // Poses for 4s
    ASSERT_EQ(4000u, poses.size());

    int sleep = 0;
    int maxSleep = 30;
    for (; (odomPoses.size() < 150 || tfPoses.size() < 150) &&
        sleep < maxSleep; ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    EXPECT_NE(maxSleep, sleep);

    // Odom for 3s
    ASSERT_FALSE(odomPoses.empty());
    EXPECT_EQ(150u, odomPoses.size());
    EXPECT_EQ(150u, odomLinVels.size());
    EXPECT_EQ(150u, odomAngVels.size());
    EXPECT_EQ(150u, tfPoses.size());

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
    EXPECT_NEAR(odomLinVels.back().X(), linVelCmd[0], 1e-1);
    EXPECT_NEAR(odomLinVels.back().Y(), linVelCmd[1], 1e-1);
    EXPECT_NEAR(odomLinVels.back().Z(), linVelCmd[2], 1e-1);
    EXPECT_NEAR(odomAngVels.back().X(), 0.0, 1e-1);
    EXPECT_NEAR(odomAngVels.back().Y(), 0.0, 1e-1);
    EXPECT_NEAR(odomAngVels.back().Z(), angVelCmd[2], 1e-1);

    // Check TF
    EXPECT_NEAR(poses.back().Pos().X(), tfPoses.back().Pos().X(), 1e-2);
    EXPECT_NEAR(poses.back().Pos().Y(), tfPoses.back().Pos().Y(), 1e-2);
    EXPECT_NEAR(poses.back().Pos().Z(), tfPoses.back().Pos().Z(), 1e-2);
    EXPECT_NEAR(poses.back().Rot().X(), tfPoses.back().Rot().X(), 1e-2);
    EXPECT_NEAR(poses.back().Rot().Y(), tfPoses.back().Rot().Y(), 1e-2);
    EXPECT_NEAR(poses.back().Rot().Z(), tfPoses.back().Rot().Z(), 1e-2);
  }

  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _odomTopic Odometry topic.
  protected: void TestMovement3dAtSingularity(const std::string &_sdfFile,
                                              const std::string &_odomTopic)
  {
    // Start server
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);

    Server server(serverConfig);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    // Create a system that records the body poses
    test::Relay testSystem;

    std::vector<math::Pose3d> poses;
    testSystem.OnPostUpdate([&poses](const sim::UpdateInfo &,
      const sim::EntityComponentManager &_ecm)
      {
        auto id = _ecm.EntityByComponents(
          components::Model(),
          components::Name("test_body"));
        EXPECT_NE(kNullEntity, id);

        auto poseComp = _ecm.Component<components::Pose>(id);
        ASSERT_NE(nullptr, poseComp);
        poses.push_back(poseComp->Data());
      });
    server.AddSystem(testSystem.systemPtr);

    std::vector<math::Vector3d> odomAngVels;
    // Create function to store data from odometry messages
    std::function<void(const msgs::Odometry &)> odomCb =
      [&](const msgs::Odometry &_msg)
      {
        odomAngVels.push_back(msgs::Convert(_msg.twist().angular()));
      };

    // Create node for publishing twist messages
    transport::Node node;
    auto cmdVel = node.Advertise<msgs::Twist>("/model/test_body/cmd_vel");
    node.Subscribe(_odomTopic, odomCb);

    // Set an angular velocity command that would cause pitch to update from 0
    // to PI in 1 second, crossing the singularity when pitch is PI/2.
    const math::Vector3d angVelCmd(0.0, GZ_PI, 0);
    msgs::Twist msg;
    msgs::Set(msg.mutable_linear(), math::Vector3d::Zero);
    msgs::Set(msg.mutable_angular(), angVelCmd);
    cmdVel.Publish(msg);

    // Run server while the model moves with the velocities set earlier
    server.Run(true, 1000, false);

    // Poses for 1s
    ASSERT_EQ(1000u, poses.size());

    int sleep = 0;
    int maxSleep = 30;
    // Default publishing frequency for odometryPublisher is 50Hz.
    for (; (odomAngVels.size() < 50) &&
        sleep < maxSleep; ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    EXPECT_NE(maxSleep, sleep);

    // Odom for 1s
    ASSERT_FALSE(odomAngVels.empty());
    EXPECT_EQ(50u, odomAngVels.size());

    // Check accuracy of velocities published in the odometry message
    for (size_t i = 1; i < odomAngVels.size(); ++i) {
      EXPECT_NEAR(odomAngVels[i].X(), angVelCmd[0], 1e-1);
      EXPECT_NEAR(odomAngVels[i].Y(), angVelCmd[1], 1e-1);
      EXPECT_NEAR(odomAngVels[i].Z(), angVelCmd[2], 1e-1);
    }
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
    // cppcheck-suppress knownConditionTrueFalse
    for (; odomPosesCount < 5 && sleep < maxSleep; ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_NE(maxSleep, sleep);

    EXPECT_EQ(5u, odomPosesCount);
  }

  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _odomTopic Odometry topic.
  protected: void TestOffsetTags(const std::string &_sdfFile,
                               const std::string &_odomTopic)
  {
    // Start server
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);

    Server server(serverConfig);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    std::vector<math::Pose3d> odomPoses;
    // Create function to store data from odometry messages
    std::function<void(const msgs::Odometry &)> odomCb =
      [&](const msgs::Odometry &_msg)
      {
        odomPoses.push_back(msgs::Convert(_msg.pose()));
      };
    transport::Node node;
    node.Subscribe(_odomTopic, odomCb);

    // Run server while the model moves with the velocities set earlier
    server.Run(true, 3000, false);

    int sleep = 0;
    int maxSleep = 30;
    for (; odomPoses.size() < 150 && sleep < maxSleep; ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_NE(maxSleep, sleep);

    // Run for 3s and check the pose in the last message
    ASSERT_FALSE(odomPoses.empty());
    auto lastPose = odomPoses[odomPoses.size() - 1];
    EXPECT_NEAR(lastPose.Pos().X(), 11, 1e-2);
    EXPECT_NEAR(lastPose.Pos().Y(), -11, 1e-2);
    EXPECT_NEAR(lastPose.Pos().Z(), 0, 1e-2);

    EXPECT_NEAR(lastPose.Rot().Roll(), 1.57, 1e-2);
    EXPECT_NEAR(lastPose.Rot().Pitch(), 0, 1e-2);
    EXPECT_NEAR(lastPose.Rot().Yaw(), 0, 1e-2);
  }

  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _odomTopic Odometry topic.
  protected: void TestGaussianNoise(const std::string &_sdfFile,
                               const std::string &_odomTopic)
  {
    // Start server
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);

    Server server(serverConfig);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    std::vector<math::Vector3d> odomLinVels;
    std::vector<math::Vector3d> odomAngVels;
    std::vector<math::Quaterniond> odomAngs;
    google::protobuf::RepeatedField<float> odomTwistCovariance;
    // Create function to store data from odometry messages
    std::function<void(const msgs::OdometryWithCovariance &)> odomCb =
      [&](const msgs::OdometryWithCovariance &_msg)
      {
        odomAngs.push_back(msgs::Convert(_msg.pose_with_covariance().
          pose().orientation()));
        odomLinVels.push_back(msgs::Convert(_msg.twist_with_covariance().
          twist().linear()));
        odomAngVels.push_back(msgs::Convert(_msg.twist_with_covariance().
          twist().angular()));
        odomTwistCovariance = _msg.twist_with_covariance().covariance().data();
      };
    transport::Node node;
    node.Subscribe(_odomTopic, odomCb);

    // Run server while the model moves with the velocities set earlier
    server.Run(true, 3000, false);

    int sleep = 0;
    int maxSleep = 30;
    for (; odomLinVels.size() < 500 && sleep < maxSleep; ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Verify the Gaussian noise.
    ASSERT_FALSE(odomLinVels.empty());
    ASSERT_FALSE(odomAngs.empty());
    ASSERT_FALSE(odomAngVels.empty());
    int n = odomLinVels.size();

    // Calculate the means.
    double linVelSumX = 0, linVelSumY = 0, linVelSumZ = 0;
    double angVelSumX = 0, angVelSumY = 0, angVelSumZ = 0;
    for (int i = 0; i < n; i++)
    {
      linVelSumX += odomLinVels[i].X();
      linVelSumY += odomLinVels[i].Y();
      linVelSumZ += odomLinVels[i].Z();

      angVelSumX += odomAngVels[i].X();
      angVelSumY += odomAngVels[i].Y();
      angVelSumZ += odomAngVels[i].Z();
    }

    // Check that the mean values are close to zero.
    EXPECT_NEAR(linVelSumX/n, 0, 0.5);
    EXPECT_NEAR(linVelSumY/n, 0, 0.5);
    EXPECT_NEAR(linVelSumZ/n, 0, 0.5);

    EXPECT_NEAR(angVelSumX/n, 0, 0.5);
    EXPECT_NEAR(angVelSumY/n, 0, 0.5);
    EXPECT_NEAR(angVelSumZ/n, 0, 0.5);

    // Calculate the variation (sigma^2).
    double linVelSqSumX = 0, linVelSqSumY = 0, linVelSqSumZ = 0;
    double angVelSqSumX = 0, angVelSqSumY = 0, angVelSqSumZ = 0;
    for (int i = 0; i < n; i++)
    {
      linVelSqSumX += std::pow(odomLinVels[i].X() - linVelSumX/n, 2);
      linVelSqSumY += std::pow(odomLinVels[i].Y() - linVelSumY/n, 2);
      linVelSqSumZ += std::pow(odomLinVels[i].Z() - linVelSumZ/n, 2);

      angVelSqSumX += std::pow(odomAngVels[i].X() - angVelSumX/n, 2);
      angVelSqSumY += std::pow(odomAngVels[i].Y() - angVelSumY/n, 2);
      angVelSqSumZ += std::pow(odomAngVels[i].Z() - angVelSumZ/n, 2);
    }

    // Verify the variance values.
    EXPECT_NEAR(linVelSqSumX/n, 1, 0.5);
    EXPECT_NEAR(linVelSqSumY/n, 1, 0.5);
    EXPECT_NEAR(linVelSqSumZ/n, 1, 0.5);

    EXPECT_NEAR(angVelSqSumX/n, 1, 0.5);
    EXPECT_NEAR(angVelSqSumY/n, 1, 0.5);
    EXPECT_NEAR(angVelSqSumZ/n, 1, 0.5);

    // Check the covariance matrix.
    EXPECT_EQ(odomTwistCovariance.size(), 36);
    for (int i = 0; i < 36; i++)
    {
      if (i % 7 == 0)
      {
        EXPECT_NEAR(odomTwistCovariance.Get(i), 1, 1e-2);
      }
      else
      {
        EXPECT_NEAR(odomTwistCovariance.Get(i), 0, 1e-2);
      }
    }
  }
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_P(OdometryPublisherTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Movement))
{
  TestMovement(
      std::string(PROJECT_SOURCE_PATH) + "/test/worlds/odometry_publisher.sdf",
      "/model/vehicle/odometry");
}

/////////////////////////////////////////////////
TEST_P(OdometryPublisherTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(MovementCustomTopic))
{
  TestMovement(
      std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/odometry_publisher_custom.sdf",
      "/model/bar/odom");
}

/////////////////////////////////////////////////
TEST_P(OdometryPublisherTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Movement3d))
{
  TestMovement3d(
      gz::common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "odometry_publisher_3d.sdf"),
      "/model/X3/odometry", "/model/X3/pose", "X3/odom", "X3/base_footprint");
}

/////////////////////////////////////////////////
TEST_P(OdometryPublisherTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(Movement3dAtSingularity))
{
  TestMovement3dAtSingularity(
      gz::common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "odometry_publisher_3d_singularity.sdf"),
      "/model/test_body/odometry");
}

/////////////////////////////////////////////////
TEST_P(OdometryPublisherTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(OdomFrameId))
{
  TestPublishCmd(
      std::string(PROJECT_SOURCE_PATH) + "/test/worlds/odometry_publisher.sdf",
      "/model/vehicle/odometry",
      "vehicle/odom",
      "vehicle/base_footprint");
}

/////////////////////////////////////////////////
TEST_P(OdometryPublisherTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(OdomCustomFrameId))
{
  TestPublishCmd(
      std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/odometry_publisher_custom.sdf",
      "/model/bar/odom",
      "odomCustom",
      "baseCustom");
}

/////////////////////////////////////////////////
TEST_P(OdometryPublisherTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(OffsetTagTest))
{
  TestOffsetTags(
      std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/odometry_offset.sdf",
      "/model/vehicle/odometry");
}

/////////////////////////////////////////////////
TEST_P(OdometryPublisherTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(GaussianNoiseTest))
{
  TestGaussianNoise(
      std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/odometry_noise.sdf",
      "/model/vehicle/odometry_with_covariance");
}

// Run multiple times
INSTANTIATE_TEST_SUITE_P(ServerRepeat, OdometryPublisherTest,
    ::testing::Range(1, 2));
