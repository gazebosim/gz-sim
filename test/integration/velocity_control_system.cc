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
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

/// \brief Test VelocityControl system
class VelocityControlTest : public ::testing::TestWithParam<int>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }

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
    testSystem.OnPostUpdate([&poses](const gazebo::UpdateInfo &,
      const gazebo::EntityComponentManager &_ecm)
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
    test::Relay velocityRamp;
    const double desiredLinVel = 10.5;
    const double desiredAngVel = 0.2;
    velocityRamp.OnPreUpdate(
        [&](const gazebo::UpdateInfo &/*_info*/,
            const gazebo::EntityComponentManager &)
        {
          msgs::Set(msg.mutable_linear(),
                    math::Vector3d(desiredLinVel, 0, 0));
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

    ASSERT_NE(maxSleep, sleep);

    // verify that the vehicle is moving in +x and rotating towards +y
    for (unsigned int i = 1001; i < poses.size(); ++i)
    {
      EXPECT_GT(poses[i].Pos().X(), poses[i-1].Pos().X());
      EXPECT_GT(poses[i].Pos().Y(), poses[i-1].Pos().Y());
      EXPECT_NEAR(poses[i].Pos().Z(), poses[i-1].Pos().Z(), 1e-5);
      EXPECT_NEAR(poses[i].Rot().Euler().X(),
          poses[i-1].Rot().Euler().X(), 1e-5);
      EXPECT_NEAR(poses[i].Rot().Euler().Y(),
          poses[i-1].Rot().Euler().Y(), 1e-5);
      EXPECT_GT(poses[i].Rot().Euler().Z(), poses[i-1].Rot().Euler().Z());
    }
  }
};

/////////////////////////////////////////////////
TEST_P(VelocityControlTest, PublishCmd)
{
  TestPublishCmd(
      std::string(PROJECT_SOURCE_PATH) + "/test/worlds/velocity_control.sdf",
      "/model/vehicle_blue/cmd_vel");
}

// Run multiple times
INSTANTIATE_TEST_CASE_P(ServerRepeat, VelocityControlTest,
    ::testing::Range(1, 2));
