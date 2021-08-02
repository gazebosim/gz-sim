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
#include <ignition/common/Util.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/HaltMotion.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

#define tol 10e-4

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

/// \brief Test DiffDrive system
class HaltMotionTest : public ::testing::TestWithParam<int>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(1);
    ignition::common::setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str());
  }

  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _cmdVelTopic Command velocity topic.
  /// \param[in] _odomTopic Odometry topic.
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
          components::Name("vehicle"));
        EXPECT_NE(kNullEntity, id);

        auto poseComp = _ecm.Component<components::Pose>(id);
        ASSERT_NE(nullptr, poseComp);

        poses.push_back(poseComp->Data());
      });
    testSystem.OnPreUpdate([&poses](const gazebo::UpdateInfo &,
      gazebo::EntityComponentManager & _ecm)
      {
        auto model = _ecm.EntityByComponents(
          components::Model(),
          components::Name("vehicle"));;
        EXPECT_NE(kNullEntity, model);
        if (!_ecm.Component<components::HaltMotion>(model))
        {
          _ecm.CreateComponent(model, components::HaltMotion(false));
        }
        if (poses.size() == 4000u &&
            !_ecm.Component<components::HaltMotion>(model)->Data())
        {
          _ecm.Component<components::HaltMotion>(model)->Data() = true;
        }
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

    const double desiredLinVel = 10.5;
    msgs::Set(msg.mutable_linear(),
              math::Vector3d(desiredLinVel, 0, 0));
    msgs::Set(msg.mutable_angular(),
              math::Vector3d(0.0, 0, 0));
    pub.Publish(msg);

    server.Run(true, 1000, false);

    msgs::Set(msg.mutable_linear(),
              math::Vector3d(0, 0, 0));
    msgs::Set(msg.mutable_angular(),
              math::Vector3d(0.0, 0, 0));
    pub.Publish(msg);

    server.Run(true, 2000, false);

    // Poses for 4s
    ASSERT_EQ(4000u, poses.size());

    server.Run(true, 1000, false);

    msgs::Set(msg.mutable_linear(),
              math::Vector3d(desiredLinVel, 0, 0));
    msgs::Set(msg.mutable_angular(),
              math::Vector3d(0.0, 0, 0));
    pub.Publish(msg);

    server.Run(true, 4000, false);

    EXPECT_EQ(9000u, poses.size());

    for (uint64_t i = 4001; i < poses.size(); ++i)
    {
      EXPECT_EQ(poses[3999], poses[i]);
    }
  }
};

/////////////////////////////////////////////////
TEST_P(HaltMotionTest, PublishCmd)
{
  TestPublishCmd(
      std::string(PROJECT_SOURCE_PATH) + "/test/worlds/diff_drive.sdf",
      "/model/vehicle/cmd_vel");
}

// Run multiple times
INSTANTIATE_TEST_SUITE_P(ServerRepeat, HaltMotionTest,
    ::testing::Range(1, 2));
