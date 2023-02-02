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

#include <gz/msgs/model.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test JointStatePublisher system
class JointStatePublisherTest
  : public InternalFixture<::testing::TestWithParam<int>>
{
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(JointStatePublisherTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(DefaultPublisher))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/diff_drive.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  int count = 0;
  // Check that all of joints are published.
  std::function<void(const msgs::Model &)> jointStateCb =
    [&](const msgs::Model &_msg)
    {
      bool foundLeftWheelJoint{false},
           foundRightWheelJoint{false},
           foundCasterWheel{false},
           extra{false};

      for (int i = 0; i < _msg.joint_size(); ++i)
      {
        if (_msg.joint(i).name() == "left_wheel_joint")
          foundLeftWheelJoint = true;
        else if (_msg.joint(i).name() == "right_wheel_joint")
          foundRightWheelJoint = true;
        else if (_msg.joint(i).name() == "caster_wheel")
          foundCasterWheel = true;
        else
          extra = true;
      }
      EXPECT_TRUE(foundLeftWheelJoint);
      EXPECT_TRUE(foundRightWheelJoint);
      EXPECT_TRUE(foundCasterWheel);
      EXPECT_FALSE(extra);
      count++;
    };

  transport::Node node;
  node.Subscribe("/world/diff_drive/model/vehicle/joint_state", jointStateCb);

  server.Run(true, 10, false);

  // Make sure the callback was triggered at least once.
  EXPECT_GT(count, 0);
}

/////////////////////////////////////////////////
TEST_F(JointStatePublisherTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(LimitedPublisher))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/diff_drive_limited_joint_pub.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  int count = 0;
  // Check that only the left and right wheel joints are published.
  std::function<void(const msgs::Model &)> jointStateCb =
    [&](const msgs::Model &_msg)
    {
      bool foundLeftWheelJoint{false},
           foundRightWheelJoint{false},
           extra{false};

      int rightWheelJointCount = 0;
      for (int i = 0; i < _msg.joint_size(); ++i)
      {
        if (_msg.joint(i).name() == "left_wheel_joint")
        {
          foundLeftWheelJoint = true;
        }
        else if (_msg.joint(i).name() == "right_wheel_joint")
        {
          rightWheelJointCount++;
          foundRightWheelJoint = true;
        }
        else
          extra = true;
      }
      // Test duplicate joint names do not result in repeats.
      EXPECT_EQ(1, rightWheelJointCount);

      EXPECT_TRUE(foundLeftWheelJoint);
      EXPECT_TRUE(foundRightWheelJoint);
      EXPECT_FALSE(extra);
      count++;
    };

  transport::Node node;
  node.Subscribe("/world/diff_drive/model/vehicle/joint_state", jointStateCb);

  server.Run(true, 10, false);

  // Make sure the callback was triggered at least once.
  EXPECT_GT(count, 0);
}

/////////////////////////////////////////////////
TEST_F(JointStatePublisherTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(NestedJointPublisher))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "diff_drive_nested.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  int count = 0;
  // Check that all of joints are published.
  std::function<void(const msgs::Model &)> jointStateCb =
    [&](const msgs::Model &_msg)
    {
      bool foundLeftWheelJoint{false},
           foundRightWheelJoint{false},
           foundCasterWheel{false},
           extra{false};

      for (int i = 0; i < _msg.joint_size(); ++i)
      {
        if (_msg.joint(i).name() == "left_wheel_joint")
          foundLeftWheelJoint = true;
        else if (_msg.joint(i).name() == "right_wheel_joint")
          foundRightWheelJoint = true;
        else if (_msg.joint(i).name() == "caster_wheel")
          foundCasterWheel = true;
        else
          extra = true;
      }
      EXPECT_TRUE(foundLeftWheelJoint);
      EXPECT_TRUE(foundRightWheelJoint);
      EXPECT_TRUE(foundCasterWheel);
      EXPECT_FALSE(extra);
      count++;
    };

  transport::Node node;
  node.Subscribe(
      "/world/diff_drive_nested/model/vehicle/model/vehicle_nested/joint_state",
      jointStateCb);

  server.Run(true, 10, false);

  // Make sure the callback was triggered at least once.
  EXPECT_GT(count, 0);
}
