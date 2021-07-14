/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Imu.hh"
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include "ignition/gazebo/components/Sensor.hh"

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include <ignition/msgs/stringmsg.pb.h>

#include "../helpers/Relay.hh"

#define TOL 1e-4

using namespace ignition;
using namespace gazebo;

/// \brief Test fixture for GimbalController system
class GimbalControllerTestFixture : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    ignition::common::setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str());
  }
};


/////////////////////////////////////////////////
void OnPitchStringMsg(const msgs::StringMsg &_msg)
{
  double pitchCommand = atof(_msg.data().c_str());
}

////////////////////////////////////////////////
void OnYawStringMsg(const msgs::StringMsg &_msg)
{
  double yawCommand = atof(_msg.data().c_str());
}

//////////////////////////////////////////////////
OnRollStringMsg(const msgs::StringMsg &_msg)
{
   double rollCommand = atof(_msg.data().c_str());
}

//////////////////////////////////////////////////
// Tests the GimbalController using joint position commands
TEST_F(GimbalControllerTestFixture, JointPositionCommand)
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/gimbal_test.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "rev_joint_1";

  test::Relay testSystem;
  std::vector<double> currentPos;
  testSystem.OnPreUpdate(
      [&](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
      {

        auto joint = _ecm.EntityByComponents(components::Joint(),
                          components::Name(jointName));
        // Create a JointPosition component if it doesn't exist. This signals
        // physics system to populate the component
        if (nullptr == _ecm.Component<components::JointPosition>(joint))
        {
          _ecm.CreateComponent(joint, components::JointPosition());
        }
      });

  server.AddSystem(testSystem.systemPtr);

  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, currentPos.at(0), TOL);

  // Publish the status of the joints at these topics
  transport::Node node;
  auto pub = node.Advertise<msgs::StringMsg>
  ("/model/gimbal/gimbal_pitch_status");
  auto pub = node.Advertise<msgs::StringMsg>
  ("/model/gimbal/gimbal_yaw_status");
  auto pub = node.Advertise<msgs::StringMsg>
  ("/model/gimbal/gimbal_roll_status");

  const double targetPos{4.0};

  msgs::StringMsg msg;
  std::stringstream ss;

  ss << targetPos;
  msg.set_data(ss.str());

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(120ms);

  const std::size_t testIters = 1000;
  server.Run(true, testIters , false);

  EXPECT_NEAR(targetPos, currentPos.at(0), TOL);
}