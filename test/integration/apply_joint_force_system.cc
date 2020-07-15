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

#include <ignition/msgs/double.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

#define TOL 1e-4

using namespace ignition;
using namespace gazebo;

/// \brief Test fixture for ApplyJointForce system
class ApplyJointForceTestFixture : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

/////////////////////////////////////////////////
// Tests that the ApplyJointForce accepts joint velocity commands
TEST_F(ApplyJointForceTestFixture, JointVelocityCommand)
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/apply_joint_force.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Slow update period to increase the likelihood of published data arriving
  // before iterations are completed
  server.SetUpdatePeriod(1ms);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> jointForceCmd;
  testSystem.OnPreUpdate(
      [&](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
      {
        auto joint = _ecm.EntityByComponents(components::Joint(),
                                             components::Name(jointName));
        auto forceComp = _ecm.Component<components::JointForceCmd>(joint);
        if (forceComp)
        {
          jointForceCmd.push_back(forceComp->Data()[0]);
        }
      });

  server.AddSystem(testSystem.systemPtr);

  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  EXPECT_EQ(initIters, jointForceCmd.size());
  for (const auto &jointForce : jointForceCmd)
  {
    EXPECT_NEAR(0, jointForce, TOL);
  }

  jointForceCmd.clear();

  // Publish command and check that the joint force is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_force_test/joint/j1/cmd_force");

  const double testJointForce{0.001};
  msgs::Double msg;
  msg.set_data(testJointForce);

  pub.Publish(msg);
  // Wait for the message to be published
  const std::size_t maxIters = 1000;
  for (std::size_t i = 0; i < maxIters; ++i)
  {
    server.Run(true, 1, false);
    if (std::abs(jointForceCmd.back() - testJointForce) < 1e-6)
    {
      break;
    }
  }
  EXPECT_DOUBLE_EQ(jointForceCmd.back(), testJointForce);
}

