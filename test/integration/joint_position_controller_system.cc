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

#include <ignition/msgs/double.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/Name.hh"

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

#define TOL 1e-4

using namespace ignition;
using namespace gazebo;

/// \brief Test fixture for JointPositionController system
class JointPositionControllerTestFixture : public ::testing::Test
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
// Tests that the JointPositionController accepts joint position commands
TEST_F(JointPositionControllerTestFixture, JointPositionCommand)
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/joint_position_controller.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> currentPosition;
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

  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                              const gazebo::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint, components::Name,
                  components::JointPosition>(
            [&](const ignition::gazebo::Entity &,
                const components::Joint *,
                const components::Name *_name,
                const components::JointPosition *_position) -> bool
            {
              EXPECT_EQ(_name->Data(), jointName);
              currentPosition = _position->Data();
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, currentPosition.at(0), TOL);

  // Publish command and check that the joint position is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_position_controller_test/joint/j1/0/cmd_pos");

  const double targetPosition{2.0};
  msgs::Double msg;
  msg.set_data(targetPosition);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(100ms);

  const std::size_t testIters = 1000;
  server.Run(true, testIters , false);

  EXPECT_NEAR(targetPosition, currentPosition.at(0), TOL);
}
