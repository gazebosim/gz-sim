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

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

#define TOL 1e-4

using namespace ignition;
using namespace gazebo;

/// \brief Test fixture for JointController system
class JointControllerTestFixture : public ::testing::Test
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
// Tests that the JointController accepts joint velocity commands
TEST_F(JointControllerTestFixture, JointVelocityCommand)
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/joint_controller.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string linkName = "rotor";

  test::Relay testSystem;
  std::vector<math::Vector3d> angularVelocities;
  testSystem.OnPreUpdate(
      [&](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
      {
        auto link = _ecm.EntityByComponents(components::Link(),
                                            components::Name(linkName));
        // Create an AngularVelocity component if it doesn't exist. This signals
        // physics system to populate the component
        if (nullptr == _ecm.Component<components::AngularVelocity>(link))
        {
          _ecm.CreateComponent(link, components::AngularVelocity());
        }
      });

  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                              const gazebo::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Link, components::Name,
                  components::AngularVelocity>(
            [&](const ignition::gazebo::Entity &,
                const components::Link *,
                const components::Name *_name,
                const components::AngularVelocity *_angularVel) -> bool
            {
              EXPECT_EQ(_name->Data(), linkName);
              angularVelocities.push_back(_angularVel->Data());
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  EXPECT_EQ(initIters, angularVelocities.size());
  for (const auto &angVel : angularVelocities)
  {
    EXPECT_NEAR(0, angVel.Length(), TOL);
  }

  angularVelocities.clear();

  // Publish command and check that the joint velocity is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_controller_test/joint/j1/cmd_vel");

  const double testAngVel{10.0};
  msgs::Double msg;
  msg.set_data(testAngVel);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(100ms);

  const std::size_t testIters = 1000;
  server.Run(true, testIters , false);

  EXPECT_EQ(testIters, angularVelocities.size());
  for (const auto &angVel : angularVelocities)
  {
    EXPECT_NEAR(0, angVel.X(), TOL);
    EXPECT_NEAR(0, angVel.Y(), TOL);
    EXPECT_NEAR(testAngVel, angVel.Z(), TOL);
  }
}

/////////////////////////////////////////////////
// Tests the JointController using joint force commands
TEST_F(JointControllerTestFixture, JointVelocityCommandWithForce)
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/joint_controller.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string linkName = "rotor2";

  test::Relay testSystem;
  math::Vector3d angularVelocity;
  testSystem.OnPreUpdate(
      [&](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
      {
        auto link = _ecm.EntityByComponents(components::Link(),
                                            components::Name(linkName));
        // Create an AngularVelocity component if it doesn't exist. This signals
        // physics system to populate the component
        if (nullptr == _ecm.Component<components::AngularVelocity>(link))
        {
          _ecm.CreateComponent(link, components::AngularVelocity());
        }
      });

  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                              const gazebo::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Link, components::Name,
                  components::AngularVelocity>(
            [&](const ignition::gazebo::Entity &,
                const components::Link *,
                const components::Name *_name,
                const components::AngularVelocity *_angularVel) -> bool
            {
              EXPECT_EQ(_name->Data(), linkName);
              angularVelocity = _angularVel->Data();
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  const std::size_t initIters = 10;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, angularVelocity.Length(), TOL);

  // Publish command and check that the joint velocity is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_controller_test_2/joint/j1/cmd_vel");

  const double testAngVel{20.0};
  msgs::Double msg;
  msg.set_data(testAngVel);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(100ms);

  const std::size_t testIters = 3000;
  server.Run(true, testIters , false);

  EXPECT_NEAR(0, angularVelocity.X(), 1e-2);
  EXPECT_NEAR(0, angularVelocity.Y(), 1e-2);
  EXPECT_NEAR(testAngVel, angularVelocity.Z(), 1e-2);
}
