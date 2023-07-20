/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 * Copyright (C) 2023 Benjamin Perseghetti, Rudis Laboratories
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

#include <gz/msgs/double.pb.h>
#include <gz/msgs/actuators.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

#define TOL 1e-4

using namespace gz;
using namespace sim;

/// \brief Test fixture for JointController system
class JointControllerTestFixture : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// Tests that the JointController accepts joint velocity commands
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(JointControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointVelocityCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_controller.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string linkName = "rotor";

  test::Relay testSystem;
  std::vector<math::Vector3d> angularVelocities;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
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

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Link, components::Name,
                  components::AngularVelocity>(
            [&](const Entity &,
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

  const std::size_t initIters = 1000;
  server.Run(true, initIters, false);
  EXPECT_EQ(initIters, angularVelocities.size());
  for (auto i = 0u; i < angularVelocities.size(); ++i)
  {
    if (i == 0)
    {
      EXPECT_NEAR(0.0, angularVelocities[i].Length(), TOL)
          << "Iteration [" << i << "]";
    }
    else
    {
      EXPECT_NEAR(5.0, angularVelocities[i].Length(), TOL)
          << "Iteration [" << i << "]";
    }
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
// Tests that the JointController accepts joint velocity commands
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(JointControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(
        JointVelocityMultipleJointsSubTopicCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_controller.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  // Publish command and check that the joint velocity is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_controller_test/joints");

  const double testAngVel{10.0};
  msgs::Double msg;
  msg.set_data(testAngVel);

  pub.Publish(msg);
}

/////////////////////////////////////////////////
// Tests that the JointController accepts actuator commands
TEST_F(JointControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(
        JointControllerActuatorsCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_controller.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string linkName = "rotor4";

  test::Relay testSystem;
  math::Vector3d angularVelocity;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
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

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Link, components::Name,
                  components::AngularVelocity>(
            [&](const Entity &,
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
  auto pub = node.Advertise<msgs::Actuators>(
      "/actuators");

  const double testAngVel{10.0};
  msgs::Actuators msg;
  msg.add_velocity(testAngVel);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(100ms);

  const std::size_t testIters = 3000;
  server.Run(true, testIters , false);

  EXPECT_NEAR(0, angularVelocity.X(), 1e-2);
  EXPECT_NEAR(0, angularVelocity.Y(), 1e-2);
  EXPECT_NEAR(testAngVel, angularVelocity.Z(), 1e-2);
}

/////////////////////////////////////////////////
// Tests the JointController using joint force commands
TEST_F(JointControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointVelocityCommandWithForce))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_controller.sdf"));


  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string linkName = "rotor3";

  test::Relay testSystem;
  math::Vector3d angularVelocity;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
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

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Link, components::Name,
                  components::AngularVelocity>(
            [&](const Entity &,
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

/////////////////////////////////////////////////
TEST_F(JointControllerTestFixture, InexistentJoint)
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_controller_invalid.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Run some iterations to make sure nothing explodes
  server.Run(true, 100, false);
}
