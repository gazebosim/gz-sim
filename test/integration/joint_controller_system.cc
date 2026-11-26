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

#include <chrono>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointVelocity.hh"
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
// Tests that multiple joints share one force command per step.
TEST_F(JointControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointForceCommandMultipleJoints))
{
  using namespace std::chrono_literals;

  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds", "joint_controller_force.sdf"));

  Server server(serverConfig);
  server.SetUpdatePeriod(0ns);

  test::Relay testSystem;
  std::size_t samples = 0;
  double integralError = 0.0;
  double previousError = 0.0;

  // Read the commands after JointController and before Physics clears them.
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &_info, EntityComponentManager &_ecm)
      {
        const auto j1 = _ecm.EntityByComponents(
            components::Joint(), components::Name("j1"));
        const auto j2 = _ecm.EntityByComponents(
            components::Joint(), components::Name("j2"));
        ASSERT_NE(kNullEntity, j1);
        ASSERT_NE(kNullEntity, j2);

        const auto *velocity = _ecm.Component<components::JointVelocity>(j1);
        if (!velocity || velocity->Data().empty())
          return;
        ASSERT_EQ(1u, velocity->Data().size());

        const auto *force1 = _ecm.Component<components::JointForceCmd>(j1);
        const auto *force2 = _ecm.Component<components::JointForceCmd>(j2);
        ASSERT_NE(nullptr, force1);
        ASSERT_NE(nullptr, force2);
        ASSERT_EQ(1u, force1->Data().size());
        ASSERT_EQ(1u, force2->Data().size());

        const double dt = std::chrono::duration<double>(_info.dt).count();
        ASSERT_GT(dt, 0.0);

        // Evaluate one PID update using the target and gains from the SDF.
        const double error = velocity->Data()[0] - 1.0;
        integralError += error * dt;
        const double derivativeError = (error - previousError) / dt;
        previousError = error;
        const double expectedForce =
            -(2.0 * error + integralError + 0.05 * derivativeError);
        EXPECT_NEAR(expectedForce, force1->Data()[0], 1e-9);
        EXPECT_DOUBLE_EQ(force1->Data()[0], force2->Data()[0]);
        EXPECT_NE(0.0, force1->Data()[0]);
        EXPECT_NE(0.0, force2->Data()[0]);
        ++samples;
      });

  server.AddSystem(testSystem.systemPtr);
  const std::size_t forceIters = 3;
  // Physics may need the first step to populate velocity feedback.
  server.Run(true, 1, false);
  server.Run(true, forceIters - samples, false);
  EXPECT_EQ(forceIters, samples);
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
