/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Util.hh"

#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

#define TOL 1e-4

using namespace gz;
using namespace sim;

/// \brief Test fixture for JointPositionController system
class JointPositionControllerTestFixture
  : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// Tests that the JointPositionController accepts joint position commands
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointPositionForceCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_position_controller.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> currentPosition;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
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

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint, components::Name,
                  components::JointPosition>(
            [&](const Entity &,
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

/////////////////////////////////////////////////
// Tests that the JointPositionController accepts joint position commands
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointPositionActuatorsCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_position_controller_actuators.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> currentPosition;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
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

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint, components::Name,
                  components::JointPosition>(
            [&](const Entity &,
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
  auto pub = node.Advertise<msgs::Actuators>(
      "/actuators");

  const double targetPosition{1.0};
  msgs::Actuators msg;
  msg.add_position(targetPosition);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(100ms);

  const std::size_t testIters = 3000;
  server.Run(true, testIters , false);

  EXPECT_NEAR(targetPosition, currentPosition.at(0), TOL);
}

/////////////////////////////////////////////////
// Tests that the JointPositionController accepts joint position commands
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointPositionVelocityCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_position_controller_velocity.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> currentPosition;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
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

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint, components::Name,
                  components::JointPosition>(
            [&](const Entity &,
                const components::Joint *,
                const components::Name *_name,
                const components::JointPosition *_position) -> bool
            {
              if (_name->Data() == jointName)
                currentPosition = _position->Data();
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // joint pos starts at 0
  const std::size_t initIters = 1;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, currentPosition.at(0), TOL);

  // joint moves to initial_position at -2.0
  const std::size_t initPosIters = 1;
  server.Run(true, initPosIters, false);
  double expectedInitialPosition = -2.0;
  EXPECT_NEAR(expectedInitialPosition, currentPosition.at(0), TOL);

  // Publish command and check that the joint position is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_position_controller_test/joint/j1/0/cmd_pos");

  const double targetPosition{2.0};
  msgs::Double msg;
  msg.set_data(targetPosition);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(1ms);

  const std::size_t testIters = 1;
  server.Run(true, testIters, false);

  EXPECT_NEAR(targetPosition, currentPosition.at(0), TOL);
}


/////////////////////////////////////////////////
// Tests that the JointPositionController accepts joint position
// sub_topic commands
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(
       JointPositionMultipleJointsSubTopicCommand))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_position_controller_multiple_joints_subtopic.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "j1";

  test::Relay testSystem;
  std::vector<double> currentPosition;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
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

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint, components::Name,
                  components::JointPosition>(
            [&](const Entity &,
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
      "/model/joint_position_controller_test/joints");

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

/////////////////////////////////////////////////
// Tests that the JointPositionController commands joints in
// nested models.
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(
       JointPositionNestedModels))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "test", "worlds",
      "joint_position_controller_nested_models.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  // model2 contains two joints with the same name, one is nested.
  const std::string modelName = "model2";
  const std::string jointName = "rotor_joint";
  const std::string scopedJointName = "model21::rotor_joint";

  test::Relay testSystem;
  std::vector<double> joint2Position;
  std::vector<double> joint21Position;
  testSystem.OnPreUpdate(
      [&](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
      {
        // Create a JointPosition component if it doesn't exist.
        _ecm.Each<components::Joint, components::Name>(
            [&](const gz::sim::Entity &_joint,
                const components::Joint *,
                const components::Name *_name) -> bool
            {
              if (_name->Data() == jointName)
              {
                _ecm.CreateComponent(_joint, components::JointPosition());
              }
              return true;
            });
      });

  testSystem.OnPostUpdate([&](const sim::UpdateInfo &,
                              const sim::EntityComponentManager &_ecm)
      {
        // Retrieve the parent model.
        Entity model{kNullEntity};
        _ecm.Each<components::Model,
                  components::Name>(
            [&](const gz::sim::Entity &_entity,
                const components::Model *,
                const components::Name *_name) -> bool
            {
              if (_name->Data() == modelName)
              {
                model = _entity;
              }
              return true;
            });

        // joint2
        {
          auto entities = entitiesFromScopedName(jointName, _ecm, model);
          Entity joint = *entities.begin();
          auto posComp = _ecm.Component<components::JointPosition>(joint);
          joint2Position = posComp->Data();
        }

        // joint21
        {
          auto entities = entitiesFromScopedName(scopedJointName, _ecm, model);
          Entity joint = *entities.begin();
          auto posComp = _ecm.Component<components::JointPosition>(joint);
          joint21Position = posComp->Data();
        }
      });

  server.AddSystem(testSystem.systemPtr);

  // joint pos starts at 0
  const std::size_t initIters = 1;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, joint2Position.at(0), TOL);
  EXPECT_NEAR(0, joint21Position.at(0), TOL);

  // Publish command and check that the joint position is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model21/cmd_rotor");

  const double targetPosition{1.0};
  msgs::Double msg;
  msg.set_data(targetPosition);

  pub.Publish(msg);
  // Wait for the message to be published
  std::this_thread::sleep_for(100ms);

  const std::size_t testIters = 1000;
  server.Run(true, testIters , false);

  // joint2 should not move
  EXPECT_NEAR(0, joint2Position.at(0), TOL);

  // joint21 should be at target position
  EXPECT_NEAR(targetPosition, joint21Position.at(0), TOL);
}

/////////////////////////////////////////////////
// Tests that the JointPositionController respects the maximum command
TEST_F(JointPositionControllerTestFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(JointPositonVelocityCommandWithMax))
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(PROJECT_SOURCE_PATH,
    "test", "worlds", "joint_position_controller_velocity.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string jointName = "j2";

  test::Relay testSystem;
  std::vector<double> currentPosition;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
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

  testSystem.OnPostUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Joint, components::Name,
                  components::JointPosition>(
            [&](const Entity &,
                const components::Joint *,
                const components::Name *_name,
                const components::JointPosition *_position) -> bool
            {
              if(_name->Data() == jointName)
                currentPosition = _position->Data();
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // joint pos starts at 0
  const std::size_t initIters = 1;
  server.Run(true, initIters, false);
  EXPECT_NEAR(0, currentPosition.at(0), TOL);

  // joint moves to initial_position at -2.0
  const std::size_t initPosIters = 2;
  server.Run(true, initPosIters, false);
  double expectedInitialPosition = -2.0;
  EXPECT_NEAR(expectedInitialPosition, currentPosition.at(0), TOL);

  // Publish command and check that the joint position is set
  transport::Node node;
  auto pub = node.Advertise<msgs::Double>(
      "/model/joint_position_controller_test_with_max/joint/j2/0/cmd_pos");

  const double targetPosition{2.0};
  msgs::Double msg;
  msg.set_data(targetPosition);

  int sleep{0};
  int maxSleep{30};
  for (; !pub.HasConnections() && sleep < maxSleep; ++sleep) {
    std::this_thread::sleep_for(100ms);
  }

  pub.Publish(msg);

  // Wait for the message to be published
  std::this_thread::sleep_for(1ms);

  const std::size_t testInitialIters = 1;
  server.Run(true, testInitialIters , false);

  // We should not have reached our target yet.
  EXPECT_GT(fabs(currentPosition.at(0) - targetPosition), TOL);

  // Eventually reach target
  const std::size_t testIters = 1000;
  server.Run(true, testIters , false);
  EXPECT_NEAR(currentPosition.at(0), targetPosition, TOL);
}
