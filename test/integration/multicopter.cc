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

#include <optional>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Pose.hh"

#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

class MulticopterTest : public InternalFixture<::testing::Test>
{
  protected: std::unique_ptr<Server> StartServer(const std::string &_filePath)
  {
    ServerConfig serverConfig;
    const auto sdfFile = std::string(PROJECT_SOURCE_PATH) + _filePath;
    serverConfig.SetSdfFile(sdfFile);

    auto server = std::make_unique<Server>(serverConfig);
    EXPECT_FALSE(server->Running());
    EXPECT_FALSE(*server->Running(0));

    using namespace std::chrono_literals;
    server->SetUpdatePeriod(1ns);
    return server;
  }
};

/////////////////////////////////////////////////
// Test that commanded motor speed is applied
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(MulticopterTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(CommandedMotorSpeed))
{
  // Start server
  auto server = this->StartServer("/test/worlds/quadcopter.sdf");

  test::Relay testSystem;
  transport::Node node;
  auto cmdMotorSpeed =
      node.Advertise<msgs::Actuators>("/X3/gazebo/command/motor_speed");

  const std::size_t iterTestStart{100};
  const std::size_t nIters{500};
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &_info, EntityComponentManager &_ecm)
      {
        // Create components, if the don't exist, on the first iteration
        if (_info.iterations == 1)
        {
          for (const auto &e : _ecm.EntitiesByComponents(components::Joint()))
          {
            if (!_ecm.Component<components::JointVelocity>(e))
            {
              _ecm.CreateComponent(e, components::JointVelocity());
            }
          }
        }
      });

  testSystem.OnPostUpdate(
      [&](const UpdateInfo &_info,
          const EntityComponentManager &_ecm)
      {
        // Command a motor speed
        // After nIters iterations, check angular velocity of each of the rotors
        const double cmdSpeed{100};
        if (_info.iterations == iterTestStart)
        {
          msgs::Actuators msg;
          msg.mutable_velocity()->Resize(4, cmdSpeed);
          cmdMotorSpeed.Publish(msg);
        }
        else if (_info.iterations == iterTestStart + nIters)
        {
          int count = 0;
          // Check that each rotor's velocity matches the commanded value
          for (const auto &e : _ecm.EntitiesByComponents(components::Joint()))
          {
            auto *jointVel = _ecm.Component<components::JointVelocity>(e);
            EXPECT_NE(nullptr, jointVel);
            EXPECT_FALSE(jointVel->Data().empty());
            if (jointVel->Data().size() > 0)
            {
              ++count;
              EXPECT_NEAR(cmdSpeed, std::abs(jointVel->Data()[0]), 1e-2);
            }
          }

          EXPECT_EQ(4, count);
        }
      });

  server->AddSystem(testSystem.systemPtr);
  server->Run(true, iterTestStart + nIters, false);
}

/////////////////////////////////////////////////
TEST_F(MulticopterTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(MulticopterVelocityControl))
{
  // Start server
  auto server =
      this->StartServer("/test/worlds/quadcopter_velocity_control.sdf");

  test::Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/X3/gazebo/command/twist");

  const std::size_t nIters{2000};
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &_info, EntityComponentManager &_ecm)
      {
        // Create components, if the don't exist, on the first iteration
        if (_info.iterations == 1)
        {
          Entity entity = _ecm.EntityByComponents(
              components::Link(), components::Name("base_link"));

          EXPECT_NE(kNullEntity, entity);
          if (!_ecm.Component<components::LinearVelocity>(entity))
          {
            _ecm.CreateComponent(entity, components::LinearVelocity());
          }
          if (!_ecm.Component<components::AngularVelocity>(entity))
          {
            _ecm.CreateComponent(entity, components::AngularVelocity());
          }
        }
      });

  // Add the system with just the preupdate and run for one iteration so all the
  // components are created and the controller system is initialized
  server->AddSystem(testSystem.systemPtr);
  server->Run(true, 1, false);

  math::Vector3d cmdLinVel = math::Vector3d::Zero;
  math::Vector3d cmdAngVel = math::Vector3d::Zero;
  std::optional<std::size_t> iterTestStart;

  auto resetTest =
      [&](const math::Vector3d &_cmdLinVel, const math::Vector3d &_cmdAngVel)
  {
    cmdLinVel = _cmdLinVel;
    cmdAngVel = _cmdAngVel;
    iterTestStart.reset();
  };

  testSystem.OnPostUpdate(
      [&](const UpdateInfo &_info,
          const EntityComponentManager &_ecm)
      {
        if (!iterTestStart.has_value())
        {
          iterTestStart = _info.iterations;
        }
        // Command a motor speed
        // After nIters iterations, check angular velocity of each of the
        // rotors
        if (_info.iterations == *iterTestStart)
        {
          msgs::Twist msg;
          msgs::Set(msg.mutable_linear(), cmdLinVel);
          msgs::Set(msg.mutable_angular(), cmdAngVel);
          cmdVel.Publish(msg);
        }
        else if (_info.iterations == *iterTestStart + nIters - 1)
        {
          std::size_t numTests = 0;
          // Check that the vehicles velocity matches the commanded value
          _ecm.Each<components::Link, components::LinearVelocity,
                    components::AngularVelocity>(
              [&](const Entity &, const components::Link *,
                  const components::LinearVelocity *_linVel,
                  const components::AngularVelocity *_angVel)
              {
                EXPECT_TRUE(cmdLinVel.Equal(_linVel->Data(), 1e-2))
                    << "Cmd: " << cmdLinVel << " Value: " << _linVel->Data();

                EXPECT_TRUE(cmdAngVel.Equal(_angVel->Data(), 1e-2))
                    << "Cmd: " << cmdAngVel << " Value: " << _angVel->Data();
                ++numTests;
                return true;
              });
          EXPECT_EQ(1u, numTests);
        }
      });

  resetTest({0, 0, 0.2}, {0, 0, 0});
  server->Run(true, nIters, false);

  resetTest({0.1, 0, 0.0}, {0, 0, 0});
  server->Run(true, nIters, false);

  resetTest({0.1, 0, 0.2}, {0, 0, 0});
  server->Run(true, nIters, false);

  resetTest({0.0, 0, 0.0}, {0, 0, 0.05});
  server->Run(true, nIters, false);

  resetTest({0.1, 0, 0.2}, {0, 0, 0.05});
  server->Run(true, nIters, false);
}

/////////////////////////////////////////////////
// Test the interactions between MulticopterVelocityControl and
// MulticopterMotorModel
TEST_F(MulticopterTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ModelAndVelocityControlInteraction))
{
  // Start server
  auto server =
      this->StartServer("/test/worlds/quadcopter_velocity_control.sdf");

  test::Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/X3/gazebo/command/twist");

  testSystem.OnPreUpdate(
      [&](const UpdateInfo &_info, EntityComponentManager &_ecm)
      {
        // Create components, if the don't exist, on the first iteration
        if (_info.iterations == 1)
        {
          for (const auto &e : _ecm.EntitiesByComponents(components::Joint()))
          {
            if (!_ecm.Component<components::JointVelocity>(e))
            {
              _ecm.CreateComponent(e, components::JointVelocity());
            }
          }
        }
      });

  // Add the system with just the preupdate and run for one iteration so all the
  // components are created and the controller system is initialized
  server->AddSystem(testSystem.systemPtr);
  server->Run(true, 1, false);

  // Now test that commands published to the MulticopterMotorModel are ignored
  {
    msgs::Twist msg;
    // Command to hover in place.
    msgs::Set(msg.mutable_linear(), {0, 0, 0.0});
    cmdVel.Publish(msg);
  }
  // Run for a few iterations so the rotors get to their stable velocities
  server->Run(true, 200, false);

  auto cmdMotorSpeed =
      node.Advertise<msgs::Actuators>("/X3/gazebo/command/motor_speed");

  testSystem.OnPostUpdate(
      [&](const UpdateInfo &,
          const EntityComponentManager &_ecm)
      {
        // Publish a motor speed command
        {
          msgs::Actuators msg;
          msg.mutable_velocity()->Resize(4, 60);
          cmdMotorSpeed.Publish(msg);
        }

        std::size_t numJoints = 0;
        // Check that the vehicles velocity matches the commanded value
        _ecm.Each<components::JointVelocity>(
            [&](const Entity &, const components::JointVelocity *_jointVel)
            {
              // The joint velocity for hovering is experimentally found to be
              // around 650. The commanded motor speed is an order of magnitude
              // smaller so we can safely do a comparison.
              const double expJointVel = 650;
              EXPECT_NEAR(expJointVel, std::abs(_jointVel->Data()[0]), 50);

              ++numJoints;
              return true;
            });
        EXPECT_EQ(4u, numJoints);
      });
  server->Run(true, 10, false);
}

/////////////////////////////////////////////////
TEST_F(MulticopterTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(MulticopterVelocityControlNestedModel))
{
  // test that the drone is able to take off when carrying a payload
  // (nexted model) with extra mass.

  // Start server
  auto server =
      this->StartServer("/test/worlds/quadcopter_velocity_control_nested.sdf");

  test::Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/X3/gazebo/command/twist");

  // Add the system
  server->AddSystem(testSystem.systemPtr);
  server->Run(true, 1, false);

  // get pose of drone in post update
  math::Pose3d x3Pose;
  testSystem.OnPostUpdate(
      [&](const UpdateInfo &,
          const EntityComponentManager &_ecm)
      {
          auto x3Ent = _ecm.EntityByComponents(
              components::Model(), components::Name("X3"));
          ASSERT_NE(kNullEntity, x3Ent);

          auto poseComp = _ecm.Component<components::Pose>(x3Ent);
          if (poseComp)
             x3Pose = poseComp->Data();
      });

  server->Run(true, 100, false);

  // check initial z pos
  double initialZ = x3Pose.Pos().Z();
  EXPECT_GT(0.1, initialZ);

  // run for a few interations and verify drone is still on the ground
  server->Run(true, 100, false);
  EXPECT_NEAR(initialZ, x3Pose.Pos().Z(), 1e-3);

  // send linear z vel for drone to take off
  msgs::Twist msg;
  msgs::Set(msg.mutable_linear(), math::Vector3d(0, 0, 5));
  cmdVel.Publish(msg);

  // verify drone continues to fly higher over the duration of 1 second
  double zHeight = x3Pose.Pos().Z();
  for (unsigned int i = 0; i < 10; ++i)
  {
    server->Run(true, 100, false);
    EXPECT_LT(zHeight, x3Pose.Pos().Z());
    zHeight = x3Pose.Pos().Z();
  }

  // one last check to verify drone is at least 5 meters off the ground
  EXPECT_LT(5.0, x3Pose.Pos().Z());
}
