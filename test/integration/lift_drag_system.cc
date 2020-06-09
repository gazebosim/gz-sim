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
#include <ignition/msgs/Utility.hh>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ExternalWorldWrenchCmd.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/Pose.hh"

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

#define TOL 1e-4

using namespace ignition;
using namespace gazebo;

/// \brief Test fixture for LiftDrag system
class LiftDragTestFixture : public ::testing::Test
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
/// Measure / verify force torques against analytical answers.
TEST_F(LiftDragTestFixture, VerifyVerticalForce)
{
  using namespace std::chrono_literals;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile =
      std::string(PROJECT_SOURCE_PATH) + "/test/worlds/lift_drag.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string bodyName = "body";
  const std::string bladeName = "wing_1";
  const std::string jointName = "body_joint";
  const double desiredVel = -0.2;

  test::Relay testSystem;
  std::vector<math::Vector3d> linearVelocities;
  std::vector<math::Vector3d> forces;
  testSystem.OnPreUpdate(
      [&](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
      {
        // Create velocity and acceleration components if they dont't exist.
        // This signals physics system to populate the component
        auto bladeLink = _ecm.EntityByComponents(components::Link(),
                                                 components::Name(bladeName));

        if (nullptr == _ecm.Component<components::AngularVelocity>(bladeLink))
        {
          _ecm.CreateComponent(bladeLink, components::AngularVelocity());
        }

        auto bodyLink = _ecm.EntityByComponents(components::Link(),
                                                components::Name(bodyName));

        if (nullptr ==
            _ecm.Component<components::WorldLinearVelocity>(bodyLink))
        {
          _ecm.CreateComponent(bodyLink, components::WorldLinearVelocity());
        }
        if (nullptr ==
            _ecm.Component<components::WorldLinearAcceleration>(bodyLink))
        {
          _ecm.CreateComponent(bodyLink, components::WorldLinearAcceleration());
        }
      });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1, false);

  const double kp = 100.0;
  // Set a constant velocity to the prismatic joint
  testSystem.OnPreUpdate(
      [&](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
      {
        auto joint = _ecm.EntityByComponents(components::Joint(),
                                             components::Name(jointName));

        auto bodyLink = _ecm.EntityByComponents(components::Link(),
                                                components::Name(bodyName));
        auto linVelComp =
            _ecm.Component<components::WorldLinearVelocity>(bodyLink);

        if (!linVelComp)
          return;

        auto jointCmd = kp * (desiredVel - linVelComp->Data().X());

        if (nullptr == _ecm.Component<components::JointForceCmd>(joint))
        {
          _ecm.CreateComponent(joint,
                               components::JointForceCmd({jointCmd}));
        }
        else
        {
          _ecm.Component<components::JointForceCmd>(joint)->Data()[0] =
              jointCmd;
        }
      });

  // \todo(addisu) This assumes that the this system will run after the lift
  // drag system. This is needed to capture the wrench set by the lift drag
  // system. This assumption may not hold when systems are run in parallel.
  test::Relay wrenchRecorder;
  wrenchRecorder.OnPreUpdate([&](const gazebo::UpdateInfo &,
                              const gazebo::EntityComponentManager &_ecm)
      {
        auto bladeLink = _ecm.EntityByComponents(components::Link(),
                                                 components::Name(bladeName));
        auto bodyLink = _ecm.EntityByComponents(components::Link(),
                                                components::Name(bodyName));
        auto linVelComp =
            _ecm.Component<components::WorldLinearVelocity>(bodyLink);
        auto wrenchComp =
            _ecm.Component<components::ExternalWorldWrenchCmd>(bladeLink);

        if (linVelComp)
        {
          linearVelocities.push_back(linVelComp->Data());
        }
        else
        {
          linearVelocities.push_back(math::Vector3d::Zero);
        }

        if (wrenchComp)
        {
          math::Vector3d force = msgs::Convert(wrenchComp->Data().force());
          forces.push_back(force);
        }
        else
        {
          forces.push_back(math::Vector3d::Zero);
        }
      });
  server.AddSystem(wrenchRecorder.systemPtr);

  // parameters from SDF
  const double a0 = 0.1;
  const double cla = 4.0;
  // This the angle the blade makes with the horizontal axis. It's set in the
  // <pose> of the link as the roll
  const double dihedral = 0.1;
  const double rho = 1.2041;
  const double area = 10;

  // It takes a few iterations before the system reaches a steady state
  const std::size_t testIters = 1000;
  server.Run(true, testIters , false);

  EXPECT_EQ(testIters, forces.size());
  EXPECT_EQ(testIters, linearVelocities.size());

  for (std::size_t i = forces.size() - 15; i < forces.size(); ++i)
  {
    const double v = linearVelocities[i].X();
    const double q = 0.5 * rho * v * v;
    const double cl = cla * a0 * q * area;
    const double vertForce = forces[i].Z();

    const double expVertForce = cl * cos(dihedral);
    EXPECT_NEAR(expVertForce, vertForce, TOL);

    // The test above passes if the force is zero (which can happen if the
    // system is not functioning properly), so we check here to make sure it's
    // not zero
    EXPECT_GT(vertForce, 0);
  }
}
