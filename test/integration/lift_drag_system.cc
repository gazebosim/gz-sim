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

#include <gz/msgs/double.pb.h>
#include <gz/common/Filesystem.hh>
#include <gz/msgs/Utility.hh>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/Pose.hh"

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Util.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

#define TOL 1e-4

using namespace gz;
using namespace sim;

struct VerticalForceTestParam
{
  const std::string fileName;
  const std::string bladeName;
};

std::ostream &operator<<(std::ostream &_out, const VerticalForceTestParam &_val)
{
  _out << "[" << _val.fileName << ", " << _val.bladeName << "]";
  return _out;
}


/// \brief Test fixture for LiftDrag system
class VerticalForceParamFixture
    : public InternalFixture<::testing::TestWithParam<VerticalForceTestParam>>
{
};

/////////////////////////////////////////////////
/// Measure / verify force torques against analytical answers.
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_P(VerticalForceParamFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(VerifyVerticalForce))
{
  using namespace std::chrono_literals;
  gz::common::setenv(
      "GZ_SIM_RESOURCE_PATH",
      common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds", "models"));

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile =
      common::joinPaths(PROJECT_SOURCE_PATH, GetParam().fileName);
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  const std::string bodyName = "body";
  const std::string bladeName = GetParam().bladeName;
  const std::string jointName = "body_joint";
  const double desiredVel = -0.2;

  auto firstEntityFromScopedName = [](const std::string &_scopedName,
                                 const EntityComponentManager &_ecm) -> Entity
  {
    auto entities = entitiesFromScopedName(_scopedName, _ecm);
    if (entities.size() > 0)
      return *entities.begin();
    else
      return kNullEntity;
  };

  test::Relay testSystem;
  std::vector<math::Vector3d> linearVelocities;
  std::vector<math::Vector3d> forces;
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        // Create velocity and acceleration components if they dont't exist.
        // This signals physics system to populate the component
        auto bladeLink = firstEntityFromScopedName(bladeName, _ecm);

        enableComponent<components::AngularVelocity>(_ecm, bladeLink);

        auto bodyLink = firstEntityFromScopedName(bodyName, _ecm);

        enableComponent<components::WorldLinearVelocity>(_ecm, bodyLink);
        enableComponent<components::WorldLinearAcceleration>(_ecm, bodyLink);
      });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 1, false);

  const double kp = 100.0;
  // Set a constant velocity to the prismatic joint
  testSystem.OnPreUpdate(
      [&](const UpdateInfo &, EntityComponentManager &_ecm)
      {
        auto joint = _ecm.EntityByComponents(components::Joint(),
                                             components::Name(jointName));

        auto bodyLink = firstEntityFromScopedName(bodyName, _ecm);

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
  wrenchRecorder.OnPreUpdate([&](const UpdateInfo &,
                              const EntityComponentManager &_ecm)
      {
        auto bladeLink = firstEntityFromScopedName(bladeName, _ecm);
        auto bodyLink = firstEntityFromScopedName(bodyName, _ecm);

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

INSTANTIATE_TEST_SUITE_P(
    LiftDragTests, VerticalForceParamFixture,
    ::testing::Values(
        VerticalForceTestParam{
            common::joinPaths("test", "worlds", "lift_drag.sdf"), "wing_1"},
        VerticalForceTestParam{
            common::joinPaths("test", "worlds", "lift_drag_nested_model.sdf"),
            "wing_1::base_link"}));
