/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>
#include <ignition/utils/ExtraTestMacros.hh>

#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/TestFixture.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/World.hh"

#include "ignition/gazebo/test_config.hh"
#include "../helpers/EnvTestFixture.hh"

#define PI 3.141592653

using namespace ignition;
using namespace gazebo;

class HydrodynamicsTest : public InternalFixture<::testing::Test>
{
  /// \brief Test a world file
  /// \param[in] _world Path to world file
  /// \param[in] _namespace Namespace for topic
  /// \param[in] _density Fluid density
  /// \param[in] _viscosity Fluid viscosity
  /// \param[in] _radius Body's radius
  /// \param[in] _area Body surface area
  /// \param[in] _drag_coeff Body drag coefficient
  public: void TestWorld(const std::string &_world,
    const std::string &_namespace, double _density,
     double _viscosity, double _area, double _drag_coeff);
};

//////////////////////////////////////////////////
void HydrodynamicsTest::TestWorld(const std::string &_world,
    const std::string &_namespace, double _density, double _viscosity,
    double _area, double _drag_coeff)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(_world);

  TestFixture fixture(serverConfig);

  Model model;
  Link body;
  std::vector<math::Vector3d> bodyVels;
  double dt{0.0};
  fixture.
  OnConfigure(
    [&](const Entity &_worldEntity,
      const std::shared_ptr<const sdf::Element> &/*_sdf*/,
      EntityComponentManager &_ecm,
      EventManager &/*eventMgr*/)
    {
      World world(_worldEntity);

      auto modelEntity =  world.ModelByName(_ecm, "ball");
      EXPECT_NE(modelEntity, kNullEntity);
      model = Model(modelEntity);

      auto bodyEntity = model.LinkByName(_ecm, "body");
      EXPECT_NE(bodyEntity, kNullEntity);

      body = Link(bodyEntity);
      body.EnableVelocityChecks(_ecm);
    }).
  OnPostUpdate([&](const UpdateInfo &_info,
                            const EntityComponentManager &_ecm)
    {
      dt = std::chrono::duration<double>(_info.dt).count();

      auto bodyVel = body.WorldLinearVelocity(_ecm);
      ASSERT_TRUE(bodyVel);
      bodyVels.push_back(bodyVel.value());
    }).
  Finalize();

  fixture.Server()->Run(true, 100, false);
  EXPECT_EQ(100u, bodyVels.size());

  EXPECT_NE(model.Entity(), kNullEntity);
  EXPECT_NE(body.Entity(), kNullEntity);
  EXPECT_EQ(_namespace, "ball");

  for (const auto &vel : bodyVels)
  {
    EXPECT_EQ(math::Vector3d::Zero, vel);
  }
  bodyVels.clear();

  // drag force
  // double force{0.0};
  
  // drag force, F = 6 * pi * a * nu * v
  // drag force, F = 0.5 * rho * u^2 * Cd * A
  // terminal velocity, u = 2 * sqrt((3 * pi * a * nu * v)/(rho * Cd * A))
  
  for (unsigned int i = 25; i < bodyVels.size(); ++i)
  {
    auto bodyVel = bodyVels[i];
    // It takes a few iterations to reach the terminal velocity
    auto terminalVel = 2 * sqrt((3 * IGN_PI * _area * _viscosity * bodyVel.Z())/
      (_density * _drag_coeff * _area));
    EXPECT_NEAR(terminalVel, bodyVel.Z(), 1e-2);
  }
}

/////////////////////////////////////////////////
TEST_F(HydrodynamicsTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(PIDControl))
{
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "hydrodynamics.sdf");

  double area = 4.0 * IGN_PI * 0.2 * 0.2;
  this->TestWorld(world, "ball", 1000.0, 0.01, area, 0.5);
}
