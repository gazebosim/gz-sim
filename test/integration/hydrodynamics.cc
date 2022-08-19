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
  public: std::vector<math::Vector3d> TestWorld(const std::string &_world,
   const std::string &_namespace);
};

//////////////////////////////////////////////////
std::vector<math::Vector3d> HydrodynamicsTest::TestWorld(const std::string &_world,
    const std::string &_namespace)
{
  // Maximum verbosity for debugging
  ignition::common::Console::SetVerbosity(4);

  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(_world);

  TestFixture fixture(serverConfig);

  Model model;
  Link body;
  std::vector<math::Vector3d> bodyVels;
  fixture.
  OnConfigure(
    [&](const Entity &_worldEntity,
      const std::shared_ptr<const sdf::Element> &/*_sdf*/,
      EntityComponentManager &_ecm,
      EventManager &/*eventMgr*/)
    {
      World world(_worldEntity);

      auto modelEntity =  world.ModelByName(_ecm, _namespace);
      EXPECT_NE(modelEntity, kNullEntity);
      model = Model(modelEntity);

      auto bodyEntity = model.LinkByName(_ecm, _namespace + "_link");
      EXPECT_NE(bodyEntity, kNullEntity);

      body = Link(bodyEntity);
      body.EnableVelocityChecks(_ecm);

      // Add force
      math::Vector3d force(0, 0, 10.0);
      body.AddWorldForce(_ecm, force);
    }).
  OnPostUpdate([&](const UpdateInfo &/*_info*/,
                            const EntityComponentManager &_ecm)
    {
      auto bodyVel = body.WorldLinearVelocity(_ecm);
      ASSERT_TRUE(bodyVel);
      bodyVels.push_back(bodyVel.value());
    }).
  Finalize();

  fixture.Server()->Run(true, 1000, false);
  EXPECT_EQ(1000u, bodyVels.size());

  EXPECT_NE(model.Entity(), kNullEntity);
  EXPECT_NE(body.Entity(), kNullEntity);

  return bodyVels;
}

/////////////////////////////////////////////////
/// This test evaluates whether the hydrodynamic plugin affects the motion
/// of the body when a force is applied.
TEST_F(HydrodynamicsTest, VelocityTestinOil)
{
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "hydrodynamics.sdf");

  auto sphere1Vels = this->TestWorld(world, "sphere1");
  auto sphere2Vels = this->TestWorld(world, "sphere2");

  for (unsigned int i = 0; i < 1000; ++i)
  {
    // Sanity check
    EXPECT_FLOAT_EQ(0.0, sphere1Vels[i].X());
    EXPECT_FLOAT_EQ(0.0, sphere1Vels[i].Y());
    EXPECT_FLOAT_EQ(0.0, sphere2Vels[i].X());
    EXPECT_FLOAT_EQ(0.0, sphere2Vels[i].Y());

    // Wait a couple of iterations for the body to move
    if(i > 4)
    {
      EXPECT_LT(sphere1Vels[i].Z(), sphere2Vels[i].Z());

      if (i > 900)
      {
        // Expect for the velocity to stabilize
        EXPECT_NEAR(sphere1Vels[i-1].Z(), sphere1Vels[i].Z(), 1e-6);
        EXPECT_NEAR(sphere2Vels[i-1].Z(), sphere2Vels[i].Z(), 1e-6);
      }
    }
  }
}

/////////////////////////////////////////////////
/// This test makes sure that the transforms of the hydrodynamics plugin 
/// are correct by comparing 3 cylinders in different positions and orientations. 
TEST_F(HydrodynamicsTest, TransformsTestinWater)
{
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "hydrodynamics.sdf");

  auto cylinder1Vels = this->TestWorld(world, "cylinder1");
  auto cylinder2Vels = this->TestWorld(world, "cylinder2");
  auto cylinder3Vels = this->TestWorld(world, "cylinder3");

  for (unsigned int i = 900; i < 1000; ++i)
  {
    // Expect for the velocity to stabilize
    EXPECT_NEAR(cylinder1Vels[i-1].Z(), cylinder1Vels[i].Z(), 1e-6);
    EXPECT_NEAR(cylinder2Vels[i-1].Z(), cylinder2Vels[i].Z(), 1e-6);
    EXPECT_NEAR(cylinder3Vels[i-1].Z(), cylinder3Vels[i].Z(), 1e-6);

    // Expect for final velocities to be similar
    EXPECT_NEAR(cylinder1Vels[i].Z(), cylinder2Vels[i].Z(), 1e-4);
    EXPECT_NEAR(cylinder2Vels[i].Z(), cylinder3Vels[i].Z(), 1e-4);
  }
}
