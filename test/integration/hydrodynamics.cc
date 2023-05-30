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

#include <gz/msgs/double.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/World.hh"

#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

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

  public: math::Vector3d defaultForce{0, 0, 10.0};
};

//////////////////////////////////////////////////
std::vector<math::Vector3d> HydrodynamicsTest::TestWorld(
  const std::string &_world, const std::string &_namespace)
{
  // Maximum verbosity for debugging
  common::Console::SetVerbosity(4);

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
      body.AddWorldForce(_ecm, this->defaultForce);
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
TEST_F(HydrodynamicsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(VelocityTestinOil))
{
  auto world = common::joinPaths(std::string(PROJECT_BINARY_PATH),
      "test", "worlds", "hydrodynamics.sdf");

  auto sphere1Vels = this->TestWorld(world, "sphere1");
  auto sphere2Vels = this->TestWorld(world, "sphere2");

  auto whenSphere1ExceedsSphere2Vel = 2000;

  for (unsigned int i = 0; i < 1000; ++i)
  {
    // Sanity check
    EXPECT_FLOAT_EQ(0.0, sphere1Vels[i].X());
    EXPECT_FLOAT_EQ(0.0, sphere1Vels[i].Y());
    EXPECT_FLOAT_EQ(0.0, sphere2Vels[i].X());
    EXPECT_FLOAT_EQ(0.0, sphere2Vels[i].Y());

    // Expect sphere1 to fall faster than sphere 2 as no hydro
    // drag is applied to it.
    EXPECT_LE(sphere1Vels[i].Z(), sphere2Vels[i].Z());
    if(sphere1Vels[i].Z() < sphere2Vels[i].Z()
      &&  whenSphere1ExceedsSphere2Vel > 1000)
    {
      // Mark this as the time when velocity of sphere1 exceeds sphere 2
      whenSphere1ExceedsSphere2Vel = i;
    }
    if (i > 900)
    {
      // Expect for the velocity to stabilize
      EXPECT_NEAR(sphere1Vels[i-1].Z(), sphere1Vels[i].Z(), 1e-6);
      EXPECT_NEAR(sphere2Vels[i-1].Z(), sphere2Vels[i].Z(), 1e-6);
    }
  }
  EXPECT_LT(whenSphere1ExceedsSphere2Vel, 500);
}

/////////////////////////////////////////////////
/// This test makes sure that the transforms of the hydrodynamics
/// plugin are correct by comparing 3 cylinders in different
/// positions and orientations.
TEST_F(HydrodynamicsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(TransformsTestinWater))
{
  auto world = common::joinPaths(std::string(PROJECT_BINARY_PATH),
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

/////////////////////////////////////////////////
/// This tests the current. A current of (1, 0, 0) is loaded in via a csv file
TEST_F(HydrodynamicsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(TransformsTestIn))
{
  this->defaultForce = math::Vector3d(0, 0, 0);
  auto world = common::joinPaths(std::string(PROJECT_BINARY_PATH),
      "test", "worlds", "hydrodynamics.sdf");

  auto sphereVel = this->TestWorld(world, "sphere_current");

  for (unsigned int i = 990; i < 1000; ++i)
  {
    // Expect for the velocity to stabilize
    EXPECT_NEAR(sphereVel[i-1].Z(), sphereVel[i].Z(), 1e-6);
    EXPECT_NEAR(sphereVel[i-1].Y(), sphereVel[i].Y(), 1e-6);
    EXPECT_NEAR(sphereVel[i-1].X(), sphereVel[i].X(), 1e-3);

    // Given current of  (1,0,0), vehicle should move in similar direction.
    EXPECT_NEAR(sphereVel[i-1].Z(), 0, 1e-6);
    EXPECT_NEAR(sphereVel[i-1].Y(), 0, 1e-6);
    EXPECT_GT(sphereVel[i-1].X(), 0);
  }
}
