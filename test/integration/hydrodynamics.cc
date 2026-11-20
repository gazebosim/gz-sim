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

#include <cmath>
#include <map>
#include <string>
#include <vector>

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

namespace
{
/// \brief Run a world and return the pose at the end of the link named
/// "link" of each of the given models.
/// \param[in] _world World file name, in test/worlds.
/// \param[in] _steps Number of steps to run the server.
/// \param[in] _models Model names.
/// \return Link pose by model name.
std::map<std::string, math::Pose3d> AddedMassFinalPoses(
    const std::string &_world, unsigned int _steps,
    const std::vector<std::string> &_models)
{
  TestFixture fixture(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", _world));

  std::map<std::string, math::Pose3d> poses;
  fixture.
  OnPostUpdate([&](const UpdateInfo &/*_info*/,
                   const EntityComponentManager &_ecm)
    {
      World world(worldEntity(_ecm));
      for (const auto &name : _models)
      {
        Model model(world.ModelByName(_ecm, name));
        ASSERT_NE(kNullEntity, model.Entity());
        auto link = model.LinkByName(_ecm, "link");
        ASSERT_NE(kNullEntity, link);
        poses[name] = worldPose(link, _ecm);
      }
    }).
  Finalize();

  fixture.Server()->Run(true, _steps, false);
  EXPECT_EQ(_models.size(), poses.size());
  return poses;
}
}  // namespace

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

/////////////////////////////////////////////////
/// The Coriolis terms of the plugin's added mass turn a body that moves
/// diagonally broadside to its motion, the Munk moment. The body has more
/// added mass in sway than in surge and is pushed along +x and +y from rest,
/// so it turns clockwise. Kirchhoff's equations give
/// (Izz + A66) dr/dt = (A11 - A22) u v, with u and v growing as
/// g m t / (m + A).
TEST_F(HydrodynamicsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(AddedMassCoriolisTurnsBroadside))
{
  const double m{10}, a11{1}, a22{6}, a66{0.02}, izz{0.25}, gx{1}, gy{0.5};
  const double t{0.2};
  // The yaw rate integrates u v, which grows as t^2; the yaw integrates
  // that once more.
  const double uvOverT2 = gx * gy * m * m / ((m + a11) * (m + a22));
  const double yaw = (a11 - a22) * uvOverT2 * std::pow(t, 4) / 12 /
      (izz + a66);

  auto poses = AddedMassFinalPoses("hydrodynamics_added_mass.sdf", 200,
                                   {"plugin"});
  EXPECT_LT(poses["plugin"].Rot().Yaw(), 0.0);
  EXPECT_NEAR(yaw, poses["plugin"].Rot().Yaw(), 0.05 * std::abs(yaw));
}

/////////////////////////////////////////////////
/// In still water the plugin's added mass and the physics engine's
/// <fluid_added_mass> are the same model, so the two bodies move alike.
TEST_F(HydrodynamicsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(AddedMassMatchesFluidAddedMass))
{
  auto poses = AddedMassFinalPoses("hydrodynamics_added_mass.sdf", 2000,
                                   {"plugin", "native"});
  // The "native" model starts 5 m to the left of the "plugin" model.
  const math::Vector3d offset(0, 5, 0);

  // Both moved and turned well away from where they started.
  EXPECT_GT(poses["native"].Pos().Distance(offset), 1.0);
  EXPECT_GT(std::abs(poses["native"].Rot().Yaw()), 0.2);

  EXPECT_LT(poses["plugin"].Pos().Distance(
      poses["native"].Pos() - offset), 5e-3);
  EXPECT_LT((poses["plugin"].Rot().Inverse() *
             poses["native"].Rot()).Euler().Length(), 5e-3);
}

/////////////////////////////////////////////////
/// A link with <fluid_added_mass> gets its Coriolis terms from the physics
/// engine. When the plugin is also given the added mass parameters, with
/// its added mass force disabled, it must not apply those terms a second
/// time: the body moves like the one with <fluid_added_mass> alone.
TEST_F(HydrodynamicsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(FluidAddedMassCoriolisNotCountedTwice))
{
  auto poses = AddedMassFinalPoses("hydrodynamics_added_mass.sdf", 2000,
                                   {"native", "both"});
  // The "both" model starts 5 m to the left of the "native" model.
  const math::Vector3d offset(0, 5, 0);

  EXPECT_GT(std::abs(poses["native"].Rot().Yaw()), 0.2);
  EXPECT_LT(poses["native"].Pos().Distance(
      poses["both"].Pos() - offset), 1e-6);
  EXPECT_LT((poses["native"].Rot().Inverse() *
             poses["both"].Rot()).Euler().Length(), 1e-6);
}

/////////////////////////////////////////////////
/// In a current the physics engine's added mass takes its Coriolis terms
/// against the ground. The plugin corrects them, so the body ends within
/// centimeters of the one whose added mass is in the plugin; without the
/// correction they end a quarter of a meter apart. What is left is the
/// current's turning in the body frame, which the correction leaves out.
TEST_F(HydrodynamicsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(FluidAddedMassCorrectedInCurrent))
{
  auto poses = AddedMassFinalPoses(
      "hydrodynamics_added_mass_current.sdf", 4000, {"plugin", "native"});
  const math::Vector3d offset(0, 5, 0);

  EXPECT_GT(poses["native"].Pos().Distance(offset), 1.0);
  EXPECT_LT(poses["plugin"].Pos().Distance(
      poses["native"].Pos() - offset), 0.03);
}
