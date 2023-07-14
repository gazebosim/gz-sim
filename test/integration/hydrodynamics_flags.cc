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

class HydrodynamicsFlagsTest : public InternalFixture<::testing::Test>
{
  /// \brief Test a world file.
  /// \param[in] _world Path to world file.
  /// \param[in] _modelName Name of the model.
  /// \param[in] _numsteps Number of steps to run the server.
  /// \param[out] _linearVel Linear velocityies after each step.
  /// \param[out] _angularVel Linear velocityies after each step.
  public: void TestWorld(const std::string &_world,
   const std::string &_modelName, const unsigned int &_numsteps,
   std::vector<math::Vector3d> &_linearVel,
   std::vector<math::Vector3d> &_angularVel);
};

//////////////////////////////////////////////////
void HydrodynamicsFlagsTest::TestWorld(const std::string &_world,
   const std::string &_modelName, const unsigned int &_numsteps,
   std::vector<math::Vector3d> &_linearVel,
  std::vector<math::Vector3d> &_angularVel)
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
  OnConfigure([&](const Entity &_worldEntity,
              const std::shared_ptr<const sdf::Element> &/*_sdf*/,
              EntityComponentManager &_ecm,
              EventManager &/*eventMgr*/)
    {
      World world(_worldEntity);

      auto modelEntity =  world.ModelByName(_ecm, _modelName);
      EXPECT_NE(modelEntity, kNullEntity);
      model = Model(modelEntity);

      auto bodyEntity = model.LinkByName(_ecm, "base_link");
      EXPECT_NE(bodyEntity, kNullEntity);

      body = Link(bodyEntity);
      body.EnableVelocityChecks(_ecm);

      // Forces in Y and Z are needed to make the coriolis
      // force appear.
      math::Vector3d force(0.0, 1000.0, 1000.0);
      math::Vector3d torque(0.0, 0.0, 10.0);
      body.AddWorldWrench(_ecm, force, torque);

    }).
  OnPostUpdate([&](const UpdateInfo &/*_info*/,
                   const EntityComponentManager &_ecm)
    {
      auto bodyVel = body.WorldLinearVelocity(_ecm);
      ASSERT_TRUE(bodyVel);
      _linearVel.push_back(bodyVel.value());
      auto bodyAngularVel = body.WorldAngularVelocity(_ecm);
      ASSERT_TRUE(bodyAngularVel);
      _angularVel.push_back(bodyAngularVel.value());
    }).
  Finalize();

  fixture.Server()->Run(true, _numsteps, false);
  EXPECT_EQ(_numsteps, _linearVel.size());
  EXPECT_EQ(_numsteps, _angularVel.size());

  EXPECT_NE(model.Entity(), kNullEntity);
  EXPECT_NE(body.Entity(), kNullEntity);

}

/////////////////////////////////////////////////
/// This test makes sure that the linear velocity is reuduced
/// disbling the coriolis force and also when disabling the added mass.
TEST_F(HydrodynamicsFlagsTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(AddedMassCoriolisFlags))
{
  auto world = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "hydrodynamics_flags.sdf");

  unsigned int numsteps = 2000;

  std::vector<math::Vector3d> angularVels, angularCoriolisVels,
      angularAddedMassVels;
  std::vector<math::Vector3d> linearVels, linearCoriolisVels,
      linearAddedMassVels;

  this->TestWorld(world, "tethys", numsteps, linearVels, angularVels);
  this->TestWorld(world, "triton", numsteps, linearCoriolisVels,
                  angularCoriolisVels);
  this->TestWorld(world, "daphne", numsteps, linearAddedMassVels,
                  angularAddedMassVels);

  // Find out when body is moving and coriolis appears
  unsigned int starts_moving = 0;
  for (starts_moving = 0; starts_moving < numsteps; ++starts_moving)
  {
    if (linearCoriolisVels[starts_moving].Z() < linearVels[starts_moving].Z())
    {
      break;
    }
  }

  // Check the body moved
  EXPECT_LT(starts_moving, numsteps-1);

  for (unsigned int i = starts_moving; i < numsteps; ++i)
  {
    // Angular and linear velocity should be lower
    // with the produced coriolis and added mass
    EXPECT_LT(angularCoriolisVels[i].Z(), angularVels[i].Z());
    EXPECT_LT(linearCoriolisVels[i].Z(), linearVels[i].Z());
    EXPECT_LT(angularAddedMassVels[i].Z(), angularVels[i].Z());
    EXPECT_LT(linearAddedMassVels[i].Z(), linearVels[i].Z());
  }
}
